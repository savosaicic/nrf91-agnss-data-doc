#include <date_time.h>
#include <math.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/net/http/client.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>

#include <cJSON.h>

#define M_PI 3.141593

#define AGNSS_SERVER_HOST  "42group.fr"
#define AGNSS_SERVER_PORT  4242
#define AGNSS_SERVER_PATH  "/?types=ephe,alm,iono,utc,loc"
#define AGNSS_HTTP_TIMEOUT 10000 /* ms */

#define GPS_MAX_PRN    32
#define GPS_EPOCH_UNIX 315964800 /* Jan 6 1980 00:00:00 UTC */

#define AGNSS_BUF_SIZE (64 * 1024)

LOG_MODULE_REGISTER(nrf91_gnss);

static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(time_synced, 0, 1);
static K_SEM_DEFINE(agnss_req_ready, 0, 1);
static K_SEM_DEFINE(cell_meas_ready, 0, 1);

static struct nrf_modem_gnss_agnss_data_frame gnss_agnss_req;
static struct nrf_modem_gnss_pvt_data_frame   pvt_data;

static struct lte_lc_cells_info cell_info = {0};

static int64_t gnss_start_time;
static bool    first_fix = false;

static char   rx_buf[2048];
static char   agnss_data_buf[AGNSS_BUF_SIZE];
static size_t agnss_data_len;

static int32_t rad_to_sc_scaled(double radians, int scale_bits)
{
  return (int32_t)round((radians / M_PI) * ((double)(1LL << scale_bits)));
}

static int64_t double_to_scaled(double value, int scale_bits)
{
  return (int64_t)round(value * ((double)(1LL << scale_bits)));
}

/**
 * Find a top-level JSON key and return a pointer to the value that follows it.
 * For example, given key = "utc", returns a pointer to the '{' of the value.
 * Returns NULL if the key is not found.
 */
static const char *find_json_key(const char *json, size_t len, const char *key)
{
  char pattern[64];
  int  plen = snprintk(pattern, sizeof(pattern), "\"%s\"", key);

  const char *end = json + len - plen;

  for (const char *p = json; p < end; p++) {
    if (memcmp(p, pattern, plen) == 0) {
      const char *val     = p + plen;
      const char *buf_end = json + len;

      while (val < buf_end && (*val == ' ' || *val == ':' || *val == '\t' ||
                               *val == '\n' || *val == '\r')) {
        val++;
      }
      return (val < buf_end) ? val : NULL;
    }
  }
  return NULL;
}

/**
 * Given a pointer to a '{' or '[', find the matching closing bracket.
 * Handles nested braces/brackets.  Returns pointer to the closing char,
 * or NULL on error.
 */
static const char *find_matching_bracket(const char *start, const char *buf_end)
{
  char open      = *start;
  char close     = (open == '{') ? '}' : ']';
  int  depth     = 0;
  bool in_string = false;

  for (const char *p = start; p < buf_end; p++) {
    if (*p == '"' && (p == start || *(p - 1) != '\\')) {
      in_string = !in_string;
      continue;
    }
    if (in_string) {
      continue;
    }
    if (*p == open) {
      depth++;
    } else if (*p == close) {
      depth--;
      if (depth == 0) {
        return p;
      }
    }
  }
  return NULL;
}

/**
 * Parse a single JSON object slice from the raw buffer using cJSON.
 * The caller provides a pointer to '{' and the closing '}'.
 * Returns a cJSON* that must be freed by the caller.
 */
static cJSON *parse_json_slice(const char *start, const char *end)
{
  size_t slice_len = (end - start) + 1;

  return cJSON_ParseWithLength(start, slice_len);
}

static int inject_one_almanac(cJSON *sat)
{
  struct nrf_modem_gnss_agnss_gps_data_almanac alm = {0};

  alm.sv_id     = (uint8_t)cJSON_GetObjectItem(sat, "prn")->valueint;
  alm.wn        = (uint8_t)(cJSON_GetObjectItem(sat, "week")->valueint % 256);
  alm.toa       = (uint8_t)(cJSON_GetObjectItem(sat, "toa")->valueint / 4096);
  alm.ioda      = (uint8_t)cJSON_GetObjectItem(sat, "ioda")->valueint;
  alm.sv_health = (uint8_t)cJSON_GetObjectItem(sat, "health")->valueint;

  alm.e =
    (uint16_t)double_to_scaled(cJSON_GetObjectItem(sat, "e")->valuedouble, 21);
  alm.sqrt_a = (uint32_t)double_to_scaled(
    cJSON_GetObjectItem(sat, "sqrt_a")->valuedouble, 11);

  alm.delta_i = (int16_t)rad_to_sc_scaled(
    cJSON_GetObjectItem(sat, "delta_i")->valuedouble, 19);
  alm.omega_dot = (int16_t)rad_to_sc_scaled(
    cJSON_GetObjectItem(sat, "omega_dot")->valuedouble, 38);
  alm.omega0 = (int32_t)rad_to_sc_scaled(
    cJSON_GetObjectItem(sat, "omega0")->valuedouble, 23);
  alm.w = (int32_t)rad_to_sc_scaled(
    cJSON_GetObjectItem(sat, "omega")->valuedouble, 23);
  alm.m0 =
    (int32_t)rad_to_sc_scaled(cJSON_GetObjectItem(sat, "m0")->valuedouble, 23);

  alm.af0 =
    (int16_t)double_to_scaled(cJSON_GetObjectItem(sat, "af0")->valuedouble, 20);
  alm.af1 =
    (int16_t)double_to_scaled(cJSON_GetObjectItem(sat, "af1")->valuedouble, 38);

  int err = nrf_modem_gnss_agnss_write(&alm, sizeof(alm),
                                       NRF_MODEM_GNSS_AGNSS_GPS_ALMANAC);
  if (err) {
    LOG_ERR("Almanac inject failed for PRN %u: %d", alm.sv_id, err);
  }
  return err;
}

static int inject_one_ephemeris(cJSON *sat)
{
  struct nrf_modem_gnss_agnss_gps_data_ephemeris eph = {0};

  eph.sv_id  = (uint8_t)cJSON_GetObjectItem(sat, "prn")->valueint;
  eph.health = (uint8_t)cJSON_GetObjectItem(sat, "health")->valueint;
  eph.iodc   = (uint16_t)cJSON_GetObjectItem(sat, "iodc")->valueint;

  eph.toc = (uint16_t)(cJSON_GetObjectItem(sat, "toc")->valueint / 16);
  eph.toe = (uint16_t)(cJSON_GetObjectItem(sat, "toe")->valueint / 16);

  eph.af0 =
    (int32_t)double_to_scaled(cJSON_GetObjectItem(sat, "af0")->valuedouble, 31);
  eph.af1 =
    (int16_t)double_to_scaled(cJSON_GetObjectItem(sat, "af1")->valuedouble, 43);
  eph.af2 =
    (int8_t)double_to_scaled(cJSON_GetObjectItem(sat, "af2")->valuedouble, 55);

  eph.tgd =
    (int8_t)double_to_scaled(cJSON_GetObjectItem(sat, "tgd")->valuedouble, 31);

  eph.sqrt_a = (uint32_t)double_to_scaled(
    cJSON_GetObjectItem(sat, "sqrt_a")->valuedouble, 19);
  eph.e =
    (uint32_t)double_to_scaled(cJSON_GetObjectItem(sat, "e")->valuedouble, 33);

  eph.i0 = rad_to_sc_scaled(cJSON_GetObjectItem(sat, "i0")->valuedouble, 31);
  eph.omega0 =
    rad_to_sc_scaled(cJSON_GetObjectItem(sat, "omega0")->valuedouble, 31);
  eph.w  = rad_to_sc_scaled(cJSON_GetObjectItem(sat, "omega")->valuedouble, 31);
  eph.m0 = rad_to_sc_scaled(cJSON_GetObjectItem(sat, "m0")->valuedouble, 31);

  eph.delta_n = (int16_t)rad_to_sc_scaled(
    cJSON_GetObjectItem(sat, "delta_n")->valuedouble, 43);
  eph.omega_dot =
    (int32_t)((cJSON_GetObjectItem(sat, "omega_dot")->valuedouble) / M_PI *
              ((double)(1LL << 43)));
  eph.idot = (int16_t)rad_to_sc_scaled(
    cJSON_GetObjectItem(sat, "idot")->valuedouble, 43);

  eph.cuc =
    (int16_t)double_to_scaled(cJSON_GetObjectItem(sat, "cuc")->valuedouble, 29);
  eph.cus =
    (int16_t)double_to_scaled(cJSON_GetObjectItem(sat, "cus")->valuedouble, 29);
  eph.crc =
    (int16_t)double_to_scaled(cJSON_GetObjectItem(sat, "crc")->valuedouble, 5);
  eph.crs =
    (int16_t)double_to_scaled(cJSON_GetObjectItem(sat, "crs")->valuedouble, 5);
  eph.cic =
    (int16_t)double_to_scaled(cJSON_GetObjectItem(sat, "cic")->valuedouble, 29);
  eph.cis =
    (int16_t)double_to_scaled(cJSON_GetObjectItem(sat, "cis")->valuedouble, 29);

  eph.ura     = 0;
  eph.fit_int = 0;

  int err = nrf_modem_gnss_agnss_write(&eph, sizeof(eph),
                                       NRF_MODEM_GNSS_AGNSS_GPS_EPHEMERIDES);
  if (err) {
    LOG_ERR("Ephemeris inject failed for PRN %u: %d", eph.sv_id, err);
  }
  return err;
}

static int inject_klobuchar(cJSON *iono)
{
  cJSON *alpha = cJSON_GetObjectItem(iono, "alpha");
  cJSON *beta  = cJSON_GetObjectItem(iono, "beta");

  if (!alpha || !beta) {
    LOG_ERR("Ionosphere alpha/beta arrays missing");
    return -EINVAL;
  }

  static const int alpha_scales[] = {30, 27, 24, 24};
  static const int beta_scales[]  = {11, 14, 16, 16};

  struct nrf_modem_gnss_agnss_data_klobuchar klob      = {0};
  int8_t                                    *alpha_dst = &klob.alpha0;
  int8_t                                    *beta_dst  = &klob.beta0;

  for (int i = 0; i < 4; i++) {
    double a     = cJSON_GetArrayItem(alpha, i)->valuedouble;
    alpha_dst[i] = (int8_t)round(a * ((double)(1LL << alpha_scales[i])));

    double b    = cJSON_GetArrayItem(beta, i)->valuedouble;
    beta_dst[i] = (int8_t)round(b / ((double)(1LL << beta_scales[i])));
  }

  int err = nrf_modem_gnss_agnss_write(
    &klob, sizeof(klob), NRF_MODEM_GNSS_AGNSS_KLOBUCHAR_IONOSPHERIC_CORRECTION);
  if (err) {
    LOG_ERR("Klobuchar inject failed: %d", err);
    return err;
  }

  LOG_INF("Klobuchar ionosphere injected");
  return 0;
}

static int inject_utc(cJSON *utc_obj)
{
  struct nrf_modem_gnss_agnss_gps_data_utc utc = {0};

  double a0    = cJSON_GetObjectItem(utc_obj, "a0")->valuedouble;
  double a1    = cJSON_GetObjectItem(utc_obj, "a1")->valuedouble;
  int    tot   = cJSON_GetObjectItem(utc_obj, "tot")->valueint;
  int    wnt   = cJSON_GetObjectItem(utc_obj, "wnt")->valueint;
  int    dt_ls = cJSON_GetObjectItem(utc_obj, "dt_ls")->valueint;

  utc.a0        = (int32_t)double_to_scaled(a0, 30);
  utc.a1        = (int32_t)double_to_scaled(a1, 50);
  utc.tot       = (uint8_t)(tot / 4096);
  utc.wn_t      = (uint8_t)(wnt % 256);
  utc.delta_tls = (int8_t)dt_ls;

  utc.wn_lsf     = utc.wn_t;
  utc.dn         = 1;
  utc.delta_tlsf = utc.delta_tls;

  int err = nrf_modem_gnss_agnss_write(&utc, sizeof(utc),
                                       NRF_MODEM_GNSS_AGNSS_GPS_UTC_PARAMETERS);
  if (err) {
    LOG_ERR("UTC inject failed: %d", err);
    return err;
  }

  LOG_INF("UTC parameters injected");
  return 0;
}

static int inject_location(cJSON *loc_obj)
{
  double lat = cJSON_GetObjectItem(loc_obj, "latitude")->valuedouble;
  double lon = cJSON_GetObjectItem(loc_obj, "longitude")->valuedouble;
  double alt = cJSON_GetObjectItem(loc_obj, "altitude")->valuedouble;

  struct nrf_modem_gnss_agnss_data_location loc = {0};

  loc.latitude  = (int32_t)round(lat * ((double)(1 << 23) / 90.0));
  loc.longitude = (int32_t)round(lon * ((double)(1 << 24) / 360.0));
  loc.altitude  = (int16_t)alt;

  loc.unc_semimajor     = 48;
  loc.unc_semiminor     = 48;
  loc.orientation_major = 0;
  loc.unc_altitude      = 255;
  loc.confidence        = 68;

  int err = nrf_modem_gnss_agnss_write(&loc, sizeof(loc),
                                       NRF_MODEM_GNSS_AGNSS_LOCATION);
  if (err) {
    LOG_ERR("Location inject failed: %d", err);
    return err;
  }

  LOG_INF("Location injected: lat=%.3f lon=%.3f", lat, lon);
  return 0;
}

static int inject_system_time(cJSON *dataset, int leap_seconds)
{
  int64_t unix_ts =
    (int64_t)cJSON_GetObjectItem(dataset, "timestamp")->valuedouble;
  int64_t gps_time = unix_ts - GPS_EPOCH_UNIX + leap_seconds;

  struct nrf_modem_gnss_agnss_gps_data_system_time_and_sv_tow clk = {0};

  clk.date_day     = (uint16_t)(gps_time / 86400);
  clk.time_full_s  = (uint32_t)(gps_time % 86400);
  clk.time_frac_ms = 0;
  clk.sv_mask      = 0;

  int err = nrf_modem_gnss_agnss_write(
    &clk, sizeof(clk), NRF_MODEM_GNSS_AGNSS_GPS_SYSTEM_CLOCK_AND_TOWS);
  if (err) {
    LOG_ERR("System clock inject failed: %d", err);
    return err;
  }

  LOG_INF("System clock injected: day=%u tod=%u s", clk.date_day,
          clk.time_full_s);
  return 0;
}

static int parse_and_inject_agnss_data(const char *buf, size_t len,
                                       uint32_t data_flags)
{
  const char *buf_end = buf + len;
  int         err;
  int         ret = 0;

  /* Ephemerides */
  if (data_flags & BIT(NRF_MODEM_GNSS_AGNSS_GPS_EPHEMERIDES - 1)) {
    const char *eph_arr = find_json_key(buf, len, "ephemeris");

    if (eph_arr && *eph_arr == '[') {
      const char *p              = eph_arr + 1;
      int         injected       = 0;
      int         total          = 0;
      uint32_t    integrity_mask = 0;

      while (p < buf_end) {
        while (p < buf_end && (*p == ' ' || *p == ',' || *p == '\n' ||
                               *p == '\r' || *p == '\t')) {
          p++;
        }
        if (p >= buf_end || *p == ']') {
          break;
        }
        if (*p != '{') {
          break;
        }

        const char *obj_end = find_matching_bracket(p, buf_end);
        if (!obj_end) {
          LOG_ERR("Malformed ephemeris object");
          ret = -EINVAL;
          break;
        }

        cJSON *sat = parse_json_slice(p, obj_end);

        if (sat) {
          total++;
          err = inject_one_ephemeris(sat);
          if (!err) {
            injected++;
          }

          cJSON *prn_j  = cJSON_GetObjectItem(sat, "prn");
          cJSON *hlth_j = cJSON_GetObjectItem(sat, "health");
          if (prn_j && hlth_j) {
            int prn  = prn_j->valueint;
            int hlth = hlth_j->valueint;
            if (prn >= 1 && prn <= 32 && hlth != 0) {
              integrity_mask |= BIT(prn - 1);
            }
          }
          cJSON_Delete(sat);
        }
        p = obj_end + 1;
      }
      LOG_INF("Injected %d/%d ephemerides", injected, total);

      /* Integrity (derived from health) */
      if (data_flags & BIT(NRF_MODEM_GNSS_AGPS_INTEGRITY - 1)) {
        struct nrf_modem_gnss_agps_data_integrity integ = {.integrity_mask =
                                                             integrity_mask};

        err = nrf_modem_gnss_agnss_write(&integ, sizeof(integ),
                                         NRF_MODEM_GNSS_AGPS_INTEGRITY);
        if (err) {
          LOG_ERR("Integrity inject failed: %d", err);
          ret = err;
        } else {
          LOG_INF("Integrity mask injected: "
                  "0x%08X",
                  integrity_mask);
        }
      }
    } else {
      LOG_WRN("Ephemeris array not found in response");
    }
  }

  /* Almanac */
  if (data_flags & BIT(NRF_MODEM_GNSS_AGNSS_GPS_ALMANAC - 1)) {
    const char *alm_arr = find_json_key(buf, len, "almanac");

    if (alm_arr && *alm_arr == '[') {
      const char *p        = alm_arr + 1;
      int         injected = 0;
      int         total    = 0;

      while (p < buf_end) {
        while (p < buf_end && (*p == ' ' || *p == ',' || *p == '\n' ||
                               *p == '\r' || *p == '\t')) {
          p++;
        }
        if (p >= buf_end || *p == ']') {
          break;
        }
        if (*p != '{') {
          break;
        }

        const char *obj_end = find_matching_bracket(p, buf_end);
        if (!obj_end) {
          LOG_ERR("Malformed almanac object");
          ret = -EINVAL;
          break;
        }

        cJSON *sat = parse_json_slice(p, obj_end);

        if (sat) {
          total++;
          err = inject_one_almanac(sat);
          if (!err) {
            injected++;
          }
          cJSON_Delete(sat);
        }
        p = obj_end + 1;
      }
      LOG_INF("Injected %d/%d almanacs", injected, total);
    } else {
      LOG_WRN("Almanac array not found in response");
    }
  }

  /* Ionosphere (Klobuchar) */
  if (data_flags &
      BIT(NRF_MODEM_GNSS_AGNSS_KLOBUCHAR_IONOSPHERIC_CORRECTION - 1)) {
    const char *iono_start = find_json_key(buf, len, "ionosphere");

    if (iono_start && *iono_start == '{') {
      const char *iono_end = find_matching_bracket(iono_start, buf_end);
      if (iono_end) {
        cJSON *iono = parse_json_slice(iono_start, iono_end);
        if (iono) {
          err = inject_klobuchar(iono);
          if (err) {
            ret = err;
          }
          cJSON_Delete(iono);
        }
      }
    } else {
      LOG_WRN("Ionosphere object not found");
    }
  }

  /* UTC parameters */
  int leap_seconds = 18;

  if (data_flags & BIT(NRF_MODEM_GNSS_AGNSS_GPS_UTC_PARAMETERS - 1)) {
    const char *utc_start = find_json_key(buf, len, "utc");

    if (utc_start && *utc_start == '{') {
      const char *utc_end = find_matching_bracket(utc_start, buf_end);
      if (utc_end) {
        cJSON *utc_obj = parse_json_slice(utc_start, utc_end);
        if (utc_obj) {
          cJSON *dt = cJSON_GetObjectItem(utc_obj, "dt_ls");
          if (dt) {
            leap_seconds = dt->valueint;
          }
          err = inject_utc(utc_obj);
          if (err) {
            ret = err;
          }
          cJSON_Delete(utc_obj);
        }
      }
    } else {
      LOG_WRN("UTC object not found");
    }
  }

  /* Dataset (location + system clock) */
  const char *ds_start = find_json_key(buf, len, "dataset");

  if (ds_start && *ds_start == '{') {
    const char *ds_end = find_matching_bracket(ds_start, buf_end);
    if (ds_end) {
      cJSON *dataset = parse_json_slice(ds_start, ds_end);

      if (dataset) {
        if (data_flags & BIT(NRF_MODEM_GNSS_AGNSS_LOCATION - 1)) {
          cJSON *loc = cJSON_GetObjectItem(dataset, "location");
          if (loc) {
            err = inject_location(loc);
            if (err) {
              ret = err;
            }
          }
        }
        if (data_flags &
            BIT(NRF_MODEM_GNSS_AGNSS_GPS_SYSTEM_CLOCK_AND_TOWS - 1)) {
          err = inject_system_time(dataset, leap_seconds);
          if (err) {
            ret = err;
          }
        }
        cJSON_Delete(dataset);
      }
    }
  }

  return ret;
}

static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt)
{
  LOG_INF("Latitude:       %.06f", pvt->latitude);
  LOG_INF("Longitude:      %.06f", pvt->longitude);
  LOG_INF("Altitude:       %.01f m", (double)pvt->altitude);
  LOG_INF("Time (UTC):     %02u:%02u:%02u.%03u", pvt->datetime.hour,
          pvt->datetime.minute, pvt->datetime.seconds, pvt->datetime.ms);
}

static void gnss_event_handler(int evt)
{
  int err;

  switch (evt) {
  case NRF_MODEM_GNSS_EVT_AGNSS_REQ:
    err = nrf_modem_gnss_read(&gnss_agnss_req, sizeof(gnss_agnss_req),
                              NRF_MODEM_GNSS_DATA_AGNSS_REQ);
    if (err) {
      LOG_ERR("Failed to read A-GNSS request: %d", err);
      break;
    }
    LOG_INF("A-GNSS request from modem: data_flags=0x%08X",
            gnss_agnss_req.data_flags);
    k_sem_give(&agnss_req_ready);
    break;

  case NRF_MODEM_GNSS_EVT_PVT:
    err =
      nrf_modem_gnss_read(&pvt_data, sizeof(pvt_data), NRF_MODEM_GNSS_DATA_PVT);
    if (err) {
      LOG_ERR("nrf_modem_gnss_read failed, err %d", err);
      return;
    }

    int num_satellites = 0;

    for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++) {
      if (pvt_data.sv[i].signal != 0) {
        LOG_INF("sv: %d, cn0: %d, signal: %d", pvt_data.sv[i].sv,
                pvt_data.sv[i].cn0, pvt_data.sv[i].signal);
        num_satellites++;
      }
    }
    LOG_INF("Searching... satellites in view: %d", num_satellites);

    if (pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
      print_fix_data(&pvt_data);
      if (!first_fix) {
        LOG_INF("TTFF: %2.1lld s", (k_uptime_get() - gnss_start_time) / 1000);
        first_fix = true;
      }
    }

    if (pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_DEADLINE_MISSED) {
      LOG_INF("GNSS Blocked by LTE activity");
    } else if (pvt_data.flags &
               NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME) {
      LOG_INF("Insufficient GNSS time window");
    }
    break;

  case NRF_MODEM_GNSS_EVT_PERIODIC_WAKEUP:
    LOG_INF("GNSS has woken up");
    break;

  case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_FIX:
    LOG_INF("GNSS enters sleep after fix");
    break;

  default:
    break;
  }
}

static void lte_handler(const struct lte_lc_evt *const evt)
{
  switch (evt->type) {
  case LTE_LC_EVT_NW_REG_STATUS:
    if (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME &&
        evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING) {
      break;
    }
    LOG_INF("LTE registered: %s",
            evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ? "home network"
                                                                : "roaming");
    k_sem_give(&lte_connected);
    break;

  case LTE_LC_EVT_RRC_UPDATE:
    LOG_INF("RRC mode: %s",
            evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "connected" : "idle");
    break;

  case LTE_LC_EVT_PSM_UPDATE:
    LOG_INF("PSM parameter update: TAU: %d s, Active time: %d s",
            evt->psm_cfg.tau, evt->psm_cfg.active_time);
    if (evt->psm_cfg.active_time == -1) {
      LOG_ERR("Network rejected PSM parameters. "
              "Failed to enable PSM");
    }
    break;

  case LTE_LC_EVT_EDRX_UPDATE:
    LOG_INF("eDRX parameter update: eDRX: %.2f s, PTW: %.2f s",
            (double)evt->edrx_cfg.edrx, (double)evt->edrx_cfg.ptw);
    break;

  case LTE_LC_EVT_NEIGHBOR_CELL_MEAS:
    if (evt->cells_info.current_cell.id != LTE_LC_CELL_EUTRAN_ID_INVALID) {
      memcpy(&cell_info, &evt->cells_info, sizeof(cell_info));
    }
    k_sem_give(&cell_meas_ready);
    break;

  default:
    break;
  }
}

static void date_time_event_handler(const struct date_time_evt *evt)
{
  if (evt->type == DATE_TIME_OBTAINED_MODEM ||
      evt->type == DATE_TIME_OBTAINED_NTP ||
      evt->type == DATE_TIME_OBTAINED_EXT) {
    LOG_INF("Time synced (source: %d)", evt->type);
    k_sem_give(&time_synced);
  }
}

static int modem_configure(void)
{
  int err;

  LOG_INF("Initializing modem library");
  err = nrf_modem_lib_init();
  if (err) {
    LOG_ERR("Failed to initialize the modem library, error: %d", err);
    return err;
  }

  date_time_register_handler(date_time_event_handler);

  err = lte_lc_psm_req(true);
  if (err) {
    LOG_ERR("lte_lc_psm_req, error: %d", err);
  }
  err = lte_lc_edrx_req(true);
  if (err) {
    LOG_ERR("lte_lc_edrx_req, error: %d", err);
  }

  LOG_INF("Connecting to LTE network");
  err = lte_lc_connect_async(lte_handler);
  if (err) {
    LOG_ERR("lte_lc_connect_async failed: %d", err);
    return err;
  }

  k_sem_take(&lte_connected, K_FOREVER);
  LOG_INF("Connected to LTE network");

  LOG_INF("Waiting for time sync...");
  err = k_sem_take(&time_synced, K_SECONDS(30));
  if (err) {
    LOG_ERR("Timed out waiting for time sync");
    return -ETIMEDOUT;
  }
  return 0;
}

static int http_response_cb(struct http_response *rsp,
                            enum http_final_call final_data, void *user_data)
{
  if (rsp->data_len > 0 && rsp->body_frag_start) {
    size_t frag_len = rsp->body_frag_len;
    size_t offset   = agnss_data_len;

    if (offset + frag_len > sizeof(agnss_data_buf)) {
      LOG_ERR("A-GNSS response exceeds buffer (%zu > %zu)", offset + frag_len,
              sizeof(agnss_data_buf));
      return 0;
    }
    memcpy(agnss_data_buf + offset, rsp->body_frag_start, frag_len);
    agnss_data_len += frag_len;
  }
  return 0;
}

static int agnss_request_and_inject(void)
{
  int                                    err;
  struct nrf_modem_gnss_agnss_data_frame req = {0};

  err = k_sem_take(&agnss_req_ready, K_SECONDS(10));
  if (err == 0) {
    LOG_INF("Using modem A-GNSS request: data_flags=0x%08X",
            gnss_agnss_req.data_flags);
    memcpy(&req, &gnss_agnss_req, sizeof(req));
  } else {
    LOG_WRN("EVT_AGNSS_REQ not received, falling back to "
            "expiry flags");

    struct nrf_modem_gnss_agnss_expiry expiry = {0};

    err = nrf_modem_gnss_agnss_expiry_get(&expiry);
    if (err) {
      LOG_ERR("Failed to get A-GNSS expiry: %d", err);
      return err;
    }
    if (!expiry.data_flags) {
      LOG_INF("A-GNSS data still valid, skipping request");
      return 0;
    }
    req.data_flags             = expiry.data_flags;
    req.system_count           = 2;
    req.system[0].system_id    = NRF_MODEM_GNSS_SYSTEM_GPS;
    req.system[0].sv_mask_ephe = 0xFFFFFFFF;
    req.system[0].sv_mask_alm  = 0xFFFFFFFF;
    req.system[1].system_id    = NRF_MODEM_GNSS_SYSTEM_QZSS;
    req.system[1].sv_mask_ephe = 0x3FF;
    req.system[1].sv_mask_alm  = 0x3FF;
  }

  char port_str[8];

  snprintk(port_str, sizeof(port_str), "%d", AGNSS_SERVER_PORT);

  struct zsock_addrinfo hints = {
    .ai_family   = AF_INET,
    .ai_socktype = SOCK_STREAM,
  };
  struct zsock_addrinfo *res = NULL;

  err = zsock_getaddrinfo(AGNSS_SERVER_HOST, port_str, &hints, &res);
  if (err) {
    LOG_ERR("DNS lookup failed for %s: %d", AGNSS_SERVER_HOST, err);
    return -EHOSTUNREACH;
  }

  int sock = zsock_socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if (sock < 0) {
    LOG_ERR("Failed to create socket: %d", errno);
    zsock_freeaddrinfo(res);
    return -errno;
  }

  err = zsock_connect(sock, res->ai_addr, res->ai_addrlen);
  zsock_freeaddrinfo(res);
  if (err) {
    LOG_ERR("Failed to connect to %s:%d: %d", AGNSS_SERVER_HOST,
            AGNSS_SERVER_PORT, errno);
    zsock_close(sock);
    return -errno;
  }

  char url[64];

  snprintk(url, sizeof(url), "%s?flags=0x%08X", AGNSS_SERVER_PATH,
           req.data_flags);

  struct http_request http_req = {
    .method       = HTTP_POST,
    .url          = url,
    .host         = AGNSS_SERVER_HOST,
    .protocol     = "HTTP/1.1",
    .response     = http_response_cb,
    .recv_buf     = rx_buf,
    .recv_buf_len = sizeof(rx_buf),
  };

  agnss_data_len = 0;

  LOG_INF("Requesting A-GNSS data from %s%s ...", AGNSS_SERVER_HOST, url);
  err = http_client_req(sock, &http_req, AGNSS_HTTP_TIMEOUT, NULL);
  zsock_close(sock);

  if (err < 0) {
    LOG_ERR("HTTP request failed: %d", err);
    return err;
  }
  if (agnss_data_len == 0) {
    LOG_ERR("Empty A-GNSS response");
    return -ENODATA;
  }

  LOG_INF("Received %zu bytes of A-GNSS JSON, parsing...", agnss_data_len);

  /* Null-terminate for cJSON */
  if (agnss_data_len < sizeof(agnss_data_buf)) {
    agnss_data_buf[agnss_data_len] = '\0';
  }

  err =
    parse_and_inject_agnss_data(agnss_data_buf, agnss_data_len, req.data_flags);
  if (err) {
    LOG_ERR("A-GNSS parse/inject returned: %d", err);
    return err;
  }

  /* Verify modem accepted the data */
  struct nrf_modem_gnss_agnss_expiry expiry_after = {0};

  if (nrf_modem_gnss_agnss_expiry_get(&expiry_after) == 0) {
    if (expiry_after.data_flags == 0) {
      LOG_INF("A-GNSS injection verified: modem has valid "
              "assistance data");
    } else {
      LOG_WRN("A-GNSS injection incomplete, modem still "
              "needs flags=0x%08X",
              expiry_after.data_flags);
    }
  }

  return 0;
}

static int gnss_delete_data(void)
{
  int err;

  uint32_t delete_mask =
    NRF_MODEM_GNSS_DELETE_EPHEMERIDES | NRF_MODEM_GNSS_DELETE_ALMANACS |
    NRF_MODEM_GNSS_DELETE_IONO_CORRECTION_DATA |
    NRF_MODEM_GNSS_DELETE_LAST_GOOD_FIX | NRF_MODEM_GNSS_DELETE_GPS_TOW |
    NRF_MODEM_GNSS_DELETE_GPS_WEEK | NRF_MODEM_GNSS_DELETE_UTC_DATA |
    NRF_MODEM_GNSS_DELETE_GPS_TOW_PRECISION | NRF_MODEM_GNSS_DELETE_EKF;

  err = nrf_modem_gnss_nv_data_delete(delete_mask);
  if (err) {
    LOG_ERR("Failed to delete GNSS data: %d", err);
    return err;
  }
  LOG_INF("GNSS cold start: all stored data deleted");
  return err;
}

static int gnss_init_and_start(void)
{
  int err;

  if (IS_ENABLED(CONFIG_GNSS_COLD_START)) {
    err = gnss_delete_data();
    if (err) {
      return err;
    }
  }

  err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL);
  if (err) {
    LOG_ERR("Failed to activate GNSS functional mode, error: %d", err);
    return err;
  }

  err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
  if (err) {
    LOG_ERR("Failed to set GNSS event handler, error: %d", err);
    return err;
  }

  int32_t interval =
    IS_ENABLED(CONFIG_GNSS_SINGLE_FIX) ? 0 : CONFIG_GNSS_PERIODIC_INTERVAL;

  err = nrf_modem_gnss_fix_interval_set(interval);
  if (err) {
    LOG_ERR("Failed to set GNSS fix interval, error: %d", err);
    return err;
  }

  err = nrf_modem_gnss_fix_retry_set(CONFIG_GNSS_PERIODIC_TIMEOUT);
  if (err) {
    LOG_ERR("Failed to set GNSS fix retry, error: %d", err);
    return err;
  }

  LOG_INF("Starting GNSS");
  err = nrf_modem_gnss_start();
  if (err) {
    LOG_ERR("Failed to start GNSS, error: %d", err);
    return err;
  }

  gnss_start_time = k_uptime_get();
  return 0;
}

int main(void)
{
  int err;

  err = modem_configure();
  if (err) {
    LOG_ERR("Failed to configure the modem, error: %d", err);
    return err;
  }

  err = gnss_init_and_start();
  if (err) {
    LOG_ERR("Failed to initialize and start GNSS");
    return err;
  }

  err = agnss_request_and_inject();
  if (err) {
    LOG_WRN("A-GNSS fetch failed (%d), will continue without "
            "assistance",
            err);
  }

  while (1) {
    k_sleep(K_FOREVER);
  }

  return 0;
}

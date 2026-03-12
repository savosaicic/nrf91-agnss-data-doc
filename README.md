# nRF91 GNSS Assistance Data (A-GNSS)

A reference project for injecting GNSS assistance data into the nRF91 series
modem without relying on nRF Cloud or a proprietary SUPL client.
The goal is to build an open-source pipeline that fetches assistance data from
a public or self-hosted source and injects it directly via the nRF Modem API.

---

## Background

The nRF91 SiP integrates an LTE-M/NB-IoT modem alongside a GPS/GNSS receiver.
To achieve a fast Time To First Fix (TTFF), the modem can request **Assisted GNSS (A-GNSS)**
data that are pre-computed orbital parameters, clock corrections,and environmental
corrections.That normally take minutes to download from satellites directly.

Nordic provides two official mechanisms for delivering this data:
- **nRF Cloud A-GNSS**: The client library is open-source (part of the nRF Connect SDK),
  but the backend requires a Nordic account. The binary protocol and data structures are
  fully readable in [`nrf_cloud_agnss.c`](https://github.com/nrfconnect/sdk-nrf) and
  [`nrf_cloud_agnss_schema_v1.h`](https://github.com/nrfconnect/sdk-nrf/blob/main/subsys/net/lib/nrf_cloud/common/include/nrf_cloud_agnss_schema_v1.h).
- **SUPL client**: Closed-source, distributed as a pre-built binary library.

This project aims to document the raw data structures expected by the modem
firmware so that assistance data can be injected from any source
using the public `nrf_modem_gnss` API.

---

## How Assistance Data Works

When the GNSS module needs assistance data, it raises an event that the
application can handle. The event carries a request bitmask indicating which
data types are needed. The application is then responsible for fetching the
corresponding data and injecting it back using `nrf_modem_gnss_agnss_write()`.

---

## Assistance Data Types

The modem can request up to 9 types of assistance data, identified by the following constants:

| Constant                                               | Value | Description                                 |
| ------------------------------------------------------ | ----- | ------------------------------------------- |
| `NRF_MODEM_GNSS_AGNSS_GPS_UTC_PARAMETERS`              |   1   | GPS UTC time correction parameters          |
| `NRF_MODEM_GNSS_AGNSS_GPS_EPHEMERIDES`                 |   2   | Precise orbital data per satellite          |
| `NRF_MODEM_GNSS_AGNSS_GPS_ALMANAC`                     |   3   | Coarse long-term orbital data per satellite |
| `NRF_MODEM_GNSS_AGNSS_KLOBUCHAR_IONOSPHERIC_CORRECTION`|   4   | Ionospheric delay model (GPS)               |
| `NRF_MODEM_GNSS_AGNSS_NEQUICK_IONOSPHERIC_CORRECTION`  |   5   | Ionospheric delay model (Galileo)           |
| `NRF_MODEM_GNSS_AGNSS_GPS_SYSTEM_CLOCK_AND_TOWS`       |   6   | GPS system time and per-satellite TOW       |
| `NRF_MODEM_GNSS_AGNSS_LOCATION`                        |   7   | Approximate receiver location               |
| `NRF_MODEM_GNSS_AGPS_INTEGRITY`                        |   8   | Legacy GPS satellite health mask            |
| `NRF_MODEM_GNSS_AGNSS_INTEGRITY`                       |   9   | Multi-signal satellite health mask          |

---

## Data Structure Reference

### 1. `NRF_MODEM_GNSS_AGNSS_GPS_UTC_PARAMETERS`

Provides the coefficients needed to convert GPS system time to UTC.
The GPS constellation maintains its own continuous time scale (no leap seconds),
so this polynomial correction allows the receiver to compute accurate wall-clock time.

```c
struct nrf_modem_gnss_agnss_gps_data_utc {
  int32_t a1;         // First order term of polynomial (sec/sec). Scale factor 2^-50.
                      // Range -8388608...8388607 (25 bits).
  int32_t a0;         // Constant term of polynomial (sec). Scale factor 2^-30.
  uint8_t tot;        // UTC reference GPS time-of-week (sec). Scale factor 2^12. Range 0..147.
  uint8_t wn_t;       // UTC reference GPS week number modulo 256.
  int8_t  delta_tls;  // Current or past leap second count (sec).
  uint8_t wn_lsf;     // Leap second reference GPS week number modulo 256.
  int8_t  dn;         // Leap second reference GPS day-of-week (day). Range 1...7.
  int8_t  delta_tlsf; // Current or future leap second count (sec).
};
```

---

### 2. `NRF_MODEM_GNSS_AGNSS_GPS_EPHEMERIDES`

Ephemeris data contains precise Keplerian orbital elements for individual
satellites, valid for a few hours. This is the most impactful assistance
type: without it, the receiver must decode ephemeris from the satellite signal
itself, taking up to 30 seconds per satellite.

Applies to both **GPS** (SVIDs 1–32) and **QZSS** (SVIDs 193–202).

```c
struct nrf_modem_gnss_agnss_gps_data_ephemeris {
  uint8_t  sv_id;     // Satellite ID. 1..32: GPS, 193..202: QZSS.
  uint8_t  health;    // Satellite health status.
  uint16_t iodc;      // Issue of data, clock. Range 0...2047.
  uint16_t toc;       // Clock reference time-of-week (sec). Scale 2^4. Range 0...37799.
  int8_t   af2;       // Clock drift rate (sec/sec²). Scale 2^-55.
  int16_t  af1;       // Clock drift (sec/sec). Scale 2^-43.
  int32_t  af0;       // Clock bias (sec). Scale 2^-31. Range -2097152...2097151 (22 bits).
  int8_t   tgd;       // Group delay (sec). Scale 2^-31.
  uint8_t  ura;       // User range accuracy index. Range 0...15.
  uint8_t  fit_int;   // Curve fit interval flag. Range 0...1.
  uint16_t toe;       // Ephemeris reference time-of-week (sec). Scale 2^4. Range 0...37799.
  int32_t  w;         // Argument of perigee (semi-circle). Scale 2^-31.
  int16_t  delta_n;   // Mean motion difference (semi-circle/sec). Scale 2^-43.
  int32_t  m0;        // Mean anomaly at reference time (semi-circle). Scale 2^-31.
  int32_t  omega_dot; // Rate of right ascension (semi-circle/sec). Scale 2^-43. Range ±8388607 (24 bits).
  uint32_t e;         // Eccentricity. Scale 2^-33.
  int16_t  idot;      // Rate of inclination angle (semi-circle/sec). Scale 2^-43. Range ±8191 (14 bits).
  uint32_t sqrt_a;    // Square root of semi-major axis (m^½). Scale 2^-19.
  int32_t  i0;        // Inclination angle at reference time (semi-circle). Scale 2^-31.
  int32_t  omega0;    // Longitude of ascending node (semi-circle). Scale 2^-31.
  int16_t  crs;       // Orbit radius, sine harmonic correction (m). Scale 2^-5.
  int16_t  cis;       // Inclination angle, sine harmonic correction (rad). Scale 2^-29.
  int16_t  cus;       // Argument of latitude, sine harmonic correction (rad). Scale 2^-29.
  int16_t  crc;       // Orbit radius, cosine harmonic correction (m). Scale 2^-5.
  int16_t  cic;       // Inclination angle, cosine harmonic correction (rad). Scale 2^-29.
  int16_t  cuc;       // Argument of latitude, cosine harmonic correction (rad). Scale 2^-29.
};
```

---

### 3. `NRF_MODEM_GNSS_AGNSS_GPS_ALMANAC`

Almanac data is a lower-precision, longer-lived alternative to ephemeris.
It is used by the receiver to know which satellites are currently above the
horizon and worth searching for. Valid for weeks to months.

Applies to both **GPS** (SVIDs 1–32) and **QZSS** (SVIDs 193–202).

```c
struct nrf_modem_gnss_agnss_gps_data_almanac {
  uint8_t  sv_id;     // Satellite ID. 1..32: GPS, 193..202: QZSS.
  uint8_t  wn;        // Almanac reference GPS week number modulo 256.
  uint8_t  toa;       // Almanac reference time-of-week (sec). Scale 2^12. Range 0...147.
  uint8_t  ioda;      // Issue of data, almanac. Range 0...3 (2 bits).
  uint16_t e;         // Eccentricity. Scale 2^-21.
  int16_t  delta_i;   // Correction to inclination (semi-circle). Scale 2^-19.
  int16_t  omega_dot; // Rate of right ascension (semi-circle/sec). Scale 2^-38.
  uint8_t  sv_health; // Satellite health.
  uint32_t sqrt_a;    // Square root of semi-major axis (m^½). Scale 2^-11. Range 0...16777215 (24 bits).
  int32_t  omega0;    // Longitude of ascending node (semi-circle). Scale 2^-23. Range ±8388607 (24 bits).
  int32_t  w;         // Argument of perigee (semi-circle). Scale 2^-23.
  int32_t  m0;        // Mean anomaly at reference time (semi-circle). Scale 2^-23. Range ±8388607 (24 bits).
  int16_t  af0;       // Clock bias (sec). Scale 2^-20. Range ±1023 (11 bits).
  int16_t  af1;       // Clock drift (sec/sec). Scale 2^-38. Range ±1023 (11 bits).
};
```

---

### 4. `NRF_MODEM_GNSS_AGNSS_KLOBUCHAR_IONOSPHERIC_CORRECTION`

The Klobuchar model provides coefficients to estimate the delay caused by the
ionosphere on GPS L1 signals. It corrects roughly 50% of the ionospheric
delay error. Used by GPS receivers.

```c
struct nrf_modem_gnss_agnss_data_klobuchar {
  int8_t alpha0; // Constant term (sec). Scale 2^-30.
  int8_t alpha1; // First-order coefficient (sec/semi-circle). Scale 2^-27.
  int8_t alpha2; // Second-order coefficient (sec/semi-circle²). Scale 2^-24.
  int8_t alpha3; // Third-order coefficient (sec/semi-circle³). Scale 2^-24.
  int8_t beta0;  // Constant term (sec). Scale 2^11.
  int8_t beta1;  // First-order coefficient (sec/semi-circle). Scale 2^14.
  int8_t beta2;  // Second-order coefficient (sec/semi-circle²). Scale 2^16.
  int8_t beta3;  // Third-order coefficient (sec/semi-circle³). Scale 2^16.
};
```

---

### 5. `NRF_MODEM_GNSS_AGNSS_NEQUICK_IONOSPHERIC_CORRECTION`

The NeQuick model is the ionospheric correction model used by **Galileo**.
It uses three effective ionisation level (az) parameters and storm condition
flags to model signal delay through the ionosphere.

Despite the nRF91 only receiving GPS/QZSS signals, Nordic has confirmed on
DevZone that NeQuick coefficients can be used by the device for more accurate
ionospheric corrections than Klobuchar alone — the model is applied to GPS
ranging corrections regardless of which constellation the coefficients
originate from.

```c
struct nrf_modem_gnss_agnss_data_nequick {
  int16_t ai0;         // Effective ionisation level 1st order parameter (SFU).
                       // Scale 2^-2. Range 0...2047 (11 bits).
  int16_t ai1;         // Effective ionisation level 2nd order parameter (SFU/deg).
                       // Scale 2^-8. Range -1024...1023 (11 bits).
  int16_t ai2;         // Effective ionisation level 3rd order parameter (SFU/deg²).
                       // Scale 2^-15. Range -8192...8191 (14 bits).
  uint8_t storm_cond;  // Bit mask: ionospheric storm condition active per region.
  uint8_t storm_valid; // Bit mask: indicates which region's storm_cond bits are valid.
};
```

---

### 6. `NRF_MODEM_GNSS_AGNSS_GPS_SYSTEM_CLOCK_AND_TOWS`

Provides a coarse GPS system time (to within ~1 second) and per-satellite
Time Of Week (TOW) data extracted from the satellite's Telemetry (TLM) word.
This allows the receiver to synchronise quickly without waiting to decode time
from the signal.

#### TOW element per satellite

```c
struct nrf_modem_gnss_agnss_gps_data_tow_element {
  uint16_t tlm;   // Bits [15:14]: reserved + integrity status flag.
                  // Bits [13:0]: TLM word being broadcast by the satellite.
  uint8_t  flags; // Bit 0 (LSB): anti-spoof flag. Bit 1: alert flag.
};
```

#### Full system time + TOW structure

```c
#define NRF_MODEM_GNSS_AGNSS_GPS_MAX_SV_TOW 32

struct nrf_modem_gnss_agnss_gps_data_system_time_and_sv_tow {
  uint16_t date_day;    // Days since GPS epoch: Jan 6, 1980 00:00:00 UTC (USNO).
  uint32_t time_full_s; // Full seconds part of time-of-day (s). Range 0...86399.
  uint16_t time_frac_ms;// Fractional seconds (ms). Range 0...999.
  uint32_t sv_mask;     // Bitmask of PRNs for which sv_tow[] entries are valid.
                        // Bit N corresponds to PRN N+1.
  struct nrf_modem_gnss_agnss_gps_data_tow_element
      sv_tow[NRF_MODEM_GNSS_AGNSS_GPS_MAX_SV_TOW]; // TOW data for up to 32 satellites.
};
```

---

### 7. `NRF_MODEM_GNSS_AGNSS_LOCATION`

An approximate initial position of the receiver, typically obtained from
cell tower triangulation or IP geolocation. Helps the receiver narrow its
satellite search window significantly, reducing TTFF.

All coordinates use **WGS-84**.

```c
struct nrf_modem_gnss_agnss_data_location {
  int32_t latitude;          // Coded latitude. N <= (2^23/90) * X < N+1, X in degrees [-90, 90].
                             // Range -8388607...8388607. 255 = missing.
  int32_t longitude;         // Coded longitude. N <= (2^24/360) * X < N+1, X in degrees [-180, 180].
                             // Range -8388607...8388607.
  int16_t altitude;          // Altitude relative to WGS-84 ellipsoid (m). Range -32767...32767.
                             // Positive = above, negative = below ellipsoid surface.
  uint8_t unc_semimajor;     // Uncertainty semi-major axis (m). Range 0...127, or 255 = missing.
                             // Decoded: r = 10 * (1.1^K - 1) meters.
  uint8_t unc_semiminor;     // Uncertainty semi-minor axis (m). Range 0...127, or 255 = missing.
                             // Decoded: r = 10 * (1.1^K - 1) meters.
  uint8_t orientation_major; // Orientation of major uncertainty axis from north (degrees). Range 0...179.
  uint8_t unc_altitude;      // Altitude uncertainty (m). Range 0...127, or 255 = missing.
                             // Decoded: h = 45 * (1.025^K - 1) meters.
  uint8_t confidence;        // Confidence level (%). Range 0...100. 0 = no info. Values 101..128 = treat as 0.
};
```

---

### 8. `NRF_MODEM_GNSS_AGPS_INTEGRITY` *(Legacy)*

A simple 32-bit mask indicating which GPS satellites are currently
unhealthy and should be excluded from positioning.
This is the older, GPS-only integrity type.

> Prefer `NRF_MODEM_GNSS_AGNSS_INTEGRITY` (type 9) for newer hardware.

```c
struct nrf_modem_gnss_agps_data_integrity {
  uint32_t integrity_mask; // Bitmask of unhealthy GPS PRNs.
                           // Bit N set = PRN N+1 is unhealthy.
};
```

---

### 9. `NRF_MODEM_GNSS_AGNSS_INTEGRITY`

Extended integrity data supporting multiple signals and constellations
(GPS + QZSS). Uses a 64-bit mask per signal, allowing both GPS PRNs 1–32
and QZSS PRNs 193–202 to be represented.

> Only supported on: `mfw_nrf91x1`, `mfw_nrf9151-ntn`

#### Per-signal integrity mask

```c
struct nrf_modem_gnss_agnss_data_signal_integrity {
  uint8_t  signal_id;      // Signal identifier, see nrf_modem_gnss_signal_id.
  uint64_t integrity_mask; // Bitmask of unhealthy satellites for this signal.
                           // GPS:  bits [31:0] = PRN 1..32.
                           // QZSS: bits [9:0]  = PRN 193..202.
};
```

#### Full integrity structure

```c
struct nrf_modem_gnss_agnss_data_integrity {
  uint8_t signal_count; // Number of valid entries in signal[].
  struct nrf_modem_gnss_agnss_data_signal_integrity
      signal[NRF_MODEM_GNSS_MAX_SIGNALS]; // Per-signal integrity masks.
};
```

---

## Injection Flow

```c
// 1. Handle GNSS event
void gnss_event_handler(int event) {
  if (event == NRF_MODEM_GNSS_EVT_AGNSS_REQ) {
    struct nrf_modem_gnss_agnss_data_frame req;
    nrf_modem_gnss_agnss_expiry_get(&req);
    // req.data_flags bitmask tells which types are needed
    fetch_and_inject_assistance(&req);
  }
}

// 2. Fetch from a-gnss data and inject each type
void fetch_and_inject_assistance(struct nrf_modem_gnss_agnss_data_frame *req) {
  if (req->data_flags & BIT(NRF_MODEM_GNSS_AGNSS_LOCATION - 1)) {
    struct nrf_modem_gnss_agnss_data_location loc = { /* ... */ };
    nrf_modem_gnss_agnss_write(&loc, sizeof(loc),
                               NRF_MODEM_GNSS_AGNSS_LOCATION);
  }
  // Repeat for each requested type...
}
```

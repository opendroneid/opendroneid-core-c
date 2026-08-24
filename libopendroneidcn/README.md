# GB 46750-2025 — Broadcast Remote ID Protocol

Civil Unmanned Aircraft System Operational Identification
(中华人民共和国民用无人驾驶航空器系统运营识别规范)

Published 2025-10-31, effective 2026-05-01.

This library implements the **broadcast mode** (广播模式) defined in GB 46750-2025. It is
independent of ASTM F3411 — GB 46750-2025 uses its own wire format, field set, and
transmission rules. See the [Differences from ASTM F3411](#differences-from-astm-f3411)
section below.

> **⚠ Compliance scope:** GB 46750-2025 mandates **both** broadcast mode (§5.2) and
> network mode (§5.3) for full compliance. This library implements broadcast mode only.
> Additionally, the standard specifies product-level requirements in §5.1 (transmission
> interval ≤1s, antenna placement, 10s shutdown grace period, ADS-B prohibition, etc.)
> that must be satisfied at the hardware/firmware level — these are outside the scope
> of this software library. Manufacturers must implement network mode and meet all §5.1
> requirements to achieve full GB 46750-2025 compliance.

## 1. Packet Format

Each broadcast-mode RID packet uses a **flag-bitmask chain** structure (not TLV tags):

```
┌──────────┬──────────┬──────────┬───────────────────┬──────────────────────┐
│  Type    │ Version  │ DataLen  │   Flag Bytes      │     Data             │
│  1 byte  │ 1 byte   │ 1 byte   │   1–3 bytes       │   N bytes (≤200)     │
│  0xFF    │          │  1–200   │                   │                      │
└──────────┴──────────┴──────────┴───────────────────┴──────────────────────┘
```

| Byte offset | Field    | Description |
|-------------|----------|-------------|
| 0           | Type     | Always `0xFF` — distinguishes GB 46750 packets from other beacon payload. |
| 1           | Version  | Bits 7–5: protocol major version, always `001` (V1). Bits 4–0: sub-version. **Standard note:** The standard text states bits 4–8 can encode 0–63, but 5 bits can only encode 0–31 — an arithmetic inconsistency in the standard itself. This implementation uses bits 4–0 (5 bits, range 0–31) as the sub-version, rendered as `V1.X`. |
| 2           | DataLen  | Byte count of the data content items only (1–200), excluding the flag bytes. Per standard Table 1: 数据长度 = 数据内容项的字节数. |
| 3 … 3+N-1   | Flags    | Variable-length flag bytes, chained via bit 0. Per standard: bits 1–7 (MSB-side) = content flag bits; bit 8 (LSB, 0x01) = extension flag — `1` = next byte continues, `0` = end. Currently 3 bytes defined (fields 001–021); the chain is open-ended for future CAAC extensions. |
| 3+N …       | Data     | Concatenated field values, each fixed-length per the protocol. Fields appear in ascending field-ID order (001 → 021). Omitted fields (flag bit = 0) consume zero bytes here. |

**Flag chain rules:**
- Bits 1–7 of each flag byte = content flag bits (per Table 2 mapping).
- Bit 8 (LSB, 0x01) = extension flag: `0` = end of flag field, `1` = next byte continues the chain.
- Currently 3 flag bytes are defined (fields 001–021); the chain is open-ended for future CAAC extensions.
- The receiver walks the chain to compute the total flag-region size, then uses `DataLen` to determine the data-region size.

## 2. Flag Bit Mapping

Each bit in the flag region (bits 7–1 of each flag byte) signals the presence of one
field. Fields marked **M** (Mandatory) are always transmitted; fields marked **O**
(Optional) are conditionally included.

| Flag Byte | Bit | Hex Mask | Field ID | Field Name            | M/O | Description                     |
|-----------|-----|----------|----------|-----------------------|-----|---------------------------------|
| Flag1     | 7   | `0x80`   | 001      | SN                    | M   | Product serial number           |
| Flag1     | 6   | `0x40`   | 002      | UIN                   | M   | Registration mark (last 8)      |
| Flag1     | 5   | `0x20`   | 003      | Operation Category    | O   | Open / Specific / Certified     |
| Flag1     | 4   | `0x10`   | 004      | Drone Class           | M   | Micro / Light / Small / …       |
| Flag1     | 3   | `0x08`   | 005      | GCS Position Type     | M   | Takeoff point or GCS location   |
| Flag1     | 2   | `0x04`   | 006      | GCS Latitude/Longitude| M   | 2 × int32 at 1e-7 degrees       |
| Flag1     | 1   | `0x02`   | 007      | GCS Geodetic Altitude | M   | uint16, 0.5 m resolution        |
| Flag1     | 0   | `0x01`   | —        | *(extension flag)*    | —   | `1` = Flag2 follows             |
| Flag2     | 7   | `0x80`   | 008      | UAV Latitude/Longitude| M   | 2 × int32 at 1e-7 degrees       |
| Flag2     | 6   | `0x40`   | 009      | Track Angle           | M   | uint16, 0.1° resolution         |
| Flag2     | 5   | `0x20`   | 010      | Ground Speed          | M   | uint16, 0.1 m/s resolution      |
| Flag2     | 4   | `0x10`   | 011      | Relative Altitude     | O   | Height above takeoff point      |
| Flag2     | 3   | `0x08`   | 012      | Vertical Speed        | O   | Signed 7-bit, 0.5 m/s steps     |
| Flag2     | 2   | `0x04`   | 013      | UAV Geodetic Altitude | M   | uint16, 0.5 m resolution        |
| Flag2     | 1   | `0x02`   | 014      | Barometric Altitude   | O   | uint16, 0.5 m resolution        |
| Flag2     | 0   | `0x01`   | —        | *(extension flag)*    | —   | `1` = Flag3 follows             |
| Flag3     | 7   | `0x80`   | 015      | Operation Status      | M   | Ground / Air / Emergency / …    |
| Flag3     | 6   | `0x40`   | 016      | Coordinate System     | M   | WGS-84 or CGCS2000              |
| Flag3     | 5   | `0x20`   | 017      | Horizontal Accuracy   | M   | GNSS NACp category (0–15)       |
| Flag3     | 4   | `0x10`   | 018      | Vertical Accuracy     | M   | GNSS GVA category (0–15)        |
| Flag3     | 3   | `0x08`   | 019      | Speed Accuracy        | M   | GNSS NACv category (0–15)       |
| Flag3     | 2   | `0x04`   | 020      | Timestamp             | M   | Unix ms, 6 bytes                |
| Flag3     | 1   | `0x02`   | 021      | Timestamp Accuracy    | M   | Category (0–15)                 |
| Flag3     | 0   | `0x01`   | —        | *(reserved)*          | —   | Should be `0`                   |

> Fields marked **M** (Mandatory) are always transmitted per the standard.
> Fields marked **O** (Optional) are transmitted only when the data is available.

## 3. Field Definitions

### Field Encoding Table

| ID  | Field Name           | Bytes | Encoding & Range                                          | Invalid / Absent | M/O |
|-----|----------------------|-------|-----------------------------------------------------------|------------------|-----|
| 001 | SN                   | 20    | ASCII per GB/T 41300, max 20 chars. Per standard: "高位以空字符NULL填充" — pad with `\0` at the trailing (high-address) end when shorter than 20 bytes. | —                | M   |
| 002 | UIN                  | 8     | ASCII, last 8 chars of real-name registration mark. Per standard: "高位补NULL" — pad with `\0` at the trailing end when shorter than 8 bytes. | —          | M   |
| 003 | Operation Category   | 1     | 0=Undefined, 1=Open, 2=Specific, 3=Certified, 4–15=Reserved. | —            | O   |
| 004 | Drone Class          | 1     | 0=Micro, 1=Light, 2=Small, 3=Medium, 4=Large, 5–15=Reserved. | —            | M   |
| 005 | GCS Position Type    | 1     | 0=Takeoff point, 1=GCS location, 2–15=Reserved.           | —                | M   |
| 006 | GCS Lat/Lon          | 8     | Two int32 LE, ×1e-7 degrees. Positive=N/positive=E.       | `0xFFFFFFFF` for each axis | M   |
| 007 | GCS Geodetic Altitude| 2     | uint16 LE, `(alt_m + 1000) × 2`, 0.5 m resolution.        | `0x0000`         | M   |
| 008 | UAV Lat/Lon          | 8     | Two int32 LE, ×1e-7 degrees. Positive=N/positive=E.       | `0xFFFFFFFF` for each axis | M   |
| 009 | Track Angle          | 2     | uint16 LE, `angle_deg × 10`, range 0–3599 (0.1° steps).   | `0xFFFF`         | M   |
| 010 | Ground Speed         | 2     | uint16 LE, `speed_mps × 10`, 0.1 m/s resolution.          | `0xFFFF`         | M   |
| 011 | Relative Altitude    | 2     | uint16 LE, `(alt_m + 9000) × 2`, 0.5 m resolution. Offset to takeoff point. | `0x0000` | O   |
| 012 | Vertical Speed       | 1     | Bits 6–0: magnitude × 0.5 m/s (0–127). Bit 7: sign (`0`=positive/ascending, `1`=negative/descending). `0` with sign bit `0` means `0 m/s`. | `0xFF` | O   |
| 013 | UAV Geodetic Altitude| 2     | uint16 LE, `(alt_m + 1000) × 2`, 0.5 m resolution.        | `0x0000`         | M   |
| 014 | Barometric Altitude  | 2     | uint16 LE, `(alt_m + 1000) × 2`, 0.5 m resolution. Ref: 101.325 kPa. | `0x0000` | O   |
| 015 | Operation Status     | 1     | 0=Unknown, 1=Ground, 2=Air, 3=Emergency, 4=RID failure (non-emergency), 5=RID failure (emergency), 6–15=Reserved. | — | M   |
| 016 | Coordinate System    | 1     | 0=WGS-84, 1=CGCS2000, 2–15=Reserved.                     | —                | M   |
| 017 | Horizontal Accuracy  | 1     | GNSS NACp: 0=>18.52 km, 1=<18.52 km, 2=<7.41 km, 3=<3.70 km, 4=<1852 m, 5=<926 m, 6=<556 m, 7=<185 m, 8=<92.6 m, 9=<30 m, 10=<10 m, 11=<3 m, 12=<1 m, 13–15=Reserved. 95% confidence. | — | M |
| 018 | Vertical Accuracy    | 1     | GNSS GVA: 0=>150 m, 1=<150 m, 2=<45 m, 3=<25 m, 4=<10 m, 5=<3 m, 6=<1 m, 7–15=Reserved. 95% confidence. | — | M   |
| 019 | Speed Accuracy       | 1     | GNSS NACv: 0=>10 m/s, 1=<10 m/s, 2=<3 m/s, 3=<1 m/s, 4=<0.3 m/s, 5–15=Reserved. 95% confidence. | — | M |
| 020 | Timestamp            | 6     | uint48 LE, Unix timestamp in milliseconds since epoch.     | `0x000000000000` | M   |
| 021 | Timestamp Accuracy   | 1     | 0=>0.5 s, 1=<0.5 s, 2=<0.4 s, 3=<0.3 s, 4=<0.2 s, 5=<0.1 s, 6=<50 ms, 7=<20 ms, 8=<10 ms, 9–15=Reserved. | — | M |

### Encoding Formulas — Quick Reference

| Field Group             | Wire Value                                  | Decode Formula                  |
|-------------------------|---------------------------------------------|---------------------------------|
| Latitude / Longitude    | `int32_le`                                  | `degrees = wire_val × 1e-7`     |
| Altitude (drone, GCS, barometric) | `uint16_le`                        | `meters = wire_val × 0.5 − 1000` |
| Relative Altitude       | `uint16_le`                                 | `meters = wire_val × 0.5 − 9000` |
| Track Angle             | `uint16_le`                                 | `degrees = wire_val × 0.1`      |
| Ground Speed            | `uint16_le`                                 | `m/s = wire_val × 0.1`          |
| Vertical Speed          | `(sign_bit << 7) \| (magnitude & 0x7F)`    | `m/s = magnitude × 0.5`, negative if sign bit = 1 |
| Timestamp               | `uint48_le` (6 bytes)                       | `ms = wire_val`                 |

All multi-byte integers are **little-endian**. The library uses byte-shift helpers
(`le16_to_h` / `h_to_le16`, etc.) rather than `<endian.h>`, so the code is
endian-agnostic at compile time.

## 4. RF / Broadcast Requirements

Broadcast mode supports the following transmission methods:

- **Bluetooth** — 5.0 and above
- **Wi-Fi** — 2.4 GHz / 5.8 GHz

## Differences from ASTM F3411

GB 46750-2025 is **not wire-compatible** with ASTM F3411. Key differences:

| Aspect                | ASTM F3411                          | GB 46750-2025                      |
|-----------------------|-------------------------------------|------------------------------------|
| Wire format           | 6 message types, 25-byte fixed      | Single variable-length packet, flag-bitmask chain |
| Identity              | Serial number **or** Session ID     | SN **and** UIN (registration ID) both mandatory |
| Authentication        | Auth message with signature pages   | Not defined                        |
| Operator location     | Optional                            | Mandatory (GCS position + type + altitude) |
| Accuracy reporting    | Not defined                         | 4 accuracy levels (horizontal / vertical / speed / timestamp) |
| Broadcast transport   | BT4 + BT5 + Wi-Fi NaN + Wi-Fi Beacon| Bluetooth 5.0+ + Wi-Fi (2.4 GHz / 5.8 GHz) |
| Coordinate system     | WGS-84 only                         | WGS-84 or CGCS2000                 |
| Drone classification  | Not defined                         | Mandatory (Micro → Large)          |
| Operation category    | EU Category & Class (optional)      | Open / Specific / Certified (optional) |

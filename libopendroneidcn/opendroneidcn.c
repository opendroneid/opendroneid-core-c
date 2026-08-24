/*
Copyright (C) 2026 Jun Zhang

SPDX-License-Identifier: Apache-2.0

Open Drone ID C Library — GB 46750-2025

Maintainer:
Jun Zhang
zhangjun.sole@qq.com

GB 46750-2025 — Civil Unmanned Aircraft System Operational Identification
(中华人民共和国民用无人驾驶航空器系统运营识别规范)

Parses the broadcast-mode RID packet as defined in section 5.2 of the standard.

Packet format (section 5.2.1, Table 1):
  [Type=0xFF][Ver+Reserved][DataLen][Flag bytes (chained via bit 0)][Data]

Flag byte chain: bits 1-7 = content flags, bit 8 = extension flag.
The chain is open-ended per §5.2.2; currently 3 bytes defined.

All multi-byte fields are little-endian per the spec.

Reference: GB 46750-2025 (published 2025-10-31, effective 2026-05-01)
*/

#include "opendroneidcn.h"
#include <math.h>
#include <string.h>

/* ===================== Endianness Helpers ===================== */
/*
 * GB 46750-2025 specifies all multi-byte numeric fields as little-endian.
 * Byte-shift macros work on any CPU regardless of host endianness — no
 * runtime detection needed, and the compiler optimizes them to single
 * load+rev instructions on big-endian targets.
 */
static inline uint16_t le16_to_h(const uint8_t p[2])
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t le32_to_h(const uint8_t p[4])
{
    return (uint32_t)p[0]        | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Host-to-little-endian encode helpers — mirror of le*_to_h above */
static inline void h_to_le16(uint8_t p[2], uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void h_to_le32(uint8_t p[4], uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/* ===================== Beacon Scanner ===================== */

/*
 * Validate that an ASCII field contains only printable characters
 * (0x20–0x7E) or null padding (0x00). Returns false if any byte is
 * non-printable, non-null — used to reject garbage data from
 * false-positive GB46750 packet matches.
 */
static bool is_valid_ascii_field(const uint8_t *buf, int len) {
    for (int i = 0; i < len; i++) {
        if (buf[i] == 0x00) continue;
        if (buf[i] < 0x20 || buf[i] > 0x7E) return false;
    }
    return true;
}

bool CN46750_FindPacket(const uint8_t *beacon_data, size_t beacon_len,
                        size_t *rid_offset, size_t *rid_len)
{
    if (beacon_data == NULL || beacon_len < 6)
        return false;

    for (size_t i = 0; i < beacon_len - 3; i++) {
        if (beacon_data[i] != 0xFF)
            continue;
        /* Version field: high 3 bits fixed to 001 (0x20 mask) */
        if ((beacon_data[i + 1] & 0xE0) != 0x20)
            continue;

        uint8_t data_len = beacon_data[i + 2];
        if (data_len < 1 || data_len > 200)
            continue;

        /* Count flag bytes (bit0 = extension flag, per Table 1).
         * Standard §5.2.1 Table 1: bits 1-7 = content flag bits,
         * bit 8 = extension flag (0 = end, 1 = next byte follows).
         * The chain is open-ended — currently 3 bytes defined for
         * fields 001-021; additional bytes reserved for future CAAC
         * protocol extensions. */
        size_t flag_bytes = 0;
        size_t flag_pos = i + 3;
        while (flag_pos < beacon_len) {
            flag_bytes++;
            if ((beacon_data[flag_pos] & 0x01) == 0)
                break;
            flag_pos++;
        }

        /* Must have at least 1 flag byte */
        if (flag_bytes == 0)
            continue;

        size_t total_rid_len = 3 + flag_bytes + data_len;
        if (i + total_rid_len > beacon_len)
            continue;

        if (rid_offset) *rid_offset = i;
        if (rid_len)    *rid_len = total_rid_len;
        return true;
    }
    return false;
}

/* ===================== Parse Function ===================== */

RID_Status_t CN46750_RID_Decode(const uint8_t *buf,
                               size_t       buf_len,
                               DroneRIDData_t *data,
                               uint8_t      *version)
{
    if (buf == NULL || data == NULL) {
        return RID_ERR_PARAM;
    }

    /* 1. Absolute minimum: header (3) + at least one flag byte (1) */
    if (buf_len < 4) {
        return RID_ERR_PARAM;
    }

    /* 2. Type byte — must be 0xFF per spec */
    if (buf[0] != 0xFF) {
        return RID_ERR_INVALID_TYPE;
    }

    /* Version field: high 3 bits fixed to 001 (0x20 mask), per spec section 5.2.1 */
    if ((buf[1] & 0xE0) != 0x20) {
        return RID_ERR_INVALID_TYPE;
    }

    /* Sub-version: bits 4-0 of byte 1 (V1.X).
     * NOTE: The standard §5.2.1 Table 1 states bits 4–8 encode 0–63, but
     * 5 bits can only represent 0–31 — an arithmetic inconsistency in the
     * standard text. This implementation uses the 5-bit field (0x1F mask,
     * range 0–31) as the sub-version, which is the physically encodable range. */
    if (version != NULL) {
        *version = buf[1] & 0x1F;
    }

    /* DataLen = byte count of data content items only (excludes flag bytes),
     * per standard Table 1: 数据长度 = 数据内容项的字节数. */
    uint8_t data_len = buf[2];
    if (data_len < 1 || data_len > 200) {
        return RID_ERR_INVALID_LEN;
    }

    /* 3. Walk the flag-byte chain (§5.2.1 Table 1)
     *    Standard: bits 1-7 = content flags, bit 8 = extension flag
     *    (0 = end of flag field, 1 = next byte continues the chain).
     *    The chain is open-ended — currently 3 bytes defined (fields
     *    001-021); additional bytes reserved for future CAAC extensions.
     *
     *    Forward-compatibility: we only parse the first 3 flag bytes
     *    (known fields).  Extra flag bytes are still counted (so the
     *    data pointer advances past the correct number of field bytes),
     *    but their corresponding data fields are silently skipped —
     *    unknown future fields do not break decoding of known fields. */
    uint8_t flags[3] = {0, 0, 0};
    int num_flag_bytes = 0;
    const uint8_t *fp   = &buf[3];
    const uint8_t *bend = buf + buf_len;

    while (fp < bend) {
        if (num_flag_bytes < 3) {
            flags[num_flag_bytes] = *fp;
        }
        num_flag_bytes++;
        if (!(*fp & 0x01)) break;   /* bit 0 = 0 → last flag byte */
        fp++;
    }
    if (num_flag_bytes == 0) {
        return RID_ERR_PARSE_FAIL;
    }

    /* 4. Validate total packet length against declared data_len */
    if (buf_len < (size_t)(3 + num_flag_bytes + data_len)) {
        return RID_ERR_INVALID_LEN;
    }

    memset(data, 0, sizeof(DroneRIDData_t));

    uint8_t flag1 = (num_flag_bytes >= 1) ? flags[0] : 0;
    uint8_t flag2 = (num_flag_bytes >= 2) ? flags[1] : 0;
    uint8_t flag3 = (num_flag_bytes >= 3) ? flags[2] : 0;

    /* Data fields start after the flag chain */
    const uint8_t *p   = &buf[3 + num_flag_bytes];
    const uint8_t *end = p + data_len;

    /* Safety guard: declared data_len must not overshoot the buffer */
    if (end > buf + buf_len) {
        return RID_ERR_INVALID_LEN;
    }

    /* =================================================================
     *  5. Parse fields in protocol order (Table 2 → Table 3)
     *
     *     Mandatory fields ("M" in Table 2) return RID_ERR_PARSE_FAIL if
     *     their flag bit is not set.  Optional fields ("O") are skipped.
     * ================================================================= */

    /* -------- Flag byte 1 -------- */

    /* 001 — Unique product ID (SN), mandatory.
     *       ASCII, max 20 bytes. Per standard: trailing (high-address)
     *       bytes are NULL-padded when the ID is shorter than 20 chars.
     *       Validate BEFORE copying — don't write to output on error. */
    if (flag1 & 0x80) {
        if (p + 20 > end) return RID_ERR_PARSE_FAIL;
        if (!is_valid_ascii_field(p, 20))
            return RID_ERR_PARSE_FAIL;
        memcpy(data->sn, p, 20);
        data->sn[20] = '\0';
        p += 20;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 002 — Last 8 characters of real-name registration mark, mandatory.
     *       ASCII, 8 bytes, padded with NULL when shorter.
     *       Validate BEFORE copying — don't write to output on error. */
    if (flag1 & 0x40) {
        if (p + 8 > end) return RID_ERR_PARSE_FAIL;
        if (!is_valid_ascii_field(p, 8))
            return RID_ERR_PARSE_FAIL;
        memcpy(data->uin, p, 8);
        data->uin[8] = '\0';
        p += 8;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 003 — Operation category, optional.  Lower nibble. */
    if (flag1 & 0x20) {
        if (p + 1 > end) return RID_ERR_PARSE_FAIL;
        data->has_op_category = true;
        data->op_category = (OperationCategory_t)(*p & 0x0F);
        p += 1;
    }

    /* 004 — UAV class, mandatory.  Lower nibble. */
    if (flag1 & 0x10) {
        if (p + 1 > end) return RID_ERR_PARSE_FAIL;
        data->drone_class = (DroneClass_t)(*p & 0x0F);
        p += 1;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 005 — GCS position type, mandatory.  Lower nibble. */
    if (flag1 & 0x08) {
        if (p + 1 > end) return RID_ERR_PARSE_FAIL;
        data->gcs_pos_type = (PositionType_t)(*p & 0x0F);
        p += 1;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 006 — GCS longitude + latitude, mandatory.
     *       Signed int32 × 1e-7 degrees. 0xFFFFFFFF = invalid. */
    if (flag1 & 0x04) {
        if (p + 8 > end) return RID_ERR_PARSE_FAIL;
        data->gcs_lon = (uint32_t)le32_to_h(p)    == 0xFFFFFFFF
                            ? 0.0
                            : (double)(int32_t)le32_to_h(p) / 10000000.0;
        data->gcs_lat = (uint32_t)le32_to_h(p + 4) == 0xFFFFFFFF
                            ? 0.0
                            : (double)(int32_t)le32_to_h(p + 4) / 10000000.0;
        p += 8;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 007 — GCS geodetic altitude, mandatory.
     *       uint16 × 0.5 m, offset -1000 m. code == 0 → invalid. */
    if (flag1 & 0x02) {
        if (p + 2 > end) return RID_ERR_PARSE_FAIL;
        uint16_t code = le16_to_h(p);
        data->gcs_alt = (code == 0) ? -1000.0f : (float)code / 2.0f - 1000.0f;
        p += 2;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* -------- Flag byte 2 -------- */

    /* 008 — UAV longitude + latitude, mandatory.
     *       Same encoding as field 006. */
    if (flag2 & 0x80) {
        if (p + 8 > end) return RID_ERR_PARSE_FAIL;
        data->drone_lon = (uint32_t)le32_to_h(p)    == 0xFFFFFFFF
                              ? 0.0
                              : (double)(int32_t)le32_to_h(p) / 10000000.0;
        data->drone_lat = (uint32_t)le32_to_h(p + 4) == 0xFFFFFFFF
                              ? 0.0
                              : (double)(int32_t)le32_to_h(p + 4) / 10000000.0;
        p += 8;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 009 — Track angle, mandatory.
     *       uint16 × 0.1°, range 0-3599. 0xFFFF = invalid. */
    if (flag2 & 0x40) {
        if (p + 2 > end) return RID_ERR_PARSE_FAIL;
        uint16_t code = le16_to_h(p);
        if (code == 0xFFFF) {
            data->track_angle = -1.0f;
        } else if (code > 3599) {
            return RID_ERR_PARSE_FAIL;
        } else {
            data->track_angle = (float)code / 10.0f;
        }
        p += 2;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 010 — Ground speed, mandatory.
     *       uint16 × 0.1 m/s. 0xFFFF = invalid. */
    if (flag2 & 0x20) {
        if (p + 2 > end) return RID_ERR_PARSE_FAIL;
        uint16_t code = le16_to_h(p);
        data->ground_speed = (code == 0xFFFF) ? -1.0f : (float)code / 10.0f;
        p += 2;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 011 — Relative altitude (optional).
     *       uint16 × 0.5 m, offset -9000 m. code == 0 → invalid. */
    if (flag2 & 0x10) {
        if (p + 2 > end) return RID_ERR_PARSE_FAIL;
        data->has_rel_alt = true;
        uint16_t code = le16_to_h(p);
        data->rel_alt = (code == 0) ? -9000.0f : (float)code / 2.0f - 9000.0f;
        p += 2;
    }

    /* 012 — Vertical speed (optional).
     *       Signed 7-bit value × 0.5 m/s, bit 7 = sign, 0xFF = invalid. */
    if (flag2 & 0x08) {
        if (p + 1 > end) return RID_ERR_PARSE_FAIL;
        data->has_v_speed = true;
        uint8_t code = *p++;
        if (code == 0xFF) {
            data->vertical_speed = 0.0f;
        } else {
            float spd = (float)(code & 0x7F) / 2.0f;
            data->vertical_speed = (code & 0x80) ? -spd : spd;
        }
    }

    /* 013 — UAV geodetic altitude, mandatory.
     *       Same encoding as field 007. */
    if (flag2 & 0x04) {
        if (p + 2 > end) return RID_ERR_PARSE_FAIL;
        uint16_t code = le16_to_h(p);
        data->drone_alt = (code == 0) ? -1000.0f : (float)code / 2.0f - 1000.0f;
        p += 2;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 014 — Barometric altitude (optional).
     *       Same encoding as field 007. */
    if (flag2 & 0x02) {
        if (p + 2 > end) return RID_ERR_PARSE_FAIL;
        data->has_pressure_alt = true;
        uint16_t code = le16_to_h(p);
        data->pressure_alt = (code == 0) ? -1000.0f : (float)code / 2.0f - 1000.0f;
        p += 2;
    }

    /* -------- Flag byte 3 -------- */

    /* 015 — Operation status, mandatory. Lower nibble. */
    if (flag3 & 0x80) {
        if (p + 1 > end) return RID_ERR_PARSE_FAIL;
        data->op_status = (OperationStatus_t)(*p & 0x0F);
        p += 1;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 016 — Coordinate system, mandatory. Lower nibble. */
    if (flag3 & 0x40) {
        if (p + 1 > end) return RID_ERR_PARSE_FAIL;
        data->coord_system = (CoordSystem_t)(*p & 0x0F);
        p += 1;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 017 — Horizontal accuracy (NACp), mandatory. Lower nibble. */
    if (flag3 & 0x20) {
        if (p + 1 > end) return RID_ERR_PARSE_FAIL;
        data->horizontal_accuracy = *p++ & 0x0F;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 018 — Vertical accuracy (GVA), mandatory. Lower nibble. */
    if (flag3 & 0x10) {
        if (p + 1 > end) return RID_ERR_PARSE_FAIL;
        data->vertical_accuracy = *p++ & 0x0F;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 019 — Speed accuracy (NACv), mandatory. Lower nibble. */
    if (flag3 & 0x08) {
        if (p + 1 > end) return RID_ERR_PARSE_FAIL;
        data->speed_accuracy = *p++ & 0x0F;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 020 — Timestamp, mandatory. 6 bytes, little-endian, unit: ms.
     *       Unix epoch. Padded with zero when shorter. */
    if (flag3 & 0x04) {
        if (p + 6 > end) return RID_ERR_PARSE_FAIL;
        data->timestamp_ms = (uint64_t)p[0]        |
                             ((uint64_t)p[1] << 8)  |
                             ((uint64_t)p[2] << 16) |
                             ((uint64_t)p[3] << 24) |
                             ((uint64_t)p[4] << 32) |
                             ((uint64_t)p[5] << 40);
        p += 6;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    /* 021 — Timestamp accuracy, mandatory. Lower nibble. */
    if (flag3 & 0x02) {
        if (p + 1 > end) return RID_ERR_PARSE_FAIL;
        data->timestamp_accuracy = *p++ & 0x0F;
    } else {
        return RID_ERR_PARSE_FAIL;
    }

    return (num_flag_bytes > 3) ? RID_OK_EXTENSION : RID_OK;
}

/* ===================== Encode Function ===================== */

RID_Status_t CN46750_RID_Encode(const DroneRIDData_t *data,
                                uint8_t *buf, size_t buf_size,
                                size_t *out_len)
{
    if (data == NULL || buf == NULL) {
        return RID_ERR_PARAM;
    }

    /* --- Validate numeric fields: reject NaN/Inf to prevent UB on cast --- */
    if (!isfinite(data->gcs_lon)   || !isfinite(data->gcs_lat)   ||
        !isfinite(data->drone_lon) || !isfinite(data->drone_lat) ||
        !isfinite(data->gcs_alt)   || !isfinite(data->drone_alt) ||
        !isfinite(data->track_angle) || !isfinite(data->ground_speed)) {
        return RID_ERR_PARAM;
    }
    if ((data->has_rel_alt      && !isfinite(data->rel_alt))      ||
        (data->has_v_speed      && !isfinite(data->vertical_speed)) ||
        (data->has_pressure_alt && !isfinite(data->pressure_alt))) {
        return RID_ERR_PARAM;
    }

    /* --- Validate mandatory string fields are non-empty --- */
    if (data->sn[0]  == '\0' ||
        data->uin[0] == '\0') {
        return RID_ERR_PARAM;
    }

    /* --- Build flag bytes --- */
    uint8_t flag1 = 0x80  /* 001 SN */
                  | 0x40  /* 002 UIN */
                  | 0x10  /* 004 Drone class */
                  | 0x08  /* 005 GCS position type */
                  | 0x04  /* 006 GCS lat/lon */
                  | 0x02; /* 007 GCS altitude */

    uint8_t flag2 = 0x80  /* 008 Drone lat/lon */
                  | 0x40  /* 009 Track angle */
                  | 0x20  /* 010 Ground speed */
                  | 0x04; /* 013 Drone altitude */

    uint8_t flag3 = 0x80  /* 015 Operation status */
                  | 0x40  /* 016 Coordinate system */
                  | 0x20  /* 017 Horizontal accuracy */
                  | 0x10  /* 018 Vertical accuracy */
                  | 0x08  /* 019 Speed accuracy */
                  | 0x04  /* 020 Timestamp */
                  | 0x02; /* 021 Timestamp accuracy */

    if (data->has_op_category) flag1 |= 0x20;    /* 003 */
    if (data->has_rel_alt)     flag2 |= 0x10;    /* 011 */
    if (data->has_v_speed)     flag2 |= 0x08;    /* 012 */
    if (data->has_pressure_alt) flag2 |= 0x02;   /* 014 */

    /* Chain: flag byte 1 always has bits set → bit0=1; ditto flag2 */
    flag1 |= 0x01;
    flag2 |= 0x01;
    /* flag3 is last — bit0 stays 0 */

    /* --- Compute data_len --- */
    size_t dlen = 0;

    /* Flag 1 fields */
    dlen += 20;  /* 001 SN */
    dlen += 8;   /* 002 UIN */
    if (data->has_op_category) dlen += 1;  /* 003 */
    dlen += 1;   /* 004 Drone class */
    dlen += 1;   /* 005 GCS position type */
    dlen += 8;   /* 006 GCS lat/lon */
    dlen += 2;   /* 007 GCS altitude */

    /* Flag 2 fields */
    dlen += 8;   /* 008 Drone lat/lon */
    dlen += 2;   /* 009 Track angle */
    dlen += 2;   /* 010 Ground speed */
    if (data->has_rel_alt)     dlen += 2;  /* 011 */
    if (data->has_v_speed)     dlen += 1;  /* 012 */
    dlen += 2;   /* 013 Drone altitude */
    if (data->has_pressure_alt) dlen += 2;  /* 014 */

    /* Flag 3 fields */
    dlen += 1;   /* 015 Operation status */
    dlen += 1;   /* 016 Coordinate system */
    dlen += 1;   /* 017 Horizontal accuracy */
    dlen += 1;   /* 018 Vertical accuracy */
    dlen += 1;   /* 019 Speed accuracy */
    dlen += 6;   /* 020 Timestamp */
    dlen += 1;   /* 021 Timestamp accuracy */

    size_t total = 3 + 3 + dlen;  /* header(3) + flags(3) + data */
    if (buf_size < total) {
        return RID_ERR_INVALID_LEN;
    }

    /* --- Write header --- */
    buf[0] = 0xFF;        /* Type */
    buf[1] = 0x20;        /* Version (high 3 bits=001, sub-version 0) */
    /* DataLen = byte count of data items only (excludes flag bytes).
     * Per standard Table 1: 数据长度 = 数据内容项的字节数. */
    buf[2] = (uint8_t)dlen;

    /* --- Write flag bytes --- */
    buf[3] = flag1;
    buf[4] = flag2;
    buf[5] = flag3;

    uint8_t *p = &buf[6];

    /* ============ Flag byte 1 fields ============ */

    /* 001 — SN: 20 bytes ASCII.
     * Per standard: "高位以空字符NULL填充" — pad with \0 at the
     * trailing (high-address) end when the serial number is shorter
     * than 20 characters. Data starts at the low-address (first) byte. */
    {
        size_t slen = 0;
        while (slen < 20 && data->sn[slen] != '\0') slen++;
        for (int i = 0; i < 20; i++)
            p[i] = (i < (int)slen) ? (uint8_t)data->sn[i] : 0x00;
        p += 20;
    }

    /* 002 — UIN: 8 bytes ASCII.
     * Per standard: "高位补NULL" — pad with \0 at the trailing
     * (high-address) end when the registration mark is shorter. */
    {
        size_t slen = 0;
        while (slen < 8 && data->uin[slen] != '\0') slen++;
        for (int i = 0; i < 8; i++)
            p[i] = (i < (int)slen) ? (uint8_t)data->uin[i] : 0x00;
        p += 8;
    }

    /* 003 — Operation category (optional) */
    if (data->has_op_category) {
        *p++ = (uint8_t)(data->op_category & 0x0F);
    }

    /* 004 — Drone class */
    *p++ = (uint8_t)(data->drone_class & 0x0F);

    /* 005 — GCS position type */
    *p++ = (uint8_t)(data->gcs_pos_type & 0x0F);

    /* 006 — GCS latitude + longitude: int32 × 1e-7 */
    {
        int32_t lon = (int32_t)(data->gcs_lon * 10000000.0 +
                                (data->gcs_lon >= 0 ? 0.5 : -0.5));
        int32_t lat = (int32_t)(data->gcs_lat * 10000000.0 +
                                (data->gcs_lat >= 0 ? 0.5 : -0.5));
        h_to_le32(p,     (uint32_t)lon);
        h_to_le32(p + 4, (uint32_t)lat);
        p += 8;
    }

    /* 007 — GCS altitude: uint16 × 0.5 m, offset -1000 m */
    {
        uint16_t code = 0;
        if (data->gcs_alt > -1000.0f) {
            float v = (data->gcs_alt + 1000.0f) * 2.0f + 0.5f;
            if (v > 65535.0f) v = 65535.0f;
            if (v < 1.0f)     v = 1.0f;
            code = (uint16_t)v;
        }
        h_to_le16(p, code);
        p += 2;
    }

    /* ============ Flag byte 2 fields ============ */

    /* 008 — Drone longitude + latitude */
    {
        int32_t lon = (int32_t)(data->drone_lon * 10000000.0 +
                                (data->drone_lon >= 0 ? 0.5 : -0.5));
        int32_t lat = (int32_t)(data->drone_lat * 10000000.0 +
                                (data->drone_lat >= 0 ? 0.5 : -0.5));
        h_to_le32(p,     (uint32_t)lon);
        h_to_le32(p + 4, (uint32_t)lat);
        p += 8;
    }

    /* 009 — Track angle: uint16 × 0.1° */
    {
        uint16_t code = 0xFFFF;
        if (data->track_angle >= 0.0f) {
            /* Spec: "向下取整" (floor). C float→uint cast truncates
             * toward zero, which equals floor for non-negative values. */
            float v = data->track_angle * 10.0f;
            if (v > 3599.0f) v = 3599.0f;
            code = (uint16_t)v;
        }
        h_to_le16(p, code);
        p += 2;
    }

    /* 010 — Ground speed: uint16 × 0.1 m/s */
    {
        uint16_t code = 0xFFFF;
        if (data->ground_speed >= 0.0f) {
            /* Spec: "向下取整" (floor). C float→uint cast truncates
             * toward zero, which equals floor for non-negative values. */
            float v = data->ground_speed * 10.0f;
            if (v > 65534.0f) v = 65534.0f;
            code = (uint16_t)v;
        }
        h_to_le16(p, code);
        p += 2;
    }

    /* 011 — Relative altitude (optional): uint16 × 0.5 m, offset -9000 m */
    if (data->has_rel_alt) {
        uint16_t code = 0;
        if (data->rel_alt > -9000.0f) {
            float v = (data->rel_alt + 9000.0f) * 2.0f + 0.5f;
            if (v > 65535.0f) v = 65535.0f;
            if (v < 1.0f)     v = 1.0f;
            code = (uint16_t)v;
        }
        h_to_le16(p, code);
        p += 2;
    }

    /* 012 — Vertical speed (optional): sign + 7-bit × 0.5 m/s */
    if (data->has_v_speed) {
        float abs_spd = (data->vertical_speed >= 0.0f)
            ? data->vertical_speed : -data->vertical_speed;
        float mag_f = abs_spd * 2.0f + 0.5f;
        if (mag_f > 127.0f) mag_f = 127.0f;
        uint8_t mag = (uint8_t)mag_f;
        if (mag > 0x7F) mag = 0x7F;
        uint8_t code = mag;
        if (data->vertical_speed < 0.0f) code |= 0x80;
        *p++ = code;
    }

    /* 013 — Drone altitude */
    {
        uint16_t code = 0;
        if (data->drone_alt > -1000.0f) {
            float v = (data->drone_alt + 1000.0f) * 2.0f + 0.5f;
            if (v > 65535.0f) v = 65535.0f;
            if (v < 1.0f)     v = 1.0f;
            code = (uint16_t)v;
        }
        h_to_le16(p, code);
        p += 2;
    }

    /* 014 — Barometric altitude (optional) */
    if (data->has_pressure_alt) {
        uint16_t code = 0;
        if (data->pressure_alt > -1000.0f) {
            float v = (data->pressure_alt + 1000.0f) * 2.0f + 0.5f;
            if (v > 65535.0f) v = 65535.0f;
            if (v < 1.0f)     v = 1.0f;
            code = (uint16_t)v;
        }
        h_to_le16(p, code);
        p += 2;
    }

    /* ============ Flag byte 3 fields ============ */

    /* 015 — Operation status */
    *p++ = (uint8_t)(data->op_status & 0x0F);

    /* 016 — Coordinate system */
    *p++ = (uint8_t)(data->coord_system & 0x0F);

    /* 017 — Horizontal accuracy */
    *p++ = (uint8_t)(data->horizontal_accuracy & 0x0F);

    /* 018 — Vertical accuracy */
    *p++ = (uint8_t)(data->vertical_accuracy & 0x0F);

    /* 019 — Speed accuracy */
    *p++ = (uint8_t)(data->speed_accuracy & 0x0F);

    /* 020 — Timestamp: 6 bytes LE, ms */
    {
        uint64_t ts = data->timestamp_ms;
        p[0] = (uint8_t)(ts & 0xFF);
        p[1] = (uint8_t)((ts >> 8) & 0xFF);
        p[2] = (uint8_t)((ts >> 16) & 0xFF);
        p[3] = (uint8_t)((ts >> 24) & 0xFF);
        p[4] = (uint8_t)((ts >> 32) & 0xFF);
        p[5] = (uint8_t)((ts >> 40) & 0xFF);
        p += 6;
    }

    /* 021 — Timestamp accuracy */
    *p++ = (uint8_t)(data->timestamp_accuracy & 0x0F);

    if (out_len) *out_len = total;
    return RID_OK;
}

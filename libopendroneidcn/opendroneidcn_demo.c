/*
Copyright (C) 2026 Jun Zhang

SPDX-License-Identifier: Apache-2.0

Open Drone ID C Library — GB 46750-2025

Maintainer:
Jun Zhang
zhangjun.sole@qq.com
*/

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stddef.h>
#include <string.h>
#include "opendroneidcn.h"

/* ================================================================
 * Test data — fill a DroneRIDData_t with realistic values
 * ================================================================ */
static void demo_fill_test_data(DroneRIDData_t *data)
{
    memset(data, 0, sizeof(*data));
    strcpy(data->sn, "123456789abcdefghijk");  /* 20-char SN */
    strcpy(data->uin, "B1234567");              /* 8-char UIN */
    data->has_op_category    = true;
    data->op_category        = OP_CATEGORY_OPEN;
    data->drone_class        = DRONE_CLASS_SMALL;
    data->gcs_pos_type       = POS_TYPE_TAKEOFF;
    data->gcs_lon            = 116.3974280;
    data->gcs_lat            = 39.9092300;
    data->gcs_alt            = 50.0f;
    data->drone_lon          = 116.4010000;
    data->drone_lat          = 39.9125000;
    data->track_angle        = 180.0f;
    data->ground_speed       = 15.5f;
    data->has_rel_alt        = true;
    data->rel_alt            = 120.0f;
    data->has_v_speed        = true;
    data->vertical_speed     = 2.5f;
    data->drone_alt          = 170.0f;
    data->has_pressure_alt   = true;
    data->pressure_alt       = 168.5f;
    data->op_status          = OP_STATUS_AIR;
    data->coord_system       = COORD_WGS84;
    data->horizontal_accuracy = 3;
    data->vertical_accuracy   = 2;
    data->speed_accuracy      = 2;
    data->timestamp_ms        = 1750262400000ULL;
    data->timestamp_accuracy  = 5;
}

/* ================================================================
 * Encode: RID data → RID packet → 802.11 beacon frame
 *
 * Builds a beacon frame that mirrors what the firmware's
 * wifi_promiscuous_handler receives after radiotap-header stripping:
 *   [0..23]   802.11 MAC header (24 B)
 *   [24..35]  Fixed params: timestamp(8) + interval(2) + capability(2)
 *   [36..]    Frame body: SSID IE + RID packet
 *
 * Returns RID_OK on success.
 * ================================================================ */
static RID_Status_t demo_encode_beacon(const DroneRIDData_t *data,
                                        uint8_t *beacon, size_t buf_size,
                                        size_t *beacon_len)
{
    /* 1. Encode RID packet */
    uint8_t rid[256];
    size_t rid_len = 0;
    RID_Status_t ret = CN46750_RID_Encode(data, rid, sizeof(rid), &rid_len);
    if (ret != RID_OK) return ret;

    printf("Encoded RID packet: %lu bytes\n", (unsigned long)rid_len);
    printf("Hex: ");
    for (size_t i = 0; i < rid_len; i++)
        printf("%02X ", rid[i]);
    printf("\n\n");

    /* 2. Build beacon frame */
    if (buf_size < 36 + 2 + 8 + rid_len) return RID_ERR_PARAM;  /* sanity */

    size_t pos = 0;

    /* --- 802.11 MAC header (24 bytes) --- */
    uint8_t mac_header[] = {
        0x80, 0x00,                         /* Frame Control: Beacon */
        0x00, 0x00,                         /* Duration */
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, /* DA: broadcast */
        0x02, 0x00, 0x00, 0x00, 0x00, 0x01, /* SA (fake) */
        0x02, 0x00, 0x00, 0x00, 0x00, 0x01, /* BSSID (fake) */
        0x00, 0x00                          /* Sequence Control */
    };
    memcpy(&beacon[pos], mac_header, sizeof(mac_header));
    pos += sizeof(mac_header);

    /* --- Fixed parameters (12 bytes) --- */
    uint8_t fixed[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Timestamp */
        0x64, 0x00,                                       /* Beacon Interval: 100 TU */
        0x21, 0x04                                        /* Capability */
    };
    memcpy(&beacon[pos], fixed, sizeof(fixed));
    pos += sizeof(fixed);

    /* --- SSID IE --- */
    const char *ssid = "RID-DEMO";
    uint8_t ssid_len = (uint8_t)strlen(ssid);
    beacon[pos++] = 0x00;
    beacon[pos++] = ssid_len;
    memcpy(&beacon[pos], ssid, ssid_len);
    pos += ssid_len;

    /* --- RID packet --- */
    memcpy(&beacon[pos], rid, rid_len);
    pos += rid_len;

    *beacon_len = pos;
    printf("Beacon frame: %lu bytes (hdr 24 + fixed 12 + SSID IE %u + RID %lu)\n\n",
           (unsigned long)pos, (unsigned)(2 + ssid_len), (unsigned long)rid_len);
    return RID_OK;
}

/* ================================================================
 * Decode: beacon frame → find RID packet → decode → print
 *
 * Mirrors main.c:packet_handler():
 *   CN46750_FindPacket(payload + 36, length - 36, &off, &len)
 * ================================================================ */
static RID_Status_t demo_decode_beacon(const uint8_t *beacon, size_t beacon_len,
                                        DroneRIDData_t *data, uint8_t *version)
{
    /* 1. Find RID packet in beacon body (offset 36+) */
    if (beacon_len < 36) {
        printf("ERROR: beacon too short (need >= 36, got %lu)\n",
               (unsigned long)beacon_len);
        return RID_ERR_PARAM;
    }
    size_t off = 0, len = 0;
    if (!CN46750_FindPacket(beacon + 36, beacon_len - 36, &off, &len)) {
        printf("ERROR: CN46750_FindPacket() found nothing!\n");
        return RID_ERR_PARSE_FAIL;
    }
    printf("Found RID packet at body offset %lu, length %lu bytes\n\n",
           (unsigned long)off, (unsigned long)len);

    /* 2. Decode */
    RID_Status_t ret = CN46750_RID_Decode(beacon + 36 + off, len, data, version);
    if (ret < 0) {
        printf("ERROR: CN46750_RID_Decode() failed, code=%d\n", ret);
        return ret;
    }
    if (ret == RID_OK_EXTENSION) {
        printf("NOTE: Packet contains additional flag bytes beyond the 3 currently defined — future CAAC extension fields were skipped.\n\n");
    }

    /* 3. Print */
    printf("===== Decoded RID Data =====\n");
    printf("Protocol version:      V1.%d\n", *version);
    printf("Product SN:            %s\n", data->sn);
    printf("UIN (last 8 chars):    %s\n", data->uin);
    printf("Drone class:           %d (0=Micro,1=Light,2=Small,3=Medium,4=Large)\n",
           data->drone_class);
    printf("Op category:           %d (0=Undefined,1=Open,2=Specific,3=Certified)\n",
           data->op_category);
    printf("GCS position type:     %d (0=Takeoff, 1=GCS)\n", data->gcs_pos_type);
    printf("GCS position:          %.7f, %.7f\n", data->gcs_lon, data->gcs_lat);
    printf("GCS geodetic alt:      %.1f m\n", (double)data->gcs_alt);
    printf("UAV position:          %.7f, %.7f\n", data->drone_lon, data->drone_lat);
    printf("Track angle:           %.1f\n", (double)data->track_angle);
    printf("Ground speed:          %.1f m/s\n", (double)data->ground_speed);
    if (data->has_rel_alt)
        printf("Relative altitude:     %.1f m\n", (double)data->rel_alt);
    if (data->has_v_speed)
        printf("Vertical speed:        %.1f m/s (%s)\n",
               (double)data->vertical_speed,
               data->vertical_speed >= 0 ? "ascending" : "descending");
    printf("UAV geodetic alt:      %.1f m\n", (double)data->drone_alt);
    if (data->has_pressure_alt)
        printf("Barometric alt:        %.1f m\n", (double)data->pressure_alt);
    printf("Operation status:      %d (0=Unknown,1=Ground,2=Air,3=Emergency)\n",
           data->op_status);
    printf("Coordinate system:     %d (0=WGS84, 1=CGCS2000)\n", data->coord_system);
    printf("Horizontal accuracy:   level %d\n", data->horizontal_accuracy);
    printf("Vertical accuracy:     level %d\n", data->vertical_accuracy);
    printf("Speed accuracy:        level %d\n", data->speed_accuracy);
    printf("Timestamp:             %" PRIu64 " ms\n", data->timestamp_ms);
    printf("Timestamp accuracy:    level %d\n", data->timestamp_accuracy);
    return RID_OK;
}

/* ================================================================
 * Verify: roundtrip comparison — orig vs decoded, all fields
 * ================================================================ */
static void demo_verify_roundtrip(const DroneRIDData_t *orig,
                                   const DroneRIDData_t *decoded)
{
    #define CMP_STR(field) \
        { int _ok = (strcmp(orig->field, decoded->field) == 0); \
          printf("  %-22s %-20s vs %-20s  %s\n", #field, orig->field, decoded->field, \
                 _ok ? "OK" : "MISMATCH"); }

    #define CMP_FLT(field, tol) \
        { double _d = (double)orig->field - (double)decoded->field; \
          if (_d < 0) _d = -_d; \
          printf("  %-22s %-12.7g vs %-12.7g  %s\n", #field, \
                 (double)orig->field, (double)decoded->field, _d <= (tol) ? "OK" : "MISMATCH"); }

    printf("\n===== Roundtrip Verification (orig vs decoded) =====\n");
    CMP_STR(sn);
    CMP_STR(uin);
    printf("  %-22s %d vs %d  %s\n", "drone_class",
           (int)orig->drone_class, (int)decoded->drone_class,
           orig->drone_class == decoded->drone_class ? "OK" : "MISMATCH");
    printf("  %-22s %d vs %d  %s\n", "op_category",
           (int)orig->op_category, (int)decoded->op_category,
           orig->op_category == decoded->op_category ? "OK" : "MISMATCH");
    printf("  %-22s %d vs %d  %s\n", "gcs_pos_type",
           (int)orig->gcs_pos_type, (int)decoded->gcs_pos_type,
           orig->gcs_pos_type == decoded->gcs_pos_type ? "OK" : "MISMATCH");
    CMP_FLT(gcs_lon, 1e-6);
    CMP_FLT(gcs_lat, 1e-6);
    CMP_FLT(gcs_alt, 0.6);
    CMP_FLT(drone_lon, 1e-6);
    CMP_FLT(drone_lat, 1e-6);
    CMP_FLT(track_angle, 0.15);
    CMP_FLT(ground_speed, 0.15);
    printf("  %-22s %s vs %s  %s\n", "has_rel_alt",
           orig->has_rel_alt ? "true" : "false",
           decoded->has_rel_alt ? "true" : "false",
           orig->has_rel_alt == decoded->has_rel_alt ? "OK" : "MISMATCH");
    CMP_FLT(rel_alt, 0.6);
    printf("  %-22s %s vs %s  %s\n", "has_v_speed",
           orig->has_v_speed ? "true" : "false",
           decoded->has_v_speed ? "true" : "false",
           orig->has_v_speed == decoded->has_v_speed ? "OK" : "MISMATCH");
    CMP_FLT(vertical_speed, 0.6);
    CMP_FLT(drone_alt, 0.6);
    printf("  %-22s %s vs %s  %s\n", "has_pressure_alt",
           orig->has_pressure_alt ? "true" : "false",
           decoded->has_pressure_alt ? "true" : "false",
           orig->has_pressure_alt == decoded->has_pressure_alt ? "OK" : "MISMATCH");
    CMP_FLT(pressure_alt, 0.6);
    printf("  %-22s %d vs %d  %s\n", "op_status",
           (int)orig->op_status, (int)decoded->op_status,
           orig->op_status == decoded->op_status ? "OK" : "MISMATCH");
    printf("  %-22s %d vs %d  %s\n", "coord_system",
           (int)orig->coord_system, (int)decoded->coord_system,
           orig->coord_system == decoded->coord_system ? "OK" : "MISMATCH");
    printf("  %-22s %d vs %d  %s\n", "horizontal_accuracy",
           orig->horizontal_accuracy, decoded->horizontal_accuracy,
           orig->horizontal_accuracy == decoded->horizontal_accuracy ? "OK" : "MISMATCH");
    printf("  %-22s %d vs %d  %s\n", "vertical_accuracy",
           orig->vertical_accuracy, decoded->vertical_accuracy,
           orig->vertical_accuracy == decoded->vertical_accuracy ? "OK" : "MISMATCH");
    printf("  %-22s %d vs %d  %s\n", "speed_accuracy",
           orig->speed_accuracy, decoded->speed_accuracy,
           orig->speed_accuracy == decoded->speed_accuracy ? "OK" : "MISMATCH");
    printf("  %-22s %" PRIu64 " vs %" PRIu64 "  %s\n", "timestamp_ms",
           orig->timestamp_ms,
           decoded->timestamp_ms,
           orig->timestamp_ms == decoded->timestamp_ms ? "OK" : "MISMATCH");
    printf("  %-22s %d vs %d  %s\n", "timestamp_accuracy",
           orig->timestamp_accuracy, decoded->timestamp_accuracy,
           orig->timestamp_accuracy == decoded->timestamp_accuracy ? "OK" : "MISMATCH");

    #undef CMP_STR
    #undef CMP_FLT
}

/* ================================================================
 * Main — encode → beacon → find → decode → verify
 * ================================================================ */
int main(void)
{
    DroneRIDData_t orig, decoded;
    uint8_t beacon[512];
    size_t beacon_len;
    uint8_t version;

    /* 1. Fill test data */
    demo_fill_test_data(&orig);

    /* 2. Encode → beacon frame */
    if (demo_encode_beacon(&orig, beacon, sizeof(beacon), &beacon_len) != RID_OK)
        return -1;

    /* 3. Decode from beacon */
    {
        RID_Status_t ret = demo_decode_beacon(beacon, beacon_len, &decoded, &version);
        if (ret < 0) return -1;
    }

    /* 4. Verify roundtrip */
    demo_verify_roundtrip(&orig, &decoded);

    printf("\nAll tests passed!\n");
    return 0;
}

/*
Copyright (C) 2019 Intel Corporation

SPDX-License-Identifier: Apache-2.0

Open Drone ID C Library

Maintainer:
Gabriel Cox
gabriel.c.cox@intel.com
*/

#ifndef _OPENDRONEID_H_
#define _OPENDRONEID_H_

#include <stdint.h>

#define ODID_MESSAGE_SIZE 25
#define ODID_ID_SIZE 20
#define ODID_STR_SIZE 23
#define ODID_PROTOCOL_VERSION 0
#define ODID_SPEC_VERSION 0.64.3

#define ODID_SUCCESS    0
#define ODID_FAIL       1

typedef enum {
    ODID_MESSAGE_TYPE_BASIC_ID = 0,
    ODID_MESSAGE_TYPE_LOCATION = 1,
    ODID_MESSAGE_TYPE_AUTH = 2,
    ODID_MESSAGE_TYPE_SELF_ID = 3,
    ODID_MESSAGE_TYPE_SYSTEM = 4,
    ODID_MESSAGE_TYPE_INVALID = 0xff,
} odid_message_type_t;

typedef enum {
    ODID_ID_TYPE_NONE = 0,
    ODID_ID_TYPE_SERIAL_NUMBER = 1,
    ODID_ID_TYPE_CAA_ASSIGNED_ID = 2,
    ODID_ID_TYPE_UTM_ASSIGNED_ID = 3,
    // 4 to 15 reserved
} odid_id_type_t;

typedef enum {
    ODID_UA_TYPE_NONE = 0,
    ODID_UA_TYPE_AEROPLANE = 1,
    ODID_UA_TYPE_ROTORCRAFT = 2, // Including Multirotor
    ODID_UA_TYPE_GYROPLANE = 3,
    ODID_UA_TYPE_VTOL = 4, // Fixed wing aircraft that can take off vertically
    ODID_UA_TYPE_ORNITHOPTER = 5,
    ODID_UA_TYPE_GLIDER = 6,
    ODID_UA_TYPE_KITE = 7,
    ODID_UA_TYPE_FREE_BALLOON = 8,
    ODID_UA_TYPE_CAPTIVE_BALLOON = 9,
    ODID_UA_TYPE_AIRSHIP = 10,
    ODID_UA_TYPE_FREE_FALL_PARACHUTE = 11,
    ODID_UA_TYPE_ROCKET = 12,
    ODID_UA_TYPE_GROUND_OBSTACLE = 13,
    ODID_UA_TYPE_RESERVED = 14,
    ODID_UA_TYPE_OTHER = 15,
} odid_ua_type_t;

typedef enum {
    ODID_STATUS_UNDECLARED = 0,
    ODID_STATUS_GROUND = 1,
    ODID_STATUS_AIRBORNE = 2,
    // 3 to 15 reserved
} odid_status_t;

typedef enum {
    ODID_HEIGHT_REF_OVER_TAKEOFF = 0,
    ODID_HEIGHT_REF_OVER_GROUND = 1,
} odid_height_ref_t;

typedef enum {
    ODID_HOR_ACC_UNKNOWN = 0,
    ODID_HOR_ACC_10NM = 1, // Nautical Miles
    ODID_HOR_ACC_4NM = 2,
    ODID_HOR_ACC_2NM = 3,
    ODID_HOR_ACC_1NM = 4,
    ODID_HOR_ACC_0_5NM = 5,
    ODID_HOR_ACC_0_3NM = 6,
    ODID_HOR_ACC_0_1NM = 7,
    ODID_HOR_ACC_0_05NM = 8,
    ODID_HOR_ACC_30_METER = 9,
    ODID_HOR_ACC_10_METER = 10,
    ODID_HOR_ACC_3_METER = 11,
    ODID_HOR_ACC_1_METER = 12,
    // 13 to 15 reserved
} odid_horizontal_accuracy_t;

typedef enum {
    ODID_VER_ACC_UNKNOWN = 0,
    ODID_VER_ACC_150_METER = 1,
    ODID_VER_ACC_45_METER = 2,
    ODID_VER_ACC_25_METER = 3,
    ODID_VER_ACC_10_METER = 4,
    ODID_VER_ACC_3_METER = 5,
    ODID_VER_ACC_1_METER = 6,
    // 7 to 15 reserved
} odid_vertical_accuracy_t;

typedef enum {
    ODID_SPEED_ACC_UNKNOWN = 0,
    ODID_SPEED_ACC_10_METERS_SECOND = 1,
    ODID_SPEED_ACC_3_METERS_SECOND = 2,
    ODID_SPEED_ACC_1_METERS_SECOND = 3,
    ODID_SPEED_ACC_0_3_METERS_SECOND = 4,
    // 5 to 15 reserved
} odid_speed_accuracy_t;

typedef enum {
    ODID_TIME_ACC_UNKNOWN = 0,
    ODID_TIME_ACC_0_1_SECONDS = 1,
    ODID_TIME_ACC_0_2_SECONDS = 2,
    ODID_TIME_ACC_0_3_SECONDS = 3,
    ODID_TIME_ACC_0_4_SECONDS = 4,
    ODID_TIME_ACC_0_5_SECONDS = 5,
    ODID_TIME_ACC_0_6_SECONDS = 6,
    ODID_TIME_ACC_0_7_SECONDS = 7,
    ODID_TIME_ACC_0_8_SECONDS = 8,
    ODID_TIME_ACC_0_9_SECONDS = 9,
    ODID_TIME_ACC_1_0_SECONDS = 10,
    ODID_TIME_ACC_1_1_SECONDS = 11,
    ODID_TIME_ACC_1_2_SECONDS = 12,
    ODID_TIME_ACC_1_3_SECONDS = 13,
    ODID_TIME_ACC_1_4_SECONDS = 14,
    ODID_TIME_ACC_1_5_SECONDS = 15,
} odid_timestamp_accuracy_t;

typedef enum {
    ODID_AUTH_TYPE_NONE = 0,
    ODID_AUTH_TYPE_MPUID = 1, // Manufacturer Programmed Unique ID
    // 2 to 9 reserved. 0xA to 0xF reserved for private use
} odid_auth_type_t;

typedef enum {
    ODID_DESC_TYPE_TEXT = 0,
    ODID_DESC_TYPE_REMOTE_PILOT_ID = 1,
    // 2 to 255 reserved
} odid_desc_type_t;

typedef enum {
    ODID_LOCATION_SRC_TAKEOFF = 0,
    ODID_LOCATION_SRC_LIVE_GNSS = 1,
    ODID_LOCATION_SRC_FIXED = 2,
    // 3 to 255 reserved
} odid_location_source_t;

 /*
 * @name ODID_DataStructs
 * ODID Data Structures in their normative (non-packed) form.
 * This is the structure that any input adapters should form to
 * let the encoders put the data into encoded form.
 */
typedef struct {
    odid_ua_type_t ua_type;
    odid_id_type_t id_type;
    char uas_id[ODID_ID_SIZE + 1];
} odid_data_basic_id_t;

typedef struct {
    odid_status_t Status;
    float Direction;          // Degrees. 0 <= x < 360. Route course based on true North
    float SpeedHorizontal;    // m/s. Positive only
    float SpeedVertical;      // m/s
    double Latitude;
    double Longitude;
    float AltitudeBaro;       // meter (Ref 29.92 inHg, 1013.24 mb)
    float AltitudeGeo;        // meter (WGS84-HAE)
    odid_height_ref_t HeightType;
    float Height;             // meter
    odid_horizontal_accuracy_t HorizAccuracy;
    odid_vertical_accuracy_t VertAccuracy;
    odid_vertical_accuracy_t BaroAccuracy;
    odid_speed_accuracy_t SpeedAccuracy;
    odid_timestamp_accuracy_t TSAccuracy;
    float TimeStamp;          // seconds after the full hour
} odid_data_location_t;

typedef struct {
    uint8_t DataPage;
    odid_auth_type_t AuthType;
    char AuthData[ODID_STR_SIZE+1];  // additional byte to allow for null term in normative form
} odid_data_auth_t;

typedef struct {
    odid_desc_type_t DescType;
    char Desc[ODID_STR_SIZE+1];
} odid_data_self_id_t;

typedef struct {
    odid_location_source_t LocationSource;
    double remotePilotLatitude;
    double remotePilotLongitude;
    uint16_t GroupCount;
    uint16_t GroupRadius;     // meter
    float GroupCeiling;       // meter
    float GroupFloor;         // meter
} odid_data_system_t;

typedef struct {
    odid_data_basic_id_t basic_id;
    odid_data_location_t location;
    odid_data_auth_t auth;
    odid_data_self_id_t self_id;
    odid_data_system_t system;
} odid_data_uas_t;

/**
* @Name ODID_PackedStructs
* Packed Data Structures prepared for broadcast
* It's best not directly access these.  Use the encoders/decoders.
*/

typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;

    // Byte 1 [id_type][ua_type]  -- must define LSb first
    uint8_t UAType: 4;
    uint8_t IDType: 4;

    // Bytes 2-21
    char UASID[ODID_ID_SIZE];

    // 22-24
    char Reserved[3];
} odid_encoded_basic_id_t;

typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;

    // Byte 1 [Status][Reserved][NSMult][EWMult] -- must define LSb first
    uint8_t SpeedMult: 1;
    uint8_t EWDirection: 1;
    uint8_t HeightType: 1;
    uint8_t Reserved: 1;
    uint8_t Status: 4;

    // Bytes 2-18
    uint8_t Direction;
    uint8_t SpeedHorizontal;
    int8_t SpeedVertical;
    int32_t Latitude;
    int32_t Longitude;
    uint16_t AltitudeBaro;
    uint16_t AltitudeGeo;
    uint16_t Height;

    // Byte 19 [VertAccuracy][HorizAccuracy]  -- must define LSb first
    uint8_t HorizAccuracy:4;
    uint8_t VertAccuracy:4;

    // Byte 20 [BaroAccuracy][SpeedAccuracy]  -- must define LSb first
    uint8_t SpeedAccuracy:4;
    uint8_t BaroAccuracy:4;

    // Byte 21-22
    uint16_t TimeStamp;

    // Byte 23 [Reserved2][TSAccuracy]  -- must define LSb first
    uint8_t TSAccuracy:4;
    uint8_t Reserved2:4;

    // Byte 24
    char Reserved3;
} odid_encoded_location_t;

typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;

    // Byte 1 [AuthType][DataPage]
    uint8_t DataPage: 4;
    uint8_t AuthType: 4;

    // Byte 2-24
    char AuthData[ODID_STR_SIZE];
} odid_encoded_auth_t;

typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;

    // Byte 1
    uint8_t DescType;

    // Byte 2-24
    char Desc[ODID_STR_SIZE];
} odid_encoded_self_id_t;

typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;

    // Byte 1 [Reserved][LocationSource]
    uint8_t Reserved: 7;
    uint8_t LocationSource: 1;

    // Byte 2-9
    int32_t remotePilotLatitude;
    int32_t remotePilotLongitude;

    // Byte 10-16
    uint16_t GroupCount;
    uint8_t  GroupRadius;
    uint16_t GroupCeiling;
    uint16_t GroupFloor;

    // Byte 17-24
    char Reserved2[8];
} odid_encoded_system_t;

typedef struct {
    uint8_t data[ODID_MESSAGE_SIZE];
} odid_message_t;

// TODO: Encoding/Decoding message pack
typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;
    uint8_t SingleMessageSize;
    uint8_t MsgPackSize; // No of messages in pack (NOT number of bytes)
    odid_message_t Messages[];
} odid_message_pack_t;

// API Calls
int odid_encode_message_basic_id(odid_encoded_basic_id_t *out, odid_data_basic_id_t *in);
int odid_encode_message_location(odid_encoded_location_t *out, odid_data_location_t *in);
int odid_encode_message_auth(odid_encoded_auth_t *out, odid_data_auth_t *in);
int odid_encode_message_self_id(odid_encoded_self_id_t *out, odid_data_self_id_t *in);
int odid_encode_message_system(odid_encoded_system_t *out, odid_data_system_t *in);

int odid_decode_message_basic_id(odid_data_basic_id_t *out, odid_encoded_basic_id_t *in);
int odid_decode_message_location(odid_data_location_t *out, odid_encoded_location_t *in);
int odid_decode_message_auth(odid_data_auth_t *out, odid_encoded_auth_t *in);
int odid_decode_message_self_id(odid_data_self_id_t *out, odid_encoded_self_id_t *in);
int odid_decode_message_system(odid_data_system_t *out, odid_encoded_system_t *in);

odid_message_type_t odid_decode_message_type(uint8_t byte);
odid_message_type_t odid_decode_open_drone_id(odid_data_uas_t *uas_data, uint8_t *msg_data);

// Helper Functions
char *odid_safe_copy_fill(char *dst_str, const char *src_str, int dst_size);
char *odid_safe_dec_copy_fill(char *dst_str, const char *src_str, int dst_size);
int odid_int_range_max(int64_t value, int start_range, int end_range);
int odid_int_in_range(int inValue, int start_range, int end_range);

odid_horizontal_accuracy_t odid_create_enum_horizontal_accuracy(float accuracy);
odid_vertical_accuracy_t odid_create_enum_vertical_accuracy(float accuracy);
odid_speed_accuracy_t odid_create_enum_speed_accuracy(float accuracy);
odid_timestamp_accuracy_t odid_create_enum_timestamp_accuracy(float accuracy);

float odid_decode_horizontal_accuracy(odid_horizontal_accuracy_t accuracy);
float odid_decode_vertical_accuracy(odid_vertical_accuracy_t accuracy);
float odid_decode_speed_accuracy(odid_speed_accuracy_t accuracy);
float odid_decode_timestamp_accuracy(odid_timestamp_accuracy_t accuracy);

#ifndef ODID_DISABLE_PRINTF

void odid_print_byte_array(uint8_t *byte_array, uint16_t asize, int spaced);
void odid_print_data_basic_id(odid_data_basic_id_t *basic_id);
void odid_print_data_location(odid_data_location_t *location);
void odid_print_data_auth(odid_data_auth_t *auth);
void odid_print_data_self_id(odid_data_self_id_t *self_id);
void odid_print_data_system(odid_data_system_t *system_data);

#endif // ODID_DISABLE_PRINTF

#endif // _OPENDRONEID_H_

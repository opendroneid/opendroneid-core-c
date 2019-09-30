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
#define ODID_SPEC_VERSION 0.8
#define ODID_AUTH_MAX_PAGES 5
#define ODID_AUTH_PAGE_0_DATA_SIZE 6
#define ODID_PACK_MAX_MESSAGES 10

#define ODID_SUCCESS    0
#define ODID_FAIL       1

typedef enum ODID_messagetype {
    ODID_MESSAGETYPE_BASIC_ID = 0,
    ODID_MESSAGETYPE_LOCATION = 1,
    ODID_MESSAGETYPE_AUTH = 2,
    ODID_MESSAGETYPE_SELF_ID = 3,
    ODID_MESSAGETYPE_SYSTEM = 4,
    ODID_MESSAGETYPE_OPERATOR_ID = 5,
    ODID_MESSAGETYPE_PACKED = 0xF,
    ODID_MESSAGETYPE_INVALID = 0xFF,
} ODID_messagetype_t;

typedef enum ODID_idtype {
    ODID_IDTYPE_NONE = 0,
    ODID_IDTYPE_SERIAL_NUMBER = 1,
    ODID_IDTYPE_CAA_REGISTRATION_ID = 2, // Civil Aviation Authority
    ODID_IDTYPE_UTM_ASSIGNED_UUID = 3, // UAS (Unmanned Aircraft System) Traffic Management
    // 4 to 15 reserved
} ODID_idtype_t;

typedef enum ODID_uatype {
    ODID_UATYPE_NONE = 0,
    ODID_UATYPE_AEROPLANE = 1, // Fixed wing
    ODID_UATYPE_ROTORCRAFT = 2, // Including Multirotor
    ODID_UATYPE_GYROPLANE = 3,
    ODID_UATYPE_VTOL = 4, // Fixed wing aircraft that can take off vertically
    ODID_UATYPE_ORNITHOPTER = 5,
    ODID_UATYPE_GLIDER = 6,
    ODID_UATYPE_KITE = 7,
    ODID_UATYPE_FREE_BALLOON = 8,
    ODID_UATYPE_CAPTIVE_BALLOON = 9,
    ODID_UATYPE_AIRSHIP = 10, // Such as a blimp
    ODID_UATYPE_FREE_FALL_PARACHUTE = 11,
    ODID_UATYPE_ROCKET = 12,
    ODID_UATYPE_TETHERED_POWERED_AIRCRAFT = 13,
    ODID_UATYPE_GROUND_OBSTACLE = 14,
    ODID_UATYPE_OTHER = 15,
} ODID_uatype_t;

typedef enum ODID_status {
    ODID_STATUS_UNDECLARED = 0,
    ODID_STATUS_GROUND = 1,
    ODID_STATUS_AIRBORNE = 2,
    // 3 to 15 reserved
} ODID_status_t;

typedef enum ODID_Height_reference {
    ODID_HEIGHT_REF_OVER_TAKEOFF = 0,
    ODID_HEIGHT_REF_OVER_GROUND = 1,
} ODID_Height_reference_t;

typedef enum ODID_Horizontal_accuracy {
    ODID_HOR_ACC_UNKNOWN = 0,
    ODID_HOR_ACC_10NM = 1,      // Nautical Miles. 18.52 km
    ODID_HOR_ACC_4NM = 2,       // 7.408 km
    ODID_HOR_ACC_2NM = 3,       // 3.704 km
    ODID_HOR_ACC_1NM = 4,       // 1.852 km
    ODID_HOR_ACC_0_5NM = 5,     // 926 m
    ODID_HOR_ACC_0_3NM = 6,     // 555.6 m
    ODID_HOR_ACC_0_1NM = 7,     // 185.2 m
    ODID_HOR_ACC_0_05NM = 8,    // 92.6 m
    ODID_HOR_ACC_30_METER = 9,
    ODID_HOR_ACC_10_METER = 10,
    ODID_HOR_ACC_3_METER = 11,
    ODID_HOR_ACC_1_METER = 12,
    // 13 to 15 reserved
} ODID_Horizontal_accuracy_t;

typedef enum ODID_Vertical_accuracy {
    ODID_VER_ACC_UNKNOWN = 0,
    ODID_VER_ACC_150_METER = 1,
    ODID_VER_ACC_45_METER = 2,
    ODID_VER_ACC_25_METER = 3,
    ODID_VER_ACC_10_METER = 4,
    ODID_VER_ACC_3_METER = 5,
    ODID_VER_ACC_1_METER = 6,
    // 7 to 15 reserved
} ODID_Vertical_accuracy_t;

typedef enum ODID_Speed_accuracy {
    ODID_SPEED_ACC_UNKNOWN = 0,
    ODID_SPEED_ACC_10_METERS_PER_SECOND = 1,
    ODID_SPEED_ACC_3_METERS_PER_SECOND = 2,
    ODID_SPEED_ACC_1_METERS_PER_SECOND = 3,
    ODID_SPEED_ACC_0_3_METERS_PER_SECOND = 4,
    // 5 to 15 reserved
} ODID_Speed_accuracy_t;

typedef enum ODID_Timestamp_accuracy {
    ODID_TIME_ACC_UNKNOWN = 0,
    ODID_TIME_ACC_0_1_SECOND = 1,
    ODID_TIME_ACC_0_2_SECOND = 2,
    ODID_TIME_ACC_0_3_SECOND = 3,
    ODID_TIME_ACC_0_4_SECOND = 4,
    ODID_TIME_ACC_0_5_SECOND = 5,
    ODID_TIME_ACC_0_6_SECOND = 6,
    ODID_TIME_ACC_0_7_SECOND = 7,
    ODID_TIME_ACC_0_8_SECOND = 8,
    ODID_TIME_ACC_0_9_SECOND = 9,
    ODID_TIME_ACC_1_0_SECOND = 10,
    ODID_TIME_ACC_1_1_SECOND = 11,
    ODID_TIME_ACC_1_2_SECOND = 12,
    ODID_TIME_ACC_1_3_SECOND = 13,
    ODID_TIME_ACC_1_4_SECOND = 14,
    ODID_TIME_ACC_1_5_SECOND = 15,
} ODID_Timestamp_accuracy_t;

typedef enum ODID_authtype {
    ODID_AUTH_NONE = 0,
    ODID_AUTH_UAS_ID_SIGNATURE = 1, // Unmanned Aircraft System
    ODID_AUTH_OPERATOR_ID_SIGNATURE = 2,
    ODID_AUTH_MESSAGE_SET_SIGNATURE = 3,
    ODID_AUTH_NETWORK_REMOTE_ID = 4, // Authentication provided by Network Remote ID
    // 5 to 9 reserved for the specification. 0xA to 0xF reserved for private use
} ODID_authtype_t;

typedef enum ODID_desctype {
    ODID_DESC_TYPE_TEXT = 0,
    // 1 to 200 reserved
    // 201 to 255 available for private use
} ODID_desctype_t;

typedef enum ODID_operatorIdType {
    ODID_OPERATOR_ID = 0,
    // 1 to 200 reserved
    // 201 to 255 available for private use
} ODID_operatorIdType_t;

typedef enum ODID_location_source {
    ODID_LOCATION_SRC_TAKEOFF = 0,
    ODID_LOCATION_SRC_LIVE_GNSS = 1,
    ODID_LOCATION_SRC_FIXED = 2,
    // 3 to 255 reserved
} ODID_location_source_t;

 /*
 * @name ODID_DataStructs
 * ODID Data Structures in their normative (non-packed) form.
 * This is the structure that any input adapters should form to
 * let the encoders put the data into encoded form.
 */
typedef struct {
    ODID_uatype_t UAType;
    ODID_idtype_t IDType;
    char UASID[ODID_ID_SIZE+1]; // Additional byte to allow for null term in normative form
} ODID_BasicID_data;

typedef struct {
    ODID_status_t Status;
    float Direction;          // Degrees. 0 <= x < 360. Route course based on true North. Invalid, No Value, or Unknown: 361deg
    float SpeedHorizontal;    // m/s. Positive only. Invalid, No Value, or Unknown: 255m/s. If speed is >= 254.25 m/s: 254.25m/s
    float SpeedVertical;      // m/s. Invalid, No Value, or Unknown: 63m/s. If speed is >= 62m/s: 62m/s
    double Latitude;          // Invalid, No Value, or Unknown: 0 deg (both Lat/Lon)
    double Longitude;         // Invalid, No Value, or Unknown: 0 deg (both Lat/Lon)
    float AltitudeBaro;       // meter (Ref 29.92 inHg, 1013.24 mb). Invalid, No Value, or Unknown: -1000m
    float AltitudeGeo;        // meter (WGS84-HAE). Invalid, No Value, or Unknown: -1000m
    ODID_Height_reference_t HeightType;
    float Height;             // meter. Invalid, No Value, or Unknown: -1000m
    ODID_Horizontal_accuracy_t HorizAccuracy;
    ODID_Vertical_accuracy_t VertAccuracy;
    ODID_Vertical_accuracy_t BaroAccuracy;
    ODID_Speed_accuracy_t SpeedAccuracy;
    ODID_Timestamp_accuracy_t TSAccuracy;
    float TimeStamp;          // seconds after the full hour
} ODID_Location_data;

/*
 * The Authentication message can have two different formats.
 * Five data pages are supported.
 * For data page 0, the fields PageCount, Length and TimeStamp are present and
 * AuthData is only 17 bytes.
 * For data page 1 through 4, PageCount,Length and TimeStamp are not present and
 * the size of AuthData is 23 bytes.
 */
typedef struct {
    uint8_t DataPage;   // 0 - 4
    ODID_authtype_t AuthType;
    uint8_t PageCount;  // Page 0 only. Maximum ODID_AUTH_MAX_PAGES
    uint8_t Length;     // Page 0 only. Bytes. Total of AuthData from all data pages
    uint32_t Timestamp; // Page 0 only. Relative to 00:00:00 01/01/2019
    char AuthData[ODID_STR_SIZE+1]; // Additional byte to allow for null term in normative form
} ODID_Auth_data;

typedef struct {
    ODID_desctype_t DescType;
    char Desc[ODID_STR_SIZE+1]; // Additional byte to allow for null term in normative form
} ODID_SelfID_data;

typedef struct {
    ODID_location_source_t LocationSource;
    double OperatorLatitude;  // Invalid, No Value, or Unknown: 0 deg (both Lat/Lon)
    double OperatorLongitude; // Invalid, No Value, or Unknown: 0 deg (both Lat/Lon)
    uint16_t AreaCount;       // Default 1
    uint16_t AreaRadius;      // meter. Default 0
    float AreaCeiling;        // meter. Invalid, No Value, or Unknown: -1000m
    float AreaFloor;          // meter. Invalid, No Value, or Unknown: -1000m
} ODID_System_data;

typedef struct {
    ODID_operatorIdType_t OperatorIdType;
    char OperatorId[ODID_ID_SIZE+1]; // Additional byte to allow for null term in normative form
} ODID_OperatorID_data;

typedef struct {
    ODID_BasicID_data BasicID;
    ODID_Location_data Location;
    ODID_Auth_data Auth[ODID_AUTH_MAX_PAGES];
    ODID_SelfID_data SelfID;
    ODID_System_data System;
    ODID_OperatorID_data OperatorID;

    uint8_t BasicIDValid;
    uint8_t LocationValid;
    uint8_t AuthValid[ODID_AUTH_MAX_PAGES];
    uint8_t SelfIDValid;
    uint8_t SystemValid;
    uint8_t OperatorIDValid;
} ODID_UAS_Data;

/**
* @Name ODID_PackedStructs
* Packed Data Structures prepared for broadcast
* It's best not directly access these.  Use the encoders/decoders.
*/

typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;

    // Byte 1 [IDType][UAType]  -- must define LSb first
    uint8_t UAType: 4;
    uint8_t IDType: 4;

    // Bytes 2-21
    char UASID[ODID_ID_SIZE];

    // 22-24
    char Reserved[3];
} ODID_BasicID_encoded;

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
} ODID_Location_encoded;

typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;

    // Byte 1 [AuthType][DataPage]
    uint8_t DataPage: 4;
    uint8_t AuthType: 4;

    // Bytes 2-7
    uint8_t PageCount;
    uint8_t Length;
    uint32_t Timestamp;

    // Byte 8-24
    char AuthData[ODID_STR_SIZE - ODID_AUTH_PAGE_0_DATA_SIZE];
} ODID_Auth_encoded_page_0;

typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;

    // Byte 1 [AuthType][DataPage]
    uint8_t DataPage: 4;
    uint8_t AuthType: 4;

    // Byte 2-24
    char AuthData[ODID_STR_SIZE];
} ODID_Auth_encoded_page_1_4;

typedef union {
    ODID_Auth_encoded_page_0 page_0;
    ODID_Auth_encoded_page_1_4 page_1_4;
} ODID_Auth_encoded;

typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;

    // Byte 1
    uint8_t DescType;

    // Byte 2-24
    char Desc[ODID_STR_SIZE];
} ODID_SelfID_encoded;

typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;

    // Byte 1 [Reserved][LocationSource]
    uint8_t Reserved: 7;
    uint8_t LocationSource: 1;

    // Byte 2-9
    int32_t OperatorLatitude;
    int32_t OperatorLongitude;

    // Byte 10-16
    uint16_t AreaCount;
    uint8_t  AreaRadius;
    uint16_t AreaCeiling;
    uint16_t AreaFloor;

    // Byte 17-24
    char Reserved2[8];
} ODID_System_encoded;

typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;

    // Byte 1
    uint8_t OperatorIdType;

    // Bytes 2-21
    char OperatorId[ODID_ID_SIZE];

    // 22-24
    char Reserved[3];
} ODID_OperatorID_encoded;

typedef union {
    uint8_t rawData[ODID_MESSAGE_SIZE];
    ODID_BasicID_encoded basicId;
    ODID_Location_encoded location;
    ODID_Auth_encoded auth;
    ODID_SelfID_encoded selfId;
    ODID_System_encoded system;
    ODID_OperatorID_encoded operatorId;
} ODID_Messages_encoded;

typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;

    // Byte 1 - 2
    uint8_t SingleMessageSize;
    uint8_t MsgPackSize;

    // Byte 3 - 252
    ODID_Messages_encoded Messages[ODID_PACK_MAX_MESSAGES];

    // Byte 253 - 255
    char Reserved[3];
} ODID_MessagePack_encoded;

typedef struct {
    uint8_t SingleMessageSize; // Must always be ODID_MESSAGE_SIZE
    uint8_t MsgPackSize; // Number of messages in pack (NOT number of bytes)

    ODID_Messages_encoded Messages[ODID_PACK_MAX_MESSAGES];
} ODID_MessagePack_data;

// API Calls
int encodeBasicIDMessage(ODID_BasicID_encoded *outEncoded, ODID_BasicID_data *inData);
int encodeLocationMessage(ODID_Location_encoded *outEncoded, ODID_Location_data *inData);
int encodeAuthMessage(ODID_Auth_encoded *outEncoded, ODID_Auth_data *inData);
int encodeSelfIDMessage(ODID_SelfID_encoded *outEncoded, ODID_SelfID_data *inData);
int encodeSystemMessage(ODID_System_encoded *outEncoded, ODID_System_data *inData);
int encodeOperatorIDMessage(ODID_OperatorID_encoded *outEncoded, ODID_OperatorID_data *inData);
int encodeMessagePack(ODID_MessagePack_encoded *outEncoded, ODID_MessagePack_data *inData);

int decodeBasicIDMessage(ODID_BasicID_data *outData, ODID_BasicID_encoded *inEncoded);
int decodeLocationMessage(ODID_Location_data *outData, ODID_Location_encoded *inEncoded);
int decodeAuthMessage(ODID_Auth_data *outData, ODID_Auth_encoded *inEncoded);
int decodeSelfIDMessage(ODID_SelfID_data *outData, ODID_SelfID_encoded *inEncoded);
int decodeSystemMessage(ODID_System_data *outData, ODID_System_encoded *inEncoded);
int decodeOperatorIDMessage(ODID_OperatorID_data *outData, ODID_OperatorID_encoded *inEncoded);
int decodeMessagePack(ODID_UAS_Data *uasData, ODID_MessagePack_encoded *pack);

int getAuthPageNum(ODID_Auth_encoded *inEncoded, int *pageNum);
ODID_messagetype_t decodeMessageType(uint8_t byte);
ODID_messagetype_t decodeOpenDroneID(ODID_UAS_Data *uas_data, uint8_t *msg_data);

// Helper Functions
ODID_Horizontal_accuracy_t createEnumHorizontalAccuracy(float Accuracy);
ODID_Vertical_accuracy_t createEnumVerticalAccuracy(float Accuracy);
ODID_Speed_accuracy_t createEnumSpeedAccuracy(float Accuracy);
ODID_Timestamp_accuracy_t createEnumTimestampAccuracy(float Accuracy);

float decodeHorizontalAccuracy(ODID_Horizontal_accuracy_t Accuracy);
float decodeVerticalAccuracy(ODID_Vertical_accuracy_t Accuracy);
float decodeSpeedAccuracy(ODID_Speed_accuracy_t Accuracy);
float decodeTimestampAccuracy(ODID_Timestamp_accuracy_t Accuracy);

#ifndef ODID_DISABLE_PRINTF
void printByteArray(uint8_t *byteArray, uint16_t asize, int spaced);
void printBasicID_data(ODID_BasicID_data *BasicID);
void printLocation_data(ODID_Location_data *Location);
void printAuth_data(ODID_Auth_data *Auth);
void printSelfID_data(ODID_SelfID_data *SelfID);
void printOperatorID_data(ODID_OperatorID_data *OperatorID);
void printSystem_data(ODID_System_data *System_data);
#endif // ODID_DISABLE_PRINTF

#endif // _OPENDRONEID_H_

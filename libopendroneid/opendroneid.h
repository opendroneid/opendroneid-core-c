/*
Copyright (C) 2019 Intel Corporation

SPDX-License-Identifier: Apache-2.0

Open Drone ID C Library

Maintainer:
Gabriel Cox
gabriel.c.cox@intel.com
*/

#include <stdint.h>
#define ODID_MESSAGE_SIZE 25
#define ODID_ID_SIZE 20
#define ODID_STR_SIZE 23
#define ODID_PROTOCOL_VERSION 0
#define ODID_SPEC_VERSION 0.64.3

typedef enum ODID_messagetype {
    ODID_MESSAGETYPE_BASIC_ID = 0,
    ODID_MESSAGETYPE_LOCATION = 1,
    ODID_MESSAGETYPE_AUTH = 2,
    ODID_MESSAGETYPE_SELF_ID = 3,
    ODID_MESSAGETYPE_SYSTEM = 4,
} ODID_messagetype_t;

typedef enum ODID_idtype {
    ODID_IDTYPE_NONE = 0,
    ODID_IDTYPE_SERIAL_NUMBER = 1,
    ODID_IDTYPE_CAA_ASSIGNED_ID = 2,
    ODID_IDTYPE_UTM_ASSIGNED_ID = 3,
    // 4 to 15 reserved
} ODID_idtype_t;

typedef enum ODID_uastype {
    ODID_UASTYPE_NONE = 0,
    ODID_UASTYPE_FIXED_WING_POWERED = 1,
    ODID_UASTYPE_ROTORCRAFT_MULTIROTOR = 2,
    ODID_UASTYPE_LTA_POWERED = 3,    /* Lighter Than Air (such as a Blimp) */
    ODID_UASTYPE_LTA_UNPOWERED = 4,  /* example: Balloon */
    ODID_UASTYPE_VTOL = 5,           /* Fixed wing aircraft that can take off vertically) */
    ODID_UASTYPE_FREE_FALL = 6,      /* example: Parachute */
    ODID_UASTYPE_ROCKET = 7,
    ODID_UASTYPE_GLIDER = 8,
    ODID_UASTYPE_OTHER = 9,
    // 10 to 15 reserved
} ODID_uastype_t;

typedef enum ODID_status {
    ODID_STATUS_UNDECLARED = 0,
    ODID_STATUS_GROUND = 1,
    ODID_STATUS_AIRBORNE = 2,
    // 3 to 15 reserved
} ODID_status_t;

typedef enum ODID_Horizontal_accuracy {
    ODID_HOR_ACC_UNKNOWN = 0,
    ODID_HOR_ACC_10NM = 1,
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
    ODID_SPEED_ACC_10_METERS_SECOND = 1,
    ODID_SPEED_ACC_3_METERS_SECOND = 2,
    ODID_SPEED_ACC_1_METERS_SECOND = 3,
    ODID_SPEED_ACC_0_3_METERS_SECOND = 4,
    // 5 to 15 reserved
} ODID_Speed_accuracy_t;

typedef enum ODID_authtype {
    ODID_AUTH_NONE = 0,
    ODID_AUTH_MPUID = 1, // Manufacturer Programmed Unique ID
    // 2 to 9 reserved. 0xA to 0xF reserved for private use
} ODID_authtype_t;

typedef enum ODID_desctype {
    ODID_DESC_TYPE_TEXT = 0,
    // 1 to 255 reserved
} ODID_desctype_t;

typedef enum ODID_location_source {
    ODID_LOCATION_SRC_TAKEOFF = 0,
    ODID_LOCATION_SRC_LIVE_GNSS = 1,
    // 2 to 255 reserved
} ODID_location_source_t;

 /*
 * @name ODID_DataStructs
 * ODID Data Structures in their normative (non-packed) form.
 * This is the structure that any input adapters should form to
 * let the encoders put the data into encoded form.
 */
typedef struct {
    ODID_uastype_t UASType;
    ODID_idtype_t IDType;
    char UASID[ODID_ID_SIZE+1];
} ODID_BasicID_data;

typedef struct {
    ODID_status_t Status;
    float SpeedNS;            // m/s
    float SpeedEW;            // m/s
    float SpeedVertical;      // m/s
    double Latitude;
    double Longitude;
    float AltitudeBaro;       // meter
    float AltitudeGeo;        // meter
    float HeightAboveTakeoff; // meter
    float HorizAccuracy;      // meter
    float VertAccuracy;       // meter
    float SpeedAccuracy;      // m/s
    float TSAccuracy;         // seconds
    float TimeStamp;          // seconds after the full hour
} ODID_Location_data;

typedef struct {
    uint8_t DataPage;
    ODID_authtype_t AuthType;
    char AuthData[ODID_STR_SIZE+1];  // additional byte to allow for null term in normative form
} ODID_Auth_data;

typedef struct {
    uint8_t DescType;
    char Desc[ODID_STR_SIZE+1];
} ODID_SelfID_data;

typedef struct {
    uint8_t LocationSource;
    double Latitude;
    double Longitude;
    uint16_t GroupCount;
    uint16_t GroupRadius;     // meter
    float GroupCeiling;       // meter
} ODID_System_data;

typedef struct {
    ODID_BasicID_data BasicID;
    ODID_Location_data Location;
    ODID_Auth_data Auth;
    ODID_SelfID_data SelfID;
    ODID_System_data System;
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

    // Byte 1 [IDType][UASType]  -- must define LSb first
    uint8_t UASType: 4;
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
    uint8_t EWMult: 1;
    uint8_t NSMult: 1;
    uint8_t Reserved: 2;
    uint8_t Status: 4;

    // Bytes 2-18
    uint8_t SpeedNS;
    uint8_t SpeedEW;
    int8_t SpeedVertical;
    int32_t Latitude;
    int32_t Longitude;
    uint16_t AltitudeBaro;
    uint16_t AltitudeGeo;
    uint16_t HeightAboveTakeoff;

    // Byte 19 [VertAccuracy][HorizAccuracy]  -- must define LSb first
    uint8_t HorizAccuracy:4;
    uint8_t VertAccuracy:4;

    // Byte 20 [Reserved2][SpeedAccuracy]  -- must define LSb first
    uint8_t SpeedAccuracy:4;
    uint8_t TSAccuracy:4;

    // Byte 21-22
    uint16_t TimeStamp;

    // Byte 23-24
    char Reserved2[2];
} ODID_Location_encoded;

typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;

    // Byte 1 [AuthType][DataPage]
    uint8_t DataPage: 4;
    uint8_t AuthType: 4;

    // Byte 2-24
    char AuthData[ODID_STR_SIZE];
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
    int32_t Latitude;
    int32_t Longitude;

    // Byte 10-14
    uint16_t GroupCount;
    uint8_t  GroupRadius;
    uint16_t GroupCeiling;

    // Byte 15-24
    char Reserved2[10];
} ODID_System_encoded;

typedef struct {
    uint8_t msgData[ODID_MESSAGE_SIZE];
} ODID_Message;

// TODO: Encoding/Decoding message pack
typedef struct __attribute__((__packed__)) {
    // Byte 0 [MessageType][ProtoVersion]  -- must define LSb first
    uint8_t ProtoVersion: 4;
    uint8_t MessageType : 4;
    uint8_t SingleMessageSize;
    uint8_t MsgPackSize; // No of messages in pack (NOT number of bytes)
    ODID_Message Messages[];
} ODID_Message_Pack;


// API Calls
int encodeBasicIDMessage(ODID_BasicID_encoded *outEncoded, ODID_BasicID_data *inData);
int encodeLocationMessage(ODID_Location_encoded *outEncoded, ODID_Location_data *inData);
int encodeAuthMessage(ODID_Auth_encoded *outEncoded, ODID_Auth_data *inData);
int encodeSelfIDMessage(ODID_SelfID_encoded *outEncoded, ODID_SelfID_data *inData);
int encodeSystemMessage(ODID_System_encoded *outEncoded, ODID_System_data *inData);

int decodeBasicIDMessage(ODID_BasicID_data *outData, ODID_BasicID_encoded *inEncoded);
int decodeLocationMessage(ODID_Location_data *outData, ODID_Location_encoded *inEncoded);
int decodeAuthMessage(ODID_Auth_data *outData, ODID_Auth_encoded *inEncoded);
int decodeSelfIDMessage(ODID_SelfID_data *outData, ODID_SelfID_encoded *inEncoded);
int decodeSystemMessage(ODID_System_data *outData, ODID_System_encoded *inEncoded);

// Helper Functions
char *safe_copyfill(char *dstStr, const char *srcStr, int dstSize);
char *safe_dec_copyfill(char *dstStr, const char *srcStr, int dstSize);
int intRangeMax(int64_t inValue, int startRange, int endRange);
int intInRange(int inValue, int startRange, int endRange);

#ifndef ODID_DISABLE_PRINTF
void printByteArray(uint8_t *byteArray, uint16_t asize, int spaced);
void printBasicID_data(ODID_BasicID_data BasicID);
void printLocation_data(ODID_Location_data Location);
void printAuth_data(ODID_Auth_data Auth);
void printSelfID_data(ODID_SelfID_data SelfID);
void printSystem_data(ODID_System_data System_data);
void test_InOut(void);
void ODID_getSimData(uint8_t *message, uint8_t msgType);
void test_sim(void);
#endif // ODID_DISABLE_PRINTF

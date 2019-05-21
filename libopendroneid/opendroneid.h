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

/*
ENUMERATION NOTES
----------------------------------------------------
  MSG Type
    0: Basic ID
    1: Location/Vector
    2: Auth
    3: Self ID
    4: Operator

  UAS ID Type:
    1: Serial Number to CTA-2063 PSN
    2: Civil aviation authority assigned ID
    3: UTM Assigned ID

  UAS Type:
    0: None
    1: Fixed Wing Powered
    2: Rotorcraft/Multirotor
    3: LTA (Lighter than Air) Powered
    4: LTA Unpowered (Balloon)
    5: VTOL (Fixed wing aircraft that can take off vertically)
    6: Free Fall/Parachute
    7: Rocket
    8: Glider
    9: Other
    10-15: Reserved

  UAS Status Type:
    0: Undeclared
    1: Ground,
    2: Airborne (manual control)
    3-15: Reserved

  Horizontal Accuracy Level:
    0:  = 18.52 km (10NM)  Unknown accuracy
    1:  < 18.52 km (10NM)
    2:  < 7.408 km (4NM)
    3:  < 3.704 km (2NM)
    4:  < 1 852 m (1NM)
    5:  < 926 m (0.5NM)
    6:  < 555.6 m (0.3NM
    7:  < 185.2 m (0.1NM
    8:  < 92.6 m (0.05NM)
    9:  < 30 m
    10:  < 10 m
    11:  < 3 m
    12:  < 1m
    13-15: Reserved

  Vertical Accuracy Level:
    0:  >150m  Unknown
    1:  <150m
    2:  <45m
    3:  <25m
    4:  <10m
    5:  <3m
    6:  <1m
    7:  Reserved

  Speed Accuracy
    0:  >10m/s or Unknown
    1:  <10m/s
    2:  <3m/s
    3:  <1m/s
    4:  <0.3m/s

  Auth Type:
    0:  None
    1:  MFG Programmed Unique ID (MPIUD)
    2-9:  Reserved for Open Drone ID
    10-15: Available for Private Use

  Location Source:
    0: Takeoff Location
    1: Live GNSS
---------------------------------------------------------
*/

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
} ODID_idtype_t;

typedef enum ODID_uavtype {
    ODID_UAVTYPE_NONE = 0,
    ODID_UAVTYPE_FIXED_WING_POWERED = 1,
    ODID_UAVTYPE_ROTORCRAFT_MULTIROTOR = 2,
    ODID_UAVTYPE_LTA_POWERED = 3,    /* Lighter Than Air (such as a Blimp) */
    ODID_UAVTYPE_LTA_UNPOWERED = 4,  /* example: Balloon */
    ODID_UAVTYPE_VTOL = 5,           /* Fixed wing aircraft that can take off vertically) */
    ODID_UAVTYPE_FREE_FALL = 6,      /* example: Parachute */
    ODID_UAVTYPE_ROCKET = 7,
    ODID_UAVTYPE_GLIDER = 8,
    ODID_UAVTYPE_OTHER = 9,
    // 10 to 15 reserved
} ODID_uavtype_t;

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

 /*
 * @name ODID_DataStructs
 * ODID Data Structures in their normative (non-packed) form.
 * This is the structure that any input adapters should form to
 * let the encoders put the data into encoded form.
 */
typedef struct {
    ODID_uavtype_t UASType;
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
    uint8_t AuthType;
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
    int16_t GroupCount;
    int16_t GroupRadius;      // meter
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
    int16_t GroupCount;
    int8_t  GroupRadius;
    int16_t GroupCeiling;

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

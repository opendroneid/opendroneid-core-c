/*
Copyright (C) 2019 Intel Corporation

SPDX-License-Identifier: Apache-2.0

Open Drone ID C Library

Maintainer:
Gabriel Cox
gabriel.c.cox@intel.com
*/

#include "opendroneid.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#define ENABLE_DEBUG 1

const float SPEED_DIV[2] = {0.25,0.75};
const float VSPEED_DIV = 0.5;
const int32_t LATLON_MULT = 10000000;
const float ALT_DIV = 0.5;
const int ALT_ADDER = 1000;
const int DATA_AGE_DIV = 10;

/**
* Encode speed into units defined by Open Drone ID
*
* The quantization algorithm allows for speed to be stored in units of 0.25 m/s
* on the low end of the scale and 0.75 m/s on the high end of the scale.
* This allows for more precise speeds to be represented in a single Int8 byte
* rather than using a large float value.
*
* @param Speed_data Speed (and decimal) in m/s
* @param mult a (write only) value that sets the multiplier flag
* @return Encoded Speed in a single byte or max speed if over max encoded speed.
*/
static int8_t encodeSpeed(float Speed_data, uint8_t *mult)
{
    int8_t signMult = 1;
    int big_value = 0;

    if (abs((int)Speed_data)/SPEED_DIV[0] <= INT8_MAX) {
        // Value fits within adjusted units
        (*mult) = 0;
        return Speed_data / SPEED_DIV[0];
    } else {
        // value does not fit within high resolution range
        (*mult) = 1;

        if (Speed_data < 0) signMult = -1;  // It's negative, set sign multiplier
        
        // Calculated value expressed in a big int
        big_value = (int) (Speed_data-(signMult*(INT8_MAX/SPEED_DIV[0]))) / SPEED_DIV[1];

        return (int8_t) intRangeMax(big_value, INT8_MIN, INT8_MAX);
    }
}

/**
* Encode Vertical Speed into a signed Integer ODID format
*
* @param SpeedVertical_data vertical speed (in m/s)
* @return Encoded vertical speed
*/
static int8_t encodeSpeedVertical(float SpeedVertical_data)
{
    int encValue = (int) (SpeedVertical_data / VSPEED_DIV);
    return (int8_t) intRangeMax(encValue, INT8_MIN, INT8_MAX);
}

/**
* Encode Latitude or Longitude value into a signed Integer ODID format
*
* This encodes a 64bit double into a 32 bit integer yet still maintains
* 10^7 of a degree of accuracy (about 1cm)
*
* @param LatLon_data Either Lat or Lon double float value
* @return Encoded Lat or Lon
*/
static int32_t encodeLatLon(double LatLon_data)
{
    return (int32_t) intRangeMax(LatLon_data * LATLON_MULT, -180 * LATLON_MULT, 180 * LATLON_MULT);
}

/**
* Encode Altitude value into an int16 ODID format
*
* This encodes a 32bit floating point altitude into an uint16 compressed
* scale that starts at -1000m.
*
* @param Alt_data Altitude to encode (in meters)
* @return Encoded Altitude
*/
static int16_t encodeAltitude(float Alt_data)
{
    return (uint16_t) intRangeMax( (int) ((Alt_data + ALT_ADDER) / ALT_DIV), 0, UINT16_MAX);
}

/**
* Encode horizontal accuracy in ODID format
*
* This encodes a horizontal accuracy value to the corresponding enum
*
* @param Accuracy The horizontal accuracy in meters
* @return Enum value representing the accuracy
*/
static ODID_Horizontal_accuracy_t encodeHorizontalAccuracy(float Accuracy)
{
    if (Accuracy >= 18520)
        return ODID_HOR_ACC_UNKNOWN;
    else if (Accuracy > 7408)
        return ODID_HOR_ACC_10NM;
    else if (Accuracy > 3704)
        return ODID_HOR_ACC_4NM;
    else if (Accuracy > 1852)
        return ODID_HOR_ACC_2NM;
    else if (Accuracy > 926)
        return ODID_HOR_ACC_1NM;
    else if (Accuracy > 555.6f)
        return ODID_HOR_ACC_0_5NM;
    else if (Accuracy > 185.2f)
        return ODID_HOR_ACC_0_3NM;
    else if (Accuracy > 92.6f)
        return ODID_HOR_ACC_0_1NM;
    else if (Accuracy > 30)
        return ODID_HOR_ACC_0_05NM;
    else if (Accuracy > 10)
        return ODID_HOR_ACC_30_METER;
    else if (Accuracy > 3)
        return ODID_HOR_ACC_10_METER;
    else if (Accuracy > 1)
        return ODID_HOR_ACC_3_METER;
    else if (Accuracy > 0)
        return ODID_HOR_ACC_1_METER;
    else
        return ODID_HOR_ACC_UNKNOWN;
}

/**
* Encode vertical accuracy in ODID format
*
* This encodes a vertical accuracy value to the corresponding enum
*
* @param Accuracy The vertical accuracy in meters
* @return Enum value representing the accuracy
*/
static ODID_Vertical_accuracy_t encodeVerticalAccuracy(float Accuracy)
{
    if (Accuracy >= 150)
        return ODID_VER_ACC_UNKNOWN;
    else if (Accuracy > 45)
        return ODID_VER_ACC_150_METER;
    else if (Accuracy > 25)
        return ODID_VER_ACC_45_METER;
    else if (Accuracy > 10)
        return ODID_VER_ACC_25_METER;
    else if (Accuracy > 3)
        return ODID_VER_ACC_10_METER;
    else if (Accuracy > 1)
        return ODID_VER_ACC_3_METER;
    else if (Accuracy > 0)
        return ODID_VER_ACC_1_METER;
    else
        return ODID_VER_ACC_UNKNOWN;
}

/**
* Encode speed accuracy in ODID format
*
* This encodes a speed accuracy value to the corresponding enum
*
* @param Accuracy The speed accuracy in m/s
* @return Enum value representing the accuracy
*/
static ODID_Speed_accuracy_t encodeSpeedAccuracy(float Accuracy)
{
    if (Accuracy >= 10)
        return ODID_SPEED_ACC_UNKNOWN;
    else if (Accuracy > 3)
        return ODID_SPEED_ACC_10_METERS_SECOND;
    else if (Accuracy > 1)
        return ODID_SPEED_ACC_3_METERS_SECOND;
    else if (Accuracy > 0.3f)
        return ODID_SPEED_ACC_1_METERS_SECOND;
    else if (Accuracy > 0)
        return ODID_SPEED_ACC_0_3_METERS_SECOND;
    else
        return ODID_SPEED_ACC_UNKNOWN;
}

/**
* Encode timestamp accuracy in ODID format
*
* This encodes a fractional seconds value into 1 byte
* and clamps it the range 0.1s - 1.5s
*
* @param Accuracy The timestamp accuracy in seconds
* @return Encoded timestamp accuracy (Tenths of seconds)
*/
static uint16_t encodeTimeStampAccuracy(float Accuracy)
{
    return (uint8_t) intRangeMax(round(Accuracy*10), 1, 15);
}

/**
* Encode timestamp data in ODID format
*
* This encodes a fractional seconds value into a 2 byte int16
* on a scale of tenths of seconds since after the hour.
*
* @param Seconds_data Seconds (to at least 1 decimal place) since the hour
* @return Encoded timestamp (Tenths of seconds since the hour)
*/
static uint16_t encodeTimeStamp(float Seconds_data)
{
    // max should be 60s * 60m * 10 = number of tenths within an hour
    return (uint16_t) intRangeMax(round(Seconds_data*10), 0, 60 * 60 * 10);
}

/**
* Encode group radius data in ODID format
*
* This encodes the group radius in meters into a 1 byte value
*
* @param Radius The radius of the drone group/swarm
* @return Encoded group radius
*/
static uint16_t encodeGroupRadius(uint16_t Radius)
{
    return (uint8_t) intRangeMax(Radius / 10, 0, 255);
}

/**
* Encode Basic ID message (packed, ready for broadcast)
*
* @param outEncoded output (encoded/packed) structure
* @param inData input data (non encoded/packed) structure
* @return success code (0 = failure, 1 = success)
*/
int encodeBasicIDMessage(ODID_BasicID_encoded *outEncoded, ODID_BasicID_data *inData)
{
    if (!outEncoded || !inData) {
        return 0;
    } else {
        outEncoded->MessageType = ODID_MESSAGETYPE_BASIC_ID;
        outEncoded->ProtoVersion = ODID_PROTOCOL_VERSION;
        outEncoded->IDType = inData->IDType;
        outEncoded->UASType = inData->UASType;
        safe_copyfill(outEncoded->UASID, inData->UASID, sizeof(outEncoded->UASID));
        return 1;
    }
}

/**
* Encode Location message (packed, ready for broadcast)
*
* @param outEncoded output (encoded/packed) structure
* @param inData input data (non encoded/packed) structure
* @return success code (0 = failure, 1 = success)
*/
int encodeLocationMessage(ODID_Location_encoded *outEncoded, ODID_Location_data *inData)
{
    uint8_t multflag = 0;
    if (!outEncoded || !inData) {
        return 0;
    } else {
        outEncoded->MessageType = ODID_MESSAGETYPE_LOCATION;
        outEncoded->ProtoVersion = ODID_PROTOCOL_VERSION;
        outEncoded->Status = inData->Status;
        outEncoded->Reserved = 0;
        outEncoded->SpeedNS = encodeSpeed(inData->SpeedNS, &multflag);
        outEncoded->NSMult = multflag;
        outEncoded->SpeedEW = encodeSpeed(inData->SpeedEW, &multflag);
        outEncoded->EWMult = multflag;
        outEncoded->SpeedVertical = encodeSpeedVertical(inData->SpeedVertical);
        outEncoded->Latitude = encodeLatLon(inData->Latitude);
        outEncoded->Longitude = encodeLatLon(inData->Longitude);
        outEncoded->AltitudeBaro = encodeAltitude(inData->AltitudeBaro);
        outEncoded->AltitudeGeo = encodeAltitude(inData->AltitudeGeo);
        outEncoded->HeightAboveTakeoff = encodeAltitude(inData->HeightAboveTakeoff);
        outEncoded->HorizAccuracy = encodeHorizontalAccuracy(inData->HorizAccuracy);
        outEncoded->VertAccuracy = encodeVerticalAccuracy(inData->VertAccuracy);
        outEncoded->SpeedAccuracy = encodeSpeedAccuracy(inData->SpeedAccuracy);
        outEncoded->TSAccuracy = encodeTimeStampAccuracy(inData->TSAccuracy);
        outEncoded->TimeStamp = encodeTimeStamp(inData->TimeStamp);
        memset(outEncoded->Reserved2, 0, sizeof(outEncoded->Reserved2));
        return 1;
    }
}

/**
* Encode Auth message (packed, ready for broadcast)
*
* @param outEncoded output (encoded/packed) structure
* @param inData input data (non encoded/packed) structure
* @return success code (0 = failure, 1 = success)
*/
int encodeAuthMessage(ODID_Auth_encoded *outEncoded, ODID_Auth_data *inData)
{
    if (!inData || !intInRange(inData->AuthType,0,15) || !outEncoded) {
        return 0;
    } else {
        outEncoded->MessageType = ODID_MESSAGETYPE_AUTH;
        outEncoded->ProtoVersion = ODID_PROTOCOL_VERSION;
        outEncoded->AuthType = inData->AuthType;
        // TODO: Implement Multi-page support (for now, this will handle a single DataPage)
        outEncoded->DataPage = 0;
        safe_copyfill(outEncoded->AuthData, inData->AuthData, sizeof(outEncoded->AuthData));
        return 1;
    }
}

/**
* Encode Self ID message (packed, ready for broadcast)
*
* @param outEncoded output (encoded/packed) structure
* @param inData input data (non encoded/packed) structure
* @return success code (0 = failure, 1 = success)
*/
int encodeSelfIDMessage(ODID_SelfID_encoded *outEncoded, ODID_SelfID_data *inData)
{
    if (!inData || !intInRange(inData->DescType,0,UINT8_MAX) || !outEncoded) {
        return 0;
    } else {
        outEncoded->MessageType = ODID_MESSAGETYPE_SELF_ID;
        outEncoded->ProtoVersion = ODID_PROTOCOL_VERSION;
        outEncoded->DescType = inData->DescType;
        safe_copyfill(outEncoded->Desc, inData->Desc, sizeof(outEncoded->Desc));
        return 1;
    }
}

/**
* Encode System message (packed, ready for broadcast)
*
* @param outEncoded output (encoded/packed) structure
* @param inData input data (non encoded/packed) structure
* @return success code (0 = failure, 1 = success)
*/
int encodeSystemMessage(ODID_System_encoded *outEncoded, ODID_System_data *inData)
{
    if (!inData || !intInRange(inData->LocationSource,0,15) || !outEncoded) {
        return 0;
    } else {
        outEncoded->MessageType = ODID_MESSAGETYPE_SYSTEM;
        outEncoded->ProtoVersion = ODID_PROTOCOL_VERSION;
        outEncoded->Reserved = 0;
        outEncoded->LocationSource = inData->LocationSource;
        outEncoded->Latitude = encodeLatLon(inData->Latitude);
        outEncoded->Longitude = encodeLatLon(inData->Longitude);
        outEncoded->GroupCount = inData->GroupCount;
        outEncoded->GroupRadius = encodeGroupRadius(inData->GroupRadius);
        outEncoded->GroupCeiling = encodeAltitude(inData->GroupCeiling);
        memset(outEncoded->Reserved2, 0, sizeof(outEncoded->Reserved2));
        return 1;
    }
}

/**
* Dencode speed from Open Drone ID packed message
*
* @param Speed_enc encoded speed
* @param mult multiplier flag
* @return decoded speed in m/s
*/
static float decodeSpeed(int8_t Speed_enc, uint8_t mult)
{
    float retValue = 0;
    if (mult == 0) {
        retValue = (float) Speed_enc * SPEED_DIV[0];
    } else {
        if (Speed_enc < 0) {
            retValue = (float) Speed_enc * SPEED_DIV[1] - (INT8_MAX * SPEED_DIV[0]);
        } else {
            retValue = (float) Speed_enc * SPEED_DIV[1] + (INT8_MAX * SPEED_DIV[0]);
        }
    }
    return retValue;
}

/**
* Decode Vertical Speed from Open Drone ID Packed Message
*
* @param SpeedVertical_enc Encoded Vertical Speed
* @return decoded Vertical Speed in m/s
*/
static float decodeSpeedVertical(int8_t SpeedVertical_enc)
{
    return (float) SpeedVertical_enc * VSPEED_DIV;
}

/**
* Decode Latitude or Longitude value into a signed Integer ODID format
*
* @param LatLon_enc Either Lat or Lon ecoded int value
* @return decoded (double) Lat or Lon
*/
static double decodeLatLon(int32_t LatLon_enc)
{
    return (double) LatLon_enc / LATLON_MULT;
}

/**
* Decode Altitude from ODID packed format
*
* @param Alt_enc Encoded Altitude to decode
* @return decoded Altitude (in meters)
*/
static float decodeAltitude(uint16_t Alt_enc)
{
    return (float) ((float) Alt_enc * ALT_DIV - ALT_ADDER) ;
}

/**
* Decode horizontal accuracy from ODID format
*
* This decodes a horizontal accuracy enum to the corresponding value
*
* @param Accuracy Enum value representing the accuracy
* @return The maximum horizontal accuracy in meters
*/
static float decodeHorizontalAccuracy(ODID_Horizontal_accuracy_t Accuracy)
{
    switch (Accuracy)
    {
    case ODID_HOR_ACC_UNKNOWN:
        return 18520;
    case ODID_HOR_ACC_10NM:
        return 18520;
    case ODID_HOR_ACC_4NM:
        return 7808;
    case ODID_HOR_ACC_2NM:
        return 3704;
    case ODID_HOR_ACC_1NM:
        return 1852;
    case ODID_HOR_ACC_0_5NM:
        return 926;
    case ODID_HOR_ACC_0_3NM:
        return 555.6f;
    case ODID_HOR_ACC_0_1NM:
        return 185.2f;
    case ODID_HOR_ACC_0_05NM:
        return 92.6f;
    case ODID_HOR_ACC_30_METER:
        return 30;
    case ODID_HOR_ACC_10_METER:
        return 10;
    case ODID_HOR_ACC_3_METER:
        return 3;
    case ODID_HOR_ACC_1_METER:
        return 1;
    default:
        return 18520;
    }
}

/**
* Decode vertical accuracy from ODID format
*
* This decodes a vertical accuracy enum to the corresponding value
*
* @param Accuracy Enum value representing the accuracy
* @return The maximum vertical accuracy in meters
*/
static float decodeVerticalAccuracy(ODID_Vertical_accuracy_t Accuracy)
{
    switch (Accuracy)
    {
    case ODID_VER_ACC_UNKNOWN:
        return 150;
    case ODID_VER_ACC_150_METER:
        return 150;
    case ODID_VER_ACC_45_METER:
        return 45;
    case ODID_VER_ACC_25_METER:
        return 25;
    case ODID_VER_ACC_10_METER:
        return 10;
    case ODID_VER_ACC_3_METER:
        return 3;
    case ODID_VER_ACC_1_METER:
        return 1;
    default:
        return 150;
    }
}

/**
* Decode speed accuracy from ODID format
*
* This decodes a speed accuracy enum to the corresponding value
*
* @param Accuracy Enum value representing the accuracy
* @return The maximum speed accuracy in m/s
*/
static float decodeSpeedAccuracy(ODID_Speed_accuracy_t Accuracy)
{
    switch (Accuracy)
    {
    case ODID_SPEED_ACC_UNKNOWN:
        return 10;
    case ODID_SPEED_ACC_10_METERS_SECOND:
        return 10;
    case ODID_SPEED_ACC_3_METERS_SECOND:
        return 3;
    case ODID_SPEED_ACC_1_METERS_SECOND:
        return 1;
    case ODID_SPEED_ACC_0_3_METERS_SECOND:
        return 0.3f;
    default:
        return 10;
    }
}

/**
* Decode timestamp accuracy from ODID format
*
* This decodes a 1 byte value to a fractional seconds value
*
* @param Accuracy Encoded timestamp accuracy (Tenths of seconds)
* @return The maximum timestamp accuracy in seconds
*/
static float decodeTimeStampAccuracy(uint16_t Accuracy)
{
    return (float) Accuracy / 10;
}

/**
* Decode timestamp data from ODID packed format
*
* @param Seconds_enc Encoded Timestamp
* @return Decoded timestamp (seconds since the hour)
*/
static float decodeTimeStamp(uint16_t Seconds_enc)
{
    return (float) Seconds_enc / 10;
}

/**
* Decode group radius data from ODID format
*
* This decodes a 1 byte value to the group radius in meters
*
* @param Radius_enc Encoded group radius
* @return The radius of the drone group/swarm in meters
*/
static uint16_t decodeGroupRadius(uint8_t Radius_enc)
{
    return (uint16_t) Radius_enc * 10;
}

/**
* Decode Basic ID data from packed message
*
* @param outData output: decoded message
* @param inEncoded input message (encoded/packed) structure
* @return success code (0 = failure, 1 = success)
*/
int decodeBasicIDMessage(ODID_BasicID_data *outData, ODID_BasicID_encoded *inEncoded)
{
    if (!outData || !inEncoded) {
        return 0;
    } else {
        outData->IDType = (ODID_idtype_t) inEncoded->IDType;
        outData->UASType = (ODID_uavtype_t) inEncoded->UASType;
        safe_dec_copyfill(outData->UASID, inEncoded->UASID, sizeof(outData->UASID));
        return 1;
    }
}

/**
* Decode Location data from packed message
*
* @param outData output: decoded message
* @param inEncoded input message (encoded/packed) structure
* @return success code (0 = failure, 1 = success)
*/
int decodeLocationMessage(ODID_Location_data *outData, ODID_Location_encoded *inEncoded)
{
    if (!outData || !inEncoded) {
        return 0;
    } else {
        outData->Status = (ODID_status_t) inEncoded->Status;
        outData->SpeedNS = decodeSpeed(inEncoded->SpeedNS, inEncoded->NSMult);
        outData->SpeedEW = decodeSpeed(inEncoded->SpeedEW, inEncoded->EWMult);
        outData->SpeedVertical = decodeSpeedVertical(inEncoded->SpeedVertical);
        outData->Latitude = decodeLatLon(inEncoded->Latitude);
        outData->Longitude = decodeLatLon(inEncoded->Longitude);
        outData->AltitudeBaro = decodeAltitude(inEncoded->AltitudeBaro);
        outData->AltitudeGeo = decodeAltitude(inEncoded->AltitudeGeo);
        outData->HeightAboveTakeoff = decodeAltitude(inEncoded->HeightAboveTakeoff);
        outData->HorizAccuracy = decodeHorizontalAccuracy((ODID_Horizontal_accuracy_t) inEncoded->HorizAccuracy);
        outData->VertAccuracy = decodeVerticalAccuracy((ODID_Vertical_accuracy_t) inEncoded->VertAccuracy);
        outData->SpeedAccuracy = decodeSpeedAccuracy((ODID_Speed_accuracy_t) inEncoded->SpeedAccuracy);
        outData->TSAccuracy = decodeTimeStampAccuracy(inEncoded->TSAccuracy);
        outData->TimeStamp = decodeTimeStamp(inEncoded->TimeStamp);
        return 1;
    }
}

/**
* Decode Auth data from packed message
*
* @param outData output decoded message
* @param inEncoded input message (encoded/packed) structure
* @return success code (0 = failure, 1 = success)
*/
int decodeAuthMessage(ODID_Auth_data *outData, ODID_Auth_encoded *inEncoded)
{
    if (!inEncoded || !intInRange(inEncoded->AuthType,0,15) || !outData) {
        return 0;
    } else {
        // TODO: Implement Multi-page support (for now, this will handle a single DataPage)
        outData->AuthType = inEncoded->AuthType;
        outData->DataPage = 0;
        safe_dec_copyfill(outData->AuthData, inEncoded->AuthData, sizeof(outData->AuthData));
        return 1;
    }
}

/**
* Decode Self ID data from packed message
*
* @param outData output: decoded message
* @param inEncoded input message (encoded/packed) structure
* @return success code (0 = failure, 1 = success)
*/
int decodeSelfIDMessage(ODID_SelfID_data *outData, ODID_SelfID_encoded *inEncoded)
{
    if (!inEncoded || !intInRange(inEncoded->DescType,0,UINT8_MAX) || !outData) {
        return 0;
    } else {
        outData->DescType = inEncoded->DescType;
        safe_dec_copyfill(outData->Desc, inEncoded->Desc, sizeof(outData->Desc));
        return 1;
    }
}

/**
* Decode System data from packed message
*
* @param outData output: decoded message
* @param inEncoded input message (encoded/packed) structure
* @return success code (0 = failure, 1 = success)
*/
int decodeSystemMessage(ODID_System_data *outData, ODID_System_encoded *inEncoded)
{
    if (!inEncoded || !intInRange(inEncoded->LocationSource,0,15) || !outData) {
        return 0;
    } else {
        outData->LocationSource = inEncoded->LocationSource;
        outData->Latitude = decodeLatLon(inEncoded->Latitude);
        outData->Longitude = decodeLatLon(inEncoded->Longitude);
        outData->GroupCount = inEncoded->GroupCount;
        outData->GroupRadius = decodeGroupRadius(inEncoded->GroupRadius);
        outData->GroupCeiling = decodeAltitude(inEncoded->GroupCeiling);
        return 1;
    }
}

/**
* Safely fill then copy string to destination
*
* This prevents overrun and guarantees copy behavior (fully null padded)
*
* @param dstStr Destination string
* @param srcStr Source string
* @param dstSize Destination size
*/
char *safe_copyfill(char *dstStr, const char *srcStr, int dstSize)
{
    memset(dstStr,0,dstSize);  // fills destination with nulls
    strncpy(dstStr,srcStr,dstSize); // copy only up to dst size (no overruns)
    return dstStr;
}

/**
* Safely fill then copy string to destination (when decoding)
*
* This prevents overrun and guarantees copy behavior (fully null padded)
* This function was specially made because the encoded data may not be null terminated (if full size)
* Therefore, the destination must use the last byte for a null (and is +1 in size)
*
* @param dstStr Destination string
* @param srcStr Source string
* @param dstSize Destination size
*/
char *safe_dec_copyfill(char *dstStr, const char *srcStr, int dstSize)
{
    memset(dstStr,0,dstSize);  // fills destination with nulls
    strncpy(dstStr,srcStr,dstSize-1); // copy only up to dst size-1 (no overruns)
    return dstStr;
}

/**
* Safely range check a value and return the minimum or max within the range if exceeded
*
* @param inValue Value to range-check
* @param startRange Start of range to compare
* @param endRange End of range to compare
* @return same value if it fits, otherwise, min or max of range as appropriate.
*/
int intRangeMax(int64_t inValue, int startRange, int endRange) {
    if ( inValue < startRange ) {
        return startRange;
    } else if (inValue > endRange) {
        return endRange;
    } else {
        return (int) inValue;
    }
}

/**
 * Determine if an Int is in range
 *
 * @param inValue Value to range-check
 * @param startRange Start of range to compare
 * @param endRange End of range to compare
 * @return 1 = yes, 0 = no
 */
int intInRange(int inValue, int startRange, int endRange)
{
    if (inValue < startRange || inValue > endRange) {
        return 0;
    } else {
        return 1;
    }
}

#ifndef ODID_DISABLE_PRINTF

/**
* Print array of bytes as a hex string
*
* @param byteArray Array of bytes to be printed
* @param asize Size of array of bytes to be printed
*/

void printByteArray(uint8_t *byteArray, uint16_t asize, int spaced)
{
    if (ENABLE_DEBUG) {
        int x;
        for (x=0;x<asize;x++) {
            printf("%02x", (unsigned int) byteArray[x]);
            if (spaced) {
                printf(" ");
            }
        }
        printf("\n");
    }
}

/**
* Print formatted BasicID Data
*
* @param BasicID structure to be printed
*/
void printBasicID_data(ODID_BasicID_data BasicID)
{
    const char ODID_BasicID_data_format[] = "UASType: %d\nIDType: %d\nUASID: %s\n";
    printf(ODID_BasicID_data_format, BasicID.IDType, BasicID.UASType, BasicID.UASID);
}

/**
* Print formatted Location Data
*
* @param Location structure to be printed
*/
void printLocation_data(ODID_Location_data Location)
{
    const char ODID_Location_data_format[] = "Status: %d\nSpeedNS/EW: %.2f, %.2f\nSpeedVert: %.2f\nLat/Lon: %.7f, %.7f\nAlt: Baro, Geo, AboveTO: %.2f, %.2f, %.2f\nHoriz, Vert, Speed, TS Accuracy: %.1f, %.1f, %.1f, %.1f\nTimeStamp: %.2f\n";
    printf(ODID_Location_data_format, Location.Status, Location.SpeedNS, Location.SpeedEW,
        Location.SpeedVertical, Location.Latitude, Location.Longitude, Location.AltitudeBaro,
        Location.AltitudeGeo, Location.HeightAboveTakeoff, Location.HorizAccuracy,
        Location.VertAccuracy, Location.SpeedAccuracy, Location.TSAccuracy, Location.TimeStamp);
}

/**
* Print formatted Auth Data
*
* @param Auth structure to be printed
*/
void printAuth_data(ODID_Auth_data Auth)
{
    const char ODID_Auth_data_format[] = "AuthType: %d\nDataPage: %d\nAuthData: %s\n";
    printf(ODID_Auth_data_format, Auth.AuthType, Auth.DataPage, Auth.AuthData);
}

/**
* Print formatted SelfID Data
*
* @param SelfID structure to be printed
*/
void printSelfID_data(ODID_SelfID_data SelfID)
{
    const char ODID_SelfID_data_format[] = "DescType: %d\nDesc: %s\n";
    printf(ODID_SelfID_data_format, SelfID.DescType, SelfID.Desc);
}

/**
* Print formatted System Data
*
* @param System_data structure to be printed
*/
void printSystem_data(ODID_System_data System_data)
{
    const char ODID_System_data_format[] = "Location Source: %d\nLat/Lon: %.7f, %.7f\nGroup Count, Radius, Ceiling: %d, %d, %.2f\n";
    printf(ODID_System_data_format, System_data.LocationSource, System_data.Latitude, System_data.Longitude, System_data.GroupCount, System_data.GroupRadius, System_data.GroupCeiling);
}

#endif // ODID_DISABLE_PRINTF

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

const float SPEED_DIV[2] = {0.25f, 0.75f};
const float VSPEED_DIV = 0.5;
const int32_t LATLON_MULT = 10000000;
const float ALT_DIV = 0.5;
const int ALT_ADDER = 1000;
const int DATA_AGE_DIV = 10;

/**
* Encode direction as defined by Open Drone ID
*
* The encoding method uses 8 bits for the direction in degrees and
* one extra bit for indicating the East/West direction.
*
* @param Direcction in degrees. 0 <= x < 360. Route course based on true North
* @param EWDirection Bit flag indicating whether the direction is towards
                     East (0 - 179 degrees) or West (180 - 359)
* @return Encoded Direction in a single byte
*/
static uint8_t encodeDirection(float Direction, uint8_t *EWDirection)
{
    if (Direction < 0)
        Direction = 0;

    unsigned int direction_int = (unsigned int) roundf(Direction) % 360;
    if (direction_int < 180) {
        *EWDirection = 0;
    } else {
        *EWDirection = 1;
        direction_int -= 180;
    }
    return (uint8_t) odid_int_range_max(direction_int, 0, UINT8_MAX);
}

/**
* Encode speed into units defined by Open Drone ID
*
* The quantization algorithm allows for speed to be stored in units of 0.25 m/s
* on the low end of the scale and 0.75 m/s on the high end of the scale.
* This allows for more precise speeds to be represented in a single Uint8 byte
* rather than using a large float value.
*
* @param Speed_data Speed (and decimal) in m/s
* @param mult a (write only) value that sets the multiplier flag
* @return Encoded Speed in a single byte or max speed if over max encoded speed.
*/
static uint8_t encodeSpeedHorizontal(float Speed_data, uint8_t *mult)
{
    if (Speed_data < 0)
        Speed_data = 0;

    if (Speed_data <= UINT8_MAX * SPEED_DIV[0]) {
        *mult = 0;
        return (uint8_t) (Speed_data / SPEED_DIV[0]);
    } else {
        *mult = 1;
        int big_value = (int) ((Speed_data - (UINT8_MAX * SPEED_DIV[0])) / SPEED_DIV[1]);
        return (uint8_t) odid_int_range_max(big_value, 0, UINT8_MAX);
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
    return (int8_t) odid_int_range_max(encValue, INT8_MIN, INT8_MAX);
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
    return (int32_t) odid_int_range_max(LatLon_data * LATLON_MULT, -180 * LATLON_MULT, 180 * LATLON_MULT);
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
    return (uint16_t) odid_int_range_max((int) ((Alt_data + ALT_ADDER) / ALT_DIV), 0, UINT16_MAX);
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
    return (uint16_t) odid_int_range_max(round(Seconds_data * 10), 0, 60 * 60 * 10);
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
    return (uint8_t) odid_int_range_max(Radius / 10, 0, 255);
}

/**
* Encode Basic ID message (packed, ready for broadcast)
*
* @param out output (encoded/packed) structure
* @param in input data (non encoded/packed) structure
* @return ODID_SUCCESS or ODID_FAIL;
*/
int odid_encode_message_basic_id(odid_encoded_basic_id_t *out, odid_data_basic_id_t *in)
{
    if (!out || !in ||
        !odid_int_in_range(in->id_type, 0, 15) ||
        !odid_int_in_range(in->ua_type, 0, 15)) {
        return ODID_FAIL;
    } else {
        out->MessageType = ODID_MESSAGE_TYPE_BASIC_ID;
        out->ProtoVersion = ODID_PROTOCOL_VERSION;
        out->IDType = in->id_type;
        out->UAType = in->ua_type;
        odid_safe_copy_fill(out->UASID, in->uas_id, sizeof(out->UASID));
        return ODID_SUCCESS;
    }
}

/**
* Encode Location message (packed, ready for broadcast)
*
* @param out output (encoded/packed) structure
* @param in input data (non encoded/packed) structure
* @return ODID_SUCCESS or ODID_FAIL;
*/
int odid_encode_message_location(odid_encoded_location_t *out, odid_data_location_t *in)
{
    uint8_t bitflag;
    if (!out || !in ||
        !odid_int_in_range(in->Status, 0, 15) ||
        !odid_int_in_range(in->HorizAccuracy, 0, 15) ||
        !odid_int_in_range(in->VertAccuracy, 0, 15) ||
        !odid_int_in_range(in->BaroAccuracy, 0, 15) ||
        !odid_int_in_range(in->SpeedAccuracy, 0, 15) ||
        !odid_int_in_range(in->TSAccuracy, 0, 15)) {
        return ODID_FAIL;
    } else {
        out->MessageType = ODID_MESSAGE_TYPE_LOCATION;
        out->ProtoVersion = ODID_PROTOCOL_VERSION;
        out->Status = in->Status;
        out->Reserved = 0;
        out->Direction = encodeDirection(in->Direction, &bitflag);
        out->EWDirection = bitflag;
        out->SpeedHorizontal = encodeSpeedHorizontal(in->SpeedHorizontal, &bitflag);
        out->SpeedMult = bitflag;
        out->SpeedVertical = encodeSpeedVertical(in->SpeedVertical);
        out->Latitude = encodeLatLon(in->Latitude);
        out->Longitude = encodeLatLon(in->Longitude);
        out->AltitudeBaro = encodeAltitude(in->AltitudeBaro);
        out->AltitudeGeo = encodeAltitude(in->AltitudeGeo);
        out->HeightType = in->HeightType;
        out->Height = encodeAltitude(in->Height);
        out->HorizAccuracy = in->HorizAccuracy;
        out->VertAccuracy = in->VertAccuracy;
        out->BaroAccuracy = in->BaroAccuracy;
        out->SpeedAccuracy = in->SpeedAccuracy;
        out->TSAccuracy = in->TSAccuracy;
        out->Reserved2 = 0;
        out->TimeStamp = encodeTimeStamp(in->TimeStamp);
        out->Reserved3 = 0;
        return ODID_SUCCESS;
    }
}

/**
* Encode Auth message (packed, ready for broadcast)
*
* @param out output (encoded/packed) structure
* @param in input data (non encoded/packed) structure
* @return ODID_SUCCESS or ODID_FAIL;
*/
int odid_encode_message_auth(odid_encoded_auth_t *out, odid_data_auth_t *in)
{
    if (!out || !in || !odid_int_in_range(in->AuthType, 0, 15)) {
        return ODID_FAIL;
    } else {
        out->MessageType = ODID_MESSAGE_TYPE_AUTH;
        out->ProtoVersion = ODID_PROTOCOL_VERSION;
        out->AuthType = in->AuthType;
        // TODO: Implement Multi-page support (for now, this will handle a single DataPage)
        out->DataPage = 0;
        odid_safe_copy_fill(out->AuthData, in->AuthData, sizeof(out->AuthData));
        return ODID_SUCCESS;
    }
}

/**
* Encode Self ID message (packed, ready for broadcast)
*
* @param out output (encoded/packed) structure
* @param in input data (non encoded/packed) structure
* @return ODID_SUCCESS or ODID_FAIL;
*/
int odid_encode_message_self_id(odid_encoded_self_id_t *out, odid_data_self_id_t *in)
{
    if (!out || !in) {
        return ODID_FAIL;
    } else {
        out->MessageType = ODID_MESSAGE_TYPE_SELF_ID;
        out->ProtoVersion = ODID_PROTOCOL_VERSION;
        out->DescType = in->DescType;
        odid_safe_copy_fill(out->Desc, in->Desc, sizeof(out->Desc));
        return ODID_SUCCESS;
    }
}

/**
* Encode System message (packed, ready for broadcast)
*
* @param out output (encoded/packed) structure
* @param in input data (non encoded/packed) structure
* @return ODID_SUCCESS or ODID_FAIL;
*/
int odid_encode_message_system(odid_encoded_system_t *out, odid_data_system_t *in)
{
    if (!out || !in) {
        return ODID_FAIL;
    } else {
        out->MessageType = ODID_MESSAGE_TYPE_SYSTEM;
        out->ProtoVersion = ODID_PROTOCOL_VERSION;
        out->Reserved = 0;
        out->LocationSource = in->LocationSource;
        out->remotePilotLatitude = encodeLatLon(in->remotePilotLatitude);
        out->remotePilotLongitude = encodeLatLon(in->remotePilotLongitude);
        out->GroupCount = in->GroupCount;
        out->GroupRadius = encodeGroupRadius(in->GroupRadius);
        out->GroupCeiling = encodeAltitude(in->GroupCeiling);
        out->GroupFloor = encodeAltitude(in->GroupFloor);
        memset(out->Reserved2, 0, sizeof(out->Reserved2));
        return ODID_SUCCESS;
    }
}

/**
* Dencode direction from Open Drone ID packed message
*
* @param Direction_enc encoded direction
* @param EWDirection East/West direction flag
* @return direction in degrees (0 - 359)
*/
static float decodeDirection(uint8_t Direction_enc, uint8_t EWDirection)
{
    if (EWDirection)
        return Direction_enc + 180;
    else
        return Direction_enc;
}

/**
* Dencode speed from Open Drone ID packed message
*
* @param Speed_enc encoded speed
* @param mult multiplier flag
* @return decoded speed in m/s
*/
static float decodeSpeedHorizontal(uint8_t Speed_enc, uint8_t mult)
{
    if (mult)
        return ((float) Speed_enc * SPEED_DIV[1]) + (UINT8_MAX * SPEED_DIV[0]);
    else
        return (float) Speed_enc * SPEED_DIV[0];
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
* @param out output: decoded message
* @param in input message (encoded/packed) structure
* @return ODID_SUCCESS or ODID_FAIL;
*/
int odid_decode_message_basic_id(odid_data_basic_id_t *out, odid_encoded_basic_id_t *in)
{
    if (!out || !in ||
        !odid_int_in_range(in->IDType, 0, 15) ||
        !odid_int_in_range(in->UAType, 0, 15)) {
        return ODID_FAIL;
    } else {
        out->id_type = (odid_id_type_t) in->IDType;
        out->ua_type = (odid_ua_type_t) in->UAType;
        odid_safe_dec_copy_fill(out->uas_id, in->UASID, sizeof(out->uas_id));
        return ODID_SUCCESS;
    }
}

/**
* Decode Location data from packed message
*
* @param out output: decoded message
* @param in input message (encoded/packed) structure
* @return ODID_SUCCESS or ODID_FAIL;
*/
int odid_decode_message_location(odid_data_location_t *out, odid_encoded_location_t *in)
{
    if (!out || !in || !odid_int_in_range(in->Status, 0, 15)) {
        return ODID_FAIL;
    } else {
        out->Status = (odid_status_t) in->Status;
        out->Direction = decodeDirection(in->Direction, in-> EWDirection);
        out->SpeedHorizontal = decodeSpeedHorizontal(in->SpeedHorizontal, in->SpeedMult);
        out->SpeedVertical = decodeSpeedVertical(in->SpeedVertical);
        out->Latitude = decodeLatLon(in->Latitude);
        out->Longitude = decodeLatLon(in->Longitude);
        out->AltitudeBaro = decodeAltitude(in->AltitudeBaro);
        out->AltitudeGeo = decodeAltitude(in->AltitudeGeo);
        out->HeightType = (odid_height_ref_t) in->HeightType;
        out->Height = decodeAltitude(in->Height);
        out->HorizAccuracy = (odid_horizontal_accuracy_t) in->HorizAccuracy;
        out->VertAccuracy = (odid_vertical_accuracy_t) in->VertAccuracy;
        out->BaroAccuracy = (odid_vertical_accuracy_t) in->BaroAccuracy;
        out->SpeedAccuracy = (odid_speed_accuracy_t) in->SpeedAccuracy;
        out->TSAccuracy = (odid_timestamp_accuracy_t) in->TSAccuracy;
        out->TimeStamp = decodeTimeStamp(in->TimeStamp);
        return ODID_SUCCESS;
    }
}

/**
* Decode Auth data from packed message
*
* @param out output decoded message
* @param in input message (encoded/packed) structure
* @return ODID_SUCCESS or ODID_FAIL;
*/
int odid_decode_message_auth(odid_data_auth_t *out, odid_encoded_auth_t *in)
{
    if (!out || !in || !odid_int_in_range(in->AuthType, 0, 15)) {
        return ODID_FAIL;
    } else {
        // TODO: Implement Multi-page support (for now, this will handle a single DataPage)
        out->AuthType = (odid_auth_type_t) in->AuthType;
        out->DataPage = 0;
        odid_safe_dec_copy_fill(out->AuthData, in->AuthData, sizeof(out->AuthData));
        return ODID_SUCCESS;
    }
}

/**
* Decode Self ID data from packed message
*
* @param out output: decoded message
* @param in input message (encoded/packed) structure
* @return ODID_SUCCESS or ODID_FAIL;
*/
int odid_decode_message_self_id(odid_data_self_id_t *out, odid_encoded_self_id_t *in)
{
    if (!out || !in) {
        return ODID_FAIL;
    } else {
        out->DescType = (odid_desc_type_t) in->DescType;
        odid_safe_dec_copy_fill(out->Desc, in->Desc, sizeof(out->Desc));
        return ODID_SUCCESS;
    }
}

/**
* Decode System data from packed message
*
* @param out output: decoded message
* @param in input message (encoded/packed) structure
* @return ODID_SUCCESS or ODID_FAIL;
*/
int odid_decode_message_system(odid_data_system_t *out, odid_encoded_system_t *in)
{
    if (!out || !in) {
        return ODID_FAIL;
    } else {
        out->LocationSource = (odid_location_source_t) in->LocationSource;
        out->remotePilotLatitude = decodeLatLon(in->remotePilotLatitude);
        out->remotePilotLongitude = decodeLatLon(in->remotePilotLongitude);
        out->GroupCount = in->GroupCount;
        out->GroupRadius = decodeGroupRadius(in->GroupRadius);
        out->GroupCeiling = decodeAltitude(in->GroupCeiling);
        out->GroupFloor = decodeAltitude(in->GroupFloor);
        return ODID_SUCCESS;
    }
}

/**
* Decodes the message type of a packed Open Drone ID message
*
* @param byte   The first byte of the message
* @return       The message type: ODID_messagetype_t
*/
odid_message_type_t odid_decode_message_type(uint8_t byte)
{
    switch (byte >> 4)
    {
    case ODID_MESSAGE_TYPE_BASIC_ID:
        return ODID_MESSAGE_TYPE_BASIC_ID;
    case ODID_MESSAGE_TYPE_LOCATION:
        return ODID_MESSAGE_TYPE_LOCATION;
    case ODID_MESSAGE_TYPE_AUTH:
        return ODID_MESSAGE_TYPE_AUTH;
    case ODID_MESSAGE_TYPE_SELF_ID:
        return ODID_MESSAGE_TYPE_SELF_ID;
    case ODID_MESSAGE_TYPE_SYSTEM:
        return ODID_MESSAGE_TYPE_SYSTEM;
    default:
        return ODID_MESSAGE_TYPE_INVALID;
    }
}

/**
* Parse encoded Open Drone ID data to identify the message type and decode
* from Open Drone ID packed format into the appropriate Open Drone ID structure
*
* This function assumes that msg_data points to a buffer conaining all
* ODID_MESSAGE_SIZE bytes of an Open Drone ID message.
*
* @param uas_data   Structure containing buffers for holding all message data
* @param msg_data   Pointer to a buffer containing a full encoded Open Drone ID
*                   message
* @return           The message type: ODID_messagetype_t
*/
odid_message_type_t odid_decode_open_drone_id(odid_data_uas_t *uas_data, uint8_t *msg_data)
{
    if (!uas_data || !msg_data)
        return ODID_MESSAGE_TYPE_INVALID;

    switch (odid_decode_message_type(msg_data[0]))
    {
    case ODID_MESSAGE_TYPE_BASIC_ID:
        if (odid_decode_message_basic_id(&uas_data->basic_id,
                                         (odid_encoded_basic_id_t *) msg_data) == ODID_SUCCESS)
            return ODID_MESSAGE_TYPE_BASIC_ID;
        break;

    case ODID_MESSAGE_TYPE_LOCATION:
        if (odid_decode_message_location(&uas_data->location,
                                         (odid_encoded_location_t *) msg_data) == ODID_SUCCESS)
            return ODID_MESSAGE_TYPE_LOCATION;
        break;

    case ODID_MESSAGE_TYPE_AUTH:
        if (odid_decode_message_auth(&uas_data->auth,
                                     (odid_encoded_auth_t *) msg_data) == ODID_SUCCESS)
            return ODID_MESSAGE_TYPE_AUTH;
        break;

    case ODID_MESSAGE_TYPE_SELF_ID:
        if (odid_decode_message_self_id(&uas_data->self_id,
                                        (odid_encoded_self_id_t *) msg_data) == ODID_SUCCESS)
            return ODID_MESSAGE_TYPE_SELF_ID;
        break;

    case ODID_MESSAGE_TYPE_SYSTEM:
        if (odid_decode_message_system(&uas_data->system,
                                       (odid_encoded_system_t *) msg_data) == ODID_SUCCESS)
            return ODID_MESSAGE_TYPE_SYSTEM;
        break;

    default:
        break;
    }

    return ODID_MESSAGE_TYPE_INVALID;
}

/**
* Safely fill then copy string to destination
*
* This prevents overrun and guarantees copy behavior (fully null padded)
*
* @param dst_str Destination string
* @param src_str Source string
* @param dst_size Destination size
*/
char *odid_safe_copy_fill(char *dst_str, const char *src_str, int dst_size)
{
    memset(dst_str, 0, dst_size);  // fills destination with nulls
    strncpy(dst_str, src_str, dst_size); // copy only up to dst size (no overruns)
    return dst_str;
}

/**
* Safely fill then copy string to destination (when decoding)
*
* This prevents overrun and guarantees copy behavior (fully null padded)
* This function was specially made because the encoded data may not be null terminated (if full size)
* Therefore, the destination must use the last byte for a null (and is +1 in size)
*
* @param dst_str Destination string
* @param src_str Source string
* @param dst_size Destination size
*/
char *odid_safe_dec_copy_fill(char *dst_str, const char *src_str, int dst_size)
{
    memset(dst_str, 0, dst_size);  // fills destination with nulls
    strncpy(dst_str, src_str, dst_size - 1); // copy only up to dst size-1 (no overruns)
    return dst_str;
}

/**
* Safely range check a value and return the minimum or max within the range if exceeded
*
* @param value Value to range-check
* @param start_range Start of range to compare
* @param end_range End of range to compare
* @return same value if it fits, otherwise, min or max of range as appropriate.
*/
int odid_int_range_max(int64_t value, int start_range, int end_range) {
    if (value < start_range ) {
        return start_range;
    } else if (value > end_range) {
        return end_range;
    } else {
        return (int) value;
    }
}

/**
 * Determine if an Int is in range
 *
 * @param inValue Value to range-check
 * @param start_range Start of range to compare
 * @param end_range End of range to compare
 * @return 1 = yes, 0 = no
 */
int odid_int_in_range(int inValue, int start_range, int end_range)
{
    if (inValue < start_range || inValue > end_range) {
        return 0;
    } else {
        return 1;
    }
}

/**
* This converts a horizontal accuracy float value to the corresponding enum
*
* @param accuracy The horizontal accuracy in meters
* @return Enum value representing the accuracy
*/
odid_horizontal_accuracy_t odid_create_enum_horizontal_accuracy(float accuracy)
{
    if (accuracy >= 18520)
        return ODID_HOR_ACC_UNKNOWN;
    else if (accuracy > 7408)
        return ODID_HOR_ACC_10NM;
    else if (accuracy > 3704)
        return ODID_HOR_ACC_4NM;
    else if (accuracy > 1852)
        return ODID_HOR_ACC_2NM;
    else if (accuracy > 926)
        return ODID_HOR_ACC_1NM;
    else if (accuracy > 555.6f)
        return ODID_HOR_ACC_0_5NM;
    else if (accuracy > 185.2f)
        return ODID_HOR_ACC_0_3NM;
    else if (accuracy > 92.6f)
        return ODID_HOR_ACC_0_1NM;
    else if (accuracy > 30)
        return ODID_HOR_ACC_0_05NM;
    else if (accuracy > 10)
        return ODID_HOR_ACC_30_METER;
    else if (accuracy > 3)
        return ODID_HOR_ACC_10_METER;
    else if (accuracy > 1)
        return ODID_HOR_ACC_3_METER;
    else if (accuracy > 0)
        return ODID_HOR_ACC_1_METER;
    else
        return ODID_HOR_ACC_UNKNOWN;
}

/**
* This converts a vertical accuracy float value to the corresponding enum
*
* @param accuracy The vertical accuracy in meters
* @return Enum value representing the accuracy
*/
odid_vertical_accuracy_t odid_create_enum_vertical_accuracy(float accuracy)
{
    if (accuracy >= 150)
        return ODID_VER_ACC_UNKNOWN;
    else if (accuracy > 45)
        return ODID_VER_ACC_150_METER;
    else if (accuracy > 25)
        return ODID_VER_ACC_45_METER;
    else if (accuracy > 10)
        return ODID_VER_ACC_25_METER;
    else if (accuracy > 3)
        return ODID_VER_ACC_10_METER;
    else if (accuracy > 1)
        return ODID_VER_ACC_3_METER;
    else if (accuracy > 0)
        return ODID_VER_ACC_1_METER;
    else
        return ODID_VER_ACC_UNKNOWN;
}

/**
* This converts a speed accuracy float value to the corresponding enum
*
* @param accuracy The speed accuracy in m/s
* @return Enum value representing the accuracy
*/
odid_speed_accuracy_t odid_create_enum_speed_accuracy(float accuracy)
{
    if (accuracy >= 10)
        return ODID_SPEED_ACC_UNKNOWN;
    else if (accuracy > 3)
        return ODID_SPEED_ACC_10_METERS_SECOND;
    else if (accuracy > 1)
        return ODID_SPEED_ACC_3_METERS_SECOND;
    else if (accuracy > 0.3f)
        return ODID_SPEED_ACC_1_METERS_SECOND;
    else if (accuracy > 0)
        return ODID_SPEED_ACC_0_3_METERS_SECOND;
    else
        return ODID_SPEED_ACC_UNKNOWN;
}

/**
* This converts a timestamp accuracy float value to the corresponding enum
*
* @param accuracy The timestamp accuracy in seconds
* @return Enum value representing the accuracy
*/
odid_timestamp_accuracy_t odid_create_enum_timestamp_accuracy(float accuracy)
{
    if (accuracy > 1.5f)
        return ODID_TIME_ACC_UNKNOWN;
    else if (accuracy > 1.4f)
        return ODID_TIME_ACC_1_5_SECONDS;
    else if (accuracy > 1.3f)
        return ODID_TIME_ACC_1_4_SECONDS;
    else if (accuracy > 1.2f)
        return ODID_TIME_ACC_1_3_SECONDS;
    else if (accuracy > 1.1f)
        return ODID_TIME_ACC_1_2_SECONDS;
    else if (accuracy > 1.0f)
        return ODID_TIME_ACC_1_1_SECONDS;
    else if (accuracy > 0.9f)
        return ODID_TIME_ACC_1_0_SECONDS;
    else if (accuracy > 0.8f)
        return ODID_TIME_ACC_0_9_SECONDS;
    else if (accuracy > 0.7f)
        return ODID_TIME_ACC_0_8_SECONDS;
    else if (accuracy > 0.6f)
        return ODID_TIME_ACC_0_7_SECONDS;
    else if (accuracy > 0.5f)
        return ODID_TIME_ACC_0_6_SECONDS;
    else if (accuracy > 0.4f)
        return ODID_TIME_ACC_0_5_SECONDS;
    else if (accuracy > 0.3f)
        return ODID_TIME_ACC_0_4_SECONDS;
    else if (accuracy > 0.2f)
        return ODID_TIME_ACC_0_3_SECONDS;
    else if (accuracy > 0.1f)
        return ODID_TIME_ACC_0_2_SECONDS;
    else if (accuracy > 0.0f)
        return ODID_TIME_ACC_0_1_SECONDS;
    else
        return ODID_TIME_ACC_UNKNOWN;
}

/**
* This decodes a horizontal accuracy enum to the corresponding float value
*
* @param accuracy Enum value representing the accuracy
* @return The maximum horizontal accuracy in meters
*/
float odid_decode_horizontal_accuracy(odid_horizontal_accuracy_t accuracy)
{
    switch (accuracy)
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
* This decodes a vertical accuracy enum to the corresponding float value
*
* @param accuracy Enum value representing the accuracy
* @return The maximum vertical accuracy in meters
*/
float odid_decode_vertical_accuracy(odid_vertical_accuracy_t accuracy)
{
    switch (accuracy)
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
* This decodes a speed accuracy enum to the corresponding float value
*
* @param accuracy Enum value representing the accuracy
* @return The maximum speed accuracy in m/s
*/
float odid_decode_speed_accuracy(odid_speed_accuracy_t accuracy)
{
    switch (accuracy)
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
* This decodes a timestamp accuracy enum to the corresponding float value
*
* @param accuracy Enum value representing the accuracy
* @return The maximum timestamp accuracy in seconds
*/
float odid_decode_timestamp_accuracy(odid_timestamp_accuracy_t accuracy)
{
    switch (accuracy)
    {
    case ODID_TIME_ACC_UNKNOWN:
        return 0.0f;
    case ODID_TIME_ACC_0_1_SECONDS:
        return 0.1f;
    case ODID_TIME_ACC_0_2_SECONDS:
        return 0.2f;
    case ODID_TIME_ACC_0_3_SECONDS:
        return 0.3f;
    case ODID_TIME_ACC_0_4_SECONDS:
        return 0.4f;
    case ODID_TIME_ACC_0_5_SECONDS:
        return 0.5f;
    case ODID_TIME_ACC_0_6_SECONDS:
        return 0.6f;
    case ODID_TIME_ACC_0_7_SECONDS:
        return 0.7f;
    case ODID_TIME_ACC_0_8_SECONDS:
        return 0.8f;
    case ODID_TIME_ACC_0_9_SECONDS:
        return 0.9f;
    case ODID_TIME_ACC_1_0_SECONDS:
        return 1.0f;
    case ODID_TIME_ACC_1_1_SECONDS:
        return 1.1f;
    case ODID_TIME_ACC_1_2_SECONDS:
        return 1.2f;
    case ODID_TIME_ACC_1_3_SECONDS:
        return 1.3f;
    case ODID_TIME_ACC_1_4_SECONDS:
        return 1.4f;
    case ODID_TIME_ACC_1_5_SECONDS:
        return 1.5f;
    default:
        return 0.0f;
    }
}

#ifndef ODID_DISABLE_PRINTF

/**
* Print array of bytes as a hex string
*
* @param byte_array Array of bytes to be printed
* @param asize Size of array of bytes to be printed
*/

void odid_print_byte_array(uint8_t *byte_array, uint16_t asize, int spaced)
{
    if (ENABLE_DEBUG) {
        int x;
        for (x=0;x<asize;x++) {
            printf("%02x", (unsigned int) byte_array[x]);
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
* @param basic_id structure to be printed
*/
void odid_print_data_basic_id(odid_data_basic_id_t *basic_id)
{
    const char ODID_BasicID_data_format[] =
        "ua_type: %d\nIDType: %d\nUASID: %s\n";
    printf(ODID_BasicID_data_format, basic_id->id_type, basic_id->ua_type,
           basic_id->uas_id);
}

/**
* Print formatted Location Data
*
* @param location structure to be printed
*/
void odid_print_data_location(odid_data_location_t *location)
{
    const char ODID_Location_data_format[] =
        "Status: %d\nDirection: %.1f\nSpeedHori: %.2f\nSpeedVert: \
        %.2f\nLat/Lon: %.7f, %.7f\nAlt: Baro, Geo, Height above %s: %.2f, \
        %.2f, %.2f\nHoriz, Vert, Baro, Speed, TS Accuracy: %.1f, %.1f, %.1f, \
        %.1f, %.1f\nTimeStamp: %.2f\n";
    printf(ODID_Location_data_format, location->Status, location->Direction,
           location->SpeedHorizontal, location->SpeedVertical, location->Latitude,
           location->Longitude, location->HeightType ? "Ground" : "TakeOff",
           location->AltitudeBaro, location->AltitudeGeo, location->Height,
           odid_decode_horizontal_accuracy(location->HorizAccuracy),
           odid_decode_vertical_accuracy(location->VertAccuracy),
           odid_decode_vertical_accuracy(location->BaroAccuracy),
           odid_decode_speed_accuracy(location->SpeedAccuracy),
           odid_decode_timestamp_accuracy(location->TSAccuracy),
           location->TimeStamp);
}

/**
* Print formatted Auth Data
*
* @param auth structure to be printed
*/
void odid_print_data_auth(odid_data_auth_t *auth)
{
    const char ODID_Auth_data_format[] =
        "AuthType: %d\nDataPage: %d\nAuthData: %s\n";
    printf(ODID_Auth_data_format, auth->AuthType, auth->DataPage,
           auth->AuthData);
}

/**
* Print formatted SelfID Data
*
* @param self_id structure to be printed
*/
void odid_print_data_self_id(odid_data_self_id_t *self_id)
{
    const char ODID_SelfID_data_format[] = "DescType: %d\nDesc: %s\n";
    printf(ODID_SelfID_data_format, self_id->DescType, self_id->Desc);
}

/**
* Print formatted System Data
*
* @param system_data structure to be printed
*/
void odid_print_data_system(odid_data_system_t *system_data)
{
    const char ODID_System_data_format[] = "location Source: %d\nLat/Lon: \
        %.7f, %.7f\nGroup Count, Radius, Ceiling, Floor: %d, %d, %.2f, %.2f\n";
    printf(ODID_System_data_format, system_data->LocationSource,
           system_data->remotePilotLatitude, system_data->remotePilotLongitude,
           system_data->GroupCount, system_data->GroupRadius,
           system_data->GroupCeiling, system_data->GroupFloor);
}

#endif // ODID_DISABLE_PRINTF

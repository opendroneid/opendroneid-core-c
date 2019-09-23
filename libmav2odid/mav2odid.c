/*
Copyright (C) 2019 Intel Corporation

SPDX-License-Identifier: Apache-2.0

Mavlink to Open Drone ID C Library

Maintainer:
Soren Friis
soren.friis@intel.com
*/

#include "mav2odid.h"

/**
* Set up the schedule in which messages are broadcast and init the structures
* for converting Mavlink messages to Open Drone ID data
*
* Every second message is a Location message, since it is declared dynamic
* in the specification and thus must be broadcast more often than the rest.
*
* If one or more of the optional message types (Authentication, SelfID, System)
* are not used in a particular implementation, they should be excluded from
* the schedule list.
*
* @param m2o    Instance structure containing working buffers/structures
* @return       Success or fail
*/
int m2o_init(mav2odid_t *m2o)
{
    if (!m2o)
        return ODID_FAIL;

    memset(m2o, 0, sizeof(mav2odid_t));

    memset(m2o->droneidSchedule, ODID_MESSAGETYPE_LOCATION, DRONEID_SCHEDULER_SIZE);
    m2o->droneidSchedule[0] = ODID_MESSAGETYPE_BASIC_ID;
    m2o->droneidSchedule[2] = ODID_MESSAGETYPE_AUTH;
    m2o->droneidSchedule[4] = ODID_MESSAGETYPE_SELF_ID;
    m2o->droneidSchedule[6] = ODID_MESSAGETYPE_SYSTEM;

    if (encodeBasicIDMessage(&m2o->basicIdEnc, &m2o->basicId))
        return ODID_FAIL;
    if (encodeLocationMessage(&m2o->locationEnc, &m2o->location))
        return ODID_FAIL;
    if (encodeAuthMessage(&m2o->authenticationEnc, &m2o->authentication))
        return ODID_FAIL;
    if (encodeSelfIDMessage(&m2o->selfIdEnc, &m2o->selfId))
        return ODID_FAIL;
    if (encodeSystemMessage(&m2o->systemEnc, &m2o->system))
        return ODID_FAIL;
    return ODID_SUCCESS;
}

/**
* Cycle through the various DroneID messages according to the schedule defined
* in droneidSchedule.
*
* It is expected that this function is called with an interval faster than
* (BcMinStaticRefreshRate seconds / DRONEID_SCHEDULER_SIZE) = 3 / 8 = 375 ms
* in order to comply with the timing restraints in the specification.
*
* This function will copy the relevant DroneID data to the provided data buffer.
*
* @param m2o    Instance structure containing working buffers
* @param data   Pointer to the buffer into which the current message data will
*               be copied
* @return       Success or fail
*/
int m2o_cycleMessages(mav2odid_t *m2o, uint8_t *data)
{
    if (!m2o || !data)
        return ODID_FAIL;

    switch (m2o->droneidSchedule[m2o->scheduleIdx])
    {
    case ODID_MESSAGETYPE_BASIC_ID:
        memcpy(data, &m2o->basicIdEnc, sizeof(ODID_BasicID_encoded));
        break;
    case ODID_MESSAGETYPE_LOCATION:
        memcpy(data, &m2o->locationEnc, sizeof(ODID_Location_encoded));
        break;
    case ODID_MESSAGETYPE_AUTH:
        memcpy(data, &m2o->authenticationEnc, sizeof(ODID_Auth_encoded));
        break;
    case ODID_MESSAGETYPE_SELF_ID:
        memcpy(data, &m2o->selfIdEnc, sizeof(ODID_SelfID_encoded));
        break;
    case ODID_MESSAGETYPE_SYSTEM:
        memcpy(data, &m2o->systemEnc, sizeof(ODID_System_encoded));
        break;
    default:
        return ODID_FAIL;
    }

    m2o->scheduleIdx = ((m2o->scheduleIdx + 1) % DRONEID_SCHEDULER_SIZE);
    return ODID_SUCCESS;
}

/**
* Convert basic ID Mavlink message to encoded Open Drone ID structure
*/
static int m2o_basicId(mav2odid_t *m2o, mavlink_open_drone_id_basic_id_t *basicId)
{
    m2o->basicId.IDType = (ODID_idtype_t) basicId->id_type;
    m2o->basicId.UAType = (ODID_uatype_t) basicId->ua_type;
    for (int i = 0; i < MAVLINK_MSG_OPEN_DRONE_ID_BASIC_ID_FIELD_UAS_ID_LEN; i++)
        m2o->basicId.UASID[i] = basicId->uas_id[i];

    return encodeBasicIDMessage(&m2o->basicIdEnc, &m2o->basicId);
}

/**
* Convert location Mavlink message to encoded Open Drone ID structure
*/
static int m2o_location(mav2odid_t *m2o, mavlink_open_drone_id_location_t *location)
{
    m2o->location.Status = (ODID_status_t) location->status;
    m2o->location.Direction = (float) location->direction / 100;
    m2o->location.SpeedHorizontal = (float) location->speed_horizontal / 100;
    m2o->location.SpeedVertical = (float) location->speed_vertical / 100;
    m2o->location.Latitude = (float) location->latitude / 1E7;
    m2o->location.Longitude = (float) location->longitude / 1E7;
    m2o->location.AltitudeBaro = location->altitude_barometric;
    m2o->location.AltitudeGeo = location->altitude_geodetic;
    m2o->location.HeightType = (ODID_Height_reference_t) location->height_reference;
    m2o->location.Height = location->height;
    m2o->location.HorizAccuracy = (ODID_Horizontal_accuracy_t) location->horizontal_accuracy;
    m2o->location.VertAccuracy = (ODID_Vertical_accuracy_t) location->vertical_accuracy;
    m2o->location.BaroAccuracy = (ODID_Vertical_accuracy_t) location->barometer_accuracy;
    m2o->location.SpeedAccuracy = (ODID_Speed_accuracy_t) location->speed_accuracy;
    m2o->location.TSAccuracy = (ODID_Timestamp_accuracy_t) location->timestamp_accuracy;
    m2o->location.TimeStamp = location->timestamp;

    return encodeLocationMessage(&m2o->locationEnc, &m2o->location);
}

/**
* Convert authentication Mavlink message to encoded Open Drone ID structure
*/
static int m2o_authentication(mav2odid_t *m2o, mavlink_open_drone_id_authentication_t *authentication)
{
    m2o->authentication.DataPage = authentication->data_page;
    m2o->authentication.AuthType = (ODID_authtype_t) authentication->authentication_type;
    for (int i = 0; i < MAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_AUTHENTICATION_DATA_LEN; i++)
        m2o->authentication.AuthData[i] = authentication->authentication_data[i];

    return encodeAuthMessage(&m2o->authenticationEnc, &m2o->authentication);
}

/**
* Convert self ID Mavlink message to encoded Open Drone ID structure
*/
static int m2o_selfId(mav2odid_t *m2o, mavlink_open_drone_id_selfid_t *selfId)
{
    m2o->selfId.DescType = (ODID_desctype_t) selfId->description_type;
    for (int i = 0; i < MAVLINK_MSG_OPEN_DRONE_ID_SELFID_FIELD_DESCRIPTION_LEN; i++)
        m2o->selfId.Desc[i] = selfId->description[i];

    return encodeSelfIDMessage(&m2o->selfIdEnc, &m2o->selfId);
}

/**
* Convert system Mavlink message to encoded Open Drone ID structure
*/
static int m2o_system(mav2odid_t *m2o, mavlink_open_drone_id_system_t *system)
{
    m2o->system.LocationSource = (ODID_location_source_t) system->flags;
    m2o->system.OperatorLatitude = (float) system->remote_pilot_latitude / 1E7;
    m2o->system.OperatorLongitude = (float) system->remote_pilot_longitude / 1E7;
    m2o->system.AreaCount = system->group_count;
    m2o->system.AreaRadius = system->group_radius;
    m2o->system.AreaCeiling = system->group_ceiling;
    m2o->system.AreaFloor = system->group_floor;

    return encodeSystemMessage(&m2o->systemEnc, &m2o->system);
}

/**
* Parse incoming data for detecting Mavlink messages
*
* This function must be called for each byte of data received.
*
* When the string of data bytes from subsequent calls match a Mavlink message,
* the full message will be decoded and the data from the message stored in the
* corresponding Open Drone ID structure.
*
* @param  m2o   Instance structure containing working buffers
* @param  data  One byte of data to be parsed
* @return       The type of message decoded
*/
ODID_messagetype_t m2o_parseMavlink(mav2odid_t *m2o, uint8_t data)
{
    if (!m2o)
        return ODID_MESSAGETYPE_INVALID;

    mavlink_message_t message;
    mavlink_status_t status;
    mavlink_open_drone_id_basic_id_t basicId;
    mavlink_open_drone_id_location_t location;
    mavlink_open_drone_id_authentication_t authentication;
    mavlink_open_drone_id_selfid_t selfId;
    mavlink_open_drone_id_system_t system;

    if (mavlink_parse_char(MAVLINK_COMM_0, data, &message, &status))
    {
        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID:
            mavlink_msg_open_drone_id_basic_id_decode(&message, &basicId);
            if (m2o_basicId(m2o, &basicId) == ODID_SUCCESS)
                return ODID_MESSAGETYPE_BASIC_ID;
            break;

        case MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION:
            mavlink_msg_open_drone_id_location_decode(&message, &location);
            if (m2o_location(m2o, &location) == ODID_SUCCESS)
                return ODID_MESSAGETYPE_LOCATION;
            break;

        case MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION:
            mavlink_msg_open_drone_id_authentication_decode(&message, &authentication);
            if (m2o_authentication(m2o, &authentication) == ODID_SUCCESS)
                return ODID_MESSAGETYPE_AUTH;
            break;

        case MAVLINK_MSG_ID_OPEN_DRONE_ID_SELFID:
            mavlink_msg_open_drone_id_selfid_decode(&message, &selfId);
            if (m2o_selfId(m2o, &selfId) == ODID_SUCCESS)
                return ODID_MESSAGETYPE_SELF_ID;
            break;

        case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM:
            mavlink_msg_open_drone_id_system_decode(&message, &system);
            if (m2o_system(m2o, &system) == ODID_SUCCESS)
                return ODID_MESSAGETYPE_SYSTEM;
            break;

        default:
            break;
        }
    }
    return ODID_MESSAGETYPE_INVALID;
}

/**
* Convert non-encoded Open Drone ID basic ID structure to Mavlink message
*/
void m2o_basicId2Mavlink(mavlink_open_drone_id_basic_id_t *mavBasicId,
                         ODID_BasicID_data *basicId)
{
    mavBasicId->id_type = (MAV_ODID_IDTYPE) basicId->IDType;
    mavBasicId->ua_type = (MAV_ODID_UATYPE) basicId->UAType;
    for (int i = 0; i < ODID_ID_SIZE; i++)
        mavBasicId->uas_id[i] = basicId->UASID[i];
}

/**
* Convert non-encoded Open Drone ID basic ID structure to Mavlink message
*/
void m2o_location2Mavlink(mavlink_open_drone_id_location_t *mavLocation,
                          ODID_Location_data *location)
{
    mavLocation->status = (MAV_ODID_STATUS) location->Status;
    mavLocation->direction = (uint16_t) (location->Direction * 100);
    mavLocation->speed_horizontal = (uint16_t) (location->SpeedHorizontal * 100);
    mavLocation->speed_vertical = (uint16_t) (location->SpeedVertical * 100);
    mavLocation->latitude = (int32_t) (location->Latitude * 1E7);
    mavLocation->longitude = (int32_t) (location->Longitude * 1E7);
    mavLocation->altitude_barometric = location->AltitudeBaro;
    mavLocation->altitude_geodetic = location->AltitudeGeo;
    mavLocation->height_reference = (MAV_ODID_HEIGHT_REF) location->HeightType;
    mavLocation->height = location->Height ;
    mavLocation->horizontal_accuracy = (MAV_ODID_HOR_ACC) location->HorizAccuracy;
    mavLocation->vertical_accuracy = (MAV_ODID_VER_ACC) location->VertAccuracy;
    mavLocation->barometer_accuracy = (MAV_ODID_VER_ACC) location->BaroAccuracy;
    mavLocation->speed_accuracy = (MAV_ODID_SPEED_ACC) location->SpeedAccuracy;
    mavLocation->timestamp_accuracy = (MAV_ODID_TIME_ACC) location->TSAccuracy;
    mavLocation->timestamp = location->TimeStamp;
}

/**
* Convert non-encoded Open Drone ID authentication structure to Mavlink message
*/
void m2o_authentication2Mavlink(mavlink_open_drone_id_authentication_t *mavAuth,
                                ODID_Auth_data *Auth)
{
    mavAuth->authentication_type = (MAV_ODID_AUTH) Auth->AuthType;
    mavAuth->data_page = Auth->DataPage;
    for (int i = 0; i < ODID_STR_SIZE; i++)
        mavAuth->authentication_data[i] = Auth->AuthData[i];
}

/**
* Convert non-encoded Open Drone ID self ID structure to Mavlink message
*/
void m2o_selfId2Mavlink(mavlink_open_drone_id_selfid_t *mavSelfID,
                        ODID_SelfID_data *selfID)
{
    mavSelfID->description_type = (MAV_ODID_DESC_TYPE) selfID->DescType;
    for (int i = 0; i < ODID_STR_SIZE; i++)
        mavSelfID->description[i] = selfID->Desc[i];
}

/**
* Convert non-encoded Open Drone ID system structure to Mavlink message
*/
void m2o_system2Mavlink(mavlink_open_drone_id_system_t *mavSystem,
                        ODID_System_data *system)
{
    mavSystem->flags = (MAV_ODID_LOCATION_SRC) system->LocationSource;
    mavSystem->remote_pilot_latitude = (int32_t) (system->OperatorLatitude * 1E7);
    mavSystem->remote_pilot_longitude = (int32_t) (system->OperatorLongitude * 1E7);
    mavSystem->group_count = system->AreaCount;
    mavSystem->group_radius = system->AreaRadius;
    mavSystem->group_ceiling = system->AreaCeiling;
    mavSystem->group_floor = system->AreaFloor;
}

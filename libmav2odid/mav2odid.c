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
* If one or more of the optional message types (Authentication, SelfID, System,
* OperatorID) are not used in a particular implementation, they should be
* excluded from the schedule list.
*
* Note: The MessagePack message type is not included, since the pack already
* contains all the message data and there is no need to cycle through individual
* messages then.
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
    m2o->droneidSchedule[2] = ODID_MESSAGETYPE_AUTH; // Add an amount of
    m2o->droneidSchedule[4] = ODID_MESSAGETYPE_AUTH; // auth messages equal to
    m2o->droneidSchedule[6] = ODID_MESSAGETYPE_AUTH; // ODID_AUTH_MAX_PAGES
    m2o->droneidSchedule[8] = ODID_MESSAGETYPE_AUTH;
    m2o->droneidSchedule[10] = ODID_MESSAGETYPE_AUTH;
    m2o->droneidSchedule[12] = ODID_MESSAGETYPE_SELF_ID;
    m2o->droneidSchedule[14] = ODID_MESSAGETYPE_SYSTEM;
    m2o->droneidSchedule[16] = ODID_MESSAGETYPE_OPERATOR_ID;

    union {
        ODID_BasicID_data basicId;
        ODID_Location_data location;
        ODID_Auth_data auth;
        ODID_SelfID_data selfId;
        ODID_System_data system;
        ODID_OperatorID_data operatorId;
    } data;

    odid_initBasicIDData(&data.basicId);
    if (encodeBasicIDMessage(&m2o->basicIdEnc, &data.basicId))
        return ODID_FAIL;

    odid_initLocationData(&data.location);
    if (encodeLocationMessage(&m2o->locationEnc, &data.location))
        return ODID_FAIL;

    odid_initAuthData(&data.auth);
    for (int i = 0; i < ODID_AUTH_MAX_PAGES; i++) {
        data.auth.DataPage = i;
        if (encodeAuthMessage(&m2o->authEnc[i], &data.auth))
            return ODID_FAIL;
    }

    odid_initSelfIDData(&data.selfId);
    if (encodeSelfIDMessage(&m2o->selfIdEnc, &data.selfId))
        return ODID_FAIL;

    odid_initSystemData(&data.system);
    if (encodeSystemMessage(&m2o->systemEnc, &data.system))
        return ODID_FAIL;

    odid_initOperatorIDData(&data.operatorId);
    if (encodeOperatorIDMessage(&m2o->operatorIdEnc, &data.operatorId))
        return ODID_FAIL;

    return ODID_SUCCESS;
}

/**
* Cycle through the various DroneID messages according to the schedule defined
* in droneidSchedule.
*
* This is useful e.g. when transmitting on Bluetooth Legacy broadcast where
* only one message at a time can be transmitted.
*
* It is expected that this function is called with an interval faster than
* (BcMinStaticRefreshRate seconds / DRONEID_SCHEDULER_SIZE) = 3 / 18 = 166 ms
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

    static uint8_t authIndex = 0;

    switch (m2o->droneidSchedule[m2o->scheduleIdx])
    {
    case ODID_MESSAGETYPE_BASIC_ID:
        if (m2o->basicIDEncValid)
            memcpy(data, &m2o->basicIdEnc, sizeof(ODID_BasicID_encoded));
        break;
    case ODID_MESSAGETYPE_LOCATION:
        if (m2o->locationEncValid)
            memcpy(data, &m2o->locationEnc, sizeof(ODID_Location_encoded));
        break;
    case ODID_MESSAGETYPE_AUTH:
        if (m2o->authEncValid[authIndex])
            memcpy(data, &m2o->authEnc[authIndex], sizeof(ODID_Auth_encoded));
        authIndex++;
        if (authIndex == ODID_AUTH_MAX_PAGES)
            authIndex = 0;
        break;
    case ODID_MESSAGETYPE_SELF_ID:
        if (m2o->selfIDEncValid)
            memcpy(data, &m2o->selfIdEnc, sizeof(ODID_SelfID_encoded));
        break;
    case ODID_MESSAGETYPE_SYSTEM:
        if (m2o->systemEncValid)
            memcpy(data, &m2o->systemEnc, sizeof(ODID_System_encoded));
        break;
    case ODID_MESSAGETYPE_OPERATOR_ID:
        if (m2o->operatorIDEncValid)
            memcpy(data, &m2o->operatorIdEnc, sizeof(ODID_OperatorID_encoded));
        break;
    default:
        return ODID_FAIL;
    }

    m2o->scheduleIdx = ((m2o->scheduleIdx + 1) % DRONEID_SCHEDULER_SIZE);
    return ODID_SUCCESS;
}

/**
* Collect individual encoded messages into a single encoded message pack.
*
* This is useful e.g. when transmitting on Bluetooth Long Range broadcast or
* WiFi Neighbor Awareness Networking, where it is mandated that all transmitted
* messages are packed into a message pack and transmitted at the same time.
*
* @param m2o    Instance structure containing encoded data
* @output       m2o->messagePackEnc
* @return       Success or fail
*/
int m2o_collectMessagePack(mav2odid_t *m2o)
{
    if (!m2o)
        return ODID_FAIL;

    m2o->messagePackEnc.MessageType = ODID_MESSAGETYPE_PACKED;
    m2o->messagePackEnc.ProtoVersion = ODID_PROTOCOL_VERSION;

    int i = 0;
    if (m2o->basicIDEncValid)
        memcpy(m2o->messagePackEnc.Messages[i++].rawData, &m2o->basicIdEnc, ODID_MESSAGE_SIZE);

    if (m2o->locationEncValid)
        memcpy(m2o->messagePackEnc.Messages[i++].rawData, &m2o->locationEnc, ODID_MESSAGE_SIZE);

    for (int j = 0; j < ODID_AUTH_MAX_PAGES; j++)
        if (m2o->authEncValid[j])
            memcpy(m2o->messagePackEnc.Messages[i++].rawData, &m2o->authEnc[j], ODID_MESSAGE_SIZE);

    if (m2o->selfIDEncValid)
        memcpy(m2o->messagePackEnc.Messages[i++].rawData, &m2o->selfIdEnc, ODID_MESSAGE_SIZE);

    if (m2o->systemEncValid)
        memcpy(m2o->messagePackEnc.Messages[i++].rawData, &m2o->systemEnc, ODID_MESSAGE_SIZE);

    if (m2o->operatorIDEncValid)
        memcpy(m2o->messagePackEnc.Messages[i++].rawData, &m2o->operatorIdEnc, ODID_MESSAGE_SIZE);

    m2o->messagePackEnc.SingleMessageSize = ODID_MESSAGE_SIZE;
    m2o->messagePackEnc.MsgPackSize = i;
    return ODID_SUCCESS;
}

/**
* Convert a basic ID Mavlink message to an encoded Open Drone ID structure
*/
static int m2o_basicId(mav2odid_t *m2o, mavlink_open_drone_id_basic_id_t *mavBasicId)
{
    if (!m2o || !mavBasicId)
        return ODID_FAIL;

    ODID_BasicID_data basicId;
    basicId.IDType = (ODID_idtype_t) mavBasicId->id_type;
    basicId.UAType = (ODID_uatype_t) mavBasicId->ua_type;
    for (int i = 0; i < MAVLINK_MSG_OPEN_DRONE_ID_BASIC_ID_FIELD_UAS_ID_LEN; i++)
        basicId.UASID[i] = mavBasicId->uas_id[i];

    return encodeBasicIDMessage(&m2o->basicIdEnc, &basicId);
}

/**
* Convert a location Mavlink message to an encoded Open Drone ID structure
*/
static int m2o_location(mav2odid_t *m2o, mavlink_open_drone_id_location_t *mavLocation)
{
    if (!m2o || !mavLocation)
        return ODID_FAIL;

    ODID_Location_data location;
    location.Status = (ODID_status_t) mavLocation->status;
    location.Direction = (float) mavLocation->direction / 100;
    location.SpeedHorizontal = (float) mavLocation->speed_horizontal / 100;
    location.SpeedVertical = (float) mavLocation->speed_vertical / 100;
    location.Latitude = (double) mavLocation->latitude / 1E7;
    location.Longitude = (double) mavLocation->longitude / 1E7;
    location.AltitudeBaro = mavLocation->altitude_barometric;
    location.AltitudeGeo = mavLocation->altitude_geodetic;
    location.HeightType = (ODID_Height_reference_t) mavLocation->height_reference;
    location.Height = mavLocation->height;
    location.HorizAccuracy = (ODID_Horizontal_accuracy_t) mavLocation->horizontal_accuracy;
    location.VertAccuracy = (ODID_Vertical_accuracy_t) mavLocation->vertical_accuracy;
    location.BaroAccuracy = (ODID_Vertical_accuracy_t) mavLocation->barometer_accuracy;
    location.SpeedAccuracy = (ODID_Speed_accuracy_t) mavLocation->speed_accuracy;
    location.TSAccuracy = (ODID_Timestamp_accuracy_t) mavLocation->timestamp_accuracy;
    location.TimeStamp = mavLocation->timestamp;

    return encodeLocationMessage(&m2o->locationEnc, &location);
}

/**
* Convert an authentication Mavlink message to an encoded Open Drone ID structure
*/
static int m2o_authentication(mav2odid_t *m2o, mavlink_open_drone_id_authentication_t *mavAuthentication)
{
    if (!m2o || !mavAuthentication ||
        mavAuthentication->data_page >= ODID_AUTH_MAX_PAGES)
        return ODID_FAIL;

    ODID_Auth_data authentication;
    authentication.DataPage = mavAuthentication->data_page;
    authentication.AuthType = (ODID_authtype_t) mavAuthentication->authentication_type;

    int size = MAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_AUTHENTICATION_DATA_LEN;
    if (authentication.DataPage == 0)
    {
        size -= ODID_AUTH_PAGE_ZERO_DATA_SIZE;
        authentication.PageCount = mavAuthentication->page_count;
        authentication.Length = mavAuthentication->length;
        authentication.Timestamp = mavAuthentication->timestamp;
    }

    for (int i = 0; i < size; i++)
        authentication.AuthData[i] = mavAuthentication->authentication_data[i];

    uint8_t ret = encodeAuthMessage(&m2o->authEnc[mavAuthentication->data_page],
                                    &authentication);
    if (ret != ODID_SUCCESS)
        return ret;

    m2o->authEncValid[mavAuthentication->data_page] = 1;
    return ODID_SUCCESS;
}

/**
* Convert a self ID Mavlink message to an encoded Open Drone ID structure
*/
static int m2o_selfId(mav2odid_t *m2o, mavlink_open_drone_id_self_id_t *mavSelfId)
{
    if (!m2o || !mavSelfId)
        return ODID_FAIL;

    ODID_SelfID_data selfId;
    selfId.DescType = (ODID_desctype_t) mavSelfId->description_type;
    for (int i = 0; i < MAVLINK_MSG_OPEN_DRONE_ID_SELF_ID_FIELD_DESCRIPTION_LEN; i++)
        selfId.Desc[i] = mavSelfId->description[i];

    return encodeSelfIDMessage(&m2o->selfIdEnc, &selfId);
}

/**
* Convert a system Mavlink message to an encoded Open Drone ID structure
*/
static int m2o_system(mav2odid_t *m2o, mavlink_open_drone_id_system_t *mavSystem)
{
    if (!m2o || !mavSystem)
        return ODID_FAIL;

    ODID_System_data system;
    system.OperatorLocationType =
        (ODID_operator_location_type_t) mavSystem->operator_location_type;
    system.ClassificationType =
        (ODID_classification_type_t) mavSystem->classification_type;
    system.OperatorLatitude = (double) mavSystem->operator_latitude / 1E7;
    system.OperatorLongitude = (double) mavSystem->operator_longitude / 1E7;
    system.AreaCount = mavSystem->area_count;
    system.AreaRadius = mavSystem->area_radius;
    system.AreaCeiling = mavSystem->area_ceiling;
    system.AreaFloor = mavSystem->area_floor;
    system.CategoryEU = (ODID_category_EU_t) mavSystem->category_eu;
    system.ClassEU = (ODID_class_EU_t) mavSystem->class_eu;

    return encodeSystemMessage(&m2o->systemEnc, &system);
}

/**
* Convert an operator ID Mavlink message to an encoded Open Drone ID structure
*/
static int m2o_operatorId(mav2odid_t *m2o, mavlink_open_drone_id_operator_id_t *mavOperatorId)
{
    if (!m2o || !mavOperatorId)
        return ODID_FAIL;

    ODID_OperatorID_data operatorId;
    operatorId.OperatorIdType = (ODID_operatorIdType_t) mavOperatorId->operator_id_type;
    for (int i = 0; i < MAVLINK_MSG_OPEN_DRONE_ID_OPERATOR_ID_FIELD_OPERATOR_ID_LEN; i++)
        operatorId.OperatorId[i] = mavOperatorId->operator_id[i];

    return encodeOperatorIDMessage(&m2o->operatorIdEnc, &operatorId);
}

/**
* Convert a message pack Mavlink message to an encoded Open Drone ID structure
*/
static int m2o_messagePack(mav2odid_t *m2o, mavlink_open_drone_id_message_pack_t *mavMessagePack)
{
    if (!m2o || !mavMessagePack)
        return ODID_FAIL;

    ODID_MessagePack_data messagePack;
    messagePack.SingleMessageSize = mavMessagePack->single_message_size;
    messagePack.MsgPackSize = mavMessagePack->msg_pack_size;
    for (int i = 0; i < mavMessagePack->msg_pack_size; i++)
        for (int j = 0; j < ODID_MESSAGE_SIZE; j++)
            messagePack.Messages[i].rawData[j] = mavMessagePack->messages[i*ODID_MESSAGE_SIZE + j];
    return encodeMessagePack(&m2o->messagePackEnc, &messagePack);
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

    union {
        mavlink_open_drone_id_basic_id_t basicId;
        mavlink_open_drone_id_location_t location;
        mavlink_open_drone_id_authentication_t authentication;
        mavlink_open_drone_id_self_id_t selfId;
        mavlink_open_drone_id_system_t system;
        mavlink_open_drone_id_operator_id_t operatorId;
        mavlink_open_drone_id_message_pack_t messagePack;
    } msg;
    mavlink_message_t message;

    // Note: this struct can in principle be set to null, to reduce stack usage.
    // The code in mavlink_helpers.h will not write to it, if it has not been
    // allocated and this parse function does not need the status information.
    mavlink_status_t status;

    // Enhance this, if used in a system transmitting on other than channel 0
    if (mavlink_parse_char(MAVLINK_COMM_0, data, &message, &status))
    {
        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID:
            mavlink_msg_open_drone_id_basic_id_decode(&message, &msg.basicId);
            if (m2o_basicId(m2o, &msg.basicId) == ODID_SUCCESS) {
                m2o->basicIDEncValid = 1;
                return ODID_MESSAGETYPE_BASIC_ID;
            }
            break;

        case MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION:
            mavlink_msg_open_drone_id_location_decode(&message, &msg.location);
            if (m2o_location(m2o, &msg.location) == ODID_SUCCESS) {
                m2o->locationEncValid = 1;
                return ODID_MESSAGETYPE_LOCATION;
            }
            break;

        case MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION:
            mavlink_msg_open_drone_id_authentication_decode(&message, &msg.authentication);
            if (m2o_authentication(m2o, &msg.authentication) == ODID_SUCCESS)
                return ODID_MESSAGETYPE_AUTH;
            break;

        case MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID:
            mavlink_msg_open_drone_id_self_id_decode(&message, &msg.selfId);
            if (m2o_selfId(m2o, &msg.selfId) == ODID_SUCCESS) {
                m2o->selfIDEncValid = 1;
                return ODID_MESSAGETYPE_SELF_ID;
            }
            break;

        case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM:
            mavlink_msg_open_drone_id_system_decode(&message, &msg.system);
            if (m2o_system(m2o, &msg.system) == ODID_SUCCESS) {
                m2o->systemEncValid = 1;
                return ODID_MESSAGETYPE_SYSTEM;
            }
            break;

        case MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID:
            mavlink_msg_open_drone_id_operator_id_decode(&message, &msg.operatorId);
            if (m2o_operatorId(m2o, &msg.operatorId) == ODID_SUCCESS) {
                m2o->operatorIDEncValid = 1;
                return ODID_MESSAGETYPE_OPERATOR_ID;
            }
            break;

        case MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK:
            mavlink_msg_open_drone_id_message_pack_decode(&message, &msg.messagePack);
            if (m2o_messagePack(m2o, &msg.messagePack) == ODID_SUCCESS) {
                m2o->messagePackEncValid = 1;
                return ODID_MESSAGETYPE_PACKED;
            }
            break;

        default:
            break;
        }
    }
    return ODID_MESSAGETYPE_INVALID;
}

/**
* Convert a non-encoded Open Drone ID basic ID structure to a Mavlink message
*/
void m2o_basicId2Mavlink(mavlink_open_drone_id_basic_id_t *mavBasicId,
                         ODID_BasicID_data *basicId)
{
    if (!mavBasicId || !basicId)
        return;

    mavBasicId->id_type = (MAV_ODID_ID_TYPE) basicId->IDType;
    mavBasicId->ua_type = (MAV_ODID_UA_TYPE) basicId->UAType;
    for (int i = 0; i < ODID_ID_SIZE; i++)
        mavBasicId->uas_id[i] = basicId->UASID[i];
}

/**
* Convert a non-encoded Open Drone ID basic ID structure to a Mavlink message
*/
void m2o_location2Mavlink(mavlink_open_drone_id_location_t *mavLocation,
                          ODID_Location_data *location)
{
    if (!mavLocation || !location)
        return;

    mavLocation->status = (MAV_ODID_STATUS) location->Status;
    mavLocation->direction = (uint16_t) (location->Direction * 100);
    mavLocation->speed_horizontal = (uint16_t) (location->SpeedHorizontal * 100);
    mavLocation->speed_vertical = (int16_t) (location->SpeedVertical * 100);
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
* Convert a non-encoded Open Drone ID authentication structure to a Mavlink message
*/
void m2o_authentication2Mavlink(mavlink_open_drone_id_authentication_t *mavAuth,
                                ODID_Auth_data *Auth)
{
    if (!mavAuth || !Auth)
        return;

    mavAuth->authentication_type = (MAV_ODID_AUTH_TYPE) Auth->AuthType;
    mavAuth->data_page = Auth->DataPage;

    int size = ODID_STR_SIZE;
    if (Auth->DataPage == 0)
    {
        size -= ODID_AUTH_PAGE_ZERO_DATA_SIZE;
        mavAuth->page_count = Auth->PageCount;
        mavAuth->length = Auth->Length;
        mavAuth->timestamp = Auth->Timestamp;
    }

    for (int i = 0; i < size; i++)
        mavAuth->authentication_data[i] = Auth->AuthData[i];
}

/**
* Convert a non-encoded Open Drone ID self ID structure to a Mavlink message
*/
void m2o_selfId2Mavlink(mavlink_open_drone_id_self_id_t *mavSelfID,
                        ODID_SelfID_data *selfID)
{
    if (!mavSelfID || !selfID)
        return;

    mavSelfID->description_type = (MAV_ODID_DESC_TYPE) selfID->DescType;
    for (int i = 0; i < ODID_STR_SIZE; i++)
        mavSelfID->description[i] = selfID->Desc[i];
}

/**
* Convert a non-encoded Open Drone ID system structure to a Mavlink message
*/
void m2o_system2Mavlink(mavlink_open_drone_id_system_t *mavSystem,
                        ODID_System_data *system)
{
    if (!mavSystem || !system)
        return;

    mavSystem->operator_location_type =
        (MAV_ODID_OPERATOR_LOCATION_TYPE) system->OperatorLocationType;
    mavSystem->classification_type =
        (MAV_ODID_CLASSIFICATION_TYPE) system->ClassificationType;
    mavSystem->operator_latitude = (int32_t) (system->OperatorLatitude * 1E7);
    mavSystem->operator_longitude = (int32_t) (system->OperatorLongitude * 1E7);
    mavSystem->area_count = system->AreaCount;
    mavSystem->area_radius = system->AreaRadius;
    mavSystem->area_ceiling = system->AreaCeiling;
    mavSystem->area_floor = system->AreaFloor;
    mavSystem->category_eu = (ODID_category_EU_t) system->CategoryEU;
    mavSystem->class_eu = (ODID_class_EU_t) system->ClassEU;
}

/**
* Convert a non-encoded Open Drone ID operator ID structure to a Mavlink message
*/
void m2o_operatorId2Mavlink(mavlink_open_drone_id_operator_id_t *mavOperatorID,
                            ODID_OperatorID_data *operatorID)
{
    if (!mavOperatorID || !operatorID)
        return;

    mavOperatorID->operator_id_type = (MAV_ODID_OPERATOR_ID_TYPE) operatorID->OperatorIdType;
    for (int i = 0; i < ODID_ID_SIZE; i++)
        mavOperatorID->operator_id[i] = operatorID->OperatorId[i];
}

/**
* Convert a non-encoded Open Drone ID message pack structure to a Mavlink message
*/
void m2o_messagePack2Mavlink(mavlink_open_drone_id_message_pack_t *mavMessagePack,
                             ODID_MessagePack_data *messagePack)
{
    if (!mavMessagePack || !messagePack)
        return;

    mavMessagePack->single_message_size = messagePack->SingleMessageSize;
    mavMessagePack->msg_pack_size = messagePack->MsgPackSize;
    for (int i = 0; i < messagePack->MsgPackSize; i++)
        for (int j = 0; j < ODID_MESSAGE_SIZE; j++)
            mavMessagePack->messages[i*ODID_MESSAGE_SIZE + j] = messagePack->Messages[i].rawData[j];
}

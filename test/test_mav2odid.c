/*
Copyright (C) 2019 Intel Corporation

SPDX-License-Identifier: Apache-2.0

Mavlink to Open Drone ID C Library

Maintainer:
Soren Friis
soren.friis@intel.com
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <mav2odid.h>
#include <common/mavlink.h>

#define MAVLINK_SYSTEM_ID       1
#define MAVLINK_COMPONENT_ID    1

static void print_mavlink_basicID(mavlink_open_drone_id_basic_id_t *basic_id)
{
    // Ensure the ID is null-terminated
    char buf[MAVLINK_MSG_OPEN_DRONE_ID_BASIC_ID_FIELD_UAS_ID_LEN + 1] = { 0 };
    memcpy(buf, basic_id->uas_id, MAVLINK_MSG_OPEN_DRONE_ID_BASIC_ID_FIELD_UAS_ID_LEN);

    printf("ID type: %d, UA type: %d, UAS ID: %s\n",
           basic_id->id_type, basic_id->ua_type, buf);
}

static void print_mavlink_location(mavlink_open_drone_id_location_t *location)
{
    printf("Status: %d, Direction: %d cdeg, SpeedHori: %d cm/s, SpeedVert: %d cm/s, \n"\
           "Lat/Lon: %d, %d degE7, Alt: Baro, Geo, Height above %s: %.2f, %.2f, %.2f\n"\
           "Horiz, Vert, Baro, Speed, TS Accuracy: %d, %d, %d, %d, %d, TimeStamp: %.2f\n",
           location->status, location->direction, location->speed_horizontal,
           location->speed_vertical, location->latitude, location->longitude,
           location->height_reference ? "Ground" : "TakeOff",
           (double) location->altitude_barometric,
           (double) location->altitude_geodetic, (double) location->height,
           location->horizontal_accuracy, location->vertical_accuracy,
           location->barometer_accuracy, location->speed_accuracy,
           location->timestamp_accuracy, (double) location->timestamp);
}

static void print_mavlink_auth(mavlink_open_drone_id_authentication_t *auth)
{
    printf("Data page: %d, auth type: %d, ",
           auth->data_page, auth->authentication_type);
    int size = MAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_AUTHENTICATION_DATA_LEN;
    if (auth->data_page == 0)
    {
        size -= ODID_AUTH_PAGE_ZERO_DATA_SIZE;
        printf("page_count: %d, length: %d, timestamp: %d, ",
               auth->page_count, auth->length, auth->timestamp);
    }
    printf("\n");
    for (int i = 0; i < size; i++)
        printf("0x%02X ", auth->authentication_data[i]);
    printf("\n");
}

static void print_mavlink_selfID(mavlink_open_drone_id_self_id_t *self_id)
{
    // Ensure the description is null-terminated
    char buf[MAVLINK_MSG_OPEN_DRONE_ID_SELF_ID_FIELD_DESCRIPTION_LEN + 1] = { 0 };
    memcpy(buf, self_id->description, MAVLINK_MSG_OPEN_DRONE_ID_SELF_ID_FIELD_DESCRIPTION_LEN);

    printf("Type: %d, description: %s\n",
           self_id->description_type, buf);
}

static void print_mavlink_system(mavlink_open_drone_id_system_t *system)
{
    printf("Operator Location Type: %d, Classification Type: %d\n"
           "Lat/Lon: %d, %d degE7, \n"
           "Area Count, Radius: %d, %d, Ceiling, Floor: %.2f, %.2f m\n"
           "Category EU: %d, Class EU: %d\n",
           system->operator_location_type, system->classification_type,
           system->operator_latitude, system->operator_longitude,
           system->area_count, system->area_radius,
           (double) system->area_ceiling, (double) system->area_floor,
           system->category_eu, system->class_eu);
}

static void print_mavlink_operatorID(mavlink_open_drone_id_operator_id_t *operator_id)
{
    // Ensure the ID is null-terminated
    char buf[MAVLINK_MSG_OPEN_DRONE_ID_OPERATOR_ID_FIELD_OPERATOR_ID_LEN + 1] = { 0 };
    memcpy(buf, operator_id->operator_id, MAVLINK_MSG_OPEN_DRONE_ID_OPERATOR_ID_FIELD_OPERATOR_ID_LEN);

    printf("Type: %d, operator ID: %s\n",
           operator_id->operator_id_type, buf);
}

/**
* Simulates "sending" the Mavlink message in buf to the OpenDroneID transmitter,
* parsing the received message and decoding it
*/
static void send_parse_tx_rx(mav2odid_t *m2o, mavlink_message_t *msg,
                             uint8_t *src, ODID_UAS_Data *uas_data,
                             ODID_messagetype_t *msgType)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_to_send_buffer(buf, msg);

    ODID_messagetype_t msg_type = ODID_MESSAGETYPE_INVALID;
    int i = 0;
    while (msg_type == ODID_MESSAGETYPE_INVALID && i < sizeof(buf))
        msg_type = m2o_parseMavlink(m2o, buf[i++]);

    if (msg_type == ODID_MESSAGETYPE_INVALID)
        printf("ERROR: Parsing Mavlink message failed\n");

    // m2o_parseMavlink transfered the data into the src buffer
    // Copy the received OpenDroneID bytestring to tx_buf
    uint8_t tx_buf[ODID_MESSAGE_SIZE] = { 0 };
    memcpy(tx_buf, src, ODID_MESSAGE_SIZE);

    printf("\nEncoded: ");
    printByteArray(tx_buf, ODID_MESSAGE_SIZE, 1);
    printf("\n");

    // The content of tx_buf has been transmitted via Bluetooth or WiFi
    // The receiver decodes the data and returns the message type and data
    *msgType = decodeOpenDroneID(uas_data, tx_buf);
}

/**
* The following test functions all simulate creation and transmission of data
* for each Open Drone ID message type.
*
* The data is first encapsulated in a Mavlink message, which is converted to
* a bytestring simulating the transfer of the data between e.g. a flight
* controller and a Bluetooth/WiFi transmitter.
*
* The Mavlink bytestring is converted to the packed OpenDroneID format for
* Bluetooth/WiFi transmission.
*
* The received OpenDroneID data is then unpacked, printed and copied into
* a mavlink structure for further transmission e.g. from a HW receiver to
* a ground control station.
*/
static void test_basicId(mav2odid_t *m2o, ODID_UAS_Data *uas_data)
{
    mavlink_message_t msg = { 0 };
    mavlink_open_drone_id_basic_id_t basic_id = {
        .ua_type = MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR,
        .id_type = MAV_ODID_ID_TYPE_SERIAL_NUMBER,
        .uas_id = "9876543210ABCDEFGHJK" };

    printf("\n--------------------------Basic ID------------------------\n\n");
    print_mavlink_basicID(&basic_id);

    mavlink_msg_open_drone_id_basic_id_encode(MAVLINK_SYSTEM_ID,
                                              MAVLINK_COMPONENT_ID,
                                              &msg, &basic_id);

    ODID_messagetype_t msgType;
    send_parse_tx_rx(m2o, &msg, (uint8_t *) &m2o->basicIdEnc,
                     uas_data, &msgType);

    if (msgType != ODID_MESSAGETYPE_BASIC_ID)
        printf("ERROR: Open Drone ID message type was not Basic ID\n");

    printBasicID_data(&uas_data->BasicID);

    // The received data is transferred into a Mavlink structure
    mavlink_open_drone_id_basic_id_t basic_id2 = { 0 };
    m2o_basicId2Mavlink(&basic_id2, &uas_data->BasicID);
    printf("\n");
    print_mavlink_basicID(&basic_id2);
}

static void test_location(mav2odid_t *m2o, ODID_UAS_Data *uas_data)
{
    mavlink_message_t msg = { 0 };
    mavlink_open_drone_id_location_t location = {
        .status = MAV_ODID_STATUS_AIRBORNE,
        .direction = (uint16_t) (27.4f * 100),
        .speed_horizontal = (uint16_t) (4.25f * 100),
        .speed_vertical = (int16_t) (4.5f * 100),
        .latitude = (int32_t) (51.477 * 1E7),
        .longitude = (int32_t) (0.0005 * 1E7),
        .altitude_barometric = 37.5f,
        .altitude_geodetic = 36.5f,
        .height_reference = MAV_ODID_HEIGHT_REF_OVER_GROUND,
        .height = 25.5f,
        .horizontal_accuracy = MAV_ODID_HOR_ACC_3_METER,
        .vertical_accuracy = MAV_ODID_VER_ACC_1_METER,
        .barometer_accuracy = MAV_ODID_VER_ACC_3_METER,
        .speed_accuracy = MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND,
        .timestamp_accuracy = MAV_ODID_TIME_ACC_0_1_SECOND,
        .timestamp = 3243.4f };

    printf("\n\n------------------------Location------------------------\n\n");
    print_mavlink_location(&location);

    mavlink_msg_open_drone_id_location_encode(MAVLINK_SYSTEM_ID,
                                              MAVLINK_COMPONENT_ID,
                                              &msg, &location);

    ODID_messagetype_t msgType;
    send_parse_tx_rx(m2o, &msg, (uint8_t *) &m2o->locationEnc,
                     uas_data, &msgType);

    if (msgType != ODID_MESSAGETYPE_LOCATION)
        printf("ERROR: Open Drone ID message type was not Location\n");

    printLocation_data(&uas_data->Location);

    // The received data is transferred into a Mavlink structure
    mavlink_open_drone_id_location_t location2 = { 0 };
    m2o_location2Mavlink(&location2, &uas_data->Location);
    printf("\n");
    print_mavlink_location(&location2);
}

static void test_authentication(mav2odid_t *m2o, ODID_UAS_Data *uas_data)
{
    mavlink_message_t msg = { 0 };
    mavlink_open_drone_id_authentication_t auth = {
        .data_page = 0,
        .authentication_type = MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE,
        .authentication_data = "98765432101234567",
        .page_count = 1,
        .length = 17,
        .timestamp = 23000000 };
    printf("\n\n---------------------Authentication---------------------\n\n");
    print_mavlink_auth(&auth);

    mavlink_msg_open_drone_id_authentication_encode(MAVLINK_SYSTEM_ID,
                                                    MAVLINK_COMPONENT_ID,
                                                    &msg, &auth);

    ODID_messagetype_t msgType;
    send_parse_tx_rx(m2o, &msg, (uint8_t *) &m2o->authEnc,
                     uas_data, &msgType);

    if (msgType != ODID_MESSAGETYPE_AUTH)
        printf("ERROR: Open Drone ID message type was not Authentication\n");

    if (uas_data->AuthValid[0])
        printAuth_data(&uas_data->Auth[0]);

    // The received data is transferred into a Mavlink structure
    mavlink_open_drone_id_authentication_t auth2 = { 0 };
    if (uas_data->AuthValid[0])
        m2o_authentication2Mavlink(&auth2, &uas_data->Auth[0]);
    printf("\n");
    print_mavlink_auth(&auth2);
}

static void test_selfID(mav2odid_t *m2o, ODID_UAS_Data *uas_data)
{
    mavlink_message_t msg = { 0 };
    mavlink_open_drone_id_self_id_t selfID = {
        .description_type = MAV_ODID_DESC_TYPE_TEXT,
        .description = "Description of flight" };

    printf("\n\n------------------------Self ID------------------------\n\n");
    print_mavlink_selfID(&selfID);

    mavlink_msg_open_drone_id_self_id_encode(MAVLINK_SYSTEM_ID,
                                             MAVLINK_COMPONENT_ID,
                                             &msg, &selfID);

    ODID_messagetype_t msgType;
    send_parse_tx_rx(m2o, &msg, (uint8_t *) &m2o->selfIdEnc,
                     uas_data, &msgType);

    if (msgType != ODID_MESSAGETYPE_SELF_ID)
        printf("ERROR: Open Drone ID message type was not Self ID\n");

    printSelfID_data(&uas_data->SelfID);

    // The received data is transferred into a Mavlink structure
    mavlink_open_drone_id_self_id_t selfID2 = { 0 };
    m2o_selfId2Mavlink(&selfID2, &uas_data->SelfID);
    printf("\n");
    print_mavlink_selfID(&selfID2);
}

static void test_system(mav2odid_t *m2o, ODID_UAS_Data *uas_data)
{
    mavlink_message_t msg = { 0 };
    mavlink_open_drone_id_system_t system = {
        .operator_location_type = MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF,
        .classification_type = MAV_ODID_CLASSIFICATION_TYPE_EU,
        .operator_latitude = (int32_t) (51.477 * 1E7),
        .operator_longitude = (int32_t) (0.0005 * 1E7),
        .area_count = 350,
        .area_radius = 55,
        .area_ceiling = 75.5f,
        .area_floor = 26.5f,
        .category_eu = MAV_ODID_CATEGORY_EU_CERTIFIED,
        .class_eu = MAV_ODID_CLASS_EU_CLASS_5 };

    printf("\n\n------------------------System------------------------\n\n");
    print_mavlink_system(&system);

    mavlink_msg_open_drone_id_system_encode(MAVLINK_SYSTEM_ID,
                                            MAVLINK_COMPONENT_ID,
                                            &msg, &system);

    ODID_messagetype_t msgType;
    send_parse_tx_rx(m2o, &msg, (uint8_t *) &m2o->systemEnc,
                     uas_data, &msgType);

    if (msgType != ODID_MESSAGETYPE_SYSTEM)
        printf("ERROR: Open Drone ID message type was not System\n");

    printSystem_data(&uas_data->System);

    // The received data is transferred into a Mavlink structure
    mavlink_open_drone_id_system_t system2 = { 0 };
    m2o_system2Mavlink(&system2, &uas_data->System);
    printf("\n");
    print_mavlink_system(&system2);
}

static void test_operatorID(mav2odid_t *m2o, ODID_UAS_Data *uas_data)
{
    mavlink_message_t msg = { 0 };
    mavlink_open_drone_id_operator_id_t operatorID = {
        .operator_id_type = MAV_ODID_OPERATOR_ID_TYPE_CAA,
        .operator_id = "ABCDEFGHJK0123456789" };

    printf("\n\n----------------------Operator ID-----------------------\n\n");
    print_mavlink_operatorID(&operatorID);

    mavlink_msg_open_drone_id_operator_id_encode(MAVLINK_SYSTEM_ID,
                                                 MAVLINK_COMPONENT_ID,
                                                 &msg, &operatorID);

    ODID_messagetype_t msgType;
    send_parse_tx_rx(m2o, &msg, (uint8_t *) &m2o->operatorIdEnc,
                     uas_data, &msgType);

    if (msgType != ODID_MESSAGETYPE_OPERATOR_ID)
        printf("ERROR: Open Drone ID message type was not Operator ID\n");

    printOperatorID_data(&uas_data->OperatorID);

    // The received data is transferred into a Mavlink structure
    mavlink_open_drone_id_operator_id_t operatorID2 = { 0 };
    m2o_operatorId2Mavlink(&operatorID2, &uas_data->OperatorID);
    printf("\n");
    print_mavlink_operatorID(&operatorID2);
}

void test_mav2odid()
{
    mav2odid_t m2o;
    if (m2o_init(&m2o))
        printf("ERROR: Initialising mav2odid data failed\n");

    ODID_UAS_Data uas_data;
    memset(&uas_data, 0, sizeof(ODID_UAS_Data));

    test_basicId(&m2o, &uas_data);
    test_location(&m2o, &uas_data);
    test_authentication(&m2o, &uas_data);
    test_selfID(&m2o, &uas_data);
    test_system(&m2o, &uas_data);
    test_operatorID(&m2o, &uas_data);

    printf("\n-------------------------------------------------------------------------------\n");
    printf("-------------------------------------  End  -----------------------------------\n");
    printf("-------------------------------------------------------------------------------\n\n");
}

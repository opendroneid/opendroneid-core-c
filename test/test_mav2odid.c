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
    printf("ID type: %d, UA type: %d, UAS ID: %s\n",
           basic_id->id_type, basic_id->ua_type, basic_id->uas_id);
}

static void print_mavlink_location(mavlink_open_drone_id_location_t *location)
{
    printf("Status: %d, Direction: %d cdeg, SpeedHori: %d cm/s, SpeedVert: %d cm/s, \n"\
           "Lat/Lon: %d, %d degE7, Alt: Baro, Geo, Height above %s: %.2f, %.2f, %.2f\n"\
           "Horiz, Vert, Baro, Speed, TS Accuracy: %d, %d, %d, %d, %d, TimeStamp: %.2f\n",
           location->status, location->direction, location->speed_horizontal,
           location->speed_vertical, location->latitude, location->longitude,
           location->height_reference ? "Ground" : "TakeOff",
           location->altitude_barometric, location->altitude_geodetic,
           location->height,
           location->horizontal_accuracy, location->vertical_accuracy,
           location->barometer_accuracy, location->speed_accuracy,
           location->timestamp_accuracy, location->timestamp);
}

static void print_mavlink_auth(mavlink_open_drone_id_authentication_t *auth)
{
    printf("Data page: %d, auth type: %d, data: ",
           auth->data_page, auth->authentication_type);
    for (int i = 0; i < MAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_AUTHENTICATION_DATA_LEN; i++)
        printf("0x%02X ", auth->authentication_data[i]);
    printf("\n");
}

static void print_mavlink_selfID(mavlink_open_drone_id_selfid_t *self_id)
{
    printf("Type: %d, description: %s\n",
           self_id->description_type, self_id->description);
}

static void print_mavlink_system(mavlink_open_drone_id_system_t *system)
{
    printf("location Source: %d\nLat/Lon: %d, %d degE7, Group Count, Radius: %d, %d, \n"\
           "Ceiling, Floor: %.2f, %.2f m\n",
           system->flags, system->remote_pilot_latitude,
           system->remote_pilot_longitude, system->group_count,
           system->group_radius, system->group_ceiling, system->group_floor);
}

/**
* Simulates "sending" the Mavlink message in buf to the OpenDroneID transmitter,
* parsing the received message and decoding it
*/
static void send_parse_tx_rx(mav2odid_t *m2o, mavlink_message_t *msg,
                             uint8_t *src, odid_data_uas_t *uas_data,
                             odid_message_type_t *msgType)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t length = mavlink_msg_to_send_buffer(buf, msg);

    odid_message_type_t msg_type = ODID_MESSAGE_TYPE_INVALID;
    int i = 0;
    while (msg_type == ODID_MESSAGE_TYPE_INVALID && i < sizeof(buf))
        msg_type = m2o_parseMavlink(m2o, buf[i++]);

    if (msg_type == ODID_MESSAGE_TYPE_INVALID)
        printf("ERROR: Parsing Mavlink message failed\n");

    // m2o_parseMavlink transfered the data into the src buffer
    // Copy the received OpenDroneID bytestring to tx_buf
    uint8_t tx_buf[ODID_MESSAGE_SIZE] = { 0 };
    memcpy(tx_buf, src, ODID_MESSAGE_SIZE);

    printf("\nEncoded: ");
    odid_print_byte_array(tx_buf, ODID_MESSAGE_SIZE, 1);
    printf("\n");

    // The content of tx_buf has been transmitted via Bluetooth or WiFi
    // The receiver decodes the data and returns the message type and data
    *msgType = odid_decode_open_drone_id(uas_data, tx_buf);
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
static void test_basicId(mav2odid_t *m2o, odid_data_uas_t *uas_data)
{
    mavlink_message_t msg = { 0 };
    mavlink_open_drone_id_basic_id_t basic_id = {
        .ua_type = MAV_ODID_UATYPE_ROTORCRAFT,
        .id_type = MAV_ODID_IDTYPE_SERIAL_NUMBER,
        .uas_id = "987654321ABCDEFGHJK" };

    printf("\n--------------------------Basic ID------------------------\n\n");
    print_mavlink_basicID(&basic_id);

    mavlink_msg_open_drone_id_basic_id_encode(MAVLINK_SYSTEM_ID,
                                              MAVLINK_COMPONENT_ID,
                                              &msg, &basic_id);

    odid_message_type_t msgType;
    send_parse_tx_rx(m2o, &msg, (uint8_t *) &m2o->basicIdEnc,
                     uas_data, &msgType);

    if (msgType != ODID_MESSAGE_TYPE_BASIC_ID)
        printf("ERROR: Open Drone ID message type was not Basic ID\n");

    odid_print_data_basic_id(&uas_data->basic_id);

    // The received data is transferred into a Mavlink structure
    mavlink_open_drone_id_basic_id_t basic_id2 = { 0 };
    m2o_basicId2Mavlink(&basic_id2, &uas_data->basic_id);
    printf("\n");
    print_mavlink_basicID(&basic_id2);
}

static void test_location(mav2odid_t *m2o, odid_data_uas_t *uas_data)
{
    mavlink_message_t msg = { 0 };
    mavlink_open_drone_id_location_t location = {
        .status = MAV_ODID_STATUS_AIRBORNE,
        .direction = (uint16_t) (27.4f * 100),
        .speed_horizontal = (uint16_t) (4.25f * 100),
        .speed_vertical = (uint16_t) (4.5f * 100),
        .latitude = (int32_t) (51.477f * 1E7),
        .longitude = (int32_t) (0.0005 * 1E7),
        .altitude_barometric = 37.5f,
        .altitude_geodetic = 36.5f,
        .height_reference = MAV_ODID_HEIGHT_REF_OVER_GROUND,
        .height = 25.5f,
        .horizontal_accuracy = MAV_ODID_HOR_ACC_3_METER,
        .vertical_accuracy = MAV_ODID_VER_ACC_1_METER,
        .barometer_accuracy = MAV_ODID_VER_ACC_3_METER,
        .speed_accuracy = MAV_ODID_SPEED_ACC_1_METER_PER_SECOND,
        .timestamp_accuracy = MAV_ODID_TIME_ACC_0_1_SECOND,
        .timestamp = 3243.4f };

    printf("\n\n------------------------location------------------------\n\n");
    print_mavlink_location(&location);

    mavlink_msg_open_drone_id_location_encode(MAVLINK_SYSTEM_ID,
                                              MAVLINK_COMPONENT_ID,
                                              &msg, &location);

    odid_message_type_t msgType;
    send_parse_tx_rx(m2o, &msg, (uint8_t *) &m2o->locationEnc,
                     uas_data, &msgType);

    if (msgType != ODID_MESSAGE_TYPE_LOCATION)
        printf("ERROR: Open Drone ID message type was not location\n");

    odid_print_data_location(&uas_data->location);

    // The received data is transferred into a Mavlink structure
    mavlink_open_drone_id_location_t location2 = { 0 };
    m2o_location2Mavlink(&location2, &uas_data->location);
    printf("\n");
    print_mavlink_location(&location2);
}

static void test_authentication(mav2odid_t *m2o, odid_data_uas_t *uas_data)
{
    mavlink_message_t msg = { 0 };
    mavlink_open_drone_id_authentication_t auth = {
        .data_page = 0,
        .authentication_type = MAV_ODID_AUTH_MPUID,
        .authentication_data = "987654321" };
    printf("\n\n---------------------Authentication---------------------\n\n");
    print_mavlink_auth(&auth);

    mavlink_msg_open_drone_id_authentication_encode(MAVLINK_SYSTEM_ID,
                                                    MAVLINK_COMPONENT_ID,
                                                    &msg, &auth);

    odid_message_type_t msgType;
    send_parse_tx_rx(m2o, &msg, (uint8_t *) &m2o->authenticationEnc,
                     uas_data, &msgType);

    if (msgType != ODID_MESSAGE_TYPE_AUTH)
        printf("ERROR: Open Drone ID message type was not Authentication\n");

    odid_print_data_auth(&uas_data->auth);

    // The received data is transferred into a Mavlink structure
    mavlink_open_drone_id_authentication_t auth2 = { 0 };
    m2o_authentication2Mavlink(&auth2, &uas_data->auth);
    printf("\n");
    print_mavlink_auth(&auth2);
}

static void test_selfID(mav2odid_t *m2o, odid_data_uas_t *uas_data)
{
    mavlink_message_t msg = { 0 };
    mavlink_open_drone_id_selfid_t selfID = {
        .description_type = MAV_ODID_DESC_TYPE_TEXT,
        .description = "Description of flight" };

    printf("\n\n------------------------Self ID------------------------\n\n");
    print_mavlink_selfID(&selfID);

    mavlink_msg_open_drone_id_selfid_encode(MAVLINK_SYSTEM_ID,
                                            MAVLINK_COMPONENT_ID,
                                            &msg, &selfID);

    odid_message_type_t msgType;
    send_parse_tx_rx(m2o, &msg, (uint8_t *) &m2o->selfIdEnc,
                     uas_data, &msgType);

    if (msgType != ODID_MESSAGE_TYPE_SELF_ID)
        printf("ERROR: Open Drone ID message type was not Self ID\n");

    odid_print_data_self_id(&uas_data->self_id);

    // The received data is transferred into a Mavlink structure
    mavlink_open_drone_id_selfid_t selfID2 = { 0 };
    m2o_selfId2Mavlink(&selfID2, &uas_data->self_id);
    printf("\n");
    print_mavlink_selfID(&selfID2);
}

static void test_system(mav2odid_t *m2o, odid_data_uas_t *uas_data)
{
    mavlink_message_t msg = { 0 };
    mavlink_open_drone_id_system_t system = {
        .flags = MAV_ODID_LOCATION_SRC_TAKEOFF,
        .remote_pilot_latitude = (int32_t) (51.477f * 1E7),
        .remote_pilot_longitude = (int32_t) (0.0005f * 1E7),
        .group_count = 350,
        .group_radius = 55,
        .group_ceiling = 75.5f,
        .group_floor = 26.5f };

    printf("\n\n------------------------system------------------------\n\n");
    print_mavlink_system(&system);

    mavlink_msg_open_drone_id_system_encode(MAVLINK_SYSTEM_ID,
                                            MAVLINK_COMPONENT_ID,
                                            &msg, &system);

    odid_message_type_t msgType;
    send_parse_tx_rx(m2o, &msg, (uint8_t *) &m2o->systemEnc,
                     uas_data, &msgType);

    if (msgType != ODID_MESSAGE_TYPE_SYSTEM)
        printf("ERROR: Open Drone ID message type was not system\n");

    odid_print_data_system(&uas_data->system);

    // The received data is transferred into a Mavlink structure
    mavlink_open_drone_id_system_t system2 = { 0 };
    m2o_system2Mavlink(&system2, &uas_data->system);
    printf("\n");
    print_mavlink_system(&system2);
}

void test_mav2odid()
{
    mav2odid_t m2o;
    if (m2o_init(&m2o))
        printf("ERROR: Initialising mav2odid data failed\n");

    odid_data_uas_t uas_data;
    memset(&uas_data, 0, sizeof(odid_data_uas_t));

    test_basicId(&m2o, &uas_data);
    test_location(&m2o, &uas_data);
    test_authentication(&m2o, &uas_data);
    test_selfID(&m2o, &uas_data);
    test_system(&m2o, &uas_data);

    printf("\n-------------------------------------------------------------------------------\n");
    printf("-------------------------------------  End  -----------------------------------\n");
    printf("-------------------------------------------------------------------------------\n\n");
}

/*
Copyright (C) 2019 Intel Corporation

SPDX-License-Identifier: Apache-2.0

Open Drone ID C Library

Maintainer:
Gabriel Cox
gabriel.c.cox@intel.com
*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <opendroneid.h>

odid_encoded_basic_id_t BasicID_enc;
odid_data_basic_id_t BasicID;
odid_data_basic_id_t BasicID_out;

odid_encoded_location_t Location_enc;
odid_data_location_t Location;
odid_data_location_t Location_out;

odid_encoded_auth_t Auth_enc;
odid_data_auth_t Auth;
odid_data_auth_t Auth_out;

odid_encoded_self_id_t SelfID_enc;
odid_data_self_id_t SelfID;
odid_data_self_id_t SelfID_out;

odid_encoded_system_t System_enc;
odid_data_system_t System_data;
odid_data_system_t System_out;

void test_InOut()
{
    printf("\n-------------------------------------Source Data-----------------------------------\n");
    BasicID.id_type = ODID_ID_TYPE_CAA_ASSIGNED_ID;
    BasicID.ua_type = ODID_UA_TYPE_ROTORCRAFT;
    odid_safe_copy_fill(BasicID.uas_id, "123456789012345678901", sizeof(BasicID.uas_id));
    printf("basic_id\n-------\n");
    odid_print_data_basic_id(&BasicID);
    odid_encode_message_basic_id(&BasicID_enc, &BasicID);

    Location.Status = ODID_STATUS_AIRBORNE;
    Location.Direction = 215.7;
    Location.SpeedHorizontal = 5.4;
    Location.SpeedVertical = 5.25;
    Location.Latitude = 45.539309;
    Location.Longitude = -122.966389;
    Location.AltitudeBaro = 100;
    Location.AltitudeGeo = 110;
    Location.HeightType = ODID_HEIGHT_REF_OVER_GROUND;
    Location.Height = 80;
    Location.HorizAccuracy = odid_create_enum_horizontal_accuracy(2.5f);
    Location.VertAccuracy = odid_create_enum_vertical_accuracy(0.5f);
    Location.BaroAccuracy = odid_create_enum_vertical_accuracy(1.5f);
    Location.SpeedAccuracy = odid_create_enum_speed_accuracy(0.5f);
    Location.TSAccuracy = odid_create_enum_timestamp_accuracy(0.2f);
    Location.TimeStamp = 3600.52;
    printf("\nlocation\n--------\n");
    odid_print_data_location(&Location);
    odid_encode_message_location(&Location_enc, &Location);

    Auth.AuthType = ODID_AUTH_TYPE_MPUID;
    Auth.DataPage = 0;
    odid_safe_copy_fill(Auth.AuthData, "1234567890123456789012", ODID_STR_SIZE);
    printf("\nauth\n--------------\n");
    odid_print_data_auth(&Auth);
    odid_encode_message_auth(&Auth_enc, &Auth);

    SelfID.DescType = ODID_DESC_TYPE_TEXT;
    odid_safe_copy_fill(SelfID.Desc, "DronesRUS: Real Estate", sizeof(SelfID.Desc));
    printf("\nself_id\n------\n");
    odid_print_data_self_id(&SelfID);
    odid_encode_message_self_id(&SelfID_enc, &SelfID);

    System_data.LocationSource = ODID_LOCATION_SRC_TAKEOFF;
    System_data.remotePilotLatitude = Location.Latitude + 0.00001;
    System_data.remotePilotLongitude = Location.Longitude + 0.00001;
    System_data.GroupCount = 35;
    System_data.GroupRadius = 75;
    System_data.GroupCeiling = 176.9;
    System_data.GroupFloor = 41.7;
    printf("\nsystem\n------\n");
    odid_print_data_system(&System_data);
    odid_encode_message_system(&System_enc, &System_data);
    printf("\n-------------------------------------Encoded Data-----------------------------------\n");
    printf("          0- 1- 2- 3- 4- 5- 6- 7- 8- 9- 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24\n");
    printf("basic_id:  ");
    odid_print_byte_array((uint8_t * ) & BasicID_enc, ODID_MESSAGE_SIZE, 1);

    printf("location: ");
    odid_print_byte_array((uint8_t * ) & Location_enc, ODID_MESSAGE_SIZE, 1);

    printf("auth:     ");
    odid_print_byte_array((uint8_t * ) & Auth_enc, ODID_MESSAGE_SIZE, 1);

    printf("self_id:   ");
    odid_print_byte_array((uint8_t * ) & SelfID_enc, ODID_MESSAGE_SIZE, 1);

    printf("system:   ");
    odid_print_byte_array((uint8_t * ) & System_enc, ODID_MESSAGE_SIZE, 1);

    printf("\n-------------------------------------Decoded Data-----------------------------------\n");
    // Now for the reverse -- decode test
    odid_decode_message_basic_id(&BasicID_out, &BasicID_enc);
    printf("basic_id\n-------\n");
    odid_print_data_basic_id(&BasicID_out);

    odid_decode_message_location(&Location_out, &Location_enc);
    printf("\nlocation\n--------\n");
    odid_print_data_location(&Location_out);

    odid_decode_message_auth(&Auth_out, &Auth_enc);
    printf("\nauth\n-------\n");
    odid_print_data_auth(&Auth_out);

    odid_decode_message_self_id(&SelfID_out, &SelfID_enc);
    printf("\nself_id\n------\n");
    odid_print_data_self_id(&SelfID_out);

    odid_decode_message_system(&System_out, &System_enc);
    printf("\nsystem\n------\n");
    odid_print_data_system(&System_out);
    printf("\n-------------------------------------------------------------------------------\n");
    printf("-------------------------------------  End  -----------------------------------\n");
    printf("-------------------------------------------------------------------------------\n\n");

}

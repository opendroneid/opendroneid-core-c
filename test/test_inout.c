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

ODID_BasicID_encoded BasicID_enc;
ODID_BasicID_data BasicID;
ODID_BasicID_data BasicID_out;

ODID_Location_encoded Location_enc;
ODID_Location_data Location;
ODID_Location_data Location_out;

ODID_Auth_encoded Auth_enc;
ODID_Auth_data Auth;
ODID_Auth_data Auth_out;

ODID_SelfID_encoded SelfID_enc;
ODID_SelfID_data SelfID;
ODID_SelfID_data SelfID_out;

ODID_System_encoded System_enc;
ODID_System_data System_data;
ODID_System_data System_out;

void test_InOut()
{
    printf("\n-------------------------------------Source Data-----------------------------------\n");
    BasicID.IDType = ODID_IDTYPE_CAA_ASSIGNED_ID;
    BasicID.UASType = ODID_UAVTYPE_ROTORCRAFT_MULTIROTOR;
    safe_copyfill(BasicID.UASID,"123456789012345678901", sizeof(BasicID.UASID));
    printf("BasicID\n-------\n");
    printBasicID_data(BasicID);
    encodeBasicIDMessage(&BasicID_enc, &BasicID);

    Location.Status = ODID_STATUS_AIRBORNE;
    Location.SpeedNS = 5.25;
    Location.SpeedEW = 3.5;
    Location.SpeedVertical = 5.25;
    Location.Latitude = 45.539309;
    Location.Longitude = -122.966389;
    Location.AltitudeBaro = 100;
    Location.AltitudeGeo = 110;
    Location.HeightAboveTakeoff = 80;
    Location.HorizAccuracy = 2.5f;
    Location.VertAccuracy = 0.5f;
    Location.SpeedAccuracy = 0.5f;
    Location.TSAccuracy = 0.2f;
    Location.TimeStamp = 3600.52;
    printf("\nLocation\n--------\n");
    printLocation_data(Location);
    encodeLocationMessage(&Location_enc, &Location);

    Auth.AuthType = 1;
    Auth.DataPage = 0;
    safe_copyfill(Auth.AuthData, "1234567890123456789012", ODID_STR_SIZE);
    printf("\nAuth\n--------------\n");
    printAuth_data(Auth);
    encodeAuthMessage(&Auth_enc, &Auth);

    SelfID.DescType = 0;
    safe_copyfill(SelfID.Desc,"DronesRUS: Real Estate",sizeof(SelfID.Desc));
    printf("\nSelfID\n------\n");
    printSelfID_data(SelfID);
    encodeSelfIDMessage(&SelfID_enc, &SelfID);

    System_data.LocationSource = 0;
    System_data.Latitude = Location.Latitude + 0.00001;
    System_data.Longitude = Location.Longitude + 0.00001;
    System_data.GroupCount = 0;
    System_data.GroupRadius = 0;
    System_data.GroupCeiling = 0;
    printf("\nSystem\n------\n");
    printSystem_data(System_data);
    encodeSystemMessage(&System_enc, &System_data);
    printf("\n-------------------------------------Encoded Data-----------------------------------\n");
    printf("          0- 1- 2- 3- 4- 5- 6- 7- 8- 9- 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24\n");
    printf("BasicID:  ");
    printByteArray((uint8_t*) &BasicID_enc, ODID_MESSAGE_SIZE, 1);

    printf("Location: ");
    printByteArray((uint8_t*) &Location_enc, ODID_MESSAGE_SIZE, 1);

    printf("Auth:     ");
    printByteArray((uint8_t*) &Auth_enc, ODID_MESSAGE_SIZE, 1);

    printf("SelfID:   ");
    printByteArray((uint8_t*) &SelfID_enc, ODID_MESSAGE_SIZE, 1);

    printf("System:   ");
    printByteArray((uint8_t*) &System_enc, ODID_MESSAGE_SIZE, 1);

    printf("\n-------------------------------------Decoded Data-----------------------------------\n");
    // Now for the reverse -- decode test
    decodeBasicIDMessage(&BasicID_out, &BasicID_enc);
    printf("BasicID\n-------\n");
    printBasicID_data(BasicID_out);

    decodeLocationMessage(&Location_out, &Location_enc);
    printf("\nLocation\n--------\n");
    printLocation_data(Location_out);

    decodeAuthMessage(&Auth_out, &Auth_enc);
    printf("\nAuth\n-------\n");
    printAuth_data(Auth_out);

    decodeSelfIDMessage(&SelfID_out, &SelfID_enc);
    printf("\nSelfID\n------\n");
    printSelfID_data(SelfID_out);

    decodeSystemMessage(&System_out, &System_enc);
    printf("\nSystem\n------\n");
    printSystem_data(System_out);
    printf("\n-------------------------------------  End  -----------------------------------\n");

}

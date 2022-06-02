/*
Copyright (C) 2019 Intel Corporation

SPDX-License-Identifier: Apache-2.0

Open Drone ID C Library

Maintainer:
Gabriel Cox
gabriel.c.cox@intel.com
*/
#include <string.h>
#include <stdio.h>
#include <opendroneid.h>

ODID_BasicID_encoded BasicID_enc;
ODID_BasicID_data BasicID;
ODID_BasicID_data BasicID_out;

ODID_Location_encoded Location_enc;
ODID_Location_data Location;
ODID_Location_data Location_out;

ODID_Auth_encoded Auth0_enc;
ODID_Auth_encoded Auth1_enc;
ODID_Auth_data Auth0;
ODID_Auth_data Auth1;
ODID_Auth_data Auth0_out;
ODID_Auth_data Auth1_out;

ODID_SelfID_encoded SelfID_enc;
ODID_SelfID_data SelfID;
ODID_SelfID_data SelfID_out;

ODID_System_encoded System_enc;
ODID_System_data System_data;
ODID_System_data System_out;

ODID_OperatorID_encoded OperatorID_enc;
ODID_OperatorID_data operatorID;
ODID_OperatorID_data operatorID_out;

ODID_MessagePack_encoded pack_enc;
ODID_MessagePack_data pack;
ODID_UAS_Data uasData;

#define MINIMUM(a,b) (((a)<(b))?(a):(b))

void test_InOut()
{
    if (ODID_AUTH_MAX_PAGES < 2) {
        fprintf(stderr, "Program compiled with ODID_AUTH_MAX_PAGES < 2\n");
        return;
    }

    printf("\n-------------------------------------Source Data-----------------------------------\n");
    odid_initBasicIDData(&BasicID);
    BasicID.IDType = ODID_IDTYPE_CAA_REGISTRATION_ID;
    BasicID.UAType = ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;
    char id[] = "12345678901234567890";
    strncpy(BasicID.UASID, id, sizeof(BasicID.UASID));
    printf("BasicID\n-------\n");
    printBasicID_data(&BasicID);
    encodeBasicIDMessage(&BasicID_enc, &BasicID);

    odid_initLocationData(&Location);
    Location.Status = ODID_STATUS_AIRBORNE;
    Location.Direction = 215.7f;
    Location.SpeedHorizontal = 5.4f;
    Location.SpeedVertical = 5.25f;
    Location.Latitude = 45.539309;
    Location.Longitude = -122.966389;
    Location.AltitudeBaro = 100;
    Location.AltitudeGeo = 110;
    Location.HeightType = ODID_HEIGHT_REF_OVER_GROUND;
    Location.Height = 80;
    Location.HorizAccuracy = createEnumHorizontalAccuracy(2.5f);
    Location.VertAccuracy = createEnumVerticalAccuracy(0.5f);
    Location.BaroAccuracy = createEnumVerticalAccuracy(1.5f);
    Location.SpeedAccuracy = createEnumSpeedAccuracy(0.5f);
    Location.TSAccuracy = createEnumTimestampAccuracy(0.2f);
    Location.TimeStamp = 360.52f;
    printf("\nLocation\n--------\n");
    printLocation_data(&Location);
    encodeLocationMessage(&Location_enc, &Location);

    odid_initAuthData(&Auth0);
    Auth0.AuthType = ODID_AUTH_UAS_ID_SIGNATURE;
    Auth0.DataPage = 0;
    Auth0.LastPageIndex = 1;
    Auth0.Length = 40;
    Auth0.Timestamp = 28000000;
    char auth0_data[] = "12345678901234567";
    memcpy(Auth0.AuthData, auth0_data, MINIMUM(sizeof(auth0_data), sizeof(Auth0.AuthData)));
    printf("\nAuth0\n--------------\n");
    printAuth_data(&Auth0);
    encodeAuthMessage(&Auth0_enc, &Auth0);

    odid_initAuthData(&Auth1);
    Auth1.AuthType = ODID_AUTH_UAS_ID_SIGNATURE;
    Auth1.DataPage = 1;
    char auth1_data[] = "12345678901234567890123";
    memcpy(Auth1.AuthData, auth1_data, MINIMUM(sizeof(auth1_data), sizeof(Auth1.AuthData)));
    printf("\nAuth1\n--------------\n");
    printAuth_data(&Auth1);
    encodeAuthMessage(&Auth1_enc, &Auth1);

    odid_initSelfIDData(&SelfID);
    SelfID.DescType = ODID_DESC_TYPE_TEXT;
    char description[] = "DronesRUS: Real Estate";
    strncpy(SelfID.Desc, description, sizeof(SelfID.Desc));
    printf("\nSelfID\n------\n");
    printSelfID_data(&SelfID);
    encodeSelfIDMessage(&SelfID_enc, &SelfID);

    odid_initSystemData(&System_data);
    System_data.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
    System_data.ClassificationType = ODID_CLASSIFICATION_TYPE_EU;
    System_data.OperatorLatitude = Location.Latitude + 0.00001;
    System_data.OperatorLongitude = Location.Longitude + 0.00001;
    System_data.AreaCount = 35;
    System_data.AreaRadius = 75;
    System_data.AreaCeiling = 176.9f;
    System_data.AreaFloor = 41.7f;
    System_data.CategoryEU = ODID_CATEGORY_EU_SPECIFIC;
    System_data.ClassEU = ODID_CLASS_EU_CLASS_3;
    System_data.OperatorAltitudeGeo = 20.5f;
    System_data.Timestamp = 28000000;
    printf("\nSystem\n------\n");
    printSystem_data(&System_data);
    encodeSystemMessage(&System_enc, &System_data);

    odid_initOperatorIDData(&operatorID);
    operatorID.OperatorIdType = ODID_OPERATOR_ID;
    char operatorId[] = "98765432100123456789";
    strncpy(operatorID.OperatorId, operatorId, sizeof(operatorID.OperatorId));
    printf("\nOperatorID\n------\n");
    printOperatorID_data(&operatorID);
    encodeOperatorIDMessage(&OperatorID_enc, &operatorID);

    odid_initMessagePackData(&pack);
    pack.MsgPackSize = 7;
    memcpy(&pack.Messages[0], &BasicID_enc, ODID_MESSAGE_SIZE);
    memcpy(&pack.Messages[1], &Location_enc, ODID_MESSAGE_SIZE);
    memcpy(&pack.Messages[2], &Auth0_enc, ODID_MESSAGE_SIZE);
    memcpy(&pack.Messages[3], &Auth1_enc, ODID_MESSAGE_SIZE);
    memcpy(&pack.Messages[4], &SelfID_enc, ODID_MESSAGE_SIZE);
    memcpy(&pack.Messages[5], &System_enc, ODID_MESSAGE_SIZE);
    memcpy(&pack.Messages[6], &OperatorID_enc, ODID_MESSAGE_SIZE);
    encodeMessagePack(&pack_enc, &pack);

    printf("\n-------------------------------------Encoded Data-----------------------------------\n");
    printf("            0- 1- 2- 3- 4- 5- 6- 7- 8- 9- 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24\n");
    printf("BasicID:    ");
    printByteArray((uint8_t*) &BasicID_enc, ODID_MESSAGE_SIZE, 1);

    printf("Location:   ");
    printByteArray((uint8_t*) &Location_enc, ODID_MESSAGE_SIZE, 1);

    printf("Auth0:      ");
    printByteArray((uint8_t*) &Auth0_enc, ODID_MESSAGE_SIZE, 1);

    printf("Auth1:      ");
    printByteArray((uint8_t*) &Auth1_enc, ODID_MESSAGE_SIZE, 1);

    printf("SelfID:     ");
    printByteArray((uint8_t*) &SelfID_enc, ODID_MESSAGE_SIZE, 1);

    printf("System:     ");
    printByteArray((uint8_t*) &System_enc, ODID_MESSAGE_SIZE, 1);

    printf("OperatorID: ");
    printByteArray((uint8_t*) &OperatorID_enc, ODID_MESSAGE_SIZE, 1);

    printf("\n-------------------------------------Decoded Data-----------------------------------\n");
    // Now for the reverse -- decode test
    decodeBasicIDMessage(&BasicID_out, &BasicID_enc);
    printf("BasicID\n-------\n");
    printBasicID_data(&BasicID_out);

    decodeLocationMessage(&Location_out, &Location_enc);
    printf("\nLocation\n--------\n");
    printLocation_data(&Location_out);

    decodeAuthMessage(&Auth0_out, &Auth0_enc);
    printf("\nAuth0\n-------\n");
    printAuth_data(&Auth0_out);

    decodeAuthMessage(&Auth1_out, &Auth1_enc);
    printf("\nAuth1\n-------\n");
    printAuth_data(&Auth1_out);

    decodeSelfIDMessage(&SelfID_out, &SelfID_enc);
    printf("\nSelfID\n------\n");
    printSelfID_data(&SelfID_out);

    decodeSystemMessage(&System_out, &System_enc);
    printf("\nSystem\n------\n");
    printSystem_data(&System_out);

    decodeOperatorIDMessage(&operatorID_out, &OperatorID_enc);
    printf("\nOperatorID\n------\n");
    printOperatorID_data(&operatorID_out);

    decodeMessagePack(&uasData, &pack_enc);
    printf("\nPack\n------\n");
    if (uasData.BasicIDValid[0])
        printBasicID_data(&uasData.BasicID[0]);
    if (uasData.LocationValid)
        printLocation_data(&uasData.Location);
    if (uasData.AuthValid[0])
        printAuth_data(&uasData.Auth[0]);
    if (uasData.AuthValid[1])
        printAuth_data(&uasData.Auth[1]);
    if (uasData.SelfIDValid)
        printSelfID_data(&uasData.SelfID);
    if (uasData.SystemValid)
        printSystem_data(&uasData.System);
    if (uasData.OperatorIDValid)
        printOperatorID_data(&uasData.OperatorID);

    printf("\n-------------------------------------------------------------------------------\n");
    printf("-------------------------------------  End  -----------------------------------\n");
    printf("-------------------------------------------------------------------------------\n\n");

}

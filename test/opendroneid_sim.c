/*
Copyright (C) 2019 Intel Corporation

SPDX-License-Identifier: Apache-2.0

Open Drone ID C Library

Maintainer:
Gabriel Cox
gabriel.c.cox@intel.com
*/

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <opendroneid.h>
#define SLEEPMS 1000000 //ns

ODID_BasicID_encoded basicID_enc;
ODID_Location_encoded location_enc;
ODID_Auth_encoded auth_enc;
ODID_SelfID_encoded selfID_enc;
ODID_System_encoded system_enc;
ODID_OperatorID_encoded operatorID_enc;

ODID_BasicID_data basicID_data;
ODID_Location_data location_data;
ODID_Auth_data auth_data;
ODID_SelfID_data selfID_data;
ODID_System_data system_data;
ODID_OperatorID_data operatorID_data;

const int SIM_STEPS = 20;
const float SIM_STEP_SIZE = 0.0001;
const float DISTANCE_PER_LAT = 111699.0;


double simLat = 45.5393092;
double simLon = -122.9663894;
double simGndLat = 45.5393082;
double simGndLon = -122.9663884;
float simDirection = 0;
float simSpeedHorizontal;

enum compdirs {E,S,W,N};
enum compdirs direction = E;
int stepCount = 0;

void updateLocation(void)
{
    simSpeedHorizontal = DISTANCE_PER_LAT * SIM_STEP_SIZE;
    stepCount++;
    switch(direction) {
        case E:
            simLon+= SIM_STEP_SIZE;
            simDirection = 90;
            if (stepCount >= SIM_STEPS) {
                direction = S;
                stepCount = 0;
            }
            break;
        case S:
            simLat-= SIM_STEP_SIZE;
            simDirection = 180;
            if (stepCount >= SIM_STEPS) {
                direction = W;
                stepCount = 0;
            }
            break;
        case W:
            simLon-= SIM_STEP_SIZE;
            simDirection = 270;
            if (stepCount >= SIM_STEPS) {
                direction = N;
                stepCount = 0;
            }
            break;
        case N:
            simLat+= SIM_STEP_SIZE;
            simDirection = 0;
            if (stepCount >= SIM_STEPS) {
                direction = E;
                stepCount = 0;
            }
            break;
    }
}

void ODID_getSimData(uint8_t *message, uint8_t msgType)
{
    switch (msgType) {
        case 0:
            basicID_data.IDType = ODID_IDTYPE_SERIAL_NUMBER;
            basicID_data.UAType = ODID_UATYPE_ROTORCRAFT;
            // 4 chr mfg code, 1chr Len, 15chr serial
            char id[] = "INTCE123456789012345";
            strncpy(basicID_data.UASID, id, sizeof(id));

            encodeBasicIDMessage(&basicID_enc, &basicID_data);
            memcpy(message, &basicID_enc, ODID_MESSAGE_SIZE);
            break;

        case 1:
            updateLocation();
            location_data.Status = ODID_STATUS_AIRBORNE;
            location_data.Direction = simDirection;
            location_data.SpeedHorizontal = simSpeedHorizontal;
            location_data.SpeedVertical = 2;
            location_data.Latitude = simLat;
            location_data.Longitude = simLon;
            location_data.AltitudeBaro = 100;
            location_data.AltitudeGeo = 100;
            location_data.HeightType = ODID_HEIGHT_REF_OVER_GROUND;
            location_data.Height = 50;
            location_data.HorizAccuracy = createEnumHorizontalAccuracy(2.5f);
            location_data.VertAccuracy = createEnumVerticalAccuracy(2.5f);
            location_data.BaroAccuracy = createEnumVerticalAccuracy(3.5f);
            location_data.SpeedAccuracy = createEnumSpeedAccuracy(0.2f);
            location_data.TSAccuracy = createEnumTimestampAccuracy(0.5f);
            location_data.TimeStamp = 60;

            encodeLocationMessage(&location_enc, &location_data);
            memcpy(message, &location_enc, ODID_MESSAGE_SIZE);
            break;

        case 2:
            auth_data.AuthType = ODID_AUTH_UAS_ID_SIGNATURE;
            auth_data.DataPage = 0;
            auth_data.PageCount = 1;
            auth_data.Length = 12;
            auth_data.Timestamp = 23000000;
            char data[] = "030a0cd033a3";
            strncpy(auth_data.AuthData, data, sizeof(data));

            encodeAuthMessage(&auth_enc, &auth_data);
            memcpy(message, &auth_enc, ODID_MESSAGE_SIZE);
            break;

        case 3:
            selfID_data.DescType = ODID_DESC_TYPE_TEXT;
            char description[] = "Real Estate Photos";
            strncpy(selfID_data.Desc, description, sizeof(description));
            encodeSelfIDMessage(&selfID_enc, &selfID_data);
            memcpy(message, &selfID_enc, ODID_MESSAGE_SIZE);
            break;

        case 4:
            system_data.LocationSource = ODID_LOCATION_SRC_TAKEOFF;
            system_data.OperatorLatitude = simGndLat;
            system_data.OperatorLongitude = simGndLon;
            system_data.AreaCount = 35;
            system_data.AreaRadius = 75;
            system_data.AreaCeiling = 176.9;
            system_data.AreaFloor = 41.7;
            encodeSystemMessage(&system_enc, &system_data);
            memcpy(message, &system_enc, ODID_MESSAGE_SIZE);
            break;

        case 5:
            operatorID_data.OperatorIdType = ODID_OPERATOR_ID;
            char operatorId[] = "98765432100123456789";
            strncpy(operatorID_data.OperatorId, operatorId, sizeof(operatorId));
            encodeOperatorIDMessage(&operatorID_enc, &operatorID_data);
            memcpy(message, &operatorID_enc, ODID_MESSAGE_SIZE);
            break;
    }
}

void test_sim()
{
    uint8_t testBytes[ODID_MESSAGE_SIZE];
    int x;
    while (1)
    {
        for (x = 0; x <= 5; x++) {
            memset(testBytes,0,ODID_MESSAGE_SIZE);
            ODID_getSimData(testBytes,x);
            printByteArray(testBytes,ODID_MESSAGE_SIZE,1);
            usleep(SLEEPMS);
        }
    }

}

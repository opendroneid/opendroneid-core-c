/*
Copyright (C) 2019 Intel Corporation

SPDX-License-Identifier: Apache-2.0

Open Drone ID C Library

Maintainer:
Gabriel Cox
gabriel.c.cox@intel.com
*/

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
const double SIM_STEP_SIZE = 0.0001;
const double DISTANCE_PER_LAT = 111699.0;


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
    simSpeedHorizontal = (float) DISTANCE_PER_LAT * (float) SIM_STEP_SIZE;
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

        default:
            break;
    }
}

void ODID_getSimData(uint8_t *message, uint8_t msgType)
{
    switch (msgType) {
        case ODID_MESSAGETYPE_BASIC_ID:
            basicID_data.IDType = ODID_IDTYPE_SERIAL_NUMBER;
            basicID_data.UAType = ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;
            // 4 chr mfg code, 1chr Len, 15chr serial
            char id[] = "INTCE123456789012345";
            strncpy(basicID_data.UASID, id, sizeof(basicID_data.UASID));

            encodeBasicIDMessage(&basicID_enc, &basicID_data);
            memcpy(message, &basicID_enc, ODID_MESSAGE_SIZE);
            break;

        case ODID_MESSAGETYPE_LOCATION:
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

        case ODID_MESSAGETYPE_AUTH:
            auth_data.AuthType = ODID_AUTH_UAS_ID_SIGNATURE;
            auth_data.DataPage = 0;
            auth_data.PageCount = 1;
            auth_data.Length = 12;
            auth_data.Timestamp = 23000000;
            char data[] = "030a0cd033a3";
            strncpy(auth_data.AuthData, data, sizeof(auth_data.AuthData));

            encodeAuthMessage(&auth_enc, &auth_data);
            memcpy(message, &auth_enc, ODID_MESSAGE_SIZE);
            break;

        case ODID_MESSAGETYPE_SELF_ID:
            selfID_data.DescType = ODID_DESC_TYPE_TEXT;
            char description[] = "Real Estate Photos";
            strncpy(selfID_data.Desc, description, sizeof(selfID_data.Desc));
            encodeSelfIDMessage(&selfID_enc, &selfID_data);
            memcpy(message, &selfID_enc, ODID_MESSAGE_SIZE);
            break;

        case ODID_MESSAGETYPE_SYSTEM:
            system_data.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
            system_data.ClassificationType = ODID_CLASSIFICATION_TYPE_EU;
            system_data.OperatorLatitude = simGndLat;
            system_data.OperatorLongitude = simGndLon;
            system_data.AreaCount = 35;
            system_data.AreaRadius = 75;
            system_data.AreaCeiling = 176.9f;
            system_data.AreaFloor = 41.7f;
            system_data.CategoryEU = ODID_CATEGORY_EU_SPECIFIC;
            system_data.ClassEU = ODID_CLASS_EU_CLASS_3;
            system_data.OperatorAltitudeGeo = 16.5f;
            encodeSystemMessage(&system_enc, &system_data);
            memcpy(message, &system_enc, ODID_MESSAGE_SIZE);
            break;

        case ODID_MESSAGETYPE_OPERATOR_ID:
            operatorID_data.OperatorIdType = ODID_OPERATOR_ID;
            char operatorId[] = "98765432100123456789";
            strncpy(operatorID_data.OperatorId, operatorId, sizeof(operatorID_data.OperatorId));
            encodeOperatorIDMessage(&operatorID_enc, &operatorID_data);
            memcpy(message, &operatorID_enc, ODID_MESSAGE_SIZE);
            break;

        default:
            break;
    }
}

_Noreturn void test_sim()
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

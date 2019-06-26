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

ODID_BasicID_data basicID_data;
ODID_Location_data location_data;
ODID_Auth_data auth_data;
ODID_SelfID_data selfID_data;
ODID_System_data system_data;

const int SIM_STEPS = 20;
const float SIM_STEP_SIZE = 0.0001;
const float DISTANCE_PER_LAT = 111699.0;


double simLat = 45.5393092;
double simLon = -122.9663894;
double simGndLat = 45.5393082;
double simGndLon = -122.9663884;
float simSpeedNS = 0;
float simSpeedEW = 0;

enum compdirs {E,S,W,N};
enum compdirs direction = E;
int stepCount = 0;

void updateLocation(void)
{
    stepCount++;
    switch(direction) {
        case E:
            simLon+= SIM_STEP_SIZE;
            simSpeedNS = 0;
            simSpeedEW = (cos(simLat * (M_PI/180)) * DISTANCE_PER_LAT) * SIM_STEP_SIZE;
            if (stepCount >= SIM_STEPS) {
                direction = S;
                stepCount = 0;
            }
            break;
        case S:
            simLat-= SIM_STEP_SIZE;
            simSpeedNS = -1.0 * DISTANCE_PER_LAT * SIM_STEP_SIZE;
            simSpeedEW = 0;
            if (stepCount >= SIM_STEPS) {
                direction = W;
                stepCount = 0;
            }
            break;
        case W:
            simLon-= SIM_STEP_SIZE;
            simSpeedNS = 0;
            simSpeedEW = -1 * (cos(simLat * (M_PI/180)) * DISTANCE_PER_LAT) * SIM_STEP_SIZE;
            if (stepCount >= SIM_STEPS) {
                direction = N;
                stepCount = 0;
            }
            break;
        case N:
            simLat+= SIM_STEP_SIZE;
            simSpeedNS = DISTANCE_PER_LAT * SIM_STEP_SIZE;
            simSpeedEW = 0;
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
            basicID_data.UASType = ODID_UAVTYPE_ROTORCRAFT_MULTIROTOR;
            // 4 chr mfg code, 1chr Len, 15chr serial
            safe_copyfill(basicID_data.UASID, "INTCE123456789012345", sizeof(basicID_data.UASID));

            encodeBasicIDMessage(&basicID_enc, &basicID_data);
            memcpy(message, &basicID_enc, ODID_MESSAGE_SIZE);
            break;

        case 1:
            updateLocation();
            location_data.Status = ODID_STATUS_AIRBORNE;
            location_data.SpeedNS = simSpeedNS;
            location_data.SpeedEW = simSpeedEW;
            location_data.SpeedVertical = 2;
            location_data.Latitude = simLat;
            location_data.Longitude = simLon;
            location_data.AltitudeBaro = 100;
            location_data.AltitudeGeo = 100;
            location_data.HeightAboveTakeoff = 50;
            location_data.HorizAccuracy = 2.5f;
            location_data.VertAccuracy = 2.5f;
            location_data.TSAccuracy = 0.2f;
            location_data.SpeedAccuracy = 0.5f;
            location_data.TimeStamp = 60;

            encodeLocationMessage(&location_enc, &location_data);
            memcpy(message, &location_enc, ODID_MESSAGE_SIZE);
            break;

        case 2:
            auth_data.AuthType = 1;
            auth_data.DataPage = 0;
            safe_copyfill(auth_data.AuthData, "030a0cd033a3",sizeof(auth_data.AuthData));

            encodeAuthMessage(&auth_enc, &auth_data);
            memcpy(message, &auth_enc, ODID_MESSAGE_SIZE);
            break;

        case 3:
            selfID_data.DescType = 0;
            safe_copyfill(selfID_data.Desc, "Real Estate Photos", sizeof(selfID_data.Desc));
            encodeSelfIDMessage(&selfID_enc, &selfID_data);
            memcpy(message, &selfID_enc, ODID_MESSAGE_SIZE);
            break;

        case 4:
            system_data.LocationSource = 0; // 0 = Takeoff Point
            system_data.Latitude = simGndLat;
            system_data.Longitude = simGndLon;
            system_data.GroupCount = 0;
            system_data.GroupRadius = 0;
            system_data.GroupCeiling = 0;
            encodeSystemMessage(&system_enc, &system_data);
            memcpy(message, &system_enc, ODID_MESSAGE_SIZE);
            break;
    }
}

void test_sim()
{
    uint8_t testBytes[ODID_MESSAGE_SIZE];
    int x;
    while (1)
    {
        for (x=0; x<=4; x++) {
            memset(testBytes,0,ODID_MESSAGE_SIZE);
            ODID_getSimData(testBytes,x);
            printByteArray(testBytes,ODID_MESSAGE_SIZE,1);
            usleep(SLEEPMS);
        }
    }

}

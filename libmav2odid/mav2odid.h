/*
Copyright (C) 2019 Intel Corporation

SPDX-License-Identifier: Apache-2.0

Mavlink to Open Drone ID C Library

Maintainer:
Soren Friis
soren.friis@intel.com
*/

#ifndef _MAV2ODID_H_
#define _MAV2ODID_H_

#include <opendroneid.h>

/*
// Mavlink internally allocates two global buffers. If this is unacceptable from
// a system perspective, these can be allocated either globally here by
// uncommenting the below code or the below definitions can be changed to
// pointers and the memory allocated somewhere else and the pointers initialized
// before calling any functions in mav2odid.
// If Mavlink's virtual channel functionality is not used, some memory can be
// saved by defining MAVLINK_COMM_NUM_BUFFERS to be equal to 1, before including
// mavlink_types.h

#include "../mavlink_c_library_v2/mavlink_types.h"

#define MAVLINK_EXTERNAL_RX_BUFFER
mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];

#define MAVLINK_EXTERNAL_RX_STATUS
mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
*/

#include <common/mavlink.h>

#define DRONEID_SCHEDULER_SIZE 8

typedef struct {
    uint8_t droneidSchedule[DRONEID_SCHEDULER_SIZE];
    uint8_t scheduleIdx;

    odid_data_basic_id_t basicId;
    odid_encoded_basic_id_t basicIdEnc;
    odid_data_location_t location;
    odid_encoded_location_t locationEnc;
    odid_data_auth_t authentication;
    odid_encoded_auth_t authenticationEnc;
    odid_data_self_id_t selfId;
    odid_encoded_self_id_t selfIdEnc;
    odid_data_system_t system;
    odid_encoded_system_t systemEnc;
} mav2odid_t;

int m2o_init(mav2odid_t *m2o);
int m2o_cycleMessages(mav2odid_t *m2o, uint8_t *data);
odid_message_type_t m2o_parseMavlink(mav2odid_t *m2o, uint8_t data);

void m2o_basicId2Mavlink(mavlink_open_drone_id_basic_id_t *mavBasicId,
                         odid_data_basic_id_t *basicId);
void m2o_location2Mavlink(mavlink_open_drone_id_location_t *mavLocation,
                          odid_data_location_t *location);
void m2o_authentication2Mavlink(mavlink_open_drone_id_authentication_t *mavAuth,
                                odid_data_auth_t *Auth);
void m2o_selfId2Mavlink(mavlink_open_drone_id_selfid_t *mavSelfID,
                        odid_data_self_id_t *selfID);
void m2o_system2Mavlink(mavlink_open_drone_id_system_t *mavSystem,
                        odid_data_system_t *system);

#endif /* _MAV2ODID_H_ */

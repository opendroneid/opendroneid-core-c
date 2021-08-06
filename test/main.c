/*
Copyright (C) 2019 Intel Corporation

SPDX-License-Identifier: Apache-2.0

Open Drone ID C Library

Maintainer:
Gabriel Cox
gabriel.c.cox@intel.com
*/

#include <stdio.h>
#include <opendroneid.h>

void test_InOut(void);
void ODID_getSimData(uint8_t *message, uint8_t msgType);
_Noreturn void test_sim(void);
void test_mav2odid();

int main(int argc, char const *argv[]) {

    // Perform test that takes all nominal unpacked structures displays them,
    // encodes them, decodes them, displays result
    test_InOut();

    // Test the Mavlink to OpenDroneID conversion functionality
    printf("\nPress enter to run the Mavlink to Open Drone ID test");
    getchar();
    test_mav2odid();

    // Simulates a moving drone, encodes and displays data
    printf("\nPress enter to begin simulator messages...");
    getchar();
    test_sim();
}

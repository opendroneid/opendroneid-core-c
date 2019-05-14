# Copyright (C) 2019 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0
#
# Open Drone ID C Library
#
# Maintainer:
# Gabriel Cox
# gabriel.c.cox@intel.com

CFLAGS=-D_FORTIFY_SOURCE=2 -fstack-protector -fno-delete-null-pointer-checks -fwrapv -O0
all: odidtest

odidtest: opendroneid.c opendroneid_sim.c test_inout.c main.c
	gcc $(CFLAGS) -o $@ $^ -lm

clean: 
	$(RM) odidtest *.o

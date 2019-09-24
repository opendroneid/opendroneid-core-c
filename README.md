# opendroneid-core-c

Open Drone ID Core C Library

This provides a function library for encoding and decoding (packing/unpacking) Open Drone ID messages as the format is defined in the specification.

To build the library and the sample app:

```
git submodule init
git submodule update
cmake .
make
```

The outputs will be `libopendroneid/libopendroneid.so` and the `test/odidtest` sample application.

The sample application will do a test encode/decode, then continuously generate sample messages.

The intended architecture is to take whatever input you wish, and to put it into the nominal structures as defined in `libopendroneid/opendroneid.h`.

![Core SDK Scope](img/core-arch.png "Core SDK Scope")

These are nominal (non-encoded) structures:

```
odid_data_basic_id_t
odid_data_location_t
odid_data_auth_t
odid_data_self_id_t
odid_data_system_t
```

The SDK functions will encode to (or decode from) the following structures:

```
odid_encoded_basic_id_t
odid_encoded_location_t
odid_encoded_auth_t
odid_encoded_self_id_t
odid_encoded_system_t
```

Once you have the encoded data, then you are ready to assemble and transmit over any of the acceptable broadcast methods in Open Drone ID.

The primary SDK calls are the following:

```
int odid_encode_message_basic_id(odid_encoded_basic_id_t *outEncoded, odid_data_basic_id_t *inData);
int odid_encode_message_location(odid_encoded_location_t *outEncoded, odid_data_location_t *inData);
int odid_encode_message_auth(odid_encoded_auth_t *outEncoded, odid_data_auth_t *inData);
int odid_encode_message_self_id(odid_encoded_self_id_t *outEncoded, odid_data_self_id_t *inData);
int odid_encode_message_system(odid_encoded_system_t *outEncoded, odid_data_system_t *inData);

int odid_decode_message_basic_id(odid_data_basic_id_t *outData, odid_encoded_basic_id_t *inEncoded);
int odid_decode_message_location(odid_data_location_t *outData, odid_encoded_location_t *inEncoded);
int odid_decode_message_auth(odid_data_auth_t *outData, odid_encoded_auth_t *inEncoded);
int odid_decode_message_self_id(odid_data_self_id_t *outData, odid_encoded_self_id_t *inEncoded);
int odid_decode_message_system(odid_data_system_t *outData, odid_encoded_system_t *inEncoded);
```

Specific messages have been added to the Mavlink message set to accomodate data for Open Drone ID implementations:

https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID

The functions in `mav2odid.c` can be used to convert these Mavlink messages into suitable `opendroneid.h` data structures and back again. See the example usages in `test/test_mav2odid.c`.

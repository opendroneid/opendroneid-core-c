# opendroneid-core-c

## Open Drone ID Core C Library

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

## Build Options

### MavLink

MavLink OpenDroneID support is included by default.

To disable, use the ```BUILD_MAVLINK``` parameter, i.e.:

```
cmake -DBUILD_MAVLINK=off .
```

MavLinks requires the mavlink_c_library_v2 to be installed in the respective folder

### WiFi reference implementation

WiFi reference implementation is built by default.

To disable, use the ```BUILD_WIFI``` parameter, i.e.:

```
cmake -DBUILD_WIFI=off .
```

It requires the libgps, libnl-3 and libnl-genl-3 support. Install the dependencies on your build host, e.g. on Debian/Ubuntu use

```
sudo apt-get install libgps-dev libnl-genl-3-dev
```

If available, the WiFi reference implementation will link against libnl-tiny instead of libnl*-3 if available.

## Architecture

![Core SDK Scope](img/core-arch.png "Core SDK Scope")

These are nominal (non-encoded) structures:

```
ODID_BasicID_data
ODID_Location_data
ODID_Auth_data
ODID_SelfID_data
ODID_System_data
ODID_OperatorID_data
ODID_MessagePack_data
```

The SDK functions will encode to (or decode from) the following structures:

```
ODID_BasicID_encoded
ODID_Location_encoded
ODID_Auth_encoded
ODID_SelfID_encoded
ODID_System_encoded
ODID_OperatorID_encoded
ODID_MessagePack_encoded
```

Once you have the encoded data, then you are ready to assemble and transmit over any of the acceptable broadcast methods in Open Drone ID.

The primary SDK calls are the following:

```
int encodeBasicIDMessage(ODID_BasicID_encoded *outEncoded, ODID_BasicID_data *inData);
int encodeLocationMessage(ODID_Location_encoded *outEncoded, ODID_Location_data *inData);
int encodeAuthMessage(ODID_Auth_encoded *outEncoded, ODID_Auth_data *inData);
int encodeSelfIDMessage(ODID_SelfID_encoded *outEncoded, ODID_SelfID_data *inData);
int encodeSystemMessage(ODID_System_encoded *outEncoded, ODID_System_data *inData);
int encodeOperatorIDMessage(ODID_OperatorID_encoded *outEncoded, ODID_OperatorID_data *inData);
int encodeMessagePack(ODID_MessagePack_encoded *outEncoded, ODID_MessagePack_data *inData);

int decodeBasicIDMessage(ODID_BasicID_data *outData, ODID_BasicID_encoded *inEncoded);
int decodeLocationMessage(ODID_Location_data *outData, ODID_Location_encoded *inEncoded);
int decodeAuthMessage(ODID_Auth_data *outData, ODID_Auth_encoded *inEncoded);
int decodeSelfIDMessage(ODID_SelfID_data *outData, ODID_SelfID_encoded *inEncoded);
int decodeSystemMessage(ODID_System_data *outData, ODID_System_encoded *inEncoded);
int decodeOperatorIDMessage(ODID_OperatorID_data *outData, ODID_OperatorID_encoded *inEncoded);
int decodeMessagePack(ODID_UAS_Data *uasData, ODID_MessagePack_encoded *pack);
```

Specific messages have been added to the Mavlink message set to accomodate data for Open Drone ID implementations:

https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID

The functions in `mav2odid.c` can be used to convert these Mavlink messages into suitable `opendroneid.h` data structures and back again. See the example usages in `test/test_mav2odid.c`.

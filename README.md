# opendroneid-core-c

## Open Drone ID Core C Library

This provides a function library for encoding and decoding (packing/unpacking) Open Drone ID messages as the format is defined in the ASTM Remote ID standard available at https://www.astm.org/Standards/F3411.htm.
The code is also compatible with the upcoming European ASD-STAN Direct Remote ID standard.
This latter standard has not yet been published, but some preliminary information can be found in this [white paper](https://asd-stan.org/wp-content/uploads/ASD-STAN_DRI_Introduction_to_the_European_digital_RID_UAS_Standard.pdf) and in the recording of this [webinar](https://www.cencenelec.eu/news/events/Pages/EV-2021-15.aspx).

The opendroneid-core-c code is primarily meant for implementations that will broadcast the Remote ID information via Bluetooth or WiFi NaN.
If you are looking for code related to Network Remoted ID (via the internet), please take a look at https://github.com/interuss and https://github.com/uastech/standards.

Work is ongoing by the IETF DRIP (Drone Remote ID Protocol) task force to define how security could be supported in the context of the ASTM Remote ID standard:
https://datatracker.ietf.org/wg/drip/documents/ and https://github.com/ietf-wg-drip.

For an example Android receiver application supporting Bluetooth and WiFi NaN, see https://github.com/opendroneid/receiver-android.

Mavlink messages for drone ID are available at https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID and documentation on how to use them is available at https://mavlink.io/en/services/opendroneid.html.

An example library for transmitting Open Drone ID signals from ESP32 HW can be found at https://github.com/sxjack/uav_electronic_ids.
The implementation supports simultaneous transmission via Bluetooth Legacy Advertising and via WiFi NaN.
Please note that the ESP32 HW only supports transmitting Bluetooth Legacy Advertising signals. Long Range and Extended Advertising are not supported.
Please check if this is sufficient to comply with the rules that apply in the area in which you are flying.
The [ESP32-C3](https://www.espressif.com/en/news/ESP32_C3) and [ESP32-S3](https://www.espressif.com/en/news/ESP32_S3?position=0&list=_TQH0oNBtbw0KMnMbCVH3ol_jy3McAHwrsqIZcX6XjM) chips will both support Long Range and Extended Advertising.

To build the library and the sample app:

```
sudo apt-get install libgps-dev libnl-genl-3-dev
git submodule update --init
mkdir build && cd build
cmake ../.
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

Specific messages have been added to the MAVLink message set to accomodate data for Open Drone ID implementations:

https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID

The functions in `mav2odid.c` can be used to convert these MAVLink messages into suitable `opendroneid.h` data structures and back again. See the example usages in `test/test_mav2odid.c`.

Recomendations on how to utilize the MAVLink messages for internal distribution of Open Drone ID data of an Unmanned Aircraft System can be found here:

https://mavlink.io/en/services/opendroneid.html

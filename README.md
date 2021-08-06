# opendroneid-core-c

## Open Drone ID Core C Library

This provides a function library for encoding and decoding (packing/unpacking) Open Drone ID messages as the format is defined in the ASTM Remote ID standard available [here](https://www.astm.org/Standards/F3411.htm).
The code is also compatible with the upcoming European ASD-STAN Direct Remote ID standard.
This latter standard has not yet been published, but some preliminary information can be found in this [white paper](https://asd-stan.org/wp-content/uploads/ASD-STAN_DRI_Introduction_to_the_European_digital_RID_UAS_Standard.pdf) and in the recording of this [webinar](https://www.cencenelec.eu/news/events/Pages/EV-2021-15.aspx).
An early draft of the standard is available [here](https://asd-stan.org/downloads/din-en-4709-0022021-02/).

Please note that both standards have been updated during the first half of 2021 and the updated documents are not yet published (August 2021).
However, this implementation is already compliant with these updates. 

The opendroneid-core-c code is meant for implementations that will broadcast the Remote ID information via Bluetooth or WiFi.
If you are looking for code related to Network Remoted ID (via the internet), please take a look at https://github.com/interuss and https://github.com/uastech/standards.

Work is ongoing by the IETF DRIP (Drone Remote ID Protocol) task force to define how security could be supported in the context of the ASTM Remote ID standard:
https://datatracker.ietf.org/wg/drip/documents/ and https://github.com/ietf-wg-drip.

MAVLink messages for drone ID are available at https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID and documentation on how to use them is available at https://mavlink.io/en/services/opendroneid.html.

## Receiver examples

For an example Android receiver application supporting Bluetooth and WiFi, see https://github.com/opendroneid/receiver-android.

Examples on how to use the WireShark PC application to pick up and dissect open drone ID messages are available here: https://github.com/opendroneid/wireshark-dissector.  

## Transmitter examples

An example library for transmitting Open Drone ID signals from ESP32 HW can be found at https://github.com/sxjack/uav_electronic_ids.
The implementation supports simultaneous transmission via Bluetooth Legacy Advertising, WiFi NaN and WiFi Beacon.
Please note that the ESP32 HW only supports transmitting Bluetooth Legacy Advertising signals. Bluetooth Long Range and Extended Advertising are not supported.
Please check if this is sufficient to comply with the rules that apply in the area in which you are flying.
The [ESP32-C3](https://www.espressif.com/en/news/ESP32_C3) and [ESP32-S3](https://www.espressif.com/en/news/ESP32_S3?position=0&list=_TQH0oNBtbw0KMnMbCVH3ol_jy3McAHwrsqIZcX6XjM) chips will both support Long Range and Extended Advertising but this has not yet been tested.

A WiFi NaN transmitter implementation for Linux is available [here](https://github.com/opendroneid/opendroneid-core-c/blob/master/wifi/sender/main.c).
Better documentation is needed on what exact HW + SW environment this is functional.

A description of some very preliminary experiments using bluez and hostapd to transmit BT4 and WiFi Beacon drone ID signals from a Linux PC or Raspberry Pi can be found [here](https://github.com/opendroneid/opendroneid-core-c/issues/42).
Contributions to create fully functional transmitter implementations are very welcome.

Transmitter implementations for Bluetooth 4 and 5, based on either the TI CC2640 or the nRF52480 SoCs are known to exists but so far none have been open sourced.
Please open an issue if you have an implementation you are willing to share. A new repository under opendroneid can be made.

## How to Build

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

### Memory reductions

Some embedded systems might require a smaller memory footprint than what by default is used by opendroneid-core-c.
The following compile time options exists for reducing the memory consumption:
- ODID_AUTH_MAX_PAGES is by default configured to support 16 pages/messages of authentication data.
  See the beginning of [opedroneid.h](libopendroneid/opendroneid.h).
  If authentication messages are not used, this value can be configured between 1 and 16, e.g. by adding `-DODID_AUTH_MAX_PAGES=1` when calling cmake.


- When including MAVLink in the build (see below), if MAVLink's virtual channel functionality is not used, some memory can be saved by defining MAVLINK_COMM_NUM_BUFFERS to be equal to 1, before including mavlink_types.h
  See further details in the beginning of [mav2odid.c](libmav2odid/mav2odid.c).

### MAVLink

MAVLink OpenDroneID support is included by default.

To disable, use the ```BUILD_MAVLINK``` parameter, i.e.:

```
cmake -DBUILD_MAVLINK=off .
```

MAVLink support requires the mavlink_c_library_v2 to be installed in the respective folder

### WiFi NaN example implementation

The WiFi NaN example implementation is built by default.

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

Specific messages have been added to the MAVLink message set to accommodate data for Open Drone ID implementations:

https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID

The functions in `mav2odid.c` can be used to convert these MAVLink messages into suitable `opendroneid.h` data structures and back again. See the example usages in `test/test_mav2odid.c`.

Recommendations on how to utilize the MAVLink messages for internal distribution of Open Drone ID data of an Unmanned Aircraft System can be found here:
https://mavlink.io/en/services/opendroneid.html

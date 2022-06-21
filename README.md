# opendroneid-core-c

## Open Drone ID Core C Library

This repository provides a C-code function library for encoding and decoding (packing/unpacking) Open Drone ID messages, as the format is defined in the ASTM F3411 Remote ID and the ASD-STAN prEN 4709-002 Direct Remote ID specifications.
See further details in the [specifications](#relevant-specifications) section below.

Please note that both specifications have been updated during the first half of 2021.
Both of the updated standards have been published (May 2022).
The code available in this repository is compliant with these updates.

The opendroneid-core-c code is meant for implementations that will broadcast the Remote ID information via Bluetooth or Wi-Fi.
If you are looking for code related to Network Remoted ID (via the internet), please take a look at https://github.com/interuss and https://github.com/uastech/standards.

Work is ongoing by the IETF DRIP (Drone Remote ID Protocol) task force to define how security could be supported in the context of the ASTM Remote ID specification:
https://datatracker.ietf.org/wg/drip/documents/ and https://github.com/ietf-wg-drip.

MAVLink messages for drone ID are available at https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID and documentation on how to use them is available at https://mavlink.io/en/services/opendroneid.html.

If you want to contribute to the open source efforts for Remote ID, see [below](#contribution-suggestions).

## Receiver examples

### Android
For an example Android receiver application supporting Bluetooth and Wi-Fi, see https://github.com/opendroneid/receiver-android.

### iOS
Currently there is no open source iOS receiver application for Apple devices.
However, reception of Bluetooth 4 Legacy Advertising drone ID signals have been demonstrated to work on iOS.
Apple currently does not expose suitable APIs to receive any other transmission method for drone ID signals than BT4 legacy advertising. I.e. current versions of iOS (up to and including 15) do not support receiving BT5 Long Range + Extended Advertising, Wi-Fi NaN nor Wi-Fi Beacon.

### WireShark
Examples on how to use the WireShark PC application to pick up and dissect open drone ID messages are available here: https://github.com/opendroneid/wireshark-dissector.  

### ESP32
The [ESP32 transmitter](https://github.com/sxjack/uav_electronic_ids) example code also contains code for receiving drone ID signals on ESP32 HW.

### Smartphone receivers

A list of smartphones that have been tested for receiving Remote ID signals is available [here](https://github.com/opendroneid/receiver-android/blob/master/supported-smartphones.md).

## Transmitter examples

### ESP32
An example library for transmitting Open Drone ID signals from ESP32 HW can be found at https://github.com/sxjack/uav_electronic_ids.
The implementation supports simultaneous transmission via Bluetooth Legacy Advertising, Wi-Fi NaN and Wi-Fi Beacon.

Please note that the ESP32 HW only supports transmitting Bluetooth Legacy Advertising signals. Bluetooth Long Range and Extended Advertising are not supported.
Please check if this is sufficient to comply with the rules that apply in the area in which you are flying (most likely it is not. See [below](#relevant-specifications)).

The [ESP32-C3](https://www.espressif.com/en/news/ESP32_C3) and [ESP32-S3](https://www.espressif.com/en/news/ESP32_S3) chips both support Long Range and Extended Advertising.
External testing have confirmed that they are capable of simultaneously transmitting both BT4 Legacy advertising signals and BT5 Long Range advertising signals.
No open-source implementation demonstrating this is known though.

### Linux
A Wi-Fi NaN transmitter implementation for Linux is available [here](https://github.com/opendroneid/opendroneid-core-c/blob/master/wifi/sender/main.c).
Better documentation is needed on what exact HW + SW environment this is functional.
Functions for creating suitable Wi-Fi Beacon frames have been added to wifi.c, but those are currently not used by the sample sender application.

A simple application for sending static drone ID data via Bluetooth 4 and 5 and via Wi-Fi Beacon is available [here](https://github.com/opendroneid/transmitter-linux).
This has tested to work reasonably okay on one CometLake motherboard and partly okay on RaspberryPi 3B and 4B HW.

### nRF and TI Bluetooth chipsets
Transmitter implementations for Bluetooth 4 and 5, based on either the TI CC2640 or the nRF52480 SoCs are known to exist, but so far none have been open sourced.
Please open an issue if you have an implementation you are willing to share.
A new repository under opendroneid can be made.

### Transmitter devices

A list of devices capable of transmitting Remote ID signals is available [here](https://github.com/opendroneid/receiver-android/blob/master/transmitter-devices.md).

## How to Build

To build the library and the sample app on Linux:

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

### Wi-Fi NaN example implementation

The Wi-Fi NaN example implementation is built by default.

To disable, use the ```BUILD_WIFI``` parameter, i.e.:

```
cmake -DBUILD_WIFI=off .
```

It requires the libgps, libnl-3 and libnl-genl-3 support. Install the dependencies on your build host, e.g. on Debian/Ubuntu use

```
sudo apt-get install libgps-dev libnl-genl-3-dev
```

If available, the Wi-Fi reference implementation will link against libnl-tiny instead of libnl*-3 if available.

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

Recommendations on how to utilize the MAVLink messages for UAS internal distribution of Open Drone ID data of an Unmanned Aircraft System (UAS) can be found here:
https://mavlink.io/en/services/opendroneid.html

## Contribution suggestions

Any contribution to the open source efforts related to Remote ID will be very welcome.
Below is a list of multiple topics that would be useful to get sorted out, but anything you feel you can help with, will certainly be appreciated.

* Integration of Remote ID [MAVLink messages](https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID) into any of the open source flight controller implementations (Ardupilot, PX4, etc.), while following the [draft architecture documentation](https://mavlink.io/en/services/opendroneid.html).
This is required in order to validate their use and get the Work In Progress marks removed from the drone ID MAVLink messages and the drafted architecture.
* Provide open source transmitter implementations for the TI [CC2640](https://github.com/opendroneid/transmitter-cc2640R2) (and related) and/or the [nRF52480](https://github.com/opendroneid/transmitter-nrf) (and related) Bluetooth transmitter chips (also ST have various Bluetooth chips). Preferably implementations capable of feeding in data via MAVLink messages
* Implement open source drone ID transmission examples on the [ESP32-C3](https://www.espressif.com/en/news/ESP32_C3) and/or [ESP32-S3](https://www.espressif.com/en/news/ESP32_S3) showing how to do simultaneous BT4 and BT5 advertising
* There are [multiple issues](https://github.com/opendroneid/receiver-android/issues) open for the Android Receiver example application.
Some are new feature requests.
Particularly the new feature request from [issue 27](https://github.com/opendroneid/receiver-android/issues/27) would be good to get supported, since this will help the application be better compliant with the [ASTM Means of Compliance](#united-states) document
* The ESP32 transmitter has an unexplained problem with the [Wi-Fi Beacon signals](https://github.com/sxjack/uav_electronic_ids/issues/9).
For some Android phones it is close to impossible to pick up the signal, despite it being sent regularly and the phone easily picking up the signal from a RaspberryPi or other Linux transmitter.
* Testing of additional (Android) smartphones for expanding the [receiver compatibility list](https://github.com/opendroneid/receiver-android/blob/master/supported-smartphones.md).
* Information and updates to the [Transmitter Devices list](https://github.com/opendroneid/receiver-android/blob/master/transmitter-devices.md).

## Relevant specifications

### United States

The [ASTM F3411](https://www.astm.org/Standards/F3411.htm) Specification for Remote ID and Tracking has been defined to specify how Unmanned Aircraft (UA) or Unmanned Aircraft Systems (UAS) can publish their ID, location, altitude etc., either via direct broadcast (Bluetooth or Wi-Fi), or via an internet connection to a Remote ID server.

Version 1.0 (F3411-19) of the specification is available: https://www.astm.org/f3411-19.html

Version 1.1 (F3411-22) of the specification is available: https://www.astm.org/f3411-22.html

The updated version F3411-22 contains smaller changes/additions to make the message content etc. better suited to meet the [rule](https://www.regulations.gov/document/FAA-2019-1100-53264) defined by the [FAA](https://www.faa.gov/uas/getting_started/remote_id/) (Federal Aviation Administration) for [UAS flights](https://www.faa.gov/uas/commercial_operators/operations_over_people/) in the United States.

Additionally, a Means of Compliance document (MoC) has been drafted by the ASTM and is undergoing ballot (March 2022).
It contains further implementation requirements and test specifications.

Together, the two documents will allow manufacturers of UAS and remote ID broadcast modules/Add-ons (for retro-fit on UAs without built-in remote ID support) to implement remote ID support and create the necessary Declaration of Compliance (DoC) document, which must be submitted to the FAA for approval.

### European Union

To meet the European Commission Delegated Regulation [2019/945](https://eur-lex.europa.eu/eli/reg_del/2019/945/2020-08-09) and the Commission Implementing Regulation [2019/947](https://eur-lex.europa.eu/eli/reg_impl/2019/947/2021-08-05), ASD-STAN has developed the prEN 4709-002 Direct Remote Identification specification.
It specifies broadcast methods for Remote ID (Bluetooth and Wi-Fi) that are compliant with the ASTM F3411 v1.1 specification (first ballot round).

The final version of the standard has been published [here](http://asd-stan.org/downloads/asd-stan-pren-4709-002-p1/).
See also the summary [whitepaper](https://asd-stan.org/wp-content/uploads/ASD-STAN_DRI_Introduction_to_the_European_digital_RID_UAS_Standard.pdf) and the recording of this [webinar](https://www.cencenelec.eu/news-and-events/events/2021-02-09-european-workshop-on-uas-direct-remote-identification/).

### Protocol versions

The continued development of the relevant standards is reflected in the remote ID protocol version number transmitted in the header of each drone ID message.
The following protocol versions have been in use:

 0. ASTM [F3411-19](https://www.astm.org/f3411-19.html). Published Feb 14, 2020.

 1. ASD-STAN prEN [4709-002 P1](http://asd-stan.org/downloads/asd-stan-pren-4709-002-p1/). Published 31-Oct-2021.

    ASTM F3411 v1.1 draft sent for first ballot round autumn 2021.

 2. ASTM F3411-v1.1 draft for second ballot round Q1 2022 and published May 25th 2022. (ASTM [F3411-22](https://www.astm.org/f3411-22.html))

      The delta to protocol version 1 is small:
      - New enum values ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE, ODID_DESC_TYPE_EMERGENCY and ODID_DESC_TYPE_EXTENDED_STATUS
      - New Timestamp field in the System message

### Timelines

The timelines for the rules requiring manufacturers and drone operators to be compliant have been fluctuating.
Some information can be found behind the following links but please do your own research, since this is not necessarily the most up-to-date information.
Remember that as a manufacturer or drone operator, you are personally responsible for being compliant with all relevant laws and standards applicable to the area of operation.
 * US: https://www.faa.gov/uas/getting_started/remote_id/
 * EU: https://eur-lex.europa.eu/eli/reg_impl/2022/425/
 * Japan: https://www.mlit.go.jp/koku/drone/en/

### Comparison

A comparison of some of the more detailed parts of each specification and rule is given in the table below.
If a field is left blank, nothing specific is mentioned in the particular document related to that part.

The rules are what is defined by law and must be followed (the FAA rule in the United States and the EU rule in the European Union).
The ASTM Means of Compliance (MoC) document overrides certain parts of the ASTM specification to meet the FAA rule requirements.

| | FAA rule | ASTM v1.1 | ASTM MoC | EU rule | ASD-STAN DRI |
| --- | --- | --- | --- | --- | --- |
| Serial ANSI/CTA-2063-A | M<sup>1</sup> |  M<sup>1</sup> | M<sup>1</sup> | M | M |
| Session ID | M<sup>1</sup> | M<sup>1</sup> | M<sup>1</sup> |  | O |
| UA dynamic position | M | M |  | M | M |
| UA altitude WGS-84 | M | M |  |  | O |
| UA altitude AGL/Take-off |  | O |  | M | M |
| Timestamp (Location msg) | M | M |  | M | M |
| Timestamp (System msg) | M | O | M |  |  |
| Operational/Emergency status | M<sup>2</sup> | O | M<sup>2</sup> | M<sup>2</sup> | M<sup>2</sup> |
| Track direction | M | M |  | M | M |
| Horizontal speed | M | M |  | M | M |
| Vertical speed | M | M |  |  | O |
| Operator registration ID |  | O |  | M | M |
| EU Category & Class |  | O |  |  | R |
| Operator dynamic position | M<sup>3</sup> | O | M<sup>3</sup> | M<sup>3</sup> | M<sup>3</sup> |
| Operator altitude WGS-84 | M | O | M |  | O |
| Transmission interval<sup>4</sup> (seconds) | 1 | 1 or 3 | 1 |  | 1 or 3 |
| Transmission time | Take-off to shutdown |  | Take-off to shutdown |  | When airborne |
| BT4 Legacy Advertising |  | M<sup>5</sup> | M<sup>6, 7</sup> |  | O |
| BT5 Long Range |  | O | M<sup>6, 7</sup> |  | M<sup>5</sup> |
| Wi-Fi NaN 2.4 GHz |  | M<sup>5</sup> |  |  | M<sup>5</sup> |
| Wi-Fi NaN 5 GHz |  | O |  |  | M<sup>5</sup> |
| Wi-Fi Beacon 2.4 GHz<sup>8</sup> |  | M<sup>5</sup> | M<sup>7</sup> |  | M<sup>5</sup> |
| Wi-Fi Beacon 5 GHz<sup>8</sup> |  | M<sup>5</sup> | M<sup>7</sup> |  | M<sup>5</sup> |

M: Mandatory. O: Optional. R: Recommended

1. Either the Serial Number or the Session ID must be transmitted.
    Add-ons are not allowed to use Session ID.
2. Not required for Add-ons.
3. For Add-ons under EU rule: the Take-off location can be used instead.
    For Add-ons under FAA rule: The Take-off location is required instead.
4. Concerns only the Basic ID, Location and System messages.
    Location is always with 1 second intervals.
    Basic ID and System intervals can be 3 seconds in the specifications.
5. Only one of the Mandatory broadcast methods is required.
6. Both BT4 and BT5 must be transmitted simultaneously.
7. Only Bluetooth (BT4 + BT5 simultaneously) or Wi-Fi Beacon (2.4 GHz or 5 GHz) are allowed.
8. If any other channel than Channel 6 on 2.4 GHz or Channel 149 on 5 GHz is used, a faster transmission rate of 5 Hz is required.

#include <gtest/gtest.h>
#include <opendroneid.h>

ODID_UAS_Data testData = {
    .BasicID = {{ODID_UATYPE_HYBRID_LIFT, ODID_IDTYPE_SERIAL_NUMBER, "USS-Enterprise"},
                {ODID_UATYPE_NONE, ODID_IDTYPE_NONE, ""}},
    .Location =
        {
            .Status = ODID_STATUS_AIRBORNE,
            .Direction = 0.25f,
            .SpeedHorizontal = 62.f,
            .SpeedVertical = 62.f,
        },
    .System = { .Timestamp = 0x12345678 },
    .BasicIDValid = {1, 0},
    .LocationValid = 1,
    .SystemValid = 1
};

char mac[6] = {0x12, 0x34, 0x56, 0x21, 0x43, 0x65};

TEST(ODID, beacon_transport_build_frame)
{
    uint8_t buffer[256];

    int len = odid_wifi_build_message_pack_beacon_frame(&testData,
                                                        &mac[0],
                                                        "testSSID_0123456789A",
                                                        20,
                                                        100,
                                                        0x55,
                                                        buffer, sizeof(buffer));

    uint8_t expectedBuffer[] = {
        /* Wi-Fi Beacon frame as described in
         * IEEE Std 802.11-2016,
         * par. 9.3.3.2 "Format of management frames"
         */
        /* Frame Control: management, beacon (type 0, subtype 8) */
        0x80, 0x00,
        /* Duration : "If the DA field contains a group address, the
         * duration value is set to 0."
         * par. 9.2.4.3.3: broadcast address is a group address */
        0x00, 0x00,
        /* Address 1: DA: Destination address (final recipient):
         * broadcast */
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        /* Address 2: SA: Source address: = mac */
        0x12, 0x34, 0x56, 0x21, 0x43, 0x65,
        /* Address 3: RA: (immediate recipient):
         * same as address 1 for broadcasts */
        0x12, 0x34, 0x56, 0x21, 0x43, 0x65,
        /* Sequence control: 0 */
        0x00, 0x00,
        /* HT control: 0 or 4 bytes - here 0 ?????????? */
        /* par. 9.3.3.3 "Beacon frame format",
         * table 9-27 "beacon frame body"
         * Timestamp: par. 9.4.1.10:
         * "value of the timing synchronization function (TSF) timer"
         * not testable unless we know which timer to mock */
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        /* Beacon interval: 9.4.1.3:
         * "number of time units (TUs) between target beacon
         * transmission times (TBTTs)" */
        0x64, 0x00,
        /* Capability information: par. 9.4.1.4:
         * use B5 Short preamble, B10 Short slot time
         * "An AP sets the Short Preamble subfield to 1 in transmitted
         *  Beacon, Probe Response, Association Response,
         *  and Reassociation Response frames to indicate that the
         *  use of the short preamble, as described in 16.2.2.3, is
         *  allowed within this BSS"
         --> so we actually do not care  */
        0x20, 0x04,
        /* Service Set Identifier (SSID) par. 9.4.2.2 */
        /* SSID: Element ID: 0 (par. 9.4.2.1 table 9-77) */
        0x00,
        /* SSID Length: "The length of the SSID field is between
         * 0 and 32 octets" */
        /* hard-coded to 20 in this lib (ODID_ID_SIZE) */
        0x14,
        /* SSID */
        't','e','s','t','S','S','I','D','_','0','1','2','3','4','5','6','7','8','9','A',
        /* Supported Rates and BSS Membership Selectors */
        /* Element ID: 1 (par. 9.4.2.1 table 9-77) */
        0x01,
        /* Length: 1 (we assume that only the transmission rate used
         * to broadcast the ODID info are encoded here */
        0x01,
        /* Supported rates: bit 7 | (12) -> 12*500kbps -> 6Mbps */
        0x8C,
        /* The following Elements are not applicable to Beacon
         * frames which are not meant to advertise an actual Access Point */

        /* "Last Vendor Specific One or more vendor-specific elements
         * are optionally present.
          These elements follow all other elements."
          --> that is where the ODID should be stored
         */

        /* Wi-Fi Beacon frame as described in
         * ASTM F3411-22 "Standard Specification for Remote ID and Tracking",
         * par. 5.4.9 "Wi-Fi Beacon Transport Method"
         */
        /* Vendor Specific Tag (IE 221) */
        0xdd,
        /* Length: 8+N*25 with N=3 -> 83(0x53) */
        0x53,
        /* OUI: ASD-STAN */
        0xfa, 0x0b, 0xbc,
        /* Vendor type: Open Drone ID */
        0x0d,
        /* Message counter: hard-coded value for the test */
        0x55,
        /* MsgType 0xF and "protocol version" 2 (cf Table 4, par. 5.4.5.4 page 18) */
        0xf2,
        /* Single msg size */
        0x19,

        /* Buffer index #70: number of messages in pack: N=2
         * the test testData contains a BasicID, a Location and a System message */
        0x03,
        /* ---------------------------------------------------------- */
        /* Buffer index #71: Open Drone ID message #1 */
        /* ---------------------------------------------------------- */
        /* Message type: par 5.4.5.5: 0x00 and version: 0x02 */
        0x02,
        /* TABLE 5 Basic ID Message Details
         * ID Type: serial number (bits [7..4]
         * UA Type: 0x04 (bits [0..3] */
        0x14,
        /* Data: UAS ID within the format of ID Type (padded with nulls)
         * Max. 20 Bytes */
        'U', 'S', 'S', '-', 'E', 'n', 't', 'e', 'r', 'p', 'r', 'i', 's', 'e', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        /* 3 reserved bytes mentioned in table 5 */
        0x00, 0x00, 0x00,
        /* --> message length: 25 bytes */

        /* ---------------------------------------------------------- */
        /* Buffer index #96 (#msg 1 +25): Open Drone ID message #2 */
        /* ---------------------------------------------------------- */
        /* Message type: par 5.4.5.7: 0x01 and "protocol version": 0x02 */
        0x12,
        /* Bits [7..4]=2(airborne), bit 2=0(above TakeOff),
         * Bit 1=0 (direction<180deg), Bit 0=0(speed mult.=0.25) */
        0x20

        /* ... more testData here ... */
        /* Testing the packing itself is done in another test. */
    };

    // Replace the generated timestamp by a test value
    *((uint64_t*)(&buffer[24])) = UINT64_MAX;

    for (int i = 0; i < len; i++)
        printf("@%03d: 0x%02x (%c),\n", i, buffer[i], (buffer[i]>='0'&&buffer[i]<120)?buffer[i]:'.');

    ASSERT_EQ(len, 146);

    for (int i = 0; i < sizeof(expectedBuffer); i++)
            EXPECT_EQ(buffer[i], expectedBuffer[i]) << "failure @index " << i;
}

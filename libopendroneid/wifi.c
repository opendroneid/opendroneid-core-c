/*
Copyright (C) 2020 Simon Wunderlich, Marek Sobe
Copyright (C) 2020 Doodle Labs

SPDX-License-Identifier: Apache-2.0

Open Drone ID C Library

Maintainer:
Simon Wunderlich
sw@simonwunderlich.de
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <byteswap.h>
#include <time.h>

#include "opendroneid.h"

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define cpu_to_le16(x)  (x)
#define cpu_to_le64(x)  (x)
#else
#define cpu_to_le16(x)      (bswap_16(x))
#define cpu_to_le64(x)      (bswap_64(x))
#endif

#define IEEE80211_FCTL_FTYPE          0x000c
#define IEEE80211_FCTL_STYPE          0x00f0

#define IEEE80211_FTYPE_MGMT            0x0000
#define IEEE80211_STYPE_ACTION          0x00D0
#define IEEE80211_STYPE_BEACON          0x0080

/* Neighbor Awareness Networking Specification v3.1 in section 2.8.2
 * The NAN Cluster ID is a MAC address that takes a value from
 * 50-6F-9A-01-00-00 to 50-6F-9A-01-FF-FF and is carried in the A3 field of
 * some of the NAN frames. The NAN Cluster ID is randomly chosen by the device
 * that initiates the NAN Cluster.
 */
uint8_t* get_nan_cluster_id()
{
	static uint8_t cluster_id[6] = { 0x50, 0x6F, 0x9A, 0x01, 0x00, 0x00 };
	static int generated = 0;

	if (generated == 0)
	{
		srand(time(NULL));
		cluster_id[4] = rand() % 256;
		cluster_id[5] = rand() % 256;
		generated = 1;
	}

	return cluster_id;
}

void drone_export_gps_data(ODID_UAS_Data *UAS_Data, char *buf, size_t buf_size)
{
	int len = 0;

#define mprintf(...) {\
	len += snprintf(buf + len, buf_size - len, __VA_ARGS__); \
	if (len >= buf_size)\
		return; \
}

	mprintf("{\n\t\"Version\": \"0.8\",\n\t\"Response\": {\n");

	mprintf("\t\t\"BasicID\": {\n");
	mprintf("\t\t\t\"UAType\": %d,\n", UAS_Data->BasicID.UAType);
	mprintf("\t\t\t\"IDType\": %d,\n", UAS_Data->BasicID.IDType);
	mprintf("\t\t\t\"UASID\": %s\n", UAS_Data->BasicID.UASID);
	mprintf("\t\t},\n");

	mprintf("\t\t\"Location\": {\n");
	mprintf("\t\t\t\"Status\": %d,\n", (int)UAS_Data->Location.Status);
	mprintf("\t\t\t\"Direction\": %f,\n", (double) UAS_Data->Location.Direction);
	mprintf("\t\t\t\"SpeedHorizontal\": %f,\n", (double) UAS_Data->Location.SpeedHorizontal);
	mprintf("\t\t\t\"SpeedVertical\": %f,\n", (double) UAS_Data->Location.SpeedVertical);
	mprintf("\t\t\t\"Latitude\": %f,\n", UAS_Data->Location.Latitude);
	mprintf("\t\t\t\"Longitude\": %f,\n", UAS_Data->Location.Longitude);
	mprintf("\t\t\t\"AltitudeBaro\": %f,\n", (double) UAS_Data->Location.AltitudeBaro);
	mprintf("\t\t\t\"AltitudeGeo\": %f,\n", (double) UAS_Data->Location.AltitudeGeo);
	mprintf("\t\t\t\"HeightType\": %d,\n", UAS_Data->Location.HeightType);
	mprintf("\t\t\t\"Height\": %f,\n", (double) UAS_Data->Location.Height);
	mprintf("\t\t\t\"HorizAccuracy\": %d,\n", UAS_Data->Location.HorizAccuracy);
	mprintf("\t\t\t\"VertAccuracy\": %d,\n", UAS_Data->Location.VertAccuracy);
	mprintf("\t\t\t\"BaroAccuracy\": %d,\n", UAS_Data->Location.BaroAccuracy);
	mprintf("\t\t\t\"SpeedAccuracy\": %d,\n", UAS_Data->Location.SpeedAccuracy);
	mprintf("\t\t\t\"TSAccuracy\": %d,\n", UAS_Data->Location.TSAccuracy);
	mprintf("\t\t\t\"TimeStamp\": %f\n", (double) UAS_Data->Location.TimeStamp);
	mprintf("\t\t},\n");

	mprintf("\t\t\"Authentication\": {\n");
	mprintf("\t\t\t\"AuthType\": %d,\n", UAS_Data->Auth[0].AuthType);
	mprintf("\t\t\t\"PageCount\": %d,\n", UAS_Data->Auth[0].PageCount);
	mprintf("\t\t\t\"Length\": %d,\n", UAS_Data->Auth[0].Length);
	mprintf("\t\t\t\"Timestamp\": %d,\n", UAS_Data->Auth[0].Timestamp);
	for (int i = 0; i < UAS_Data->Auth[0].PageCount; i++) {
		mprintf("\t\t\t\"AuthData Page %d\": %s\n", i, UAS_Data->Auth[i].AuthData);
	}
	mprintf("\t\t},\n");

	mprintf("\t\t\"SelfID\": {\n");
	mprintf("\t\t\t\"Description Type\": %d\n", UAS_Data->SelfID.DescType);
	mprintf("\t\t\t\"Description\": %s\n", UAS_Data->SelfID.Desc);
	mprintf("\t\t},\n");

	mprintf("\t\t\"Operator\": {\n");
	mprintf("\t\t\t\"OperatorLocationType\": %d,\n", UAS_Data->System.OperatorLocationType);
	mprintf("\t\t\t\"ClassificationType\": %d,\n", UAS_Data->System.ClassificationType);
	mprintf("\t\t\t\"OperatorLatitude\": %f,\n", UAS_Data->System.OperatorLatitude);
	mprintf("\t\t\t\"OperatorLongitude\": %f,\n", UAS_Data->System.OperatorLongitude);
	mprintf("\t\t\t\"AreaCount\": %d,\n", UAS_Data->System.AreaCount);
	mprintf("\t\t\t\"AreaRadius\": %d,\n", UAS_Data->System.AreaRadius);
	mprintf("\t\t\t\"AreaCeiling\": %f\n", (double) UAS_Data->System.AreaCeiling);
	mprintf("\t\t\t\"AreaFloor\": %f\n", (double) UAS_Data->System.AreaFloor);
	mprintf("\t\t\t\"CategoryEU\": %d,\n", UAS_Data->System.CategoryEU);
	mprintf("\t\t\t\"ClassEU\": %d,\n", UAS_Data->System.ClassEU);
	mprintf("\t\t}\n");

	mprintf("\t\t\"OperatorID\": {\n");
	mprintf("\t\t\t\"OperatorIdType\": %d,\n", UAS_Data->OperatorID.OperatorIdType);
	mprintf("\t\t\t\"OperatorId\": \"%s\",\n", UAS_Data->OperatorID.OperatorId);
	mprintf("\t\t},\n");

	mprintf("\t}\n}");

	return;
}

int odid_message_build_pack(ODID_UAS_Data *UAS_Data, void *pack, size_t buflen)
{
	ODID_MessagePack_data msg_pack;
	ODID_MessagePack_encoded *msg_pack_enc;
	size_t len = 0;

	/* check if there is enough space for the message pack. */
	if (sizeof(*msg_pack_enc) > buflen)
		return -ENOMEM;

	/* create a complete message pack */
	msg_pack.SingleMessageSize = ODID_MESSAGE_SIZE;
	msg_pack.MsgPackSize = 10;
	encodeBasicIDMessage((void *)&msg_pack.Messages[0], &UAS_Data->BasicID);
	encodeLocationMessage((void *)&msg_pack.Messages[1], &UAS_Data->Location);
	encodeAuthMessage((void *)&msg_pack.Messages[2], &UAS_Data->Auth[0]);
	encodeAuthMessage((void *)&msg_pack.Messages[3], &UAS_Data->Auth[1]);
	encodeAuthMessage((void *)&msg_pack.Messages[4], &UAS_Data->Auth[2]);
	encodeAuthMessage((void *)&msg_pack.Messages[5], &UAS_Data->Auth[3]);
	encodeAuthMessage((void *)&msg_pack.Messages[6], &UAS_Data->Auth[4]);
	encodeSelfIDMessage((void *)&msg_pack.Messages[7], &UAS_Data->SelfID);
	encodeSystemMessage((void *)&msg_pack.Messages[8], &UAS_Data->System);
	encodeOperatorIDMessage((void *)&msg_pack.Messages[9], &UAS_Data->OperatorID);

	msg_pack_enc = (ODID_MessagePack_encoded *) pack;
	if (encodeMessagePack(msg_pack_enc, &msg_pack) != ODID_SUCCESS)
		return -1;
	len += sizeof(*msg_pack_enc);

	return len;
}

int odid_wifi_build_nan_sync_beacon_frame(char *mac, uint8_t *buf, size_t buf_size)
{
	/* Broadcast address */
	uint8_t target_addr[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	uint8_t wifi_alliance_oui[3] = { 0x50, 0x6F, 0x9A };
	/* "org.opendroneid.remoteid" hash */
	uint8_t service_id[6] = { 0x88, 0x69, 0x19, 0x9D, 0x92, 0x09 };
	uint8_t *cluster_id = get_nan_cluster_id();
	struct ieee80211_mgmt *mgmt;
	struct ieee80211_beacon *beacon;
	struct nan_master_indication_attribute *master_indication_attr;
	struct nan_cluster_attribute *cluster_attr;
	struct nan_service_id_list_attribute *nsila;
	struct timespec ts;
	long len = 0;

	/* IEEE 802.11 Management Header */
	if (len + sizeof(*mgmt) > buf_size)
		return -ENOMEM;

	mgmt = (struct ieee80211_mgmt *)(buf + len);
	memset(mgmt, 0, sizeof(*mgmt));
	mgmt->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_BEACON);
	mgmt->duration = 0;
	memcpy(mgmt->da, target_addr, sizeof(mgmt->da));
	memcpy(mgmt->sa, mac, sizeof(mgmt->sa));
	memcpy(mgmt->bssid, cluster_id, sizeof(mgmt->bssid));
	mgmt->seq_ctrl = 0;
	len += sizeof(*mgmt);

	/* Beacon */
	if (len + sizeof(*beacon) > buf_size)
		return -ENOMEM;

	beacon = (struct ieee80211_beacon *)(buf + len);
	memset(beacon, 0, sizeof(*beacon));
	clock_gettime(CLOCK_MONOTONIC, &ts);
	uint64_t mono = (uint64_t)(ts.tv_sec * 1e6 + ts.tv_nsec * 1e-3);
	beacon->timestamp = cpu_to_le64(mono);
	beacon->beacon_interval = cpu_to_le16(0x0200);
	beacon->capability = cpu_to_le16(0x0420);
	beacon->element_id = 0xDD;
	beacon->length = 0x22;
	memcpy(beacon->oui, wifi_alliance_oui, sizeof(beacon->oui));
	beacon->oui_type = 0x13;
	len += sizeof(*beacon);

	/* NAN Master Indication attribute */
	if (len + sizeof(*master_indication_attr) > buf_size)
		return -ENOMEM;

	master_indication_attr = (struct nan_master_indication_attribute *)(buf + len);
	memset(master_indication_attr, 0, sizeof(*master_indication_attr));
	master_indication_attr->header.attribute_id = 0x00;
	master_indication_attr->header.length = cpu_to_le16(0x0002);
	/* Information that is used to indicate a NAN Device’s preference to serve
	 * as the role of Master, with a larger value indicating a higher
	 * preference. Values 1 and 255 are used for testing purposes only.
	 */
	master_indication_attr->master_preference = 0xFE;
	/* Random factor value 0xEA is recommended by the European Standard */
	master_indication_attr->random_factor = 0xEA;
	len += sizeof(*master_indication_attr);

	/* NAN Cluster attribute */
	if (len + sizeof(*cluster_attr) > buf_size)
		return -ENOMEM;

	cluster_attr = (struct nan_cluster_attribute *)(buf + len);
	memset(cluster_attr, 0, sizeof(*cluster_attr));
	cluster_attr->header.attribute_id = 0x1;
	cluster_attr->header.length = cpu_to_le16(0x000D);
	memcpy(cluster_attr->device_mac, mac, sizeof(cluster_attr->device_mac));
	cluster_attr->random_factor = 0xEA;
	cluster_attr->master_preference = 0xFE;
	cluster_attr->hop_count_to_anchor_master = 0x00;
	memset(cluster_attr->anchor_master_beacon_transmission_time, 0, sizeof(cluster_attr->anchor_master_beacon_transmission_time));
	len += sizeof(*cluster_attr);

	/* NAN attributes */
	if (len + sizeof(*nsila) > buf_size)
		return -ENOMEM;

	nsila = (struct nan_service_id_list_attribute *)(buf + len);
	memset(nsila, 0, sizeof(*nsila));
	nsila->header.attribute_id = 0x02;
	nsila->header.length = cpu_to_le16(0x0006);
	memcpy(nsila->service_id, service_id, sizeof(service_id));
	len += sizeof(*nsila);

	return len;
}

int odid_wifi_build_message_pack_nan_action_frame(ODID_UAS_Data *UAS_Data, char *mac,
						  uint8_t send_counter,
						  uint8_t *buf, size_t buf_size)
{
	/* Neighbor Awareness Networking Specification v3.0 in section 2.8.1
	 * NAN Network ID calls for the destination mac to be 51-6F-9A-01-00-00 */
	uint8_t target_addr[6] = { 0x51, 0x6F, 0x9A, 0x01, 0x00, 0x00 };
	/* "org.opendroneid.remoteid" hash */
	uint8_t service_id[6] = { 0x88, 0x69, 0x19, 0x9D, 0x92, 0x09 };
	uint8_t wifi_alliance_oui[3] = { 0x50, 0x6F, 0x9A };
	uint8_t *cluster_id = get_nan_cluster_id();
	struct ieee80211_mgmt *mgmt;
	struct nan_service_discovery *nsd;
	struct nan_service_descriptor_attribute *nsda;
	struct nan_service_descriptor_extension_attribute *nsdea;
	struct ODID_service_info *si;
	long ret, len = 0;

	/* IEEE 802.11 Management Header */
	if (len + sizeof(*mgmt) > buf_size)
		return -ENOMEM;

	mgmt = (struct ieee80211_mgmt *)(buf + len);
	memset(mgmt, 0, sizeof(*mgmt));
	mgmt->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_ACTION);
	mgmt->duration = 0;
	memcpy(mgmt->sa, mac, sizeof(mgmt->sa));
	memcpy(mgmt->da, target_addr, sizeof(mgmt->da));
	memcpy(mgmt->bssid, cluster_id, sizeof(mgmt->bssid));
	mgmt->seq_ctrl = 0;

	len += sizeof(*mgmt);

	/* NAN Service Discovery header */
	if (len + sizeof(*nsd) > buf_size)
		return -ENOMEM;

	nsd = (struct nan_service_discovery *)(buf + len);
	memset(nsd, 0, sizeof(*nsd));
	nsd->category = 0x04;			/* IEEE 802.11 Public Action frame */
	nsd->action_code = 0x09;		/* IEEE 802.11 Public Action frame Vendor Specific*/
	memcpy(nsd->oui, wifi_alliance_oui, sizeof(nsd->oui));
	nsd->oui_type = 0x13;			/* Identify Type and version of the NAN */
	len += sizeof(*nsd);

	/* NAN Attribute for Service Descriptor header */
	if (len + sizeof(*nsda) > buf_size)
		return -ENOMEM;

	nsda = (struct nan_service_descriptor_attribute *)(buf + len);
	nsda->attribute_id = 0x3;		/* Service Descriptor Attribute type */
	memcpy(nsda->service_id, service_id, sizeof(service_id));
	/* always 1 */
	nsda->instance_id = 0x01;		/* always 1 */
	nsda->requestor_instance_id = 0x00;	/* from triggering frame */
	nsda->service_control = 0x10;		/* follow up */
	len += sizeof(*nsda);

	/* ODID Service Info Attribute header */
	if (len + sizeof(*si) > buf_size)
		return -ENOMEM;

	si = (struct ODID_service_info *)(buf + len);
	memset(si, 0, sizeof(*si));
	si->message_counter = send_counter;
	len += sizeof(*si);

	ret = odid_message_build_pack(UAS_Data, buf + len, buf_size - len);
	if (ret < 0)
		return ret;
	len += ret;

	/* set the lengths according to the message pack lengths */
	nsda->service_info_length = sizeof(*si) + ret;
	nsda->length = cpu_to_le16(sizeof(*nsda) - sizeof(struct nan_attribute_header) + nsda->service_info_length);

	/* NAN Attribute for Service Descriptor extension header */
	if (len + sizeof(*nsdea) > buf_size)
		return -ENOMEM;

	nsdea = (struct nan_service_descriptor_extension_attribute *)(buf + len);
	nsdea->header.attribute_id = 0xE;
	nsdea->header.length = cpu_to_le16(0x0004);
	nsdea->instance_id = 0x01;
	nsdea->control = cpu_to_le16(0x0200);
	nsdea->service_update_indicator = send_counter;
	len += sizeof(*nsdea);

	return len;
}

int odid_message_process_pack(ODID_UAS_Data *UAS_Data, uint8_t *pack, size_t buflen)
{
	ODID_MessagePack_encoded *msg_pack_enc;

	if (sizeof(*msg_pack_enc) > buflen)
		return -ENOMEM;

	msg_pack_enc = (ODID_MessagePack_encoded *) pack;
	if (decodeMessagePack(UAS_Data, msg_pack_enc) != ODID_SUCCESS)
		return -1;

	return 0;
}

int odid_wifi_receive_message_pack_nan_action_frame(ODID_UAS_Data *UAS_Data,
						    char *mac, uint8_t *buf, size_t buf_size)
{
	struct ieee80211_mgmt *mgmt;
	struct nan_service_discovery *nsd;
	struct nan_service_descriptor_attribute *nsda;
	struct ODID_service_info *si;
	uint8_t wifi_alliance_oui[3] = { 0x50, 0x6F, 0x9A };
	uint8_t service_id[6] = { 0x88, 0x69, 0x19, 0x9D, 0x92, 0x09 };
	int ret, len;

	/* basic header size check */
	if (sizeof(*mgmt) + sizeof(*nsd) + sizeof(*nsda) + sizeof(*si) > buf_size)
		return -EINVAL;
	len = 0;

	/* check for frame type and correct sender address */
	mgmt = (struct ieee80211_mgmt *)(buf + len);
	if ((mgmt->frame_control & cpu_to_le16(IEEE80211_FCTL_FTYPE | IEEE80211_FCTL_STYPE)) !=
	    cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_ACTION))
		return -EINVAL;

	memcpy(mac, mgmt->sa, sizeof(mgmt->sa));

	len += sizeof(*mgmt);

	/* check NAN service discovery frame fields */
	nsd = (struct nan_service_discovery *)(buf + len);
	if (nsd->category != 0x04)
		return -EINVAL;
	if (nsd->action_code != 0x09)
		return -EINVAL;
	if (nsd->oui_type != 0x13)
		return -EINVAL;
	if (memcmp(nsd->oui, wifi_alliance_oui, sizeof(wifi_alliance_oui)) != 0)
		return -EINVAL;
	len += sizeof(*nsd);

	/* check NAN service descriptor attribute fields */
	nsda = (struct nan_service_descriptor_attribute *)(buf + len);
	if (nsda->attribute_id != 0x3)
		return -EINVAL;
	if (memcmp(nsda->service_id, service_id, sizeof(service_id)) != 0)
		return -EINVAL;
	if (nsda->instance_id != 0x01)
		return -EINVAL;
	if (nsda->service_control != 0x10)
		return -EINVAL;
	if (len + sizeof(*nsda) + nsda->service_info_length != buf_size)
		return -EINVAL;

	len += sizeof(*nsda);

	si = (struct ODID_service_info *)(buf + len);
	len += sizeof(*si);

	ret = odid_message_process_pack(UAS_Data, buf + len, buf_size - len);
	if (ret < 0) {
		return -1;
	}

	return 0;
}

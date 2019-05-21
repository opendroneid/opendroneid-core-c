/*
Copyright (C) 2019 Intel Corporation

SPDX-License-Identifier: Apache-2.0

Open Drone ID C Library

Maintainer:
Gabriel Cox
gabriel.c.cox@intel.com
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <byteswap.h>

#include "opendroneid.h"

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define cpu_to_le16(x)  (x)
#else
#define cpu_to_le16(x)      (bswap_16(x))
#endif

#define IEEE80211_FCTL_FTYPE          0x000c
#define IEEE80211_FCTL_STYPE          0x00f0

#define IEEE80211_FTYPE_MGMT            0x0000
#define IEEE80211_STYPE_ACTION          0x00D0


char *drone_export_gps_data(ODID_UAS_Data *UAS_Data)
{
	int len = 0, total_len = 8192;
	char *drone_str;

	drone_str = malloc(total_len);
	if (!drone_str)
		return NULL;

#define mprintf(...) {\
	if (total_len - len < 2048) { \
		total_len *= 2; \
		drone_str = realloc(drone_str, total_len); \
		if (!drone_str) \
			return NULL; \
	} \
	len += snprintf(drone_str + len, total_len - len, __VA_ARGS__); \
	if (len > total_len) { \
		free(drone_str); \
		return NULL; \
	} \
}
	/* build json object from UAS_Data */
	/*
	{
		"Version": "x.x",
		"Response": {
			"BasicID": {
				"UAType": <>,
				"IDType": <>,
				"UASID": <>
			},
			"Location": {
				"Status": <>,
				"Direction": <>,
				"SpeedHorizontal": <>,
				"SpeedVertical": <>,
				"Latitude": <>,
				"Longitude": <>,
				"AltitudeBaro": <>,
				"AltitudeGeo": <>,
				"Height": <>,
				"HAccuracy": <>,
				"VAccuracy": <>,
				"SpAccuracy": <>,
				"TSAccuracy": <>,
				"TimeStamp": <>
			},
			"Authentication": {
				"AuthType": <>,
				"AuthToken": <>
			},
			"SelfID": {
				"Name": <>,
				"Description": <>
			},
			"Operator": {
				"LocationSource": <>,
				"Latitude": <>,
				"Longitude": <>,
				"AreaCount": <>,
				"AreaRadius": <>,
				"AreaCeiling": <>
			}
		}
	}
	*/

	mprintf("{\n\t\"Version\": \"0.0\",\n\t\"Response\": {\n");

	mprintf("\t\t\"BasicID\": {\n");
	mprintf("\t\t\t\"UAType\": %i,\n", UAS_Data->BasicID.UAType);
	mprintf("\t\t\t\"IDType\": %i,\n", UAS_Data->BasicID.IDType);
	mprintf("\t\t\t\"UASID\": %s\n", UAS_Data->BasicID.UASID);
	mprintf("\t\t},\n");

	mprintf("\t\t\"Location\": {\n");
	mprintf("\t\t\t\"Status\": %d,\n", (int)UAS_Data->Location.Status);
	mprintf("\t\t\t\"Direction\": %f,\n", UAS_Data->Location.Direction);
	mprintf("\t\t\t\"SpeedHorizontal\": %f,\n", UAS_Data->Location.SpeedHorizontal);
	mprintf("\t\t\t\"SpeedVertical\": %f,\n", UAS_Data->Location.SpeedVertical);
	mprintf("\t\t\t\"Latitude\": %f,\n", UAS_Data->Location.Latitude);
	mprintf("\t\t\t\"Longitude\": %f,\n", UAS_Data->Location.Longitude);
	mprintf("\t\t\t\"AltitudeBaro\": %f,\n", UAS_Data->Location.AltitudeBaro);
	mprintf("\t\t\t\"AltitudeGeo\": %f,\n", UAS_Data->Location.AltitudeGeo);
	mprintf("\t\t\t\"Height\": %f,\n", UAS_Data->Location.Height);
	mprintf("\t\t\t\"HorizAccuracy\": %d,\n", UAS_Data->Location.HorizAccuracy);
	mprintf("\t\t\t\"VertAccuracy\": %d,\n", UAS_Data->Location.VertAccuracy);
	mprintf("\t\t\t\"SpeedAccuracy\": %d,\n", UAS_Data->Location.SpeedAccuracy);
	mprintf("\t\t\t\"TSAccuracy\": %d,\n", UAS_Data->Location.TSAccuracy);
	mprintf("\t\t\t\"TimeStamp\": %f\n", UAS_Data->Location.TimeStamp);
	mprintf("\t\t},\n");

	mprintf("\t\t\"Authentication\": {\n");
	mprintf("\t\t\t\"AuthType\": %i,\n", UAS_Data->Auth[0].AuthType);
	mprintf("\t\t\t\"AuthToken\": %s\n", UAS_Data->Auth[0].AuthData);
	mprintf("\t\t},\n");

	mprintf("\t\t\"SelfID\": {\n");
	mprintf("\t\t\t\"Name\": \"string\",\n");
	mprintf("\t\t\t\"Description\": %s\n", UAS_Data->SelfID.Desc);
	mprintf("\t\t},\n");

	mprintf("\t\t\"Operator\": {\n");
	mprintf("\t\t\t\"LocationSource\": %i,\n", UAS_Data->System.LocationSource);
	mprintf("\t\t\t\"OperatorLatitude\": %f,\n", UAS_Data->System.OperatorLatitude);
	mprintf("\t\t\t\"OperatorLongitude\": %f,\n", UAS_Data->System.OperatorLongitude);
	mprintf("\t\t\t\"AreaCount\": %i,\n", UAS_Data->System.AreaCount);
	mprintf("\t\t\t\"AreaRadius\": %i,\n", UAS_Data->System.AreaRadius);
	mprintf("\t\t\t\"AreaCeiling\": %f\n", UAS_Data->System.AreaCeiling);
	mprintf("\t\t}\n");

	mprintf("\t}\n}");

	return drone_str;
}

int odid_message_encode_pack(ODID_UAS_Data *UAS_Data, void *pack, size_t buflen)
{
	ODID_MessagePack_encoded *outPack;
	size_t len = 0;

	/* check if there is enough space for the header. */
	if (sizeof(*outPack) > buflen)
		return -ENOMEM;

	/* TODO: flexibly set optional fields as available */

	outPack = (ODID_MessagePack_encoded *) pack;
	outPack->ProtoVersion = 0;
	outPack->MessageType = 0;
	outPack->SingleMessageSize = ODID_MESSAGE_SIZE;
	outPack->MsgPackSize = 5;
	len += sizeof(*outPack);

	if (len + (outPack->MsgPackSize * ODID_MESSAGE_SIZE) > buflen)
		return -ENOMEM;

	encodeBasicIDMessage((void *)&outPack->Messages[0], &UAS_Data->BasicID);
	encodeLocationMessage((void *)&outPack->Messages[1], &UAS_Data->Location);
	encodeAuthMessage((void *)&outPack->Messages[2], &UAS_Data->Auth[0]);
	encodeSelfIDMessage((void *)&outPack->Messages[3], &UAS_Data->SelfID);
	encodeSystemMessage((void *)&outPack->Messages[4], &UAS_Data->System);
	len += ODID_MESSAGE_SIZE * outPack->MsgPackSize;

	return len;
}

int odid_wifi_build_message_pack_nan_action_frame(ODID_UAS_Data *UAS_Data, char *mac,
						  uint8_t send_counter,
				     		  uint8_t *buf, size_t buf_size)
{
	uint8_t broadcast_addr[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	/* "org.opendroneid.remoteid" hash */
	uint8_t service_id[6] = { 0x88, 0x69, 0x19, 0x9D, 0x92, 0x09 };
	uint8_t wifi_alliance_oui[3] = { 0x50, 0x6F, 0x9A };
	struct ieee80211_mgmt *mgmt;
	struct nan_service_discovery *nsd;
	struct nan_service_descriptor_attribute *nsda;
	struct ODID_service_info *si;
	int ret, len = 0;

	/* IEEE 802.11 Management Header */
	if (len + sizeof(*mgmt) > buf_size)
		return -ENOMEM;

	mgmt = (struct ieee80211_mgmt *)(buf + len);
	memset(mgmt, 0, sizeof(*mgmt));
	mgmt->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_ACTION);
	mgmt->duration = 0;
	memcpy(mgmt->sa, mac, sizeof(mgmt->sa));
	memcpy(mgmt->da, broadcast_addr, sizeof(mgmt->da));
	memcpy(mgmt->bssid, mac, sizeof(mgmt->bssid));
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

	ret = odid_message_encode_pack(UAS_Data, buf + len, buf_size - len);
	if (ret < 0)
		return ret;
	len += ret;

	/* set the lengths according to the message pack lengths */
	nsda->service_info_length = sizeof(*si) + ret;
	nsda->length = cpu_to_le16(sizeof(*nsda) - sizeof(struct nan_attribute_header) + nsda->service_info_length);

	return len;
}

int odid_message_decode_pack(ODID_UAS_Data *UAS_Data, uint8_t *pack, size_t buflen)
{
	ODID_MessagePack_encoded *inPack;

	if (sizeof(*inPack) > buflen)
		return -ENOMEM;

	inPack = (ODID_MessagePack_encoded *) pack;
	if (inPack->MsgPackSize != 5)
		return -1;

	decodeBasicIDMessage(&UAS_Data->BasicID, (void *)&inPack->Messages[0]);
	decodeLocationMessage(&UAS_Data->Location, (void *)&inPack->Messages[1]);
	decodeAuthMessage(&UAS_Data->Auth[0], (void *)&inPack->Messages[2]);
	decodeSelfIDMessage(&UAS_Data->SelfID, (void *)&inPack->Messages[3]);
	decodeSystemMessage(&UAS_Data->System, (void *)&inPack->Messages[4]);

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

	ret = odid_message_decode_pack(UAS_Data, buf + len, buf_size - len);
	if (ret < 0) {
		return -1;
	}

	return 0;
}

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/un.h>
#include <netinet/ip.h>
#include <linux/types.h>
#include <linux/if_ether.h>

#include <netlink/attr.h>
#include <netlink/genl/ctrl.h>
#include <netlink/genl/genl.h>
#include <netlink/handlers.h>
#include <netlink/msg.h>
#include <netlink/netlink.h>
#include <netlink/socket.h>
#include <linux/nl80211.h>

#include <gps.h>
#include <math.h>

#include <opendroneid.h>

struct global {
	char server[1024];
	char port[16];
	char wlan_iface[16];
	char mac[6];
	uint8_t send_counter;
	int refresh_rate;
	int test_json;
	int set_ssid_string;
};

void usage(char *name)
{
	fprintf(stderr,"%s\n", name);
	fprintf(stderr,"\t-h\tGPSD Host (default: 127.0.0.1)\n");
	fprintf(stderr,"\t-p\tGPSD Port (default: "DEFAULT_GPSD_PORT"\n");
	fprintf(stderr,"\t-w\twlan interface (default: wlan0)\n");
	fprintf(stderr,"\t-i\tDrone ID (string)\n");
	fprintf(stderr,"\t-t\tDrone type (number)\n");
	fprintf(stderr,"\t-r\tRefresh rate of beacon sends, in seconds\n");
	fprintf(stderr,"\t-T\tTest JSON Input/Output (debug)\n");
	fprintf(stderr,"\t-S\tadditionally set an SSID string (debug/legacy)\n");
}

static int nl80211_id = -1;

static struct nl_sock *nl80211_socket_create(void)
{
	struct nl_sock *nl_sock = NULL;

	nl_sock = nl_socket_alloc();
	if (!nl_sock) {
		fprintf(stderr, "Failed to create netlink socket\n");
		goto err;
	}

	if (genl_connect(nl_sock)) {
		fprintf(stderr, "Failed to connect to generic netlink\n");
		goto err;
	}

	nl80211_id = genl_ctrl_resolve(nl_sock, "nl80211");
	if (nl80211_id < 0) {
		fprintf(stderr, "nl80211 not found\n");
		goto err;
	}

	return nl_sock;

err:
	nl_socket_free(nl_sock);
	return NULL;
}

int send_nl80211_action(struct nl_sock *nl_sock, int if_index, void *action, size_t len)
{
	struct nl_msg *msg = NULL;
	struct nl_cb *s_cb;
	int ret = -1;

	s_cb = nl_cb_alloc(NL_CB_DEBUG);
	if (!s_cb) {
		fprintf(stderr, "\n");
		goto nla_put_failure;
	}
	nl_socket_set_cb(nl_sock, s_cb);

	msg = nlmsg_alloc();
	if (!msg) {
		fprintf(stderr, "Could not create netlink message\n");
		goto nla_put_failure;
	}

	genlmsg_put(msg, 0, 0, nl80211_id, 0, 0, NL80211_CMD_FRAME, 0);
	NLA_PUT_U32(msg, NL80211_ATTR_IFINDEX, if_index);
	NLA_PUT(msg, NL80211_ATTR_FRAME, len, action);
	NLA_PUT_FLAG(msg, NL80211_ATTR_DONT_WAIT_FOR_ACK);
	ret = nl_send_auto_complete(nl_sock, msg);
	if (ret < 0)
		goto nla_put_failure;

	nl_wait_for_ack(nl_sock);
	nlmsg_free(msg);
	return 0;

nla_put_failure:
	nl_cb_put(s_cb);
	nlmsg_free(msg);
	return ret;

}

int read_arguments(int argc, char *argv[], ODID_UAS_Data *drone, struct global *global)
{
	int opt;

	strncpy(global->server, "127.0.0.1", sizeof(global->server));
	strncpy(global->port, (char *)DEFAULT_GPSD_PORT, sizeof(global->port));
	strncpy(global->wlan_iface, "wlan0", sizeof(global->wlan_iface));
	strncpy(drone->BasicID.UASID, "1", sizeof(drone->BasicID.UASID));
	drone->BasicID.IDType = ODID_IDTYPE_SERIAL_NUMBER;
	drone->BasicID.UAType = ODID_UATYPE_FREE_BALLOON; /* balloon */
	global->refresh_rate = 1;

	while((opt = getopt(argc, argv, "hp:H:i:t:r:TSw:")) != -1) {
		switch (opt) {
		case 'h':
			usage(argv[0]);
			exit(0);
			break;
		case 'p':
			strncpy(global->port, optarg, sizeof(global->port));
			break;
		case 'H':
			strncpy(global->server, optarg, sizeof(global->server));
			break;
		case 'w':
			strncpy(global->wlan_iface, optarg, sizeof(global->wlan_iface));
			break;

		case 'i':
			strncpy(drone->BasicID.UASID, optarg, sizeof(drone->BasicID.UASID));
			break;
		case 't':
			/* TODO: verify */
			drone->BasicID.UAType = atoi(optarg);
			break;
		case 'r':
			global->refresh_rate = atoi(optarg);
			break;
		case 'T':
			global->test_json = 1;
			break;
		case 'S':
			global->set_ssid_string = 1;
			break;
		default:
			fprintf(stderr, "unknown option\n");
			break;
		}
	}

	return 0;
}

/**
 * drone_adopt_gps_data - adopt GPS data into the drone status info
 * @gpsdata: gps data from gpsd
 * @drone: general drone status information
 */
static void drone_adopt_gps_data(ODID_UAS_Data *drone,
				 struct gps_data_t *gpsdata)
{
	uint64_t time_in_tenth;

	/*
	*	ALL READOUTS FROM GPSD
	*/
	printf("\nGPS:\tmode %d\n", gpsdata->fix.mode);

	/* Latitude/Longitude */
	drone->Location.Latitude = gpsdata->fix.latitude;
	drone->Location.Longitude = gpsdata->fix.longitude;
	drone->Location.HorizAccuracy = gpsdata->fix.epy > gpsdata->fix.epx ? gpsdata->fix.epy : gpsdata->fix.epx;

	/* Altitude */
	drone->Location.AltitudeGeo = gpsdata->fix.altitude;
	drone->Location.VertAccuracy = gpsdata->fix.epv;

	/* Horizontal movement */
	drone->Location.Direction = gpsdata->fix.track;
	drone->Location.SpeedAccuracy = gpsdata->fix.eps;
	drone->Location.SpeedHorizontal = gpsdata->fix.speed;

	/* Vertical movement */
	drone->Location.SpeedVertical = gpsdata->fix.climb;
	drone->Location.SpeedAccuracy = gpsdata->fix.epc;

	/* Time */
	time_in_tenth = gpsdata->fix.time * 10;
	drone->Location.TimeStamp = (float)((time_in_tenth % 36000) / 10);
	drone->Location.TSAccuracy = (float)gpsdata->fix.ept;

	printf("drone:\n\t"
		"TimeStamp: %f, time since last hour (100ms): %ld, TSAccuracy: %d\n\t"
		"Latitude: %f, Longitude: %f\n\t"
		"SpeedHorizontal: %f, SpeedVertical: %f\n",
		drone->Location.TimeStamp, (uint64_t)(drone->Location.TimeStamp*10)%36000, drone->Location.TSAccuracy,
		drone->Location.Latitude, drone->Location.Longitude,
		drone->Location.SpeedHorizontal, 	drone->Location.SpeedVertical
	);
}

static void drone_set_ssid(ODID_UAS_Data *drone, struct global *global)
{
	char ssid[33];
	char cmd[256];
	int ret;

	ret = snprintf(ssid, sizeof(ssid), "%7s:%2.5f:%3.5f:%3d",
			drone->BasicID.UASID,
			drone->Location.Latitude,
			drone->Location.Longitude,
			(int) drone->Location.AltitudeGeo);

	if (ret < 0)
		return;

	printf("set SSID to %s, %d\n", ssid, (int)strlen(ssid));

	system("hostapd_cli DISABLE");
	snprintf(cmd, sizeof(cmd), "hostapd_cli SET ssid \"%s\"", ssid);
	system(cmd);
	system("hostapd_cli ENABLE");
}


/**
 * drone_test_receive_data - receive and process drone information
 */
static void drone_test_receive_data(uint8_t *buf, size_t buf_size)
{
	ODID_UAS_Data rcvd;
	char mac[6];
	int ret;
	FILE *fp;
	char filename[] = "rcvd_drone.json";
	char *drone_str;

	ret = odid_wifi_receive_message_pack_nan_action_frame(&rcvd, mac, buf, buf_size);
	if (ret < 0)
		return;

	drone_str = drone_export_gps_data(&rcvd);
	if (drone_str != NULL) {
		fp = fopen(filename, "w");
		if (fp != NULL) {
			fputs(drone_str, fp);
			fclose(fp);
		}
	}
	free(drone_str);
}

/**
 * drone_send_data - send information about the drone out
 * @drone: general drone status information
 */
static void drone_send_data(ODID_UAS_Data *drone, struct global *global, struct nl_sock *nl_sock, int if_index)
{
	uint8_t frame_buf[1024];
	int ret;
	FILE *fp;
	char filename[] = "drone.json";
	char *drone_str;

	if (global->set_ssid_string)
		drone_set_ssid(drone, global);

	if (global->test_json) {
		drone_str = drone_export_gps_data(drone);
		if (drone_str != NULL) {
			fp = fopen(filename, "w");
			if (fp != NULL) {
				fputs(drone_str, fp);
				fclose(fp);
			}
		}
		free(drone_str);
	}

	ret = odid_wifi_build_message_pack_nan_action_frame(drone, global->mac, global->send_counter++, frame_buf, sizeof(frame_buf));
	if (ret < 0) {
		fprintf(stderr, "%s: odid_wifi_build_message_pack_nan_action_frame failed: %d (%s)", __func__, ret, strerror(ret));
		return;
	}

	if (global->test_json)
		drone_test_receive_data(frame_buf, (uint8_t)ret);

	ret = send_nl80211_action(nl_sock, if_index, frame_buf, ret);
	if (ret < 0) {
		fprintf(stderr, "%s: send_nl80211_action failed: %d (%s)", __func__, ret, strerror(ret));
		return;
	}
}

static int get_device_mac(const char *iface, char *mac, int *if_index)
{
	struct ifreq ifr;
	int sock;

	*if_index = if_nametoindex(iface);
	if (*if_index == 0)
		return -1;

	sock = socket(PF_INET, SOCK_STREAM, 0);
	if (sock < 0)
		return sock;

	strncpy(ifr.ifr_name, iface, sizeof(ifr.ifr_name)-1);
	ifr.ifr_name[sizeof(ifr.ifr_name) - 1] = '\0';

	if (ioctl(sock, SIOCGIFHWADDR, &ifr)== -1) {
		return -1;
	}
	close(sock);

	memcpy(mac, &ifr.ifr_hwaddr.sa_data, 6);

	return 0;
}

int main(int argc, char *argv[])
{
	ODID_UAS_Data drone;
	struct global global;
	struct gps_data_t gpsdata;
	struct nl_sock *nl_sock = NULL;
	int if_index;
	int ret, errno;

	memset(&drone, 0, sizeof(drone));
	memset(&global, 0, sizeof(global));

	if (read_arguments(argc, argv, &drone, &global) < 0) {
		usage(argv[0]);
		return -1;
	}

	if (get_device_mac(global.wlan_iface, global.mac, &if_index) < 0) {
		fprintf(stderr, "%s: Couldn't acquire wlan0 address\n", argv[0]);

		return -1;
	}

	nl_sock = nl80211_socket_create();
	if (!nl_sock) {
		fprintf(stderr, "%s: Couldn't open nl80211 socket\n", argv[0]);
		goto out;
	}

	if (gps_open(global.server, global.port, &gpsdata) != 0) {
		fprintf(stderr, "%s: gpsd error: %d, %s\n", argv[0],
			errno, gps_errstr(errno));
		goto out;
	}

	gps_stream(&gpsdata, WATCH_ENABLE | WATCH_JSON, NULL);

	while (1) {
		sleep(global.refresh_rate);

		/* read as much as we can using gps_read() */
#if GPSD_API_MAJOR_VERSION >= 7
		while ((ret = gps_read(&gpsdata, NULL, 0)) > 0);
#else
		while ((ret = gps_read(&gpsdata)) > 0);
#endif
		if (ret < 0) {
			fprintf(stderr, "%s: gpsd_read error: %d, %s\n", argv[0],
				errno, gps_errstr(errno));
		}

		drone_adopt_gps_data(&drone, &gpsdata);
		drone_send_data(&drone, &global, nl_sock, if_index);
	}

	gps_stream(&gpsdata, WATCH_DISABLE, NULL);
	gps_close(&gpsdata);
out:
	nl_socket_free(nl_sock);

	return 0;
}

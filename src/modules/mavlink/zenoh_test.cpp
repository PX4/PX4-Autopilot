#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

__BEGIN_DECLS

#include "zenoh-pico.h"

__END_DECLS

extern "C" __EXPORT int zenoh_test_main(int argc, char *argv[])
{
	char *uri = "/demo/example/zenoh-pico-pub";

	if (argc > 1) {
		uri = argv[1];
	}

	char *value = "Pub from pico!";

	if (argc > 2) {
		value = argv[2];
	}

	zn_properties_t *config = zn_config_default();

	if (argc > 3) {
		zn_properties_insert(config, ZN_CONFIG_PEER_KEY, z_string_make(argv[3]));
	}

	printf("Openning session...\n");
	zn_session_t *s = zn_open(config);

	if (s == 0) {
		printf("Unable to open session!\n");
		exit(-1);
	}

	// Start the receive and the session lease loop for zenoh-pico
	znp_start_read_task(s);
	znp_start_lease_task(s);

	printf("Declaring Resource '%s'", uri);
	unsigned long rid = zn_declare_resource(s, zn_rname(uri));
	printf(" => RId %lu\n", rid);
	zn_reskey_t reskey = zn_rid(rid);

	char buf[256];

	for (int idx = 0; idx < 5; ++idx) {
		sleep(1);
		sprintf(buf, "[%4d] %s", idx, value);
		printf("Writing Data ('%lu': '%s')...\n", rid, buf);
		zn_write(s, reskey, (const uint8_t *)buf, strlen(buf));
	}

	znp_stop_read_task(s);
	znp_stop_lease_task(s);
	zn_close(s);

	return 0;
}

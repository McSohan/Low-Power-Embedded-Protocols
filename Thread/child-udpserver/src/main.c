/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/openthread.h>
#include <openthread/thread.h>

#include <zephyr/posix/arpa/inet.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/posix/sys/socket.h>

LOG_MODULE_REGISTER(child_server, CONFIG_CHILD_SERVER_LOG_LEVEL);

static struct k_work udp_server_work;

static void run_server(struct k_work *item)
{
	ARG_UNUSED(item);

	// TODO: implement server receive loop here

}

static void initialize_server(void)
{
	// TODO: implement server initialization here

	k_work_submit(&udp_server_work);
}

static void on_thread_state_changed(otChangedFlags flags, struct openthread_context *ot_context,
				    void *user_data)
{
	if (flags & OT_CHANGED_THREAD_ROLE) {
		switch (otThreadGetDeviceRole(ot_context->instance)) {
		case OT_DEVICE_ROLE_CHILD:
		case OT_DEVICE_ROLE_ROUTER:
		case OT_DEVICE_ROLE_LEADER:
			LOG_INF("Connected to OpenThread network");
			initialize_server();
			break;

		case OT_DEVICE_ROLE_DISABLED:
		case OT_DEVICE_ROLE_DETACHED:
		default:
			break;
		}
	}
}

static struct openthread_state_changed_cb ot_state_changed_cb = {.state_changed_cb = on_thread_state_changed};

int main(void)
{
	int ret;

	LOG_INF("child-udp-server appplication started");

	k_work_init(&udp_server_work, run_server);

	ret = dk_leds_init();
	if (ret) {
		LOG_ERR("Could not initialize leds, err code: %d", ret);
		goto end;
	}

	openthread_state_changed_cb_register(openthread_get_default_context(), &ot_state_changed_cb);
	openthread_start(openthread_get_default_context());

end:
	return 0;
}

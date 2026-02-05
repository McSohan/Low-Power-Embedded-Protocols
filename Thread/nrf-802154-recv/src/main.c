/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <stdio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
	LOG_MODULE_REGISTER(recv);

#include "nrf_802154.h"

#define MAX_CHIPS_PER_US (2)
#define PSDU_MAX_SIZE (127u)
#define WAIT_BYTES (2 * PSDU_MAX_SIZE)
#define WAIT_SYMBOLS (2 * WAIT_BYTES)
#define RECEIVE_PERIOD_US ((WAIT_SYMBOLS * 32) / MAX_CHIPS_PER_US)
#define CHECK_PERIOD_US (2000000)
#if (CHECK_PERIOD_US < RECEIVE_PERIOD_US)
#error "Check period cannot be less than receive period!"
#endif
#define SLEEP_PERIOD_US (CHECK_PERIOD_US - RECEIVE_PERIOD_US)

#define MY_STACK_SIZE 500
#define MY_PRIORITY 100

extern void fword(void *, void *, void *);

K_THREAD_STACK_DEFINE(whateverstack, MY_STACK_SIZE);
struct k_thread whatever;

k_tid_t my_tid;


static void pkt_hexdump(uint8_t *pkt, int length) {
	printk("Packet: ");
	for (int i = 0; i < length; i++) {
		printk("%02x ", *(pkt + i));
	}
	printk("\n");
}

static int rf_setup()
{
	LOG_INF("RF setup started");
	printk("RF setup started\n");

	/* nrf radio driver initialization */
	nrf_802154_init();
	return 0;
}

static struct k_mutex lock_tuah;
static uint8_t *fuck = NULL;

void nrf_802154_received_raw(uint8_t *data, int8_t power, uint8_t lqi) {
	k_mutex_lock(&lock_tuah, K_FOREVER);
	fuck = data;
	k_mutex_unlock(&lock_tuah);
}

void fword(void *, void *, void *) {
	while (1) {
		k_mutex_lock(&lock_tuah, K_FOREVER);
		if (fuck != NULL) {
			pkt_hexdump(fuck, fuck[0]);
			nrf_802154_buffer_free_raw(fuck);
			fuck = NULL;
		}
		k_mutex_unlock(&lock_tuah);
	}
}

void nrf_802154_receive_failed(nrf_802154_rx_error_t error, uint32_t id) {
 	LOG_INF("rx failed error %u!", error);
}

int main(void) {
	nrf_802154_channel_set(17u);
	nrf_802154_auto_ack_set(true);
	k_mutex_init(&lock_tuah);
	
	LOG_DBG("channel: %u", nrf_802154_channel_get());
	printk("channel: %u\n", nrf_802154_channel_get());

	// set the pan_id (2 bytes, little-endian)
	uint8_t pan_id[] = {0x68, 0x68};
	nrf_802154_pan_id_set(pan_id);

	// set the extended address (8 bytes, little-endian)
	uint8_t extended_addr[] = {0x68, 0x68, 0x68, 0x68, 0x68, 0x68, 0x68, 0x68};
	nrf_802154_short_address_set(extended_addr);
	nrf_802154_src_addr_matching_method_set(NRF_802154_SRC_ADDR_MATCH_THREAD);
	my_tid = k_thread_create(&whatever, whateverstack,
                                 K_THREAD_STACK_SIZEOF(whateverstack),
                                 fword,
                                 NULL, NULL, NULL,
                                 MY_PRIORITY, 0, K_NO_WAIT);

	while (1) {
		if (nrf_802154_sleep()) {
			printk("radio sleeping zzz...\n");
		} else {
			printk("issue sleeping radio :(");
		}
		k_sleep(K_USEC(SLEEP_PERIOD_US));
		if(nrf_802154_receive()) {
			printk("radio is aware O_o\n");
		} else {
			printk("radio couldn't start :(\n");
		}
		k_sleep(K_USEC(RECEIVE_PERIOD_US));
	}
	
	return 0;
}

SYS_INIT(rf_setup, POST_KERNEL, CONFIG_PTT_RF_INIT_PRIORITY);

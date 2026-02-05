#include <stdio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
	LOG_MODULE_REGISTER(send);

#include "nrf_802154.h"
#include <inttypes.h>

#define PSDU_MAX_SIZE (127u)
#define FCS_LENGTH (2u) /* Length of the Frame Control Sequence */

#define ACK_WAIT_MS (2)

#define SW0_NODE	DT_ALIAS(sw0)

static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});

static struct gpio_callback button1_cb_data;
static int button_is_pressed;
static uint8_t seq = 0;
static struct k_condvar cond;
static struct k_mutex lock_tuah;

static void pkt_hexdump(uint8_t *pkt, uint8_t length) {
  int i;
  printk("Packet: ");
	for (i=0; i<length; i++) {
		printk("%02x ", *pkt);
		pkt++;
	}
	printk("\n");
}

static int rf_setup() {
	LOG_INF("RF setup started");

	/* nrf radio driver initialization */
	nrf_802154_init();

	return 0;
}

// callback fn when tx starts
void nrf_802154_tx_started(const uint8_t *p_frame) {
	//LOG_INF("tx started");
}

// callback fn when tx fails
void nrf_802154_transmit_failed(uint8_t *frame, nrf_802154_tx_error_t error, const nrf_802154_transmit_done_metadata_t *p_metadata) {
	//pkt_hexdump(frame, 14);
	//LOG_INF("tx failed error %u!", error);
}

// callback fn for successful tx
void nrf_802154_transmitted_raw(uint8_t *p_frame, const nrf_802154_transmit_done_metadata_t *p_metadata) {
	k_mutex_lock(&lock_tuah, K_FOREVER);
	k_condvar_signal(&cond);
	k_mutex_unlock(&lock_tuah);
	LOG_INF("frame was transmitted!");
	nrf_802154_buffer_free_raw(p_metadata->data.transmitted.p_ack);
}

void button1_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	button_is_pressed = gpio_pin_get_dt(&button1) & 0x1;
	//printk("button presses\n");
}

void init_button(const struct gpio_dt_spec* button, struct gpio_callback* button_cb_data, gpio_callback_handler_t cb) {
	int ret;
	if (!device_is_ready(button->port)) {
		printk("Error: button device %s is not ready\n", button->port->name);
		return;
	}

	ret = gpio_pin_configure_dt(button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n", ret, button->port->name, button->pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(button, GPIO_INT_EDGE_BOTH);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, button->port->name, button->pin);
		return;
	}
	gpio_init_callback(button_cb_data, cb, BIT(button->pin));
	gpio_add_callback(button->port, button_cb_data);
	printk("Set up button at %s pin %d\n", button->port->name, button->pin);
}

int main(int argc, char **argv) {
	init_button(&button1, &button1_cb_data, button1_pressed);
	nrf_802154_channel_set(17u);
	k_condvar_init(&cond);
	k_mutex_init(&lock_tuah);
	LOG_DBG("channel: %u", nrf_802154_channel_get());

	// set the pan_id (2 bytes, little-endian)
	uint8_t src_pan_id[] = {0x68, 0x68};
	nrf_802154_pan_id_set(src_pan_id);

	// set the extended address (8 bytes, little-endian)
	uint8_t src_extended_addr[] = {0xdc, 0xa9, 0x35, 0x7b, 0x73, 0x36, 0xce, 0xf4};
	nrf_802154_short_address_set(src_extended_addr);

	uint8_t dst_extended_addr[] = {0x68, 0x68, 0x68, 0x68, 0x68, 0x68, 0x68, 0x68};

	// send a tx pkt
	uint8_t pkt[PSDU_MAX_SIZE];
	pkt[0] = 25u + FCS_LENGTH; /* Length for nrf_transmit (length of pkt + FCS) */
	pkt[1] = 0b00100001; /* Frame Control Field */
	pkt[2] = 0b10001000; /* Frame Control Field */
	pkt[3] = seq; /* Sequence number */
	pkt[4] = 0x68; /* Destination PAN ID 0xffff */
	pkt[5] = 0x68; /* Destination PAN ID */
	memcpy(&pkt[6], dst_extended_addr, 2); /* Destination extended address */
	memcpy(&pkt[8], src_pan_id, 2); /* Source PAN ID */
	memcpy(&pkt[10], src_extended_addr, 2);/* Source extended address */
	pkt[12] = 0x01; /* Payload */
	char name[] = "bloodhounds";
	memcpy(&pkt[13], name, sizeof(name) - 1);
	pkt[24] = 0;
	pkt[25] = 0;

	const nrf_802154_transmit_metadata_t metadata = {
		.frame_props = NRF_802154_TRANSMITTED_FRAME_PROPS_DEFAULT_INIT,
		.cca = true
	};

	while(1) {
		// send the packet
		int send_loop = 0;
		int crash_out = 1;
		if (button_is_pressed) {
			button_is_pressed = 0;
			do {
				if(!nrf_802154_transmit_raw(pkt, &metadata)) {
					printk("driver could not schedule the transmission procedure.\n");
				}
				k_mutex_lock(&lock_tuah, K_FOREVER);
				int c = k_condvar_wait(&cond, &lock_tuah, K_MSEC(ACK_WAIT_MS));
				k_mutex_unlock(&lock_tuah);
				if (c == -EAGAIN) {
					send_loop = 1;
					if (crash_out) {
						printk("im gonna crash out forever :<\n");
						crash_out = 0;
					}
				} else {
					seq++;
					if (crash_out == 0) {
						printk("nevermind, ");
					}
					printk("Sent: ");
					pkt_hexdump(pkt,pkt[0]);
					printk("we got an ack :>\n");
					send_loop = 0;
				}
				pkt[3] = seq;
			} while (send_loop);
		}
		
		k_sleep(K_MSEC(100));
		//pkt_hexdump(pkt, 14);
	}

	return 0;
}

SYS_INIT(rf_setup, POST_KERNEL, CONFIG_PTT_RF_INIT_PRIORITY);
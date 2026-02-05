#include <stdbool.h>
#include <zephyr/types.h>
#include <zephyr/drivers/sensor.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/arch/arm/thread.h>
#define ARCH_STACK_PTR_ALIGN 4

#define MY_STACK_SIZE 500
#define MY_PRIORITY 5

void my_entry_point(void *, void *, void *);

K_THREAD_STACK_DEFINE(my_stack_area, MY_STACK_SIZE);
struct k_thread my_thread_data;

static k_tid_t my_tid = NULL;

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static uint8_t buttons = 0;

#define LAB2_SERVICE_UUID BT_UUID_128_ENCODE(0xBDFC9792, 0x8234, 0x405E, 0xAE02, 0x35EF4174B299)
#define LAB2_SERVICE_CHARACTERISTIC_UUID 0x0001

static ssize_t characteristic_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
	
// Global value that saves state for the characteristic.
uint8_t characteristic_value = 0x7;

// Set up the advertisement data.
#define DEVICE_NAME "blebleble"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN,532,533,NULL); 

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, LAB2_SERVICE_UUID)
};

// Setup the the service and characteristics.
BT_GATT_SERVICE_DEFINE(lab2_service,
	BT_GATT_PRIMARY_SERVICE(
		BT_UUID_DECLARE_128(LAB2_SERVICE_UUID)
	),
	BT_GATT_CHARACTERISTIC(
		BT_UUID_DECLARE_16(LAB2_SERVICE_CHARACTERISTIC_UUID), BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE, NULL, characteristic_write, &characteristic_value
	),
);
				
static ssize_t characteristic_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
	if (len == 1 && offset == 0) {
		buttons = ((uint8_t*)buf)[0];
		return 1;
	}
	return 0;
}


// Setup callbacks when devices connect and disconnect.
static void connected(struct bt_conn *conn, uint8_t err) {
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
	printk("Disconnected (reason 0x%02x)\n", reason);
	int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};


static void bt_ready(int err) {
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static char str[] = "bloodhounds";

static int morse[][5] = {
	{0,1, -1}, // a
	{1,0,0,0, -1}, // b
	{1,0,1,0, -1}, // c
	{1,0,0, -1}, // d
	{0, -1}, // e
	{0,0,1,0, -1}, // f
	{1,1,0, -1}, // g
	{0,0,0, -1}, // h
	{0,0, -1}, // i
	{0,1,1,1, -1}, // j
	{1,0,1, -1}, // k
	{0,1,0,0, -1}, // l
	{1,1, -1}, // m
	{1,0 , -1}, // n
	{1,1,1, -1}, // o
	{0,1,1,0, -1}, // p
	{1,1,0,1, -1}, // q
	{0,1,0, -1}, // r
	{0,0,0, -1}, // s
	{1, -1}, // t
	{0,0,1, -1}, // u
	{0,0,0,1, -1}, // v
	{0,1,1, -1}, // w
	{1,0,0,1, -1}, // x
	{1,0,1,1, -1}, // y
	{1,1,0,0, -1}, // z
	{-1} // {
};

static int idx = 0;
static int *next = NULL;
static int off = 0;

static int led_state = 0;
#define SLEEP_TIME_MAX 2000
#define SLEEP_TIME_MIN 50

void my_entry_point(void *a, void *b, void *c) {
	while (1) {
		gpio_pin_toggle_dt(&led);
		printk("BUTTONs: %d\n", buttons);
		int sleep_time = (((SLEEP_TIME_MAX - SLEEP_TIME_MIN) / 16) * buttons) + SLEEP_TIME_MIN;
		if (next == NULL) {
			idx = (idx + 1) % sizeof(str) - 1;
			// printf("idx: %d\n", idx);
			// printf("str[idx]: %d\n", str[idx] - 'a');
			// printf("morse[str[idx]]: %X\n", morse[str[idx] - 'a']);
			next = morse[str[idx] - 'a'];
		}
		
		// printf("off: %d\n", off);
		// printf("next[off]: %d\n", next[off]);
		if (str[idx] != NULL && next[off] != -1) {
			if (next[off] == 1) {
				led_state = 1;
				//printf("LED state: dash\n");
				k_msleep(sleep_time*3);
			} else {
				led_state = 1;
				//printf("LED state: dot\n");
				k_msleep(sleep_time );
			}
			off++;
		} else {
			next = NULL;
			off = 0;
		}
		led_state = 0;
		k_msleep(sleep_time);
	}
}

int main() {
	int err;

	int ret;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}
	k_thread_create(&my_thread_data, my_stack_area,
		K_THREAD_STACK_SIZEOF(my_stack_area),
		my_entry_point,
		NULL, NULL, NULL,
		MY_PRIORITY, 0, K_NO_WAIT
	);
}

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>

void six_s_timer_handler(struct k_timer *timer);
void seven_s_timer_handler(struct k_timer *timer);

K_TIMER_DEFINE(six_s_timer, six_s_timer_handler, NULL);
K_TIMER_DEFINE(seven_s_timer, seven_s_timer_handler, NULL);

void six_s_timer_handler(struct k_timer *timer) {
	printk("It has been six seconds.\n");
}
void seven_s_timer_handler(struct k_timer *timer) {
	printk("It has been seven seconds.\n");
}

void main(void) {
	k_timer_start(&six_s_timer, K_SECONDS(6), K_SECONDS(6));
	k_timer_start(&seven_s_timer, K_SECONDS(7), K_SECONDS(7));
	printk("Started 7 second timer. Please wait.\n");
	printk("Started six second timer. Please wait.\n");
}

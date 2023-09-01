#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <pulse_rgb.h>

static const struct pwm_dt_spec red_pwm_led =
	PWM_DT_SPEC_GET(DT_ALIAS(red_pwm_led));
static const struct pwm_dt_spec green_pwm_led =
	PWM_DT_SPEC_GET(DT_ALIAS(green_pwm_led));
static const struct pwm_dt_spec blue_pwm_led =
	PWM_DT_SPEC_GET(DT_ALIAS(blue_pwm_led));

#define FAST_PULSE_DELAY K_MSEC(10)
#define SLOW_PULSE_DELAY K_MSEC(10)

#define STEP_SIZE PWM_USEC(300)

void pulse_red(int num_pulses, bool fast){

	int ret;
	uint32_t pulse_red;

	if (!device_is_ready(red_pwm_led.dev))
	{
		printk("Error: red LED device not ready\n");
		return;
	}

	for (int ct = 0; ct < num_pulses; ct++){
		for (uint32_t pulse_red = 0U; pulse_red <= red_pwm_led.period; pulse_red += STEP_SIZE) 
		{
			ret = pwm_set_pulse_dt(&red_pwm_led, pulse_red);
			printk("%d", pulse_red); printk("\n");
			if (ret != 0) {
				printk("Error %d: red write failed\n",
						ret);
				return;
			}

			if (fast) k_sleep(FAST_PULSE_DELAY);
			else      k_sleep(SLOW_PULSE_DELAY);
		}

	for (pulse_red = red_pwm_led.period; pulse_red >= STEP_SIZE; pulse_red -= STEP_SIZE) 
		{
			ret = pwm_set_pulse_dt(&red_pwm_led, pulse_red);
			printk("%d", pulse_red); printk("\n");
			if (ret != 0) {
				printk("Error %d: red write failed\n",
						ret);
				return;
			}

			if (fast) k_sleep(FAST_PULSE_DELAY);
			else      k_sleep(SLOW_PULSE_DELAY);		
		}
	}

	ret = pwm_set_pulse_dt(&red_pwm_led, 0);
	if (ret != 0) {
		printk("Error %d: red write failed\n",
				ret);
		return;
	}

}

void pulse_green(int num_pulses, bool fast){

	int ret;
	uint32_t pulse_green;

	if (!device_is_ready(green_pwm_led.dev))
	{
		printk("Error: green LED device not ready\n");
		return;
	}

	for (int ct = 0; ct < num_pulses; ct++){
		for (uint32_t pulse_green = 0U; pulse_green <= green_pwm_led.period; pulse_green += STEP_SIZE) 
		{
			ret = pwm_set_pulse_dt(&green_pwm_led, pulse_green);
			printk("%d", pulse_green); printk("\n");
			if (ret != 0) {
				printk("Error %d: green write failed\n",
						ret);
				return;
			}

			if (fast) k_sleep(FAST_PULSE_DELAY);
			else      k_sleep(SLOW_PULSE_DELAY);
		}

		for (pulse_green = green_pwm_led.period; pulse_green >= STEP_SIZE; pulse_green -= STEP_SIZE) 
		{
			ret = pwm_set_pulse_dt(&green_pwm_led, pulse_green);
			printk("%d", pulse_green); printk("\n");
			if (ret != 0) {
				printk("Error %d: green write failed\n",
						ret);
				return;
			}

			if (fast) k_sleep(FAST_PULSE_DELAY);
			else      k_sleep(SLOW_PULSE_DELAY);
		}
	}

	ret = pwm_set_pulse_dt(&green_pwm_led, 0);
	if (ret != 0) {
		printk("Error %d: green write failed\n",
				ret);
		return;
	}

}

void pulse_blue(int num_pulses, bool fast){

	int ret;
	uint32_t pulse_blue;

	if (!device_is_ready(blue_pwm_led.dev))
	{
		printk("Error: bleu LED device not ready\n");
		return;
	}

	for (int ct = 0; ct < num_pulses; ct++){
		for (uint32_t pulse_blue = 0U; pulse_blue <= blue_pwm_led.period; pulse_blue += STEP_SIZE) 
		{
			ret = pwm_set_pulse_dt(&blue_pwm_led, pulse_blue);
			//printk("%d", pulse_blue); printk("\n");
			if (ret != 0) {
				printk("Error %d: blue write failed\n",
						ret);
				return;
			}

			if (fast) k_sleep(FAST_PULSE_DELAY);
			else      k_sleep(SLOW_PULSE_DELAY);
		}

		for (pulse_blue = blue_pwm_led.period; pulse_blue >= STEP_SIZE; pulse_blue -= STEP_SIZE) 
		{
			ret = pwm_set_pulse_dt(&blue_pwm_led, pulse_blue);
			//printk("%d", pulse_blue); printk("\n");
			if (ret != 0) {
				printk("Error %d: blue write failed\n",
						ret);
				return;
			}

			if (fast) k_sleep(FAST_PULSE_DELAY);
			else      k_sleep(SLOW_PULSE_DELAY);
		}
	}

	ret = pwm_set_pulse_dt(&blue_pwm_led, 0);
	if (ret != 0) {
		printk("Error %d: blue write failed\n",
				ret);
		return;
	}

}


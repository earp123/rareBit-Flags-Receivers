#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <soc.h>

// For power manager
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/watchdog.h>

// Devicetree + GPIO includes
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_ppi.h>
#include <zephyr/irq.h>

//ADC Library
#include <zephyr/drivers/adc.h>

// Bluetooth include files
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/scan.h>
#include <bluetooth/gatt_dm.h>
#include <paging_client.h>

const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#define COMPILE_ADC_PRINT 1
#define COMPILE_ON_OFF 1


#if COMPILE_ADC_PRINT
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static int16_t adc_buf;
static int val_mv;
static struct adc_sequence sequence = {
    .buffer = &adc_buf,
    /* buffer size in bytes, not number of samples */
    .buffer_size = sizeof(adc_buf),
    // Optional
    //.calibrate = true,
};
#endif

#define WDT_MAX_WINDOW  10000U
#define WDT_MIN_WINDOW  0U

int wdt_channel_id;
const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));

struct wdt_timeout_cfg wdt_config = {
    /* Reset SoC when watchdog timer expires. */
    .flags = WDT_FLAG_RESET_SOC,

    /* Expire watchdog after max window */
    .window.min = WDT_MIN_WINDOW,
    .window.max = WDT_MAX_WINDOW,
};

LOG_MODULE_REGISTER(PRO_RECV, LOG_LEVEL_DBG);

#define CONFIG_BOARD_PRO_RECEIVER 1

#define GREEN_LED_NODE      DT_ALIAS(led1)
#define BLUE_LED_NODE       DT_ALIAS(led2)
#define RED_LED_NODE        DT_ALIAS(led0)
#define BUTTON_NODE         DT_ALIAS(sw0)

#if CONFIG_BOARD_PRO_RECEIVER
#define HAPTIC_PWM_NODE     DT_ALIAS(pwm_buzz)
#define HAPTIC_PIN_NODE     DT_ALIAS(buzz_en)
#define PWM_RED_LED_NODE    DT_ALIAS(pwm_led0)
#define PWM_GREEN_LED_NODE  DT_ALIAS(pwm_led1)
#define PWM_BLUE_LED_NODE   DT_ALIAS(pwm_led2)
#define USB_DETECT_NODE     DT_ALIAS(usb_dt)
#define USB_STAT_NODE       DT_ALIAS(chrg)
#define FACTORY_ENABLE_NODE DT_ALIAS(pwron)
#else
#define HAPTIC_PWM_NODE DT_ALIAS(pwm_led0)
#define HAPTIC_PIN_NODE DT_ALIAS(led3)
#endif

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static const struct gpio_dt_spec r_led =  GPIO_DT_SPEC_GET(RED_LED_NODE, gpios);
static const struct gpio_dt_spec g_led =  GPIO_DT_SPEC_GET(GREEN_LED_NODE, gpios);
static const struct gpio_dt_spec b_led =  GPIO_DT_SPEC_GET(BLUE_LED_NODE, gpios);
//static const struct pwm_dt_spec  pwm_buzz = PWM_DT_SPEC_GET(HAPTIC_PWM_NODE);
//static const struct gpio_dt_spec buzz_en = GPIO_DT_SPEC_GET(HAPTIC_PIN_NODE, gpios);
#if CONFIG_BOARD_PRO_RECEIVER
//static const struct pwm_dt_spec pwm_ledr =    PWM_DT_SPEC_GET(PWM_RED_LED_NODE);
//static const struct pwm_dt_spec pwm_ledg =    PWM_DT_SPEC_GET(PWM_GREEN_LED_NODE);
//static const struct pwm_dt_spec pwm_ledb =    PWM_DT_SPEC_GET(PWM_BLUE_LED_NODE);
static const struct gpio_dt_spec usb_detect = GPIO_DT_SPEC_GET(USB_DETECT_NODE, gpios);
static const struct gpio_dt_spec stat_pin =   GPIO_DT_SPEC_GET(USB_STAT_NODE, gpios);
static const struct gpio_dt_spec bat_en =     GPIO_DT_SPEC_GET(FACTORY_ENABLE_NODE, gpios);
#else
static const struct pwm_dt_spec pwm_ledr = PWM_DT_SPEC_GET(HAPTIC_PWM_NODE);
static const struct pwm_dt_spec pwm_ledg = PWM_DT_SPEC_GET(HAPTIC_PWM_NODE);
static const struct pwm_dt_spec pwm_ledb = PWM_DT_SPEC_GET(HAPTIC_PWM_NODE);
#endif

//Total fade up time is DELAY * PWM_STEPS
#define HAPTIC_PWM_PERIOD_NS        500000
#define HAPTIC_FADE_DELAY_MS        1
#define HAPTIC_PWM_STEPS            1000

#define POWERON_COUNT               100  //in units of 50ms
#define POWERDOWN_COUNT             100  //in units of 50ms
#define DFU_MODE_COUNT              200  // should at least twice as long as POWERON_COUNT, in units of 50ms
#define WAIT_FOR_DFU                K_SECONDS(20)

#define POWERON_PWM_PERIOD_NS       500000
#define POWERON_FADE_UP_DELAY_MS    1
#define POWERON_PWM_STEPS           3000


#define LOW_BATTERY_STATUS_MV       800
#define LOW_BATTERY_STATUS_COUNT    100
#define BATTERY_CHECK_INTERVAL_MS   5000

#define DEVICE_NAME "rareBit PRO Receiver"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define WORQ_THREAD_STACK_SIZE  8192
#define WORKQ_PRIORITY          5

// Define stack area used by workqueue thread
static K_THREAD_STACK_DEFINE(my_stack_area, WORQ_THREAD_STACK_SIZE);

// Define queue structure
static struct k_work_q offload_work_q = {0};

struct bt_work_info {
    struct k_work work;
    char name[25];
} bt_work;

struct usb_work_info
{
    struct k_work work;
    char name[50];
} usb_work;

struct pwr_down_info
{
    struct k_work work;
    char name[50];
} pwr_down_work;

//static struct bt_gatt_exchange_params mtu_exchange_params[CONFIG_BT_MAX_CONN];

bool notify_check = false;

//static struct bt_conn *default_conn = NULL;
static struct bt_conn *connection_1 = NULL;
static struct bt_conn *connection_2 = NULL;
static struct bt_conn *connection_3 = NULL;
static struct bt_conn *connection_4 = NULL;
static struct bt_conn *connection_5 = NULL;
static struct bt_conn *connection_6 = NULL;
static struct bt_conn **app_ctx[] = {&connection_1, &connection_2, &connection_3, &connection_4, &connection_5, &connection_6};

static struct bt_pag_client pag_c;

static uint8_t conn_count;

#if COMPILE_ON_OFF
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
    
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
        0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
        0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d)
};
#endif

static uint8_t notify_func(struct bt_pag_client *pag_c, uint8_t page_alert, char page_addr[MAC_ADDRESS_LEN]) // SWR future inputs go here
{
    page_addr[17] = '\0';

	if(!notify_check)
	{
		printk("Notification, page_alert: %d\nDevice addr: %s\n", page_alert, page_addr);

		//TODO Alerting
	}

	return BT_GATT_ITER_CONTINUE;
	// SWR things will happen
}

// assign the callback for notifications
const struct bt_pag_client_cb_init pag_cb_init = {
    .alert_cb = {
        .pag_notify_cb = notify_func,
    }
};

/*
static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
							struct bt_gatt_exchange_params *params)
{
	printk("MTU exchange %u %s (%u)\n", bt_conn_index(conn),
		   err == 0U ? "successful" : "failed", bt_gatt_get_mtu(conn));
}

static int mtu_exchange(struct bt_conn *conn)
{
	uint8_t conn_index;
	int err;

	conn_index = bt_conn_index(conn);

	printk("MTU (%u): %u\n", conn_index, bt_gatt_get_mtu(conn));

	mtu_exchange_params[conn_index].func = mtu_exchange_cb;

	err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params[conn_index]);
	if (err)
	{
		printk("MTU exchange failed (err %d)", err);
	}
	else
	{
		printk("Exchange pending...");
	}

	return err;
}*/

static void discover_pag_completed(struct bt_gatt_dm *dm, void *ctx)
{
	int err;
	struct bt_pag_client *pag = ctx;
	char addr[BT_ADDR_LE_STR_LEN];
	struct bt_conn *conn = bt_pag_conn(pag);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    addr[BT_ADDR_STR_LEN] = '\0';

	printk("The discovery procedure succeeded\n");

	bt_gatt_dm_data_print(dm);

	err = bt_pag_handles_assign(dm, pag);
	if (err)
	{
		printk("Could not init Paging client object (err %d)\n", err);
		return;
	}
	else
		printk("Paging Service handles are assgined.\n");

	// pass the callback to the client structure used in bt_page_alert_subscribe
	bt_pag_cb_init(pag, &pag_cb_init);

	err = bt_page_alert_subscribe(pag);
	if (err)
	{
		printk("Subscribe failed (err %d)\n", err);
	}
	else
	{
		printk("[SUBSCRIBED]\n");
	}

	err = bt_gatt_dm_data_release(dm);
	if (err)
	{
		printk("Could not release the discovery data (err %d)\n", err);
	}
	else
		printk("Discorvery data released\n");

	if (conn_count < CONFIG_BT_MAX_CONN)
	{

		err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
		if (err)
		{
			printk("Scanning failed to start (err %d)\n", err);
		}
		k_sleep(K_MSEC(500));
	}
	else
		printk("Max devices connected");

	// mtu_exchange(conn);
}

static void discover_pag_service_not_found(struct bt_conn *conn, void *ctx)
{
	printk("No more services\n");
}

static void discover_pag_error_found(struct bt_conn *conn, int err, void *ctx)
{
	printk("The discovery procedure failed, err %d\n", err);
}

static struct bt_gatt_dm_cb discover_pag_cb = {
	.completed = discover_pag_completed,
	.service_not_found = discover_pag_service_not_found,
	.error_found = discover_pag_error_found,
};

static void scan_filter_match(struct bt_scan_device_info *device_info,
							  struct bt_scan_filter_match *filter_match,
							  bool connectable)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN];
	struct bt_conn_le_create_param *conn_params;

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    addr[BT_ADDR_STR_LEN] = '\0';

	printk("Filters matched. Address: %s connectable: %s\n",
		   addr, connectable ? "yes" : "no");

	err = bt_scan_stop();
	if (err)
	{
		printk("Stop LE scan failed (err %d)\n", err);
	}
	else printk("Scanning stopped.\n");

	conn_params = BT_CONN_LE_CREATE_PARAM(
		BT_CONN_LE_OPT_CODED | BT_CONN_LE_OPT_NO_1M,
		BT_GAP_SCAN_FAST_INTERVAL,
		BT_GAP_SCAN_FAST_INTERVAL);

    for (size_t i = 0; i < 6; i++) {
        if (*app_ctx[i] == NULL) {
            // Connection slot is free
            err = bt_conn_le_create(device_info->recv_info->addr, conn_params,
							            BT_LE_CONN_PARAM_DEFAULT, app_ctx[0]);
            break;
        }
    }
	
	if (err)
	{
		printk("Create conn failed (err %d)\n", err);

		err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
		if (err)
		{
			printk("Scanning failed to start (err %d)\n", err);
			return;
		}
	}

	printk("Connection pending\n");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, NULL, NULL);

static void scan_init(void)
{
	int err;

	/* Use active scanning and disable duplicate filtering to handle any
	 * devices that might update their advertising data at runtime. */
	struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_ACTIVE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
		.options = BT_LE_SCAN_OPT_CODED | BT_LE_SCAN_OPT_NO_1M};

	struct bt_scan_init_param scan_init = {
		.connect_if_match = 1,
		.scan_param = &scan_param,
		.conn_param = NULL};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	// SWR TODO change the filter type to short name and iterate through the device names
	//* change this  for future pairing and un-pairing implementations
	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_PAGING_SERVICE);
	if (err)
	{
		printk("Scanning filters cannot be set (err %d)\n", err);
		return;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err)
	{
		printk("Filters cannot be turned on (err %d)\n", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	int err;
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    addr[BT_ADDR_STR_LEN] = '\0';

	if (conn_err)
	{
		printk("Failed to connect to %s (%u)\n", addr, conn_err);

		bt_conn_unref(conn);
		for (size_t i = 0; i < 6; i++)
        {
            if(conn == *app_ctx[i])
            {
                *app_ctx[i] = NULL;
                break;
            }
        }

		err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
		if (err)
		{
			printk("Scanning failed to start (err %d)\n", err);
		}

		return;
	}

	err = bt_conn_get_info(conn, &info);

	if (err)
	{
		printk("Failed to get connection info\n");
	}
	else
	{
		const struct bt_conn_le_phy_info *phy_info;

		phy_info = info.le.phy;
		printk("Connected: %s, tx_phy %u, rx_phy %u\n",
			   addr, phy_info->tx_phy, phy_info->rx_phy);
	}

    for (size_t i = 0; i < 6; i++) {
        if (conn == *app_ctx[i])
        {
            err = bt_gatt_dm_start(conn, BT_UUID_PAGING_SERVICE, &discover_pag_cb, &pag_c);
            if (err)
            {
                printk("Failed to start discovery (err %d)\n", err);
            }
            else
            {
                printk("Connection %d assigned to %s\n", i+1, addr);

                conn_count++;
                printk("Devices connected: %d\n", conn_count);
                return;
                
            }
        }
    }
	
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{

	char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    addr[BT_ADDR_STR_LEN] = '\0';

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(conn);

    for (size_t i = 0; i < 6; i++)
    {
        if(conn == *app_ctx[i])
        {
            *app_ctx[i] = NULL;
            break;
        }
    }

	printk("%s unreferenced\n", addr);

	conn_count--;
	printk("Devices connected: %d\n", conn_count);

    k_msleep(300);

}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

void pwr_down_thread(struct k_work *work_item)
{

    k_msleep(300);

    int button_held_count = 0;

    while(gpio_pin_get_dt(&button))
    {
        k_msleep(50);
        button_held_count++;
        //TODO green_pwm_fade down
        gpio_pin_set_dt(&g_led, 1);
        if(button_held_count > POWERON_COUNT) break;
    }

    gpio_pin_set_dt(&g_led, 0);

    if(button_held_count >= POWERON_COUNT)
    {
        printk("Powering Off: Button held for POWERDOWN_COUNT.\n");
        k_msleep(4000);
        sys_poweroff();
    }

}

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Button Interrupt Triggered\n");
    k_work_submit_to_queue(&offload_work_q, &pwr_down_work.work);
    k_yield();

}

void usb_detect_thread(struct k_work *work_item)
{
    while(gpio_pin_get_dt(&usb_detect))
    {
        gpio_pin_set_dt(&r_led, 1);
        //red slow pwm pulse
        printk("USB PRESENT............\n");
        k_msleep(1000);
        gpio_pin_set_dt(&r_led, 0);
        k_msleep(1000);
    }

    printk("Powering Off: USB Removed.\n");
    k_msleep(4000);

    pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
    sys_poweroff();

}

#if COMPILE_ADC_PRINT
static void init_adc()
{
    int err;

    /* Define a variable of type adc_sequence and a buffer of type uint16_t */
	
	/* Validate that the ADC peripheral (SAADC) is ready */
	if (!adc_is_ready_dt(&adc_channel)) {
		LOG_ERR("ADC controller devivce %s not ready", adc_channel.dev->name);
		return;
	}
	/* Setup the ADC channel */
	err = adc_channel_setup_dt(&adc_channel);
	if (err < 0) {
		LOG_ERR("Could not setup channel #%d (%d)", 0, err);
        return;
	}
	/* Initialize the ADC sequence */
	err = adc_sequence_init_dt(&adc_channel, &sequence);
	if (err < 0) {
		LOG_ERR("Could not initalize sequnce");
		return;
	}
}
#endif

static void init_callbacks(void)
{
    int err;

    //Button Pin Callabck
    static struct gpio_callback button_cb_data;

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));

    err = gpio_add_callback(button.port, &button_cb_data);
    if (err < 0)
        printk("Button pin interrupt callback add failed with err %d\n", err);
    else
        printk("Button pin interrupt callback added.\n");
    
}

static void init_pins(void)
{
    int err;

    //Enable Pin
    err = gpio_pin_configure_dt(&bat_en, GPIO_OUTPUT_HIGH);
    if (err < 0)
        printk("Battery Enable Output failed (err %d)\n", err);
    else
        printk("Battery Enable Output configured\n");
    
    //Battery Status Pin
    err = gpio_pin_configure_dt(&stat_pin, GPIO_INPUT);
    if (err < 0)
        printk("Battery Status Iput failed (err %d)\n", err);
    else
        printk("Battery Status Iput configured\n");

    //Button Pin
    err = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (err < 0)
        printk("Button Input failed (err %d)\n", err);
    else
        printk("Button Input configured\n");

    err = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_LOW_0);
    if (err < 0)
        printk("Button pin interrupt configure failed with err %d\n", err);
    else
        printk("Button pin interrupt configured.\n");


    //USB Detect Pin
    err = gpio_pin_configure_dt(&usb_detect, GPIO_INPUT);
    if (err < 0)
        printk("USB Detect Input failed (err %d)\n", err);
    else
        printk("USB Detect Input configured\n");


    err = gpio_pin_interrupt_configure_dt(&usb_detect, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0)
        printk("USB Detect pin interrupt configure failed with err %d\n", err);
    else
        printk("USB Detect pin interrupt configured.\n");

    
}

int main(void)
{
    init_pins();
#if COMPILE_ON_OFF
    int button_held_count = 0;
#endif    

    k_work_queue_start(&offload_work_q, my_stack_area, 
                        K_THREAD_STACK_SIZEOF(my_stack_area), 
                                        WORKQ_PRIORITY, NULL);
    strcpy(usb_work.name, "USB Detect Thread");
    k_work_init(&usb_work.work, usb_detect_thread);

    if(gpio_pin_get_dt(&usb_detect))
    {
        
        k_msleep(300);
        if(gpio_pin_get_dt(&usb_detect))
        {
            printk("Jump to USB Detect Thread\n");
            k_work_submit_to_queue(&offload_work_q, &usb_work.work);
            k_yield();
            return 0;
        }
        
    }
#if COMPILE_ON_OFF
    else if(gpio_pin_get_dt(&button))
    {
        printk("BUTTON HELD CONDITION\n");
        
        k_msleep(300);

        
        while(gpio_pin_get_dt(&button))
        {
            k_msleep(50);
            button_held_count++;
            //TODO green pwm fade up
            gpio_pin_set_dt(&g_led, 1);
            if(button_held_count > POWERON_COUNT) break;
        }

        gpio_pin_set_dt(&g_led, 0);
        k_msleep(300);

        while(gpio_pin_get_dt(&button))
        {
            k_msleep(50);
            button_held_count++;

            if(button_held_count > DFU_MODE_COUNT)
            {
                gpio_pin_set_dt(&r_led, 1);
                gpio_pin_set_dt(&b_led, 1);
                break;
            } 
        }


    }

    if(button_held_count < POWERON_COUNT)
    {
        printk("Powering Off: Button not held long enough.\n");
        k_msleep(3000);
        sys_poweroff();
        k_msleep(3000);
    }
    else if(button_held_count >= DFU_MODE_COUNT)
    {
        int err;
        printk("~~~~~~~~~~~~~~~~DFU CONDITION~~~~~~~~~~~~~~~~\n");
        err = bt_enable(NULL);
        if (err)
        {
            printk("Bluetooth init failed (err %d)\n", err);
        }

        err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	    if (err) {
		    printk("Advertising failed to start (err %d)\n", err);
		    return 0;
	    }
        else
        {
            printk("Started Advertising!\n");
            k_sleep(WAIT_FOR_DFU);
        } 

    }
#endif

    // //Continues...
    printk("~~~~~~RECEIVER POWER ON~~~~~~~~~\n");

    int err;

    wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
	if (wdt_channel_id < 0) {
		printk("Watchdog install error\n");
		return 0;
	}

	err = wdt_setup(wdt, 0);
	if (err < 0) {
		printk("Watchdog setup error\n");
		return 0;
	}

    strcpy(pwr_down_work.name, "Power Down Thread");
    k_work_init(&pwr_down_work.work, pwr_down_thread);

    init_callbacks();
    
    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
    }

    scan_init();

    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err < 0)
    {
        printk("Scanning failed to start (err %d)\n", err);
    }
    else printk("Scanning Started!! \n");


#if COMPILE_ADC_PRINT
    init_adc();
    int low_batt_count = 0;
    while (1) {
   
		/* Read a sample from the ADC */
		err = adc_read(adc_channel.dev, &sequence);
		if (err < 0) {
			LOG_ERR("Could not read (%d)", err);
			continue;
		}

		val_mv = (int)adc_buf;//Might not need this

		/* Convert raw value to mV*/
		err = adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
		/* conversion to mV may not be supported, skip if not */
		if (err < 0) {
			LOG_WRN(" (value in mV not available)\n");
		} else {
			
            if(val_mv < LOW_BATTERY_STATUS_MV)
            {
                low_batt_count++;
            }
            else low_batt_count = 0;

            LOG_INF("ADC: = %d mV, Low Batt Count = %d", val_mv, low_batt_count);

            if(low_batt_count >= LOW_BATTERY_STATUS_COUNT)
            {
                gpio_pin_set_dt(&r_led, 1);
                k_msleep(300);
                gpio_pin_set_dt(&r_led, 0);
            }
            else
            {
                gpio_pin_set_dt(&b_led, 1);
                k_msleep(150);
                gpio_pin_set_dt(&b_led, 0);
                
            }
		}
#else
    while(1)
    {
#endif
        wdt_feed(wdt, wdt_channel_id);
		k_msleep(BATTERY_CHECK_INTERVAL_MS);
        //TODO Indicate Less than max connected (still scanning)
    }
    return 0;
}


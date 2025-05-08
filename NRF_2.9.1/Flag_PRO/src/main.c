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
#include <zephyr/bluetooth/services/bas.h>

const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#define COMPILE_ADC_PRINT   1
#define COMPILE_ON_OFF      0
#define COMPILE_AD_TIMEOUT  0


#if COMPILE_ADC_PRINT && CONFIG_BOARD_PRO_FLAG
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

LOG_MODULE_REGISTER(PRO_FLAG, LOG_LEVEL_DBG);

#define GREEN_LED_NODE      DT_ALIAS(led1)
#define BLUE_LED_NODE       DT_ALIAS(led2)
#define RED_LED_NODE        DT_ALIAS(led0)
#define BUTTON_NODE         DT_ALIAS(sw0)

#if CONFIG_BOARD_PRO_FLAG
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

static const struct gpio_dt_spec button =       GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static const struct gpio_dt_spec r_led =        GPIO_DT_SPEC_GET(RED_LED_NODE, gpios);
//static const struct gpio_dt_spec g_led =      GPIO_DT_SPEC_GET(GREEN_LED_NODE, gpios);
static const struct gpio_dt_spec b_led =        GPIO_DT_SPEC_GET(BLUE_LED_NODE, gpios);
static const struct pwm_dt_spec  pwm_buzz =     PWM_DT_SPEC_GET(HAPTIC_PWM_NODE);
static const struct gpio_dt_spec buzz_en =      GPIO_DT_SPEC_GET(HAPTIC_PIN_NODE, gpios);
#if CONFIG_BOARD_PRO_FLAG
static const struct pwm_dt_spec pwm_ledr =      PWM_DT_SPEC_GET(PWM_RED_LED_NODE);
static const struct pwm_dt_spec pwm_ledg =      PWM_DT_SPEC_GET(PWM_GREEN_LED_NODE);
static const struct pwm_dt_spec pwm_ledb =      PWM_DT_SPEC_GET(PWM_BLUE_LED_NODE);
static const struct gpio_dt_spec usb_detect =   GPIO_DT_SPEC_GET(USB_DETECT_NODE, gpios);
static const struct gpio_dt_spec stat_pin =     GPIO_DT_SPEC_GET(USB_STAT_NODE, gpios);
static const struct gpio_dt_spec bat_en =       GPIO_DT_SPEC_GET(FACTORY_ENABLE_NODE, gpios);
#else
static const struct pwm_dt_spec pwm_ledr = PWM_DT_SPEC_GET(HAPTIC_PWM_NODE);
static const struct pwm_dt_spec pwm_ledg = PWM_DT_SPEC_GET(HAPTIC_PWM_NODE);
static const struct pwm_dt_spec pwm_ledb = PWM_DT_SPEC_GET(HAPTIC_PWM_NODE);
#endif

#define MAX_TRANSMIT_SIZE 240//TODO figure this out


// Service: Paging Service UUID 23210001-28D5-4B7B-BA0F-7DEE1EEE1B6D
#define PAGING_SERVICE_UUID 0x6d, 0x1b, 0xee, 0x1e, 0xee, 0x7d, 0xd0, 0xba, 0x7b, \
                            0x4b, 0xd5, 0x28, 0x01, 0x00, 0x21, 0x23

// Characteristic: Paging Characteristic UUID 23210002-28D5-4B7B-BA0F-7DEE1EEE1B6D
#define PAGE_ALERT_CHARACTERISTIC_UUID 0x6d, 0x1b, 0xee, 0x1e, 0xee, 0x7d, 0xd0, 0xba, 0x7b, \
                            0x4b, 0xd5, 0x28, 0x02, 0x00, 0x21, 0x23

#define BT_UUID_PAGING_SERVICE              BT_UUID_DECLARE_128(PAGING_SERVICE_UUID)
#define BT_UUID_PAGE_ALERT_CHARACTERISTIC   BT_UUID_DECLARE_128(PAGE_ALERT_CHARACTERISTIC_UUID)

const uint8_t page_d[] = {0x01};
const uint8_t * page_p = page_d;

//Advertising Timeout period ranges from EVENTS * MIN_INTERVAL to EVENTS * MAX_INTERVAL
#define ADVERTISNG_TIMEOUT_EVENTS   255

//static bool app_button_state = false;

//Total fade up time is DELAY * PWM_STEPS
#define HAPTIC_PWM_PERIOD           PWM_USEC(500)
#define HAPTIC_FADE_DELAY_MS        1
#define HAPTIC_PWM_STEPS            1000
#define HAPTIC_PWM_INC              HAPTIC_PWM_PERIOD / HAPTIC_PWM_STEPS

#define PWM_FADE_PERIOD             PWM_USEC(500)
#define PWM_FADE_DELAY_MS           1
#define PWM_FADE_STEPS              2000            //TOTAL FADE DURATION = PWM_FADE_STEPS * PWM_FADE_DELAY_MS
#define PWM_FADE_INC                PWM_FADE_PERIOD / PWM_FADE_STEPS

#define ADV_FADE_STEPS              1000
#define ADV_FADE_DELAY_US           500
#define ADV_FADE_INC                PWM_FADE_PERIOD / ADV_FADE_STEPS

#define POWERON_COUNT               4000  //in ms plus a few extra

#define POWERON_PWM_PERIOD_NS       500000
#define POWERON_FADE_UP_DELAY_MS    1
#define POWERON_PWM_STEPS           3000

#define LOW_BATTERY_STATUS_MV       800
#define LOW_BATTERY_STATUS_COUNT    15
#define BATTERY_CHECK_INTERVAL_MS   5000

#define DEVICE_NAME "rareBit PRO Flag"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define WORQ_THREAD_STACK_SIZE  8192
#define WORKQ_PRIORITY   3

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

struct adv_work_info
{
    struct k_work work;
    char name[50];
} adv_work;

bool indicate_adv = true;

static struct bt_le_ext_adv *adv;
struct bt_conn *conn_handle = NULL;
const struct bt_gatt_attr *page_alert_attr = NULL;

bool page_flag = false;

// Bluetooth advertising data
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    // BT_DATA_BYTES(BT_DATA_UUID128_ALL,
	// 	      0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
	// 	      0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d), //DFU Service
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, PAGING_SERVICE_UUID),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};

void start_advertising_coded(void)
{
    int err;

#if COMPILE_AD_TIMEOUT
    //timeout value doesn't seem to work here, the advertising sent function gets triggered even when the timeout doesn't occur
    const struct bt_le_ext_adv_start_param start_params = {
        .num_events = ADVERTISNG_TIMEOUT_EVENTS
    };
    err = bt_le_ext_adv_start(adv, &start_params);
#else
    err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
#endif
    
    if(err)
    {
        printk("Error: Advertising NOT started. return %d\n", err);
    }
    else 
	{
		printk("Bluetooth advertising started!\n");
        k_work_submit_to_queue(&offload_work_q, &adv_work.work);
	}
}

#if COMPILE_AD_TIMEOUT && CONFIG_BOARD_PRO_FLAG
static void advertising_max_events(struct bt_le_ext_adv *adv, struct bt_le_ext_adv_sent_info *info)
{

    uint8_t adv_events = info->num_sent;

    printk("Adertising finished with %d events.\n", adv_events);

    //Turn off our advertising indicate fader
    indicate_adv = false;

    //Effectively the timeout condition, but it's not a precise interval
    if (adv_events >= ADVERTISNG_TIMEOUT_EVENTS)
    {
        printk("Advertising timed out. Shutting down... \n");
        k_msleep(500);
        for(int i = HAPTIC_PWM_PERIOD; i > 0; i -= HAPTIC_PWM_INC)
        {
            pwm_set_dt(&pwm_buzz, HAPTIC_PWM_PERIOD, i);
            k_msleep(HAPTIC_FADE_DELAY_MS);
        }
        pwm_set_pulse_dt(&pwm_buzz, 0);
        pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
        sys_poweroff();
    }
    else
    {
        //Need to check if we connected.
        if(conn_handle == NULL)
        {
            printk("We didn't connect.\n");
            start_advertising_coded();
        }
            
        //else we conncted

    }
}
#endif

static void create_advertising_coded(void)
{
    int err;

    static struct bt_le_adv_param params = BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE |
                                                        BT_LE_ADV_OPT_EXT_ADV | 
                                                        BT_LE_ADV_OPT_CODED,
                                                        BT_GAP_ADV_FAST_INT_MIN_2, //100ms
                                                        BT_GAP_ADV_FAST_INT_MAX_2, //150ms
                                                        NULL);
#if COMPILE_AD_TIMEOUT
    static struct bt_le_ext_adv_cb advertising_cb =
        {
            .sent = advertising_max_events
        };


    err = bt_le_ext_adv_create(&params, &advertising_cb, &adv);
#else
    err = bt_le_ext_adv_create(&params, NULL, &adv);
#endif
    if(err)
    {
        printk("Error %d could not create ext_adv.\n", err);
    }

    else printk("CODED PHY Advertising Configured.\n");

    err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
    if(err)
    {
        printk("Error %d could not set_data.\n", err);
    }
    else printk("CODED PHY adv data set.\n");
}



// Instantiate the service and its characteristics
BT_GATT_SERVICE_DEFINE(
    paging_service,
    
    // Simple Service
    BT_GATT_PRIMARY_SERVICE(BT_UUID_PAGING_SERVICE),

    // Page Alert Characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_PAGE_ALERT_CHARACTERISTIC,
                    BT_GATT_CHRC_NOTIFY, 
                    BT_GATT_PERM_NONE,
                    NULL,
                    NULL,
                    NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_WRITE | BT_GATT_PERM_READ)
   
);


// Connected callback function
static void connected_cb(struct bt_conn *conn, uint8_t err)
{

    struct bt_conn_info info;
    char addr[BT_ADDR_LE_STR_LEN];

    conn_handle = conn;

    page_alert_attr = &paging_service.attrs[2];

	if (err) 
	{
		printk("Connection failed (err %u)\n", err);
		return;
	}
	else if(bt_conn_get_info(conn, &info))
	{
		printk("Could not parse connection info\n");
	}
	else
	{
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
		
		printk("\n \n Connection established!\n\
		Connected to: %s					 \n\
		Role: %u							 \n\
		Connection interval: %u				 \n\
		Slave latency: %u					 \n\
		Connection supervisory timeout: %u	 \n"
		, addr, info.role, info.le.interval, info.le.latency, info.le.timeout);

        for(int i = 0; i < HAPTIC_PWM_PERIOD; i += HAPTIC_PWM_INC)
        {
            pwm_set_dt(&pwm_buzz, HAPTIC_PWM_PERIOD, i);
            k_msleep(HAPTIC_FADE_DELAY_MS);
        }
        pwm_set_pulse_dt(&pwm_buzz, 0);
	}
}

// Disconnected callback function
static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected with reason %d\n", reason);
    k_sleep(K_MSEC(100));

    conn_handle = NULL;

    gpio_pin_set_dt(&buzz_en, 1);
    k_msleep(75);
    gpio_pin_set_dt(&buzz_en, 0);
    k_msleep(75);
    gpio_pin_set_dt(&buzz_en, 1);
    k_msleep(75);
    gpio_pin_set_dt(&buzz_en, 0);

    start_advertising_coded();

}

#if LE_PARAMETER_REQUESTS_ENABLED
static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	//If acceptable params, return true, otherwise return false.
	return true; 
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	struct bt_conn_info info; 
	char addr[BT_ADDR_LE_STR_LEN];
	
	if(bt_conn_get_info(conn, &info))
	{
		printk("Could not parse connection info\n");
	}
	else
	{
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
		
		printk("Connection parameters updated!	\n\
		Connected to: %s						\n\
		New Connection Interval: %u				\n\
		New Slave Latency: %u					\n\
		New Connection Supervisory Timeout: %u	\n"
		, addr, info.le.interval, info.le.latency, info.le.timeout);
	}
}
#endif

// Connection callback structure
static struct bt_conn_cb conn_callbacks = {
    .connected = connected_cb,
    .disconnected = disconnected_cb
#if LE_PARAMETER_REQUESTS_ENABLED
    ,
    .le_param_req			= le_param_req,
	.le_param_updated		= le_param_updated
#endif
};



void bt_notify_thread(struct k_work *work_item)
{
	if ((conn_handle != NULL))
    {
        // Send the notification
        if(bt_gatt_notify(conn_handle, page_alert_attr, page_p, 1))
        {
            printk("Error, unable to send notification\n");
        }
        else 
        {
            printk("Notified Peer of %d\n", (int) page_p[0]);

            
        
            int safety_count = 0;
            while(gpio_pin_get_dt(&button) && safety_count < 3000)
            {
                gpio_pin_set_dt(&buzz_en, 1);
                safety_count++;
                k_msleep(1);
            }
            gpio_pin_set_dt(&buzz_en, 0);
            
            //disable for a bit so as to prevent stacking notifications
            gpio_pin_interrupt_configure_dt(&button, GPIO_INT_DISABLE);
            k_busy_wait(5000000);
            gpio_pin_interrupt_configure_dt(&button, GPIO_INT_LOW_0);
        }
        gpio_pin_set_dt(&buzz_en, 0);
    }
    gpio_pin_set_dt(&buzz_en, 0);
    LOG_INF("Work thread executed.");
    k_yield();
}

void adv_fade_thread(struct k_work *work_tiem)
{

    int pulse_w = 0;
    bool fade_toggle = true;
    indicate_adv = true;
    while(conn_handle == NULL && indicate_adv)
    {
        pwm_set_dt(&pwm_ledb, PWM_FADE_PERIOD, pulse_w);
        if(fade_toggle) pulse_w += ADV_FADE_INC;
        else            pulse_w -= ADV_FADE_INC;

        if(pulse_w >= PWM_FADE_PERIOD)    fade_toggle = false;
        else if (pulse_w <= ADV_FADE_INC) fade_toggle = true;
        k_usleep(ADV_FADE_DELAY_US);
    }
    pwm_set_pulse_dt(&pwm_ledb, 0);
    return;
    
}

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
        k_work_submit_to_queue(&offload_work_q, &bt_work.work);
        gpio_pin_set_dt(&buzz_en, 0);
        k_yield();
        

}

#if CONFIG_BOARD_PRO_FLAG
void usb_detect_thread(struct k_work *work_item)
{
    while(gpio_pin_get_dt(&usb_detect))
    {
        for(int i = 0; i < PWM_FADE_PERIOD; i+=PWM_FADE_INC)
        {
            pwm_set_dt(&pwm_ledr, PWM_FADE_PERIOD, i);
            k_msleep(PWM_FADE_DELAY_MS);
        }
        for(int i = PWM_FADE_PERIOD; i > 0; i-=PWM_FADE_INC)
        {
            pwm_set_dt(&pwm_ledr, PWM_FADE_PERIOD, i);
            k_msleep(PWM_FADE_DELAY_MS);
        }
        pwm_set_dt(&pwm_ledr, PWM_FADE_PERIOD, 0);
        
        printk("USB PRESENT............\n");

    }

    printk("Powering Off: USB Removed.\n");
    k_msleep(4000);

    pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
    sys_poweroff();

}
#endif

#if COMPILE_ADC_PRINT && CONFIG_BOARD_PRO_FLAG
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
#if CONFIG_BOARD_PRO_FLAG
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
#endif

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

    
}

int main(void)
{
    init_pins();

    int err;
    
    k_work_queue_start(&offload_work_q, my_stack_area, K_THREAD_STACK_SIZEOF(my_stack_area), 
                                                                        WORKQ_PRIORITY, NULL);

    int button_held_count = 0;
    ARG_UNUSED(button_held_count);

#if COMPILE_ON_OFF && CONFIG_BOARD_PRO_FLAG
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

    else if(gpio_pin_get_dt(&button))
    {
        printk("BUTTON HELD CONDITION\n");
        k_msleep(300);

        int pulse_w = 0;
        while(gpio_pin_get_dt(&button))
        {
            button_held_count++;
            pwm_set_dt(&pwm_ledg, PWM_FADE_PERIOD, pulse_w);
            k_msleep(PWM_FADE_DELAY_MS);
            pulse_w += PWM_FADE_INC;
            if(button_held_count > POWERON_COUNT) break;
        }
        pwm_set_pulse_dt(&pwm_ledg, 0);

    }
    if(button_held_count < POWERON_COUNT)
    {
        printk("Powering Off: Button not held long enough.\n");
        k_msleep(2000);
        pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
        sys_poweroff();
    }
#endif

    //Continues...
    printk("~~~~~~FLAG POWER ON~~~~~~~~~\n");

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

    init_callbacks();

    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
    }

    // Register for connection callbacks
    bt_conn_cb_register(&conn_callbacks);

    // Create the Extended Advertising
    create_advertising_coded();

    // Start advertising
    start_advertising_coded();
    
    strcpy(bt_work.name, "Bluetooth Notify thread");
    k_work_init(&bt_work.work, bt_notify_thread);

    strcpy(adv_work.name, "Indicate Adv Blue Fader");
    k_work_init(&adv_work.work, adv_fade_thread);
                            

#if COMPILE_ADC_PRINT && CONFIG_BOARD_PRO_FLAG
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
            else if(conn_handle != NULL)
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

	}

    return 0;
}

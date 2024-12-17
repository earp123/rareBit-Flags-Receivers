#include <zephyr/kernel.h>

// Devicetree + GPIO includes
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif
#include <zephyr/irq.h>
//nrfx GPIO driver
#include <hal/nrf_gpio.h>

//For power manager
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <soc.h>

// Bluetooth include files
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/bas.h>

#define COMPILE_BATTERY 0
#define COMPILE_RGBLED 0
#define COMPILE_ONOFF 0
#define COMPILE_DFU  0

#if COMPILE_BATTERY
#include <battery.h>
#endif
#if COMPILE_RGBLED
#include <pulse_rgb.h>
#endif

#define ON_HOLD_TIME  4000
#define OFF_HOLD_TIME 3000

// SMP-related code
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>

#ifdef CONFIG_MCUMGR_CMD_FS_MGMT
#include <zephyr/device.h>
#include <zephyr/fs/fs.h>
//#include "fs_mgmt/fs_mgmt.h"
#include <zephyr/fs/littlefs.h>
#endif
#ifdef CONFIG_MCUMGR_CMD_OS_MGMT
#include "zephyr/mgmt/mcumgr/grp/os_mgmt/os_mgmt.h"
#endif
#ifdef CONFIG_MCUMGR_CMD_IMG_MGMT
#include "zephyr/mgmt/mcumgr/grp/img_mgmt/img_mgmt.h"
#endif
#ifdef CONFIG_MCUMGR_CMD_STAT_MGMT
#include "zephyr/mgmt/mcumgr/grp/stat_mgmt/stat_mgmt.h"
#endif
#ifdef CONFIG_MCUMGR_CMD_SHELL_MGMT
#include "zephyr/mgmt/mcumgr/grp/shell_mgmt/shell_mgmt.h"
#endif
#ifdef CONFIG_MCUMGR_CMD_FS_MGMT
#include "zephyr/mgmt/mcumgr/grp/fs_mgmt/fs_mgmt.h"
#endif

#define SLEEP_S 1U

#define DEVICE_NAME "rareBit Flag Demo"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME)-1)


//Macros required to get absolute pin numbers for use with NRF GPIO API (stupid!)
#define GET_GPIO_ID_OF_LED(LED_ID)	DT_PHANDLE_BY_IDX(LED_ID, gpios, 0)
#define GET_PORT_OF_GPIO(GPIO_ID)	DT_PROP(GPIO_ID, port)

#define GET_PORT(LED_ID)	GET_PORT_OF_GPIO(GET_GPIO_ID_OF_LED(LED_ID))
#define GET_PIN(LED_ID)		DT_GPIO_PIN(LED_ID, gpios)

#define LED0_ID 	DT_ALIAS(led0)
#define LED1_ID 	DT_ALIAS(led1)
#define LED2_ID 	DT_ALIAS(led2)
#define LED3_ID 	DT_ALIAS(led3)
//#define PWR_ON_ID	DT_ALIAS(pwr_on)

#define SW0_ID 		DT_ALIAS(sw0)
#define SW1_ID 		DT_ALIAS(sw1)
#define SW2_ID 		DT_ALIAS(sw2)

#define GPIO1_PORT	GET_PORT(LED1_ID)

#define LED0_PIN	GET_PIN(LED0_ID)
#define LED1_PIN	GET_PIN(LED1_ID)
#define LED2_PIN	GET_PIN(LED2_ID)
#define LED3_PIN	GET_PIN(LED3_ID)
//#define PWR_ON_PIN  GET_PIN(PWR_ON_ID)

#define SW0_PIN		GET_PIN(SW0_ID)
#define SW1_PIN		GET_PIN(SW1_ID)
#define SW2_PIN		GET_PIN(SW2_ID)

#define PAGE_BUTTON_PIN 	NRF_GPIO_PIN_MAP(GPIO1_PORT, SW0_PIN)
#define USB_PIN	            NRF_GPIO_PIN_MAP(GPIO1_PORT, SW1_PIN)
#define CHG_STAT_PIN	    NRF_GPIO_PIN_MAP(GPIO1_PORT, SW2_PIN)

#define HAPTIC_MOTOR_PIN    NRF_GPIO_PIN_MAP(GPIO1_PORT, LED0_PIN)

#define RED_LED_PIN       NRF_GPIO_PIN_MAP(GPIO1_PORT, LED1_PIN)
#define GREEN_LED_PIN     NRF_GPIO_PIN_MAP(GPIO1_PORT, LED2_PIN)
#define BLUE_LED_PIN      NRF_GPIO_PIN_MAP(GPIO1_PORT, LED3_PIN)
//#define PWR_ON_ABS_PIN 	  NRF_GPIO_PIN_MAP(GPIO1_PORT, PWR_ON_PIN)

#define LOW_BATT_INDIACTE_MV 3000
#define NORMAL_BATT_INDICATE_MV 3500

#define CONSOLE_LABEL DT_LABEL(DT_CHOSEN(zephyr_console))

#define MAX_TRANSMIT_SIZE 240//TODO figure this out

// Define the custom services and characteristics

#define PAGING_SERVICE_UUID 0x6d, 0x1b, 0xee, 0x1e, 0xee, 0x7d, 0xd0, 0xba, 0x7b, \
                            0x4b, 0xd5, 0x28, 0x01, 0x00, 0x21, 0x23

// Characteristic: Paging Characteristic UUID 23210002-28D5-4B7B-BA0F-7DEE1EEE1B6D
#define PAGE_ALERT_CHARACTERISTIC_UUID 0x6d, 0x1b, 0xee, 0x1e, 0xee, 0x7d, 0xd0, 0xba, 0x7b, \
                                 0x4b, 0xd5, 0x28, 0x02, 0x00, 0x21, 0x23

#define BT_UUID_PAGING_SERVICE        BT_UUID_DECLARE_128(PAGING_SERVICE_UUID)
#define BT_UUID_PAGE_ALERT_CHARACTERISTIC   BT_UUID_DECLARE_128(PAGE_ALERT_CHARACTERISTIC_UUID)

#define WORQ_THREAD_STACK_SIZE  4096
#define WORKQ_PRIORITY   4

// Define stack area used by workqueue thread
static K_THREAD_STACK_DEFINE(my_stack_area, WORQ_THREAD_STACK_SIZE);

// Define queue structure
static struct k_work_q offload_work_q = {0};


struct work_info {
    struct k_work work;
    char name[25];
} my_work;

uint8_t data_tx[MAX_TRANSMIT_SIZE];
bool debounce_check = false;


static struct bt_le_ext_adv *adv;
struct bt_conn *conn_handle = NULL;
const struct bt_gatt_attr *page_alert_attr = NULL;

bool page_flag = false;

int my_service_init(void)
{
    int err = 0;

    memset(&data_tx, 0, MAX_TRANSMIT_SIZE);

    return err;
}

// Bluetooth advertising data
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
		      0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, PAGING_SERVICE_UUID)
};


static void create_advertising_coded(void)
{
    int err;

    struct bt_le_adv_param params = BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE |
                                                        BT_LE_ADV_OPT_CODED |
                                                        BT_LE_ADV_OPT_EXT_ADV,
                                                        BT_GAP_ADV_FAST_INT_MIN_2,
                                                        BT_GAP_ADV_FAST_INT_MAX_2,
                                                        NULL);
                
    err = bt_le_ext_adv_create(&params, NULL, &adv);
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

void start_advertising_coded(void)
{
    int err;

    err = bt_le_ext_adv_start(adv, NULL);
    if(err)
    {
        printk("Error: Advertising NOT started. return %d\n", err);
        
    }
    else 
	{
		printk("Bluetooth advertising started!\n");
		nrf_gpio_pin_set(BLUE_LED_PIN);
	}

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
	}

	nrfx_gpiote_out_task_enable(HAPTIC_MOTOR_PIN);
	nrf_gpio_pin_clear(BLUE_LED_PIN);

}

// Disconnected callback function
static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected with reason %d\n", reason);
    k_sleep(K_MSEC(100));

	conn_handle = NULL;

    start_advertising_coded();
	nrfx_gpiote_out_task_disable(HAPTIC_MOTOR_PIN);
}
/*
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
}*/

// Connection callback structure
static struct bt_conn_cb conn_callbacks = {
    .connected = connected_cb,
    .disconnected = disconnected_cb//,
    //.le_param_req			= le_param_req,
	//.le_param_updated		= le_param_updated
};


void bt_pageAlert(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    
    // Send the notification
        
	//TODO check if connected before calling bt_gatt_notify, crashes otherwise
    if(bt_gatt_notify(conn, page_alert_attr, &data, len))
    {
        printk("Error, unable to send notification\n");
    }
    else {
        printk("Notified Peer of %d\n", (int) &data);
    }

}

void offload_function(struct k_work *work_tem)
{
	uint8_t page_d[] = {0x01};
	uint8_t * page_p = page_d;

	if (conn_handle != NULL)
		bt_pageAlert(conn_handle, page_p, 1);

#if COMPILE_ONFOFF
	int button_held_time = 0;
	while (!nrf_gpio_pin_read(PAGE_BUTTON_PIN))
	{
		k_busy_wait(1000);
		button_held_time++;

		if(button_held_time > (OFF_HOLD_TIME-1000))
		{	
			nrf_gpio_pin_clear(GREEN_LED_PIN);
		}	

		if (button_held_time > OFF_HOLD_TIME)
		{
		
			pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});

			printk("Powering Off...\n");
			nrf_gpio_pin_set(GREEN_LED_PIN);
			nrfx_gpiote_out_task_force(HAPTIC_MOTOR_PIN, 1);

			k_busy_wait(2000000);//give the user time to release the button
			
		}

	}
	
	nrf_gpio_pin_set(GREEN_LED_PIN);
	nrfx_gpiote_out_task_force(HAPTIC_MOTOR_PIN, 0);
	
#endif
		
	printk("Work thread executed. \n");
}

static void button_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *context)
{
	if (debounce_check)
	{
		printk("GPIO input event callback\n");

		k_work_submit_to_queue(&offload_work_q, &my_work.work);
			
	}
	debounce_check = !debounce_check;
}

nrfx_err_t configure_haptic_button(void)
{
    nrfx_err_t err = NRFX_SUCCESS;
	uint8_t in_channel;
    uint8_t out_channel;
    uint8_t ppi_channel;

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(gpiote)),
		    DT_IRQ(DT_NODELABEL(gpiote), priority),
		    nrfx_isr, nrfx_gpiote_irq_handler, 0);

	err = nrfx_gpiote_init(0);
	if (err != NRFX_SUCCESS) {
		printk("nrfx_gpiote_init error: 0x%08X", err);
		return err;
	}

	err = nrfx_gpiote_channel_alloc(&in_channel);
	if (err != NRFX_SUCCESS) {
		printk("Failed to allocate in_channel, error: 0x%08X", err);
		return err;
	}

    err = nrfx_gpiote_channel_alloc(&out_channel);
	if (err != NRFX_SUCCESS) {
		printk("Failed to allocate out_channel, error: 0x%08X", err);
		return err;
	}

	/* Initialize input pin to generate event on high to low transition
	 * (falling edge) and call button_handler()
	 */
	static const nrfx_gpiote_input_config_t input_config = {
		.pull = NRF_GPIO_PIN_PULLUP,
	};
	const nrfx_gpiote_trigger_config_t trigger_config = {
		.trigger = NRFX_GPIOTE_TRIGGER_TOGGLE,
		.p_in_channel = &in_channel,
	};
	static const nrfx_gpiote_handler_config_t handler_config = {
		.handler = button_handler,
	};

	err = nrfx_gpiote_input_configure(PAGE_BUTTON_PIN,
					  &input_config,
					  &trigger_config,
					  &handler_config);
	if (err != NRFX_SUCCESS) {
		printk("nrfx_gpiote_input_configure error: 0x%08X", err);
		return err;
	}

    static const nrfx_gpiote_output_config_t output_config = {
		.drive = NRF_GPIO_PIN_S0S1,
		.input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
		.pull = NRF_GPIO_PIN_NOPULL,
	};
	const nrfx_gpiote_task_config_t task_config = {
		.task_ch = out_channel,
		.polarity = NRF_GPIOTE_POLARITY_TOGGLE,
		.init_val = 0,
	};
	err = nrfx_gpiote_output_configure(HAPTIC_MOTOR_PIN,
					   &output_config,
					   &task_config);
	if (err != NRFX_SUCCESS) {
		printk("nrfx_gpiote_output_configure error: 0x%08X", err);
		return err;
	}

    nrfx_gpiote_trigger_enable(PAGE_BUTTON_PIN, true);
 

    /* Allocate a (D)PPI channel. */
	err = nrfx_gppi_channel_alloc(&ppi_channel);
	if (err != NRFX_SUCCESS) {
		printk("nrfx_gppi_channel_alloc error: 0x%08X", err);
		return 0;
	}

	/* Configure endpoints of the channel so that the input pin event is
	 * connected with the output pin OUT task. This means that each time
	 * the button is pressed, the LED pin will be toggled.
	 */
	nrfx_gppi_channel_endpoints_setup(ppi_channel,
		nrfx_gpiote_in_event_addr_get(PAGE_BUTTON_PIN),
		nrfx_gpiote_out_task_addr_get(HAPTIC_MOTOR_PIN));

	/* Enable the channel. */
	nrfx_gppi_channels_enable(BIT(ppi_channel));

    return err;
}


void main(void)
{
    nrfx_err_t err;

	printk("~~~~~~~~~~rareBit Flag Demo~~~~~~~~~~~~~\n");

	//nrf_gpio_cfg_output(PWR_ON_ABS_PIN);
	//nrf_gpio_pin_set(PWR_ON_ABS_PIN);

	nrf_gpio_cfg_input(PAGE_BUTTON_PIN, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(USB_PIN, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(CHG_STAT_PIN, NRF_GPIO_PIN_NOPULL);

	/* Configure to generate PORT event (wakeup) on button 1 press. */
	nrf_gpio_cfg_sense_set(PAGE_BUTTON_PIN, NRF_GPIO_PIN_SENSE_LOW);
	nrf_gpio_cfg_sense_set(USB_PIN, NRF_GPIO_PIN_SENSE_HIGH);

	nrf_gpio_cfg_output(RED_LED_PIN);
	nrf_gpio_cfg_output(GREEN_LED_PIN);
	nrf_gpio_cfg_output(BLUE_LED_PIN);


#if COMPILE_ONOFF
	
	while(!nrf_gpio_pin_read(USB_PIN))
	{
		nrf_gpio_pin_clear(RED_LED_PIN);
		k_msleep(1000);
		while(!nrf_gpio_pin_read(CHG_STAT_PIN))
		{
			nrf_gpio_pin_set(RED_LED_PIN);
			nrf_gpio_pin_clear(GREEN_LED_PIN);
		}	
	}

	nrf_gpio_pin_set(RED_LED_PIN);
	nrf_gpio_pin_set(GREEN_LED_PIN);

	int on_button_held = 0;
	while (!nrf_gpio_pin_read(PAGE_BUTTON_PIN))
	{
		nrf_gpio_pin_clear(GREEN_LED_PIN);
		k_busy_wait(1000);
		on_button_held++;

		if (on_button_held > ON_HOLD_TIME)
		{
			nrf_gpio_pin_set(GREEN_LED_PIN);
			break;
		} 

		//TODO continue to select different modes, like DFU, 
		//for on_button_held length values
	}

	if (on_button_held > ON_HOLD_TIME)
    {
        printk("Initializing...\n");
        k_busy_wait(2000000);
    }
	else
	{
		printk("Powering Off...\n");
		nrf_gpio_pin_set(GREEN_LED_PIN);

		pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});
	    k_sleep(K_SECONDS(SLEEP_S));

	}
#endif
	//Continues...
	k_work_queue_start(&offload_work_q, my_stack_area,
					K_THREAD_STACK_SIZEOF(my_stack_area), WORKQ_PRIORITY,
					NULL);
	strcpy(my_work.name, "Bluetooth Notify thread");
	k_work_init(&my_work.work, offload_function);
	
    //Initialize Button/Motor Peripherals
    err = configure_haptic_button();
    if (err != NRFX_SUCCESS)
    {
        return;
    } 

    int rc;

#if COMPILE_BATTERY
    //Initialize Battery Status checks
    rc = battery_measure_enable(true);
    if (rc != 0) {
        printk("Failed initialize battery measurement: %d\n", err);
        return;
    }
#endif

    // Initialize the Bluetooth stack
    rc = bt_enable(NULL);
    if (rc)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    rc = my_service_init();
    if(rc)
    {
        printk("Could not initialize simple service \n");
        return;
    }

    printk("....\n Bluetooth stack init success\n");

#if COMPILE_DFU
	smp_bt_register();
	//"E: Unable to register handle 0x0010" ??	       
#endif

    // Register for connection callbacks
    bt_conn_cb_register(&conn_callbacks);

    // Create the Extended Advertising
    create_advertising_coded();

    //Start advertising
    start_advertising_coded();

	int blink_cnt = 0;
    while(1)
    {

		k_msleep(100);
		blink_cnt++;
		if (blink_cnt >= 100)
		{
			nrf_gpio_pin_set(BLUE_LED_PIN);
			k_busy_wait(200000);
			nrf_gpio_pin_clear(BLUE_LED_PIN);
			blink_cnt = 0;
			
		}

#if COMPILE_BATTERY
		int batt_mV = battery_sample();

		if (batt_mV < 0) {
			printk("Failed to read battery voltage: %d\n",
			       batt_mV);
			break;
		}
        else{
            if (batt_mV < LOW_BATT_INDIACTE_MV)
            {
                //Set Red LED
            }
            else if (batt_mV > NORMAL_BATT_INDICATE_MV)
            {
                //Clear Red LED
            }
            else{
                //Do nothing
            }
        }
#endif
#if !COMPILE_ON_OFF
		if(nrf_gpio_pin_read(USB_PIN))
		{
			if(nrf_gpio_pin_read(CHG_STAT_PIN))
			{
				nrf_gpio_pin_clear(RED_LED_PIN);
				nrf_gpio_pin_set(GREEN_LED_PIN);
			}
			else
				nrf_gpio_pin_set(RED_LED_PIN);
		}
		else
		{
			nrf_gpio_pin_clear(RED_LED_PIN);
			nrf_gpio_pin_clear(GREEN_LED_PIN);
		}
			
#endif
    }
}
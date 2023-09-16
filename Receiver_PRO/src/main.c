#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <paging_client.h>

//For power manager
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <soc.h>

//nrfx GPIO driver
#include <hal/nrf_gpio.h>
// Devicetree + GPIO includes
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif
#include <zephyr/irq.h>

// SMP-related code
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>

#ifdef CONFIG_MCUMGR_CMD_FS_MGMT
#include <zephyr/device.h>
#include <zephyr/fs/fs.h>

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

#define COMPILE_ONOFF 1

#define ON_HOLD_TIME  4000
#define OFF_HOLD_TIME 3000

//Macros required to get absolute pin numbers for use with NRF GPIO API (stupid!)
#define GET_GPIO_ID_OF_LED(LED_ID)	DT_PHANDLE_BY_IDX(LED_ID, gpios, 0)
#define GET_PORT_OF_GPIO(GPIO_ID)	DT_PROP(GPIO_ID, port)

#define GET_PORT(LED_ID)	GET_PORT_OF_GPIO(GET_GPIO_ID_OF_LED(LED_ID))
#define GET_PIN(LED_ID)		DT_GPIO_PIN(LED_ID, gpios)

#define LED0_ID 	DT_ALIAS(led0)
#define LED1_ID 	DT_ALIAS(led1)
#define LED2_ID 	DT_ALIAS(led2)
#define LED3_ID 	DT_ALIAS(led3)
#define PWR_ON_ID	DT_ALIAS(pwr_on)

#define SW0_ID 		DT_ALIAS(sw0)
#define SW1_ID 		DT_ALIAS(sw1)
#define SW2_ID 		DT_ALIAS(sw2)

#define GPIO1_PORT	GET_PORT(LED1_ID)

#define LED0_PIN	GET_PIN(LED0_ID)
#define LED1_PIN	GET_PIN(LED1_ID)
#define LED2_PIN	GET_PIN(LED2_ID)
#define LED3_PIN	GET_PIN(LED3_ID)
#define PWR_ON_PIN  GET_PIN(PWR_ON_ID)

#define SW0_PIN		GET_PIN(SW0_ID)
#define SW1_PIN		GET_PIN(SW1_ID)
#define SW2_PIN		GET_PIN(SW2_ID)

#define RECEIVER_BUTTON_PIN	NRF_GPIO_PIN_MAP(GPIO1_PORT, SW0_PIN)
#define USB_PIN	            NRF_GPIO_PIN_MAP(GPIO1_PORT, SW1_PIN)
#define CHG_STAT_PIN	    NRF_GPIO_PIN_MAP(GPIO1_PORT, SW2_PIN)

#define HAPTIC_MOTOR_PIN    NRF_GPIO_PIN_MAP(GPIO1_PORT, LED0_PIN)

#define RED_LED_PIN       NRF_GPIO_PIN_MAP(GPIO1_PORT, LED1_PIN)
#define GREEN_LED_PIN     NRF_GPIO_PIN_MAP(GPIO1_PORT, LED2_PIN)
#define BLUE_LED_PIN      NRF_GPIO_PIN_MAP(GPIO1_PORT, LED3_PIN)
#define PWR_ON_ABS_PIN 	  NRF_GPIO_PIN_MAP(GPIO1_PORT, PWR_ON_PIN)

#define WORQ_THREAD_STACK_SIZE  4096
#define WORKQ_PRIORITY   5

// Define stack area used by workqueue thread
static K_THREAD_STACK_DEFINE(my_stack_area, WORQ_THREAD_STACK_SIZE);

// Define queue structure
static struct k_work_q offload_work_q = {0};


struct power_off_work_info {
    struct k_work work;
    char name[25];
} power_off_work;

struct page_alert_work_info {
    struct k_work work;
    char name[25];
} page_alert_work;

//static struct bt_gatt_exchange_params mtu_exchange_params[CONFIG_BT_MAX_CONN];

bool debounce_check = false;
bool notify_check = false;

int alert_type = 1; //just using 1 and 2 right now

// TODO Need to modify default_conn to better handle multiple connections.
//      Currently, only one of the connections are successfully
//      unreferenced upon disconnection. The other connection reference
//      remains and is causing issues trying to reconnect.
static struct bt_conn *default_conn;
static struct bt_pag_client pag_c;

static uint8_t volatile conn_count;

char NOT_CONNECTED[] = "FF:FF:FF:FF:FF:FF";

char AR1_device[] = "FF:FF:FF:FF:FF:FF";
char AR2_device[] = "FF:FF:FF:FF:FF:FF";

// Bluetooth advertising data
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
				  0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
				  0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d)};

void start_advertising_dfu(void)
{
	int err;

	// Start advertising
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err)
	{
		printk("Advertising start failed with err %d\n", err);
		return;
	}

	printk("Bluetooth advertising started!\n");
}

static uint8_t notify_func(struct bt_pag_client *pag_c, uint8_t page_alert, char page_addr[MAC_ADDRESS_LEN]) // SWR future inputs go here
{
	if(!notify_check)
	{
		printk("Notification, page_alert: %d\nDevice addr: %s\n", page_alert, page_addr);

		if (!strcmp(page_addr, AR1_device))
		{
			alert_type = 1;
		}
		else if (!strcmp(page_addr, AR2_device))
		{
			alert_type = 2;
		}
		
		k_work_submit_to_queue(&offload_work_q, &page_alert_work.work);
	}

	return BT_GATT_ITER_CONTINUE;
	// SWR things will happen
}
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
	char addr[MAC_ADDRESS_LEN]; // BT_ADDR_LE_STR_LEN];
	struct bt_conn *conn = bt_pag_conn(pag);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

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

	// assign the callback for notifications
	struct bt_pag_client_cb_init init = {
		.alert_cb = {
			.pag_notify_cb = notify_func,
		}};

	// pass the callback to the client structure used in bt_page_alert_subscribe
	bt_pag_cb_init(pag, &init);

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
	char addr[MAC_ADDRESS_LEN]; // BT_ADDR_LE_STR_LEN];
	struct bt_conn_le_create_param *conn_params;

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

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

	err = bt_conn_le_create(device_info->recv_info->addr, conn_params,
							BT_LE_CONN_PARAM_DEFAULT,
							&default_conn);

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
	char addr[MAC_ADDRESS_LEN]; // BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err)
	{
		printk("Failed to connect to %s (%u)\n", addr, conn_err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

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

	if (conn == default_conn)
	{

		err = bt_gatt_dm_start(conn, BT_UUID_PAGING_SERVICE, &discover_pag_cb, &pag_c);
		if (err)
		{
			printk("Failed to start discovery (err %d)\n", err);
		}
		else
		{
			// save the address of the newly added device
			if (!strcmp(AR1_device, NOT_CONNECTED))
			{
				strcpy(AR1_device, addr);
				printk("Added AR1_device at Address: %s\n", addr);
			}
			else if (!strcmp(AR2_device, NOT_CONNECTED))
			{
				strcpy(AR2_device, addr);
				printk("Added AR2_device at Address: %s\n", addr);
			}

			conn_count++;
			printk("Devices connected: %d\n", conn_count);

		}
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{

	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(conn);
	printk("%s unreferenced\n", addr);

	conn_count--;
	printk("Devices connected: %d\n", conn_count);

}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

void power_off_function(struct k_work *work_item)
{

	int button_held_time = 0;
	while (!nrf_gpio_pin_read(RECEIVER_BUTTON_PIN))
	{
		k_busy_wait(1000);
		button_held_time++;

		if(button_held_time > (OFF_HOLD_TIME-1000))
		{	
			nrf_gpio_pin_set(GREEN_LED_PIN);
		}	

		if (button_held_time > OFF_HOLD_TIME)
		{
			
			pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});

			printk("Powering Off...\n");
			nrf_gpio_pin_clear(GREEN_LED_PIN);
			
			k_busy_wait(2000000);//give the user time to release the button
			
		}
	}
	
		
	printk("Work thread executed. \n");
}

void page_alert_function(struct k_work *work_item){

	notify_check = true;
	printk("Receiver work is happening.\n");

	if (alert_type == 1){
		
		nrf_gpio_pin_set(HAPTIC_MOTOR_PIN);
		k_busy_wait(500000);
		nrf_gpio_pin_clear(HAPTIC_MOTOR_PIN);
		k_busy_wait(500000);
		nrf_gpio_pin_set(HAPTIC_MOTOR_PIN);
		k_busy_wait(500000);
		nrf_gpio_pin_clear(HAPTIC_MOTOR_PIN);
		k_busy_wait(500000);
		nrf_gpio_pin_set(HAPTIC_MOTOR_PIN);
		k_busy_wait(500000);
		nrf_gpio_pin_clear(HAPTIC_MOTOR_PIN);
		
	}
	else{
		
		nrf_gpio_pin_set(HAPTIC_MOTOR_PIN);
		k_busy_wait(200000);
		nrf_gpio_pin_clear(HAPTIC_MOTOR_PIN);
		k_busy_wait(200000);
		nrf_gpio_pin_set(HAPTIC_MOTOR_PIN);
		k_busy_wait(200000);
		nrf_gpio_pin_clear(HAPTIC_MOTOR_PIN);
		k_busy_wait(200000);
		nrf_gpio_pin_set(HAPTIC_MOTOR_PIN);
		k_busy_wait(200000);
		nrf_gpio_pin_clear(HAPTIC_MOTOR_PIN);
		k_busy_wait(200000);
		nrf_gpio_pin_set(HAPTIC_MOTOR_PIN);
		k_busy_wait(200000);
		nrf_gpio_pin_clear(HAPTIC_MOTOR_PIN);
		k_busy_wait(200000);
		nrf_gpio_pin_set(HAPTIC_MOTOR_PIN);
		k_busy_wait(200000);
		nrf_gpio_pin_clear(HAPTIC_MOTOR_PIN);

	}

	notify_check = false;
	
}

static void button_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *context)
{
	if (debounce_check)
	{
		printk("GPIO input event callback\n");

		k_work_submit_to_queue(&offload_work_q, &power_off_work.work);
			
	}
	debounce_check = !debounce_check;
}

nrfx_err_t configure_haptic_button(void)
{
    nrfx_err_t err = NRFX_SUCCESS;
	uint8_t in_channel;

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

	/* Initialize input pin to generate event on high to low transition
	 * (falling edge) and call button_handler()
	 */
	static const nrfx_gpiote_input_config_t input_config = {
		.pull = NRF_GPIO_PIN_NOPULL,
	};
	const nrfx_gpiote_trigger_config_t trigger_config = {
		.trigger = NRFX_GPIOTE_TRIGGER_HITOLO,
		.p_in_channel = &in_channel,
	};
	static const nrfx_gpiote_handler_config_t handler_config = {
		.handler = button_handler,
	};

	err = nrfx_gpiote_input_configure(RECEIVER_BUTTON_PIN,
					  &input_config,
					  &trigger_config,
					  &handler_config);
	if (err != NRFX_SUCCESS) {
		printk("nrfx_gpiote_input_configure error: 0x%08X", err);
		return err;
	}

    nrfx_gpiote_trigger_enable(RECEIVER_BUTTON_PIN, true);

    return err;
}

void main(void)
{

	nrf_gpio_cfg_output(PWR_ON_ABS_PIN);
	nrf_gpio_pin_set(PWR_ON_ABS_PIN);

	int err;

	printk("~~~~~~~~~rareBit Receiver Demo~~~~~~~~~~~\n");

	nrf_gpio_cfg_input(RECEIVER_BUTTON_PIN, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(USB_PIN, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(CHG_STAT_PIN, NRF_GPIO_PIN_NOPULL);

	nrf_gpio_cfg_output(HAPTIC_MOTOR_PIN);

	/* Configure to generate PORT event (wakeup) on button 1 press. */
	nrf_gpio_cfg_sense_set(RECEIVER_BUTTON_PIN, NRF_GPIO_PIN_SENSE_LOW);
	nrf_gpio_cfg_sense_set(USB_PIN, NRF_GPIO_PIN_SENSE_HIGH);

	nrf_gpio_cfg_output(RED_LED_PIN);
	nrf_gpio_cfg_output(GREEN_LED_PIN);
	nrf_gpio_cfg_output(BLUE_LED_PIN);

	

#if COMPILE_ONOFF

	int on_button_held = 0;
	while (!nrf_gpio_pin_read(RECEIVER_BUTTON_PIN))
	{
		nrf_gpio_pin_set(GREEN_LED_PIN);
		k_busy_wait(1000);
		on_button_held++;

		if (on_button_held > ON_HOLD_TIME)
		{
			nrf_gpio_pin_clear(GREEN_LED_PIN);
			break;
		} 

		//TODO continue to select different modes, like DFU, 
		//for on_button_held length values
	}
		
	while(nrf_gpio_pin_read(USB_PIN))
	{
		nrf_gpio_pin_set(RED_LED_PIN);
		k_msleep(1000);
		while(nrf_gpio_pin_read(CHG_STAT_PIN))
		{
			nrf_gpio_pin_clear(RED_LED_PIN);
			nrf_gpio_pin_set(GREEN_LED_PIN);
		}	
	}

	nrf_gpio_pin_clear(RED_LED_PIN);
	nrf_gpio_pin_clear(GREEN_LED_PIN);

	if (on_button_held > ON_HOLD_TIME)
    {
        printk("Initializing...\n");
        k_busy_wait(2000000);
    }
	else
	{
		printk("Powering Off...\n");
		nrf_gpio_pin_clear(GREEN_LED_PIN);

		pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});
	    k_msleep(2000);

	}
#endif
	//Continues...
	k_work_queue_start(&offload_work_q, my_stack_area,
					K_THREAD_STACK_SIZEOF(my_stack_area), WORKQ_PRIORITY,
					NULL);
	strcpy(power_off_work.name, "Receiver Button Push thread");
	k_work_init(&power_off_work.work, power_off_function);

	strcpy(page_alert_work.name, "Page Alert thread");
	k_work_init(&page_alert_work.work, page_alert_function);
	

    //Initialize Button/Motor Peripherals
    err = configure_haptic_button();
    if (err != NRFX_SUCCESS)
    {
		printk("Haptic Button config failed (err %d)\n", err);
		nrf_gpio_pin_set(RED_LED_PIN);
        return;
    } 

	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		//nrf_gpio_pin_set(RED_LED_PIN);
		return;
	}

	printk("Bluetooth initialized\n");

	//TODO
	//smp_bt_register();
	//"E: Unable to register handle 0x0010"

	scan_init();

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err)
	{
		printk("Scanning failed to start (err %d)\n", err);
		//nrf_gpio_pin_set(RED_LED_PIN);
		return;
	}

	printk("Scanning successfully started\n");

	while(1)
	{
		k_msleep(100);
		//TODO handle case of USB_PIN detected while on
	} 
}

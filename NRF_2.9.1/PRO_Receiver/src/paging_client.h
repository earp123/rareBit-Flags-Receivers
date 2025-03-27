#include <zephyr/kernel.h>
#include <sys/types.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/gatt_dm.h>

#define MAC_ADDRESS_LEN 18

// Define the custom services and characteristics

#define PAGING_SERVICE_UUID 0x6d, 0x1b, 0xee, 0x1e, 0xee, 0x7d, 0xd0, 0xba, 0x7b, \
                            0x4b, 0xd5, 0x28, 0x01, 0x00, 0x21, 0x23

// Characteristic: Paging Characteristic UUID 23210002-28D5-4B7B-BA0F-7DEE1EEE1B6D
#define PAGE_ALERT_CHARACTERISTIC_UUID 0x6d, 0x1b, 0xee, 0x1e, 0xee, 0x7d, 0xd0, 0xba, 0x7b, \
                                 0x4b, 0xd5, 0x28, 0x02, 0x00, 0x21, 0x23

#define BT_UUID_PAGING_SERVICE              BT_UUID_DECLARE_128(PAGING_SERVICE_UUID)
#define BT_UUID_PAGE_ALERT_CHARACTERISTIC   BT_UUID_DECLARE_128(PAGE_ALERT_CHARACTERISTIC_UUID)

struct bt_pag_client;
struct bt_page_alert_characteristic;

struct bt_pag_client_cb{
/**
 * @brief Value notification callback.
 *
 * This function is called every time the server sends a notification
 * for a changed value.
 *
 * @param pag           Paging Client object.
 * @param page_alert    The notified page alert value, or
 *                      @ref BT_BAS_VAL_INVALID if the notification
 *                      was interrupted by the server
 *                      (NULL received from the stack).
 */
 uint8_t (*pag_notify_cb)(struct bt_pag_client *pag, uint8_t page_alert, char page_addr[MAC_ADDRESS_LEN]);
};

struct bt_page_alert_charcteristic {

	struct bt_gatt_subscribe_params notify_params;

	/** Notify callback. */
	struct bt_pag_client_cb notify_cb;

	/** Properties of the service. */
	uint8_t properties;

	uint16_t ccc_handle;
    /** handle of the page alert value*/
	uint16_t val_handle;
    /** Page Alert value*/
};

/** @brief rareBit Paging Client instance. */
//TODO add property that IDs which device sent the page alert
struct bt_pag_client {
	/** Connection handle. */
	struct bt_conn *conn;
	
	struct bt_page_alert_charcteristic page_chrc;
	
	uint8_t page_alert;
	
	/** Notification supported. */
	bool notify;
};

/** @brief PAG Client callback structure. */
struct bt_pag_client_cb_init {

	/** Callbacks provided by the user. */
	struct bt_pag_client_cb alert_cb;
};

/**
 * @brief Initialize the Paging Client instance.
 *
 * You must call this function on the Paging Client object before
 * any other function.
 *
 * @param pag  Paging Client object.
 */
void bt_pag_cb_init(struct bt_pag_client *pag,
		const struct bt_pag_client_cb_init *init_param);

/**
 * @brief Assign handles to the PAG Client instance.
 *
 * This function should be called when a connection with a peer has been
 * established, to associate the connection to this instance of the module.
 * This makes it possible to handle multiple connections and associate each
 * connection to a particular instance of this module.
 * The GATT attribute handles are provided by the GATT Discovery Manager.
 *
 * @param dm    Discovery object.
 * @param pag   Paging Client object.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 * @retval (-ENOTSUP) Special error code used when the UUID
 *         of the service does not match the expected UUID.
 */
int bt_pag_handles_assign(struct bt_gatt_dm *dm,
			  struct bt_pag_client *pag);

/**
 * @brief Subscribe to the page alert value change notification.
 *
 * @param pag Paging Client object.

 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 * @retval -ENOTSUP Special error code used if the connected server
 *         does not support notifications.
 */
int bt_page_alert_subscribe(struct bt_pag_client *pag);



/**
 * @brief Get the connection object that is used with a given Paging Client.
 *
 * @param pag Paging Client object.
 *
 * @return Connection object.
 */
struct bt_conn *bt_pag_conn(const struct bt_pag_client *pag);
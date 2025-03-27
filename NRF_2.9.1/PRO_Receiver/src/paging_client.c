#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <paging_client.h>

static uint8_t notify_process(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
	uint8_t *bytes = (uint8_t *) data;
	uint8_t page_alert = bytes[0];
	struct bt_pag_client *pag_c;

	char page_addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), page_addr, sizeof(page_addr));

	pag_c = CONTAINER_OF(params, struct bt_pag_client, page_chrc.notify_params);
	
	return pag_c->page_chrc.notify_cb.pag_notify_cb(pag_c, page_alert, page_addr);
	
}

/**
 * @brief Reinitialize the Paging Client.
 *
 * @param pag Paging Client object.
 */
static void pag_reinit(struct bt_pag_client *pag)
{
	pag->page_chrc.val_handle = 0;
	pag->page_alert = 0;//BT_PAGE?_VAL_INVALID; decided to forego the INVALID label
	pag->conn = NULL;
	pag->page_chrc.notify_cb.pag_notify_cb = NULL;
	pag->notify = true;
}


void bt_pag_cb_init(struct bt_pag_client *pag, const struct bt_pag_client_cb_init *pag_init_param)
{
	memcpy(&pag->page_chrc.notify_cb, &pag_init_param->alert_cb, sizeof(pag->page_chrc.notify_cb));

}

int bt_pag_handles_assign(struct bt_gatt_dm *dm,
				 struct bt_pag_client *pag)
{
	const struct bt_gatt_dm_attr *gatt_service_attr =
			bt_gatt_dm_service_get(dm);
	const struct bt_gatt_service_val *gatt_service =
			bt_gatt_dm_attr_service_val(gatt_service_attr);
	const struct bt_gatt_dm_attr *gatt_chrc;
	const struct bt_gatt_dm_attr *gatt_desc;
	const struct bt_gatt_chrc *chrc_val;

	if (bt_uuid_cmp(gatt_service->uuid, BT_UUID_PAGING_SERVICE)) {
		return -ENOTSUP;
	}
	printk("Getting handles from Paging service.\n");


	/* When workqueue is used its instance cannont be cleared. */
	pag_reinit(pag);

	/* Page Alert characteristic */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_PAGE_ALERT_CHARACTERISTIC);
	if (!gatt_chrc) {
		printk("No page alert characteristic found.\n");
		return -EINVAL;
	}
	chrc_val = bt_gatt_dm_attr_chrc_val(gatt_chrc);
	__ASSERT_NO_MSG(chrc_val); /* This is internal function and it has to
				    * be called with characteristic attribute
				    */
	pag->page_chrc.properties = chrc_val->properties;
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc,
					    BT_UUID_PAGE_ALERT_CHARACTERISTIC);
	if (!gatt_desc) {
		printk("No page alert characteristic value found.\n");
		return -EINVAL;
	}
	pag->page_chrc.val_handle = gatt_desc->handle;

	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc,
					    BT_UUID_GATT_CCC);
	if (!gatt_desc) {
		printk("No CCC found.\n");
		return -EINVAL;
	}
	pag->page_chrc.ccc_handle = gatt_desc->handle;

    pag->notify = true;
	

	/* Finally - save connection object */
	pag->conn = bt_gatt_dm_conn_get(dm);
	return 0;
}

int bt_page_alert_subscribe(struct bt_pag_client *pag)
{
	int err;


	if (!pag->conn) {
		return -EINVAL;
	}
	if (!(pag->page_chrc.properties & BT_GATT_CHRC_NOTIFY)) {
		return -ENOTSUP;
	}


	struct bt_gatt_subscribe_params *params = &pag->page_chrc.notify_params;

	printk("Attempting to Subscribe to Characterstic\n");

	params->notify = notify_process;
	params->value = BT_GATT_CCC_NOTIFY;
	params->value_handle = pag->page_chrc.val_handle;
	params->ccc_handle = pag->page_chrc.ccc_handle;
	
	atomic_set_bit(params->flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

	err = bt_gatt_subscribe(pag->conn, params);
	if (err) {
		printk("Report notification subscribe error: %d.\n", err);
		//pag->page_chrc.notify_cb = NULL;
		return err;
	}
	printk("Report subscribed.\n");
	return err;
}

struct bt_conn *bt_pag_conn(const struct bt_pag_client *pag)
{
	return pag->conn;
}
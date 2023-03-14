/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "main.h"
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>


#include <zephyr/settings/settings.h>

//#include <zephyr/usb/usb_device.h>

//#include <zephyr/drivers/uart.h>


#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/services/hrs.h>


LOG_MODULE_REGISTER(ppgmain);

//#include "cts.h"

//Pulse Oximeter service 0x1822
#define BT_UUID_POS_VAL 0x1822
#define BT_UUID_POS BT_UUID_DECLARE_16(BT_UUID_POS_VAL)

//0x2A5F PLX Continuous Measurement
//#define BT_UUID_PLX_MEASUREMENT_VAL 0x2A5F
//#define BT_UUID_PLX_MEASUREMENT BT_UUID_DECLARE_16(BT_UUID_PLX_MEASUREMENT_VAL)

//Fingertip Pulse Oximeter 0x0C41
#define BT_UUID_FINGERTIP_PO_MEASUREMENT_VAL 0x0C41
#define BT_UUID_FINGERTIP_PO_MEASUREMENT BT_UUID_DECLARE_16(BT_UUID_FINGERTIP_PO_MEASUREMENT_VAL)


#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

volatile bool notify_enable = 0;

K_EVENT_DEFINE(notify_event);

static void vnd_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	notify_enable = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("value %i", value);
	k_event_post(&notify_event, (value ? evt_powerup : evt_powerdown));
}

BT_GATT_SERVICE_DEFINE(ppg_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_POS),	
		BT_GATT_CHARACTERISTIC(BT_UUID_FINGERTIP_PO_MEASUREMENT,
					BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE |
			        BT_GATT_CHRC_NOTIFY, 
					BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, NULL, NULL, NULL),
			BT_GATT_CCC(vnd_ccc_cfg_changed,
					BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),				
);


static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
			   BT_UUID_16_ENCODE(BT_UUID_POS_VAL),
		      	BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
			  ),	
};

K_SEM_DEFINE(sem_initialized_max30102, 0, 1);
K_SEM_DEFINE(sem_interruptmax30102, 0, 1);
K_FIFO_DEFINE(ppg_items);

static struct gpio_callback interrupt_max30102;
static const struct gpio_dt_spec signal = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, signal_gpios);





void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	LOG_DBG("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err 0x%02x)\n", err);
	} else {
		LOG_INF("Connected\n");		
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02x)\n", reason);	
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,	
};


static void bt_ready(void)
{
	int err;

	LOG_DBG("Bluetooth initialized\n");
	

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return;
	}

	LOG_DBG("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
	
};


void max30102isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {	
	k_sem_give(&sem_interruptmax30102);			
}

void attachinterrupt(void) {
	
	__ASSERT(device_is_ready(signal.port), "custom device not ready");
	gpio_pin_configure_dt(&signal, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&signal, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&interrupt_max30102, max30102isr, BIT(signal.pin));
	gpio_add_callback(signal.port, &interrupt_max30102);	
	LOG_INF("attached");
}



void main(void)
{


// #if DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_shell_uart), zephyr_cdc_acm_uart)
// 	const struct device *dev;
// 	uint32_t dtr = 0;

// 	dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
// 	if (!device_is_ready(dev) || usb_enable(NULL)) {
// 		return;
// 	}

// 	while (!dtr) {
// 		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
// 		k_sleep(K_MSEC(100));
// 	}
// #endif	



	struct bt_gatt_attr *vnd_ind_attr;
	char str[BT_UUID_STR_LEN];
	int err;
	LOG_INF("enabling bt");
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return;
	}
	LOG_INF("bt enabled");

	bt_ready();

	bt_gatt_cb_register(&gatt_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);
	
	static struct bt_uuid_16 vnd_enc_uuid = BT_UUID_INIT_16(BT_UUID_FINGERTIP_PO_MEASUREMENT_VAL);
	vnd_ind_attr = bt_gatt_find_by_uuid(ppg_svc.attrs, ppg_svc.attr_count,
					    &vnd_enc_uuid.uuid);
						

	//bt_uuid_to_str(&vnd_enc_uuid.uuid, str, sizeof(str));
	bt_uuid_to_str(BT_UUID_POS, str, sizeof(str));
	LOG_DBG("Indicate VND attr %p (UUID %s)\n", vnd_ind_attr, str);	
	
	if(max30102_init()) {
		LOG_ERR("could not initialize");
		k_sleep(K_FOREVER);		
	}		
	attachinterrupt();	
	uint8_t max_read_state = max30102_interrupt_state(); //clear interrupt;
	LOG_DBG("max initialized, read state:%i", max_read_state);	
	k_sem_give(&sem_initialized_max30102);			
	shutdown();


	fifo_item_t *queued;
	for(;;) {
		
		queued = k_fifo_get(&ppg_items, K_FOREVER);	
		uint32_t test = 42;
		if (notify_enable) {			
			
			err = bt_gatt_notify(NULL, vnd_ind_attr, &test, sizeof(uint32_t));
			if (err) {
				LOG_ERR("Notify error: %d", err);
			} else {
				LOG_INF("Send notify ir %i", queued->item->ir);	
			}		
		}		
		k_free(queued->item);
	}
}


void max30102read(void) {	
	k_sem_take(&sem_initialized_max30102, K_FOREVER);
	
	uint8_t amount_samples;
	uint8_t isrstate;
	ppg_item_t buffer[32]; //maximum amount for ic
	while (1) {		
		k_sem_take(&sem_interruptmax30102, K_FOREVER);
		LOG_DBG("sem_interrupt_received");
		isrstate = max30102_interrupt_state();
		if (isrstate != 0b10000000) {
			LOG_WRN("unexpected max30102int: %d", isrstate);			
			continue;
		} 
		amount_samples = GetSamplesAvailable();		
		
		max30102data(buffer, amount_samples);
		
		for (int i = 0; i < amount_samples; i++)
		{						
			fifo_item_t toqueue; // = k_malloc(sizeof(fifo_item_t));			
			toqueue.item = k_malloc(sizeof(ppg_item_t));
			memcpy(toqueue.item, &buffer[i], sizeof(ppg_item_t));
			LOG_DBG("queued ir %i:", toqueue.item->ir);
			k_fifo_put(&ppg_items, &toqueue);
		} 			
	}
}
void power_thread(void) {
	for(;;) {
		uint32_t events;
		events = k_event_wait(&notify_event, (evt_powerup | evt_powerdown), true, K_FOREVER);
		LOG_DBG("received %i", events);
		if(events == evt_powerdown) {
			shutdown();		
		} else if(events == evt_powerup) {
			powerup();
		}
	}
}

K_THREAD_DEFINE(notify_read_id, 1024, power_thread, NULL, NULL,
		NULL, 8, 0, 0);

K_THREAD_DEFINE(max30102_read_thread_id, 1024, max30102read, NULL, NULL,
		NULL, 7, 0, 0);


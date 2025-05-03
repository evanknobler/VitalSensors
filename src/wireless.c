#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(wireless, LOG_LEVEL_INF);

#include "wireless.h"
#include "drivers.h"

#define BT_UUID_CUSTOM_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcde00)

#define BT_UUID_MAX86140_HR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcde11)
#define BT_UUID_MAX86140_SPO2_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcde12)

#define BT_UUID_MAX30001_ECG_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcde21)
#define BT_UUID_MAX30001_BIOZ_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcde22)
#define BT_UUID_MAX30001_RTOR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcde23)

#define BT_UUID_WRITE_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcde99)

#define MAX30001_STANDARD_PERIOD 30 // seconds
#define MAX86140_STANDARD_PERIOD 18 // seconds
#define MAX86140_ABNORMAL_PERIOD 8  // seconds

static struct bt_uuid_128 svc_uuid      = BT_UUID_INIT_128(BT_UUID_CUSTOM_SERVICE_VAL);
static struct bt_uuid_128 notify_hr = BT_UUID_INIT_128(BT_UUID_MAX86140_HR_VAL);
static struct bt_uuid_128 notify_spo2 = BT_UUID_INIT_128(BT_UUID_MAX86140_SPO2_VAL);
static struct bt_uuid_128 notify_ecg = BT_UUID_INIT_128(BT_UUID_MAX30001_ECG_VAL);
static struct bt_uuid_128 notify_bioz = BT_UUID_INIT_128(BT_UUID_MAX30001_BIOZ_VAL);
static struct bt_uuid_128 notify_rtor = BT_UUID_INIT_128(BT_UUID_MAX30001_RTOR_VAL);
static struct bt_uuid_128 write_uuid = BT_UUID_INIT_128(BT_UUID_WRITE_CHAR_VAL);

static ssize_t write_handler(
    struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf, uint16_t len,
    uint16_t offset, uint8_t flags);

BT_GATT_SERVICE_DEFINE(custom_svc,
        BT_GATT_PRIMARY_SERVICE(&svc_uuid),
    
        BT_GATT_CHARACTERISTIC(&notify_hr.uuid,
                               BT_GATT_CHRC_NOTIFY,
                               BT_GATT_PERM_NONE,
                               NULL, NULL, NULL),
        BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
        BT_GATT_CHARACTERISTIC(&notify_spo2.uuid,
                               BT_GATT_CHRC_NOTIFY,
                               BT_GATT_PERM_NONE,
                               NULL, NULL, NULL),
        BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
        BT_GATT_CHARACTERISTIC(&notify_ecg.uuid,
                               BT_GATT_CHRC_NOTIFY,
                               BT_GATT_PERM_NONE,
                               NULL, NULL, NULL),
        BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
        BT_GATT_CHARACTERISTIC(&notify_bioz.uuid,
                               BT_GATT_CHRC_NOTIFY,
                               BT_GATT_PERM_NONE,
                               NULL, NULL, NULL),
        BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

        BT_GATT_CHARACTERISTIC(&notify_rtor.uuid,
                               BT_GATT_CHRC_NOTIFY,
                               BT_GATT_PERM_NONE,
                               NULL, NULL, NULL),
        BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

        BT_GATT_CHARACTERISTIC(&write_uuid.uuid,
            BT_GATT_CHRC_WRITE,
            BT_GATT_PERM_WRITE,
            NULL, write_handler, NULL)
    );

static struct bt_conn *current_conn;
static struct k_work_delayable max30001_ble_work;
static struct k_work_delayable max86140_ble_work;
static bool max86140_abnormal_condition = false;

static ssize_t write_handler(
    struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf, uint16_t len,
    uint16_t offset, uint8_t flags)
{
    LOG_INF("Received %d bytes from central", len);
    const uint8_t *data = buf;
    for (int i = 0; i < len; i++) {
        LOG_INF("%02X ", data[i]);
    }
    LOG_INF("\n");

    // do something when data is received...

    return len;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    current_conn = bt_conn_ref(conn);
    LOG_INF("Connected");
    
    k_work_schedule(&max30001_ble_work, K_SECONDS(MAX30001_STANDARD_PERIOD));
    k_work_schedule(&max86140_ble_work, K_SECONDS(MAX86140_STANDARD_PERIOD));
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    k_work_cancel_delayable(&max30001_ble_work);
    k_work_cancel_delayable(&max86140_ble_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

void max30001_ble_handler(struct k_work *work)
{
    LOG_INF("Sending max30001");

    if (!current_conn) {
        LOG_WRN("No connection, skipping BLE message\n");
    }
    else
    {
        int err;

        // Send BLE notification
        err = bt_gatt_notify(current_conn, &custom_svc.attrs[7], &ecg, sizeof(ecg));
        err |= bt_gatt_notify(current_conn, &custom_svc.attrs[10], &bioz, sizeof(bioz));
        err |= bt_gatt_notify(current_conn, &custom_svc.attrs[13], &rtor, sizeof(rtor));
        if (err) {
            LOG_ERR("Failed to notify (err %d)\n", err);
        } else {
            LOG_INF("BLE message sent\n");
        } 
    }

    k_work_schedule(&max30001_ble_work, K_SECONDS(MAX30001_STANDARD_PERIOD));
}

void max86140_ble_handler(struct k_work *work)
{
    LOG_INF("Sending max86140");

    if (!current_conn) {
        LOG_WRN("No connection, skipping BLE message\n");
    }
    else
    {
        int err;

        // Send BLE notification
        err = bt_gatt_notify(current_conn, &custom_svc.attrs[1], &hr, sizeof(hr));
        err |= bt_gatt_notify(current_conn, &custom_svc.attrs[4], &spo2, sizeof(spo2));
        if (err) {
            LOG_ERR("Failed to notify (err %d)\n", err);
        } else {
            LOG_INF("BLE message sent\n");
        } 
    }

    int next_interval = max86140_abnormal_condition ? MAX86140_ABNORMAL_PERIOD : MAX86140_STANDARD_PERIOD;
    k_work_schedule(&max86140_ble_work, K_SECONDS(next_interval));
}

int init_wireless()
{
    int err;

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth enable failed");
        return err;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0);
    if (err) {
        LOG_ERR("Bluetooth advertising failed");
        return err;
    }

    k_work_init_delayable(&max30001_ble_work, max30001_ble_handler);
    k_work_init_delayable(&max86140_ble_work, max86140_ble_handler);

    return 0;
}

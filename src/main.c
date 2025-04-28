#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/nus.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#include <stdint.h>

#define MAX30001_SPI_NODE DT_ALIAS(max30001)
#define MAX86140_SPI_NODE DT_ALIAS(max86140)

LOG_MODULE_REGISTER(main);

static struct bt_conn *current_conn;

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
    LOG_INF("Received %d bytes from central", len);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    current_conn = bt_conn_ref(conn);
    LOG_INF("Connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb,
};

static const struct spi_dt_spec max30001_spi = {
    .bus = DEVICE_DT_GET(DT_BUS(MAX30001_SPI_NODE)),
    .config = {
        .frequency = 12000000,
        .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
        .slave = DT_REG_ADDR(MAX30001_SPI_NODE),
        .cs = NULL,
    },
};

static const struct spi_dt_spec max86140_spi = {
    .bus = DEVICE_DT_GET(DT_BUS(MAX86140_SPI_NODE)),
    .config = {
        .frequency = 4000000,
        .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
        .slave = DT_REG_ADDR(MAX86140_SPI_NODE),
        .cs = NULL,
    },
};

static int max30001_spi_write_reg(uint8_t reg, uint32_t value)
{
    uint8_t tx_buf[4] = {
        (reg << 1) | 0,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 0) & 0xFF
    };

    struct spi_buf tx_bufs = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx = {.buffers = &tx_bufs, .count = 1};

    return spi_write_dt(&max30001_spi, &tx);
}

static int max30001_spi_read_reg(uint8_t reg, uint32_t *data)
{
    uint8_t tx_buf[4] = {(reg << 1) | 1, 0, 0, 0};
    uint8_t rx_buf[4];

    struct spi_buf tx_bufs = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf rx_bufs = {.buf = rx_buf, .len = sizeof(rx_buf)};

    struct spi_buf_set tx_set = {.buffers = &tx_bufs, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx_bufs, .count = 1};

    int ret = spi_transceive_dt(&max30001_spi, &tx_set, &rx_set);
    if (ret == 0) {
        *data = ((uint32_t)rx_buf[1] << 16) | ((uint32_t)rx_buf[2] << 8) | (uint32_t)rx_buf[3];
    }
    return ret;
}

static int max30001_init(void)
{
    int ret;

    if (!device_is_ready(max30001_spi.bus)) {
        LOG_ERR("MAX30001 SPI bus not ready");
        return -ENODEV;
    }

    // Software reset
    ret = max30001_spi_write_reg(0x08, 0x000000);
    if (ret) return ret;

    k_msleep(10);
    
    // Configuration
    ret = max30001_spi_write_reg(0x10, 0x0E0000);
    if (ret) return ret;

    // EMUX configure
    ret = max30001_spi_write_reg(0x14, 0x000000);
    if (ret) return ret;

    // Enable R-R
    ret = max30001_spi_write_reg(0x1D, 0x1FD180);
    if (ret) return ret;

    // Sync command
    ret = max30001_spi_write_reg(0x09, 0x000000);
    if (ret) return ret;
    
    // Reset FIFO
    ret = max30001_spi_write_reg(0x0A, 0x000000);
    if (ret) return ret;

    return 0;
}

static int max30001_read_ecg_fifo(int32_t *sample, uint8_t *etag, uint8_t *ptag)
{
    uint32_t data;
    int ret = max30001_spi_read_reg(0x21, &data);
    if (ret == 0) {
        int32_t raw_sample = (int32_t)(data >> 6);
        if (raw_sample & (1 << 17)) {
            raw_sample |= 0xFFFC0000;
        }
        *sample = raw_sample;
        *etag = (uint8_t)((data >> 3) & 0x07);
        *ptag = (uint8_t)(data & 0x07);
    }
    return ret;
}

static int max30001_read_bioz_fifo(int32_t *sample, uint8_t *btag)
{
    uint32_t data;
    int ret = max30001_spi_read_reg(0x23, &data);
    if (ret == 0) {
        int32_t raw_sample = (int32_t)(data >> 4);
        if (raw_sample & (1 << 19)) {
            raw_sample |= 0xFFF00000;
        }
        *sample = raw_sample;
        *btag = (uint8_t)(data & 0x07);
    }
    return ret;
}

static int max30001_read_rtor(uint16_t *sample)
{
    uint32_t data;
    int ret = max30001_spi_read_reg(0x25, &data);
    if (ret == 0) {
        *sample = (uint16_t)(data >> 10);
    }
    return ret;
}

static int max86140_spi_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2] = {reg, value};

    struct spi_buf tx_bufs = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx = {.buffers = &tx_bufs, .count = 1};

    return spi_write_dt(&max86140_spi, &tx);
}

static int max86140_spi_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx_buf[1] = {reg};
    uint8_t rx_buf[1];

    struct spi_buf tx_bufs = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf rx_bufs = {.buf = rx_buf, .len = sizeof(rx_buf)};

    struct spi_buf_set tx_set = {.buffers = &tx_bufs, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx_bufs, .count = 1};

    int ret = spi_transceive_dt(&max86140_spi, &tx_set, &rx_set);
    if (ret == 0) {
        *value = rx_buf[0];
    }
    return ret;
}

static int max86140_init(void)
{
    int ret;

    if (!device_is_ready(max86140_spi.bus)) {
        LOG_ERR("MAX86140 SPI bus not ready");
        return -ENODEV;
    }

    // Reset
    ret = max86140_spi_write_reg(0x0D, 0x01);
    if (ret) return ret;

    k_msleep(2);

    // LED Sequencing
    ret = max86140_spi_write_reg(0x20, 0x21);
    if (ret) return ret;

    // LED 1 current
    ret = max86140_spi_write_reg(0x23, 0x7D);
    if (ret) return ret;

    // LED 2 current
    ret = max86140_spi_write_reg(0x24, 0x7D);
    if (ret) return ret;

    return 0;
}

static int max86140_read_fifo(uint32_t *red_sample, uint32_t *ir_sample)
{
    uint8_t tx_buf[1] = { 0x07 << 1 | 1 };
    uint8_t rx_buf[6] = { 0 };

    struct spi_buf tx_bufs = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf rx_bufs = {.buf = rx_buf, .len = sizeof(rx_buf)};

    struct spi_buf_set tx_set = {.buffers = &tx_bufs, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx_bufs, .count = 1};

    int ret = spi_transceive_dt(&max86140_spi, &tx_set, &rx_set);
    if (ret == 0) {
        *red_sample = ((uint32_t)(rx_buf[0] << 16) | (rx_buf[1] << 8) | rx_buf[2]) >> 6;
        *ir_sample  = ((uint32_t)(rx_buf[3] << 16) | (rx_buf[4] << 8) | rx_buf[5]) >> 6;
    }
    return ret;
}

void main(void)
{
    int err;
    int32_t ecg_sample;
    uint8_t etag;
    uint8_t ptag;
    int32_t bioz_sample;
    uint8_t btag;
    uint16_t rtor_sample;
    uint32_t red_sample, ir_sample;

    err = bt_enable(NULL);
    if (err) { LOG_ERR("Bluetooth enable failed"); return; }

    err = bt_nus_init(&nus_cb);
    if (err) { LOG_ERR("NUS service init failed"); return; }

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0);
    if (err) { LOG_ERR("Bluetooth advertising failed"); return; }

    err = max30001_init();
    if (err) { return; }

    err = max86140_init();
    if (err) { return; }

    while (1) {
        k_sleep(K_SECONDS(1));

        if (current_conn) {
            if (
                max30001_read_ecg_fifo(&ecg_sample, &etag, &ptag) == 0 &&
                max30001_read_bioz_fifo(&bioz_sample, &btag) == 0 &&
                max30001_read_rtor(&rtor_sample) == 0 &&
                max86140_read_fifo(&red_sample, &ir_sample) == 0
            ) {
                uint8_t payload[14];
                payload[0] = (ecg_sample >> 16) & 0xFF;
                payload[1] = (ecg_sample >> 8) & 0xFF;
                payload[2] = ecg_sample & 0xFF;
                payload[3] = (bioz_sample >> 16) & 0xFF;
                payload[4] = (bioz_sample >> 8) & 0xFF;
                payload[5] = bioz_sample & 0xFF;
                payload[6] = (rtor_sample >> 8) & 0xFF;
                payload[7] = rtor_sample & 0xFF;
                payload[8] = (red_sample >> 16) & 0xFF;
                payload[9] = (red_sample >> 8) & 0xFF;
                payload[10] = red_sample & 0xFF;
                payload[11] = (ir_sample >> 16) & 0xFF;
                payload[12] = (ir_sample >> 8) & 0xFF;
                payload[13] = ir_sample & 0xFF;

                err = bt_nus_send(NULL, payload, sizeof(payload));
                if (err) {
                    LOG_WRN("Failed to send sensor data (err %d)", err);
                } else {
                    LOG_INF("Sensor data sent (err %d)", err);
                }
            }
            else {
                LOG_WRN("Failed to read sensor data");
            }
        }
    }
}

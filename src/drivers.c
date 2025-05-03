#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(drivers, LOG_LEVEL_INF);

#include "drivers.h"

#define MAX86140_SAMPLE_RATE 4 // ms
#define MAX30001_ECG_SAMPLE_RATE 5 // ms
#define MAX30001_BIOZ_SAMPLE_RATE 20 // ms

#define MAX86140_NODE DT_NODELABEL(max86140)
#define MAX30001_NODE DT_NODELABEL(max30001)

const struct spi_dt_spec max86140_spi = SPI_DT_SPEC_GET(MAX86140_NODE, SPI_WORD_SET(8), 0);
const struct spi_dt_spec max30001_spi = SPI_DT_SPEC_GET(MAX30001_NODE, SPI_WORD_SET(8), 0);

static struct k_work_delayable max86140_sample_work;
static struct k_work_delayable max30001_ecg_sample_work;
static struct k_work_delayable max30001_bioz_sample_work;

float ecg = 0.;
float bioz = 0.;
float rtor = 0.;
float hr = 0.;
float spo2 = 0.;

int max86140_spi_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2] = {reg, value};

    struct spi_buf tx_bufs = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx = {.buffers = &tx_bufs, .count = 1};

    return spi_write_dt(&max86140_spi, &tx);
}

int max86140_spi_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx_buf[1] = {(reg << 1) | 1};
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

void max86140_sample_handler(struct k_work *work)
{
    // LOG_INF("Sampling max86140");

    // sample sensor

    k_work_schedule(&max86140_sample_work, K_MSEC(MAX86140_SAMPLE_RATE));
}

int max86140_init(void)
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

    // Clear interrupts by reading INT_STATUS1 and INT_STATUS2
    uint8_t dummy;
    max86140_spi_read_reg(0x00, &dummy);
    max86140_spi_read_reg(0x01, &dummy);

    // LED1 and LED2 Sequencing
    ret = max86140_spi_write_reg(0x20, 0x21);
    if (ret) return ret;

    // LED3 Sequencing
    ret = max86140_spi_write_reg(0x21, 0x03);
    if (ret) return ret;

    // LED current range
    ret = max86140_spi_write_reg(0x2A, 0x3F);
    if (ret) return ret;

    // LED 1 current
    ret = max86140_spi_write_reg(0x23, 0xFF);
    if (ret) return ret;

    // LED 2 current
    ret = max86140_spi_write_reg(0x24, 0xFF);
    if (ret) return ret;

    // LED 3 current
    ret = max86140_spi_write_reg(0x25, 0xFF);
    if (ret) return ret;

    // Exit shutdown
    ret = max86140_spi_write_reg(0x0D, 0x00);
    if (ret) return ret;
    
    k_work_init_delayable(&max86140_sample_work, max86140_sample_handler);
    k_work_schedule(&max86140_sample_work, K_MSEC(MAX86140_SAMPLE_RATE));

    return 0;
}

int max86140_read_fifo(uint32_t *red_sample, uint32_t *ir_sample)
{
    uint8_t tx_buf[1] = { (0x07 << 1) | 1 }; // FIFO read
    uint8_t rx_buf[6] = {0};

    struct spi_buf tx_bufs = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf rx_bufs = {.buf = rx_buf, .len = sizeof(rx_buf)};

    struct spi_buf_set tx_set = {.buffers = &tx_bufs, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx_bufs, .count = 1};

    int ret = spi_transceive_dt(&max86140_spi, &tx_set, &rx_set);
    if (ret == 0) {
        *red_sample = ((uint32_t)rx_buf[0] << 16) | ((uint32_t)rx_buf[1] << 8) | rx_buf[2];
        *ir_sample  = ((uint32_t)rx_buf[3] << 16) | ((uint32_t)rx_buf[4] << 8) | rx_buf[5];
        *red_sample >>= 6;
        *ir_sample  >>= 6;
    }
    return ret;
}

int max30001_spi_write_reg(uint8_t reg, uint32_t value)
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

int max30001_spi_read_reg(uint8_t reg, uint32_t *data)
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

void max30001_ecg_sample_handler(struct k_work *work)
{
    int32_t sample;
    uint8_t etag;
    uint8_t ptag;
    max30001_read_ecg_fifo(&sample, &etag, &ptag);

    ecg = sample * 1000.0 / (131072 * 20);

    k_work_schedule(&max30001_ecg_sample_work, K_MSEC(MAX30001_ECG_SAMPLE_RATE));
}

void max30001_bioz_sample_handler(struct k_work *work)
{
    int32_t bioz_sample;
    uint8_t btag;
    max30001_read_bioz_fifo(&bioz_sample, &btag);

    bioz = bioz_sample * 1.0 / (524288 * 0.000008 * 10);

    uint16_t rtor_sample;
    max30001_read_rtor(&rtor_sample);

    rtor = 60.0 / (rtor_sample * 0.008);

    k_work_schedule(&max30001_bioz_sample_work, K_MSEC(MAX30001_BIOZ_SAMPLE_RATE));
}

int max30001_init(void)
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
    ret = max30001_spi_write_reg(0x10, 0x2C0000);
    if (ret) return ret;

    // ECG Rate
    ret = max30001_spi_write_reg(0x15, 0x805000);
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

    k_work_init_delayable(&max30001_ecg_sample_work, max30001_ecg_sample_handler);
    k_work_init_delayable(&max30001_bioz_sample_work, max30001_bioz_sample_handler);
    k_work_schedule(&max30001_ecg_sample_work, K_MSEC(MAX30001_ECG_SAMPLE_RATE));
    k_work_schedule(&max30001_bioz_sample_work, K_MSEC(MAX30001_BIOZ_SAMPLE_RATE));

    return 0;
}

int max30001_read_ecg_fifo(int32_t *sample, uint8_t *etag, uint8_t *ptag)
{
    uint32_t data;
    int ret = max30001_spi_read_reg(0x21, &data);
    if (ret == 0) {
        int32_t raw_sample = (int32_t)(data >> 6);
        raw_sample = (raw_sample << 14) >> 14; // sign-extend from 18-bit
        *sample = raw_sample;
        *etag = (uint8_t)((data >> 3) & 0x07);
        *ptag = (uint8_t)(data & 0x07);
    }
    return ret;
}

int max30001_read_bioz_fifo(int32_t *sample, uint8_t *btag)
{
    uint32_t data;
    int ret = max30001_spi_read_reg(0x23, &data);
    if (ret == 0) {
        int32_t raw_sample = (int32_t)(data >> 4);
        raw_sample = (raw_sample << 12) >> 12; // sign-extend from 20-bit
        *sample = raw_sample;
        *btag = (uint8_t)(data & 0x07);
    }
    return ret;
}

int max30001_read_rtor(uint16_t *sample)
{
    uint32_t data;
    int ret = max30001_spi_read_reg(0x25, &data);
    if (ret == 0) {
        *sample = (uint16_t)(data >> 10);
    }
    return ret;
}
#ifndef DRIVERS_H
#define DRIVERS_H

extern float ecg;
extern float bioz;
extern float rtor;
extern float hr;
extern float spo2;

int max86140_spi_write_reg(uint8_t reg, uint8_t value);
int max86140_spi_read_reg(uint8_t reg, uint8_t *value);
int max86140_init(void);
int max86140_read_fifo(uint32_t *red_sample, uint32_t *ir_sample);

int max30001_spi_write_reg(uint8_t reg, uint32_t value);
int max30001_spi_read_reg(uint8_t reg, uint32_t *data);
int max30001_init(void);
int max30001_read_ecg_fifo(int32_t *sample, uint8_t *etag, uint8_t *ptag);
int max30001_read_bioz_fifo(int32_t *sample, uint8_t *btag);
int max30001_read_rtor(uint16_t *sample);

#endif

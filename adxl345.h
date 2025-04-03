#ifndef ADXL345_H
#define ADXL345_H

#include <stdint.h>
#include <stdbool.h>

#define ADXL345_ADDRESS 0x53

// Registros do ADXL345
typedef enum {
    ADXL345_REG_DEVID = 0x00,
    ADXL345_REG_POWER_CTL = 0x2D,
    ADXL345_REG_DATAX0 = 0x32
} adxl345_register_t;

// Inicializa o ADXL345
bool adxl345_init(uint8_t sda, uint8_t scl);

// Lê os valores de aceleração nos eixos X, Y e Z
bool adxl345_read(int16_t *x, int16_t *y, int16_t *z);

// Calcula o valor RMS das leituras
float adxl345_read_rms();

#endif

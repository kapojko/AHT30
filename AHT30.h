#ifndef AHT30_H
#define AHT30_H

#include <stdint.h>
#include <stdbool.h>

#define AHT30_I2C_ADDR 0x38

#define AHT30_POWER_ON_DELAY_MS 5
#define AHT30_MEASUREMENT_DELAY_MS 80

enum AHT30_WorkMode {
    AHT30_NOR_MODE = 0b00,
    AHT30_CYC_MODE = 0b01,
    AHT30_CMD_MODE = 0b10,
    AHT30_CMD_MODE_2 = 0b11,
};

struct AHT30_Status {
    bool busy;
    enum AHT30_WorkMode modeStatus;
    bool crcFlag;
    bool calibrationEnable;
    bool cmpInterrupt;
};

struct AHT30_Platform {
    int (*i2cWrite)(uint8_t addr7bit, const uint8_t *data, uint8_t length, uint8_t wait, uint8_t send_stop);
    int (*i2cRead)(uint8_t addr7bit, uint8_t *data, uint8_t length, int timeout);

    void (*debugPrint)(const char *fmt, ...);
};

void AHT30_Init(struct AHT30_Platform *platform);

bool AHT30_StartMeasurement(int *measurementDelay_ms);
bool AHT30_GetStatus(struct AHT30_Status *status);
bool AHT30_ReadTempHum(float *temperature, float *humidity, struct AHT30_Status *status);

#endif // AHT30_H

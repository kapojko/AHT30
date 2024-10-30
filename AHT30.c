#include "AHT30.h"

#define AHT30_I2C_TIMEOUT 10


static struct AHT30_Platform platform;

static void ParseStatus(uint8_t statusData, struct AHT30_Status *status) {
    status->busy = ((statusData >> 7) & 0x01) != 0;
    status->modeStatus = (enum AHT30_WorkMode)((statusData >> 5) & 0x03);
    status->crcFlag = ((statusData >> 4) & 0x01) != 0;
    status->calibrationEnable = ((statusData >> 3) & 0x01) != 0;
    status->cmpInterrupt = ((statusData >> 2) & 0x01) != 0;
}

//CRC check type: CRC8
//polynomial: X8+X5+X4+1
//Poly:0011 0001 0x31
static unsigned char Calc_CRC8(unsigned char *message, unsigned char Num)
{
    unsigned char i;
    unsigned char byte;
    unsigned char crc = 0xFF;
    for (byte = 0;byte < Num;byte++)
    {
        crc ^= (message[byte]);
        for (i = 8;i > 0;--i)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

void AHT30_Init(struct AHT30_Platform *platformPtr) {
    platform = *platformPtr;
}

bool AHT30_StartMeasurement(int *measurementDelay_ms) {
    // Send write measurement command
    uint8_t data[3] = { 0xAC, 0x33, 0x00 };
    int ret = platform.i2cWrite(AHT30_I2C_ADDR, data, sizeof(data), 1, 1);
    if (ret < 0) {
        platform.debugPrint("AHT30: Write start measurement command failed (%d)\r\n", -ret);
        return false;
    }

    *measurementDelay_ms = AHT30_MEASUREMENT_DELAY_MS;
    return true;
}

bool AHT30_GetStatus(struct AHT30_Status *status) {
    uint8_t data[1] = { 0 };
    int ret = platform.i2cRead(AHT30_I2C_ADDR, data, sizeof(data), AHT30_I2C_TIMEOUT);
    if (ret < 0) {
        platform.debugPrint("AHT30: Read status failed (%d)\r\n", -ret);
        return false;
    }

    ParseStatus(data[0], status);
    return true;
}

bool AHT30_ReadTempHum(float *temperature, float *humidity, struct AHT30_Status *status) {
    // Read data
    uint8_t data[7] = { 0 };
    int ret = platform.i2cRead(AHT30_I2C_ADDR, data, sizeof(data), AHT30_I2C_TIMEOUT);
    if (ret < 0) {
        platform.debugPrint("AHT30: Read temperature/humidity failed (%d)\r\n", -ret);
        return false;
    }

    // Parse status (if requested)
    if (status) {
        ParseStatus(data[0], status);
    }

    // Check CRC
    uint8_t crc = Calc_CRC8(data, 6);
    uint8_t crcRead = data[6];
    if (crc != crcRead) {
        platform.debugPrint("AHT30: CRC check failed (%02X != %02X)\r\n", crc, crcRead);
        return false;
    }

    // Read humidity
    uint32_t srhDat = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4);
    *humidity = (float)srhDat / 1048576.0f * 100.0f;
    
    // Read temperature
    uint32_t stDat = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];
    *temperature = (float)stDat / 1048576.0f * 200.0f - 50.0f;

    return true;
}

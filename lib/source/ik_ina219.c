#include "ik_ina219.h"

#include <stdint.h>
#include "I2C.h"
#include "arduino.h"

static struct {
    uint8_t address;
    uint16_t ina219_calibrationValue;
    int16_t ina219_currentDivider_mA;
    int16_t ina219_powerMultiplier_mW;
} context = {0};


void Write16Unsafe(uint8_t Register, uint16_t Value)
{
    uint8_t buff[3] = {0};
    buff[0] = Register; // register
    buff[1] = (Value >> 8) & 0xff;  // upper byte
    buff[2] = (Value >> 0) & 0xff; // lower byte
    I2CWrite(context.address, buff, sizeof(buff));
}

uint16_t Read16Unsafe()
{
    uint8_t buff[2];
    uint16_t value;

    I2CRead(context.address, buff, sizeof(buff));

    value = buff[0];
    value <<= 8;
    value |= buff[1];

    return value;
}

void INA219_Reset()
{
    if (!context.address) {
        printf("ina219 reset error \r\n");
        return;
    }

    Write16Unsafe(INA219_REG_CONFIG, INA219_CONFIG_RESET);
    delay(1);
}

void INA219_setCalibration(uint16_t CalibrationData)
{
    Write16Unsafe(INA219_REG_CALIBRATION, CalibrationData);
}

void INA219_setConfig(uint16_t Config)
{
    Write16Unsafe(INA219_REG_CONFIG, Config);
}

void INA219_setCalibration_32V_2A()
{
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                 INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                 INA219_CONFIG_SADCRES_12BIT_128S_69MS |
                 INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    context.ina219_calibrationValue = 4096;
    context.ina219_currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
    context.ina219_powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

    INA219_setCalibration(context.ina219_calibrationValue);
    INA219_setConfig(config);
}

uint8_t INA219_Init(uint8_t u8Addr, int iSpeed) {
    I2CInit(iSpeed);
    context.address = u8Addr;

    printf("start ina219\r\n");
    if (!I2CTest(context.address)) {
        // error
        printf("error ina219\r\n");
        return 0;
    }
    printf("success ina219\r\n");

    INA219_Reset();
    INA219_setCalibration_32V_2A();

    return 0;
}

int16_t INA219_ReadCurrent() {
    Write16Unsafe(INA219_REG_CURRENT, 0);
    int16_t result = Read16Unsafe(INA219_REG_CURRENT);

    // resistor 10 Om
    // resolution 10uV
    // max 320mV ~ 32mA
    // 1 bit - 1uA

    return result;
}

uint16_t INA219_ReadBusVoltage() {
    Write16Unsafe(INA219_REG_BUSVOLTAGE, 0);
    uint16_t result = Read16Unsafe();
    //return result * 1.25;
    return ((result >> 3  ) * 4);
}

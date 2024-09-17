#include "arduino.h"
#include "oled.h"
#include "string.h"
#include "ik_ina219.h"
// the setup function runs once when you press reset or power the board

config_t current_config;

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(D2, INPUT_PULLDOWN);
    pinMode(C3, INPUT_PULLUP);
    pinMode(C4, INPUT_PULLUP);
	read_config(&current_config);
	INA219_Init(INA219_ADDRESS, 400000);
	oledInit(0x3c, 400000);
	oledFill(0);
}

// the loop function runs over and over again forever

static uint8_t mode = 0;

void print_current(const char* prefix, int16_t current, uint8_t y) {
    char msg[30] = {0};
    if (current < 1000 && current > -1000) {
        sprintf(msg, "%s  %3d uA ", prefix, current);
    } else {
        uint8_t plus = current > 0;
        current = current > 0 ? current : -current;
        sprintf(msg, "%s %s%2d.%01d mA   ",prefix,  plus ? " " : "-", current/1000, (current%1000)/100 );
    }
    oledWriteString(0, y, msg, FONT_12x16, 0);
}

void print_voltage(uint16_t voltage)
{
    char msg[30] = {0};
    sprintf(msg, "V %2d.%03d v", voltage / 1000, voltage % 1000);
    oledWriteString(0, 0, msg, FONT_12x16, 0);
}

void loop() {
	static uint8_t led = 0;
	static uint8_t last_c4 = 0;
	uint16_t bus_voltage = INA219_ReadBusVoltage();
	int16_t current = -INA219_ReadCurrent();
	int16_t current_calibration = *(s16*)(&current_config.data[0]);

	digitalWrite(LED_BUILTIN, led = !led);

	if (mode == 0) {
		print_voltage(bus_voltage);
		print_current("I", current - current_calibration, 24);
	} else if (mode == 1) {
		print_current("C", current_calibration, 0);
		print_current("R", current, 24);
	}

	if (!digitalRead(C3)) {
		printf("button C3\r\n");
		if (mode == 1) {
			mode = 0;
			oledFill(0);
		}
	}

	uint8_t c4 = !digitalRead(C4);
	if (c4 && c4 != last_c4) {
		printf("button C4\r\n");
		if (mode == 1) {
			current_calibration = current;
		}
		if (mode == 0) {
			mode = 1;
			oledFill(0);
		}
		{
			*(s16*)(&current_config.data[0]) = current_calibration;
			save_config(&current_config);
		}
	}
	last_c4 = c4;

	// printf(">voltage:%d,current:%d\r\n", bus_voltage, current - current_calibration);
}

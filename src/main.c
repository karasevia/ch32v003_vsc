#include "arduino.h"
#include "oled.h"
#include "string.h"
#include "ik_ina219.h"
// the setup function runs once when you press reset or power the board

typedef struct {
	uint8_t data[62];
} config_t;

uint8_t calc_config_crc(config_t* config) {
	uint8_t res = 0x33;
	// simple crc
	for (u8 i = 0; i < sizeof(config_t); ++i) {
		res |= config->data[i];
		res += config->data[i];
	}
	return res;
}

u32 sector_address(u32 s) {
	return FLASH_BASE + s * 64;
}

void write_config_to_sector(config_t* config, u8 s)
{
	FLASH_ErasePage_Fast(sector_address(s));
	FLASH_BufReset();
	for (u8 i = 0; i < 60; i += 4) {
		FLASH_BufLoad(sector_address(s) + i, *(u32*)(&config->data[i]));
	}
	u8 tmp[4];
	tmp[0] = config->data[60];
	tmp[1] = config->data[61];
	tmp[2] = calc_config_crc(config);
	tmp[3] = 0x33;
	FLASH_BufLoad(sector_address(s) + 60, *(u32*)(tmp));
	FLASH_ProgramPage_Fast(sector_address(s));
}

void write_config(config_t* config) {
	FLASH_Unlock_Fast();
	write_config_to_sector(config, 255);
	write_config_to_sector(config, 254);
	FLASH_ErasePage_Fast(sector_address(255));
	FLASH_Lock_Fast();
}

u8 read_config_from_sector(config_t* config, u8 s) {
	u8 status = *(u8*)(sector_address(s) + 63);
	if (status != 0x33) {
		return 0;
	}
	for (u8 i = 0; i < sizeof(config_t); ++i) {
		config->data[i] = *(u8*)(sector_address(s) + i);
	}
	u8 crc = calc_config_crc(config);
	u8 readed = *(u8*)(sector_address(s) + 62);
	if (crc != readed) {
		return 0;
	}
	return 1;
}

void read_config(config_t* config) {
	if (read_config_from_sector(config, 255) || read_config_from_sector(config, 254)) {
		return;
	}
	for (u8 i = 0; i < sizeof(config_t); ++i) {
		config->data[i] = 0;
	}
}

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
			write_config(&current_config);
		}
	}
	last_c4 = c4;

	// printf(">voltage:%d,current:%d\r\n", bus_voltage, current - current_calibration);
}

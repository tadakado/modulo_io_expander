// I2C (PB0:SCL, PB1:SDA)
//   address: PB2 PC2 = NC  NC (0x20 & 0x30) / NC  GND (0x21 & 0x31)
//                      GND NC (0x22 & 0x32) / GND GND (0x23 & 0x33)
// IO Expander @ 0x20 (0x21, 0x22, 0x23)
//   register mapping (MSB to LSB) port0: PA7 PA6 PA5 PA4 PA3 PA2 PA1 NC
//                                 port1: PB7 PB6 PB5 PB4 PB3 PC4 PC3 NC
//   LPME-IO compatibility
//     BMP config:   1   2   7   8   9  10  11  12  13  14  15  16  17  18  19  20
//     LPME silk:    0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F (P0x for TCA9539)
//     AtTiny pin:  NC PA1 PA2 PA3 PA4 PA5 PA6 PA7  NC PC3 PC4 PB3 PB4 PB5 PB6 PB7
//   TCA9535 compatibility
//     register 0,1: Input Port Registers [pull-up] (PA1-7, PB3-7, PC3-4)
//     register 2,3: Output Port Registers (PA1-7, PB3-7, PC3-4)
//     register 4,5: Polarity Inversion Registers (not implemented)
//     register 6,7: Configuration Registers ['1' input (default), '0' output] (PA1-7, PB3-7)
// RGB LED controller @ 0x30 (0x31, 0x32, 0x33)
//   output : PC1
//
// PA0   : UDPI
// PA1-7 : GPIO
// PB0,1 : I2C
// PB2   : I2C address (low bit)
// PB3-7 : GPIO
// PC0   : NC
// PC1   : RGB LED (WS2812)
// PC2   : I2C address (high bit)
// PC3-4 : GPIO (input only)
// PC5   : NC
//
// Note: PC3-4 may or may not work as output ports, so that it is recommend to use them as input ports.
//       (Somehow, they conditionally work as output ports. But, the condition is not investigated.)

#include <atmel_start.h>
#include <atomic.h>
#include <avr/sleep.h>

#define I2C_IOE_SADDR 0x20
#define I2C_LED_SADDR 0x30

#define Max_LEDs 128 // max 159 (512 bytes SRAM), try smaller number if it does not work
#define Update_LEDs_REG 0xff

#define Sleep_Mode SLEEP_MODE_PWR_DOWN // SLEEP_MODE_IDLE / SLEEP_MODE_STANDBY / SLEEP_MODE_PWR_DOWN

//// test                  0, 1, 2, 3, 4, 5, 6, 7
//uint8_t PORTA_pins[8] = {0, 1, 2, 3, 4, 5, 6, 7}; // Out NG 0     In NG 0   (IO: PA1-7)
//uint8_t PORTB_pins[6] = {      2, 3, 4, 5, 6, 7}; // Out NG 2     In NG     (IO: PB3-7) (IN: PB2)
//uint8_t PORTC_pins[5] = {0,    2, 3, 4, 5      }; // Out NG 0,2,5 In NG 0,5 (IO: PC3-4) (IN: PC2)

uint8_t PORT_pins[2][8] = {{0xff, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},  // LSB to MSB (NC, PA1-7)
                           {0xff, 0x23, 0x24, 0x13, 0x14, 0x15, 0x16, 0x17}}; // LSB to MSB (NC, PC3,4,PB3-7)

uint8_t OUT_REG[2] = {0xff, 0xff};
uint8_t CONF_REG[2] = {0xff, 0xff};

uint8_t i2c_saddr = 0;

uint8_t reg;
uint16_t data_count;

uint8_t LED_data[Max_LEDs * 3] = {0};
uint8_t num_LEDs = 0;

void sleep()
{
	ENABLE_INTERRUPTS();
	sleep_mode();
}

void write_to_LEDs()
{
	// Clear the Write Collision flag, to allow writing
	SPI0.INTFLAGS = SPI0_INTFLAGS;

	// Reset TCA counter register to ensure the first rising edge of PWM is predictable
	TCA0.SINGLE.CNT = 0 /* Count: 0 */;

	// Start TCA
	TCA0.SINGLE.CTRLA = TCA_SPLIT_CLKSEL_DIV1_gc /* System Clock */
						| 1 << TCA_SPLIT_ENABLE_bp /* Module Enable: enabled */;

	for (uint8_t i=0; i < num_LEDs * 3; i++) {
		// Start SPI by writing a byte to SPI data register
		SPI0.DATA = LED_data[i];

		// Wait for transfer to complete
		while ((SPI0.INTFLAGS & SPI_RXCIF_bm) == 0) {
		}
	}

	// Stop TCA
	TCA0.SINGLE.CTRLA = TCA_SPLIT_CLKSEL_DIV1_gc /* System Clock */
						| 0 << TCA_SPLIT_ENABLE_bp /* Module Enable: disabled */;
}

void update_LEDs()
{
	if (num_LEDs > 0) {
		DISABLE_INTERRUPTS();
		write_to_LEDs();
		num_LEDs = 0;
		ENABLE_INTERRUPTS();
	}
}

void I2C_address_handler()
{
	i2c_saddr = I2C_0_read() & 0xf9; // 1111 1001 b (mask sub address)
	I2C_0_send_ack();
	if (i2c_saddr & 0x01) { // I2C read
		data_count = 0x0000;
	} else { // I2C write
		data_count = 0xffff;
	}
}

uint8_t read_input_port(uint8_t PORT_num, uint8_t portA, uint8_t portB, uint8_t portC)
{
	uint8_t data = 0x00;
	uint8_t pin;
	uint8_t port;

	if (PORT_num != 0 && PORT_num != 1) return data;

	for (uint8_t i=0; i<8; i++) {
		pin = PORT_pins[PORT_num][i] & 0x07;
		port = PORT_pins[PORT_num][i] >> 4;
		if (port == 0x0f) {
			if (CONF_REG[PORT_num] & 1 << i) { // input pin
				data |= 1 << i;
 			} else { // output pin
				if (OUT_REG[PORT_num] & 1 << i) {
					data |= 1 << i;
				}
			}
		} else if (port == 0) {
			if (portA & 1 << pin) {
				data |= 1 << i;
			}
		} else if (port == 1) {
			if (portB & 1 << pin) {
				data |= 1 << i;
			}
		} else if (port == 2) {
			if (portC & 1 << pin) {
				data |= 1 << i;
			}
		}
	}
	return data;
}

void I2C_read_handler()
{
	uint8_t data = 0x00;

	if (i2c_saddr >> 1 == I2C_LED_SADDR) {
		if (reg == Update_LEDs_REG) { // Maximum number of LEDs
			data = Max_LEDs;
		} else if (reg * 3 + data_count < Max_LEDs * 3) { // LED data
			data = LED_data[reg * 3 + data_count];
		}
	}

	if (i2c_saddr >> 1 == I2C_IOE_SADDR) {
		if (reg == 0x00 || reg == 0x01) { // Input register
			if (reg + data_count == 0x00) {
				data = read_input_port(0, PORTA.IN, PORTB.IN, PORTC.IN);
			} else if (reg + data_count == 0x01) {
				data = read_input_port(1, PORTA.IN, PORTB.IN, PORTC.IN);
			}
		} else if (reg == 0x02 || reg == 0x03) { // Output register
			if (reg + data_count == 0x02) {
				data = OUT_REG[0];
			} else if (reg + data_count == 0x03) {
				data = OUT_REG[1];
			}
		} else if (reg == 0x06 || reg == 0x07) { // Configuration register
			if (reg + data_count == 0x06) {
				data = CONF_REG[0];
			} else if (reg + data_count == 0x07) {
				data = CONF_REG[1];
			}
		}
	}

	I2C_0_write(data);
	data_count++;
}

void write_data_port(uint8_t PORT_num, uint8_t data)
{
	uint8_t pin;
	uint8_t port;

	if (PORT_num != 0 && PORT_num != 1) return;

	for (uint8_t i=0; i<8; i++) {
		pin = PORT_pins[PORT_num][i] & 0x07;
		port = PORT_pins[PORT_num][i] >> 4;
		if (port == 0) {
			if (data & 1 << i) {
				PORTA_set_pin_level(pin, true);
			} else {
				PORTA_set_pin_level(pin, false);
			}
		} else if (port == 1) {
			if (data & 1 << i) {
				PORTB_set_pin_level(pin, true);
			} else {
				PORTB_set_pin_level(pin, false);
			}
		} else if (port == 2) {
			if (data & 1 << i) {
				PORTC_set_pin_level(pin, true);
			} else {
				PORTC_set_pin_level(pin, false);
			}
		}
	}
}

void write_config_port(uint8_t PORT_num, uint8_t data)
{
	uint8_t pin;
	uint8_t port;

	if (PORT_num != 0 && PORT_num != 1) return;

	for (uint8_t i=0; i<8; i++) {
		pin = PORT_pins[PORT_num][i] & 0x07;
		port = PORT_pins[PORT_num][i] >> 4;
		if (port == 0) {
			if (data & 1 << pin) {
				PORTA_set_pin_dir(pin, PORT_DIR_IN);
				PORTA_set_pin_pull_mode(pin, PORT_PULL_UP);
			} else {
				PORTA_set_pin_dir(pin, PORT_DIR_OUT);
				PORTA_set_pin_pull_mode(pin, PORT_PULL_OFF);
			}
		} else if (port == 1) {
			if (data & 1 << pin) {
				PORTB_set_pin_dir(pin, PORT_DIR_IN);
				PORTB_set_pin_pull_mode(pin, PORT_PULL_UP);
			} else {
				PORTB_set_pin_dir(pin, PORT_DIR_OUT);
				PORTB_set_pin_pull_mode(pin, PORT_PULL_OFF);
			}
		} else if (port == 2) {
			if (data & 1 << pin) {
				PORTC_set_pin_dir(pin, PORT_DIR_IN);
				PORTC_set_pin_pull_mode(pin, PORT_PULL_UP);
			} else {
				PORTC_set_pin_dir(pin, PORT_DIR_OUT);
				PORTC_set_pin_pull_mode(pin, PORT_PULL_OFF);
			}
		}
	}
}

void I2C_write_handler()
{
	uint8_t data;

	data = I2C_0_read();
	I2C_0_send_ack();
	if (data_count == 0xffff) { // register (command)
		reg = data;
		data_count++;
		return;
	}

	if (i2c_saddr >> 1 == I2C_LED_SADDR) {
		if (reg == Update_LEDs_REG) { // Update LEDs
			num_LEDs = data;
		} else if (reg * 3 + data_count < Max_LEDs * 3) { // LED data
			LED_data[reg * 3 + data_count] = data;
		}
	}

	if (i2c_saddr >> 1 == I2C_IOE_SADDR) {
		if (reg == 0x02 || reg == 0x03) { // Output register
			if (reg + data_count == 0x02) {
				OUT_REG[0] = data;
				write_data_port(0, data);
			} else if (reg + data_count == 0x03) {
				OUT_REG[1] = data;
				write_data_port(1, data);
			}			
		} else if (reg == 0x06 || reg == 0x07) { // Configuration register
			if (reg + data_count == 0x06) {
				CONF_REG[0] = data;
				write_config_port(0, data);
			} else if (reg + data_count == 0x07) {
				CONF_REG[1] = data;
				write_config_port(1, data);
			}
		}
	}

	data_count++;
}

void I2C_stop_handler()
{
	if (i2c_saddr >> 1 == I2C_LED_SADDR) {
		if (reg == Update_LEDs_REG) {
			update_LEDs();
		}
	}

	if (i2c_saddr >> 1 == I2C_IOE_SADDR) {
		;
	}

	/* reset register */
	reg = 0x00;

	/* Sleep mode */
	ENABLE_INTERRUPTS();
	sleep_mode();
}

void I2C_error_handler()
{
}

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	/* I2C callbacks */
	I2C_0_set_read_callback(I2C_read_handler);
	I2C_0_set_write_callback(I2C_write_handler);
	I2C_0_set_address_callback(I2C_address_handler);
	I2C_0_set_stop_callback(I2C_stop_handler);
	I2C_0_set_collision_callback(I2C_error_handler);
	I2C_0_set_bus_error_callback(I2C_error_handler);

	/* Sleep mode */
	set_sleep_mode(Sleep_Mode);
	ENABLE_INTERRUPTS();
	sleep_mode();

	/* Initialize ports */
    write_data_port(0, OUT_REG[0]);
	write_data_port(1, OUT_REG[1]);
	write_config_port(0, CONF_REG[0]);
	write_config_port(1, CONF_REG[1]);

	while (1) {
	}
}

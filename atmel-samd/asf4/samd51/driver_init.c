/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_adc_base.h>
#include <hpl_rtc_base.h>

struct timer_descriptor      TIMER_0;
struct spi_m_sync_descriptor SPI_0;

struct adc_sync_descriptor ADC_0;

struct dac_sync_descriptor DAC_0;

struct i2c_m_sync_desc I2C_0;

struct usart_sync_descriptor USART_0;

struct pwm_descriptor PWM_0;

struct rand_sync_desc RAND_0;

void ADC_0_PORT_init(void)
{
}

void ADC_0_CLOCK_init(void)
{
	hri_mclk_set_APBDMASK_ADC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC0_GCLK_ID, CONF_GCLK_ADC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void ADC_0_init(void)
{
	ADC_0_CLOCK_init();
	ADC_0_PORT_init();
	adc_sync_init(&ADC_0, ADC0, (void *)NULL);
}

void DAC_0_PORT_init(void)
{
}

void DAC_0_CLOCK_init(void)
{

	hri_mclk_set_APBDMASK_DAC_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, DAC_GCLK_ID, CONF_GCLK_DAC_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void DAC_0_init(void)
{
	DAC_0_CLOCK_init();
	dac_sync_init(&DAC_0, DAC);
	DAC_0_PORT_init();
}

/**
 * \brief Timer initialization function
 *
 * Enables Timer peripheral, clocks and initializes Timer driver
 */
static void TIMER_0_init(void)
{
	hri_mclk_set_APBAMASK_RTC_bit(MCLK);
	timer_init(&TIMER_0, RTC, _rtc_get_timer());
}

void I2C_0_PORT_init(void)
{

	gpio_set_pin_pull_mode(PA08,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA08, PINMUX_PA08C_SERCOM0_PAD0);

	gpio_set_pin_pull_mode(PA09,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA09, PINMUX_PA09C_SERCOM0_PAD1);
}

void I2C_0_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_SLOW, CONF_GCLK_SERCOM0_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBAMASK_SERCOM0_bit(MCLK);
}

void I2C_0_init(void)
{
	I2C_0_CLOCK_init();
	i2c_m_sync_init(&I2C_0, SERCOM0);
	I2C_0_PORT_init();
}

void SPI_0_PORT_init(void)
{

	// Set pin direction to output
	gpio_set_pin_direction(PA00, GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PA00,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_function(PA00, PINMUX_PA00D_SERCOM1_PAD0);

	// Set pin direction to output
	gpio_set_pin_direction(PA01, GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PA01,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_function(PA01, PINMUX_PA01D_SERCOM1_PAD1);

	// Set pin direction to input
	gpio_set_pin_direction(PA18, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA18,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA18, PINMUX_PA18C_SERCOM1_PAD2);
}

void SPI_0_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBAMASK_SERCOM1_bit(MCLK);
}

void SPI_0_init(void)
{
	SPI_0_CLOCK_init();
	spi_m_sync_init(&SPI_0, SERCOM1);
	SPI_0_PORT_init();
}

void USART_0_PORT_init(void)
{

	gpio_set_pin_function(PA12, PINMUX_PA12C_SERCOM2_PAD0);

	gpio_set_pin_function(PA13, PINMUX_PA13C_SERCOM2_PAD1);
}

void USART_0_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_SLOW, CONF_GCLK_SERCOM2_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBBMASK_SERCOM2_bit(MCLK);
}

void USART_0_init(void)
{
	USART_0_CLOCK_init();
	usart_sync_init(&USART_0, SERCOM2, (void *)NULL);
	USART_0_PORT_init();
}

void delay_driver_init(void)
{
	delay_init(SysTick);
}

void PWM_0_PORT_init(void)
{
}

void PWM_0_CLOCK_init(void)
{
	hri_mclk_set_APBAMASK_TC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, TC0_GCLK_ID, CONF_GCLK_TC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void PWM_0_init(void)
{
	PWM_0_CLOCK_init();
	PWM_0_PORT_init();
	pwm_init(&PWM_0, TC0, _tc_get_pwm());
}

void RAND_0_CLOCK_init(void)
{
	hri_mclk_set_APBCMASK_TRNG_bit(MCLK);
}

void RAND_0_init(void)
{
	RAND_0_CLOCK_init();
	rand_sync_init(&RAND_0, TRNG);
}

void USB_0_PORT_init(void)
{

	gpio_set_pin_direction(PA24,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PA24,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(PA24,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA24, PINMUX_PA24H_USB_DM);

	gpio_set_pin_direction(PA25,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(PA25,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(PA25,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA25, PINMUX_PA25H_USB_DP);
}

/* The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
 * for low speed and full speed operation. */
#if (CONF_GCLK_USB_FREQUENCY > (48000000 + 48000000 / 400)) || (CONF_GCLK_USB_FREQUENCY < (48000000 - 48000000 / 400))
#warning USB clock should be 48MHz ~ 0.25% clock, check your configuration!
#endif

void USB_0_CLOCK_init(void)
{

	hri_gclk_write_PCHCTRL_reg(GCLK, USB_GCLK_ID, CONF_GCLK_USB_SRC | GCLK_PCHCTRL_CHEN);
	hri_mclk_set_AHBMASK_USB_bit(MCLK);
	hri_mclk_set_APBBMASK_USB_bit(MCLK);
}

void USB_0_init(void)
{
	USB_0_CLOCK_init();
	usb_d_init();
	USB_0_PORT_init();
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA02

	// Set pin direction to input
	gpio_set_pin_direction(PA02, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA02,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA02, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA03

	// Set pin direction to input
	gpio_set_pin_direction(PA03, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA03,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA03, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA04

	// Set pin direction to input
	gpio_set_pin_direction(PA04, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA04,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA04, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA05

	// Set pin direction to input
	gpio_set_pin_direction(PA05, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA05,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA05, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA06

	// Set pin direction to input
	gpio_set_pin_direction(PA06, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA06,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA06, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA07

	// Set pin direction to input
	gpio_set_pin_direction(PA07, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA07,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA07, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA10

	// Set pin direction to input
	gpio_set_pin_direction(PA10, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA10,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA10, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA11

	// Set pin direction to input
	gpio_set_pin_direction(PA11, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA11,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA11, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA14

	// Set pin direction to input
	gpio_set_pin_direction(PA14, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA14,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA14, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA15

	// Set pin direction to input
	gpio_set_pin_direction(PA15, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA15,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA15, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA16

	// Set pin direction to input
	gpio_set_pin_direction(PA16, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA16,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA16, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA17

	// Set pin direction to input
	gpio_set_pin_direction(PA17, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA17,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA17, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA19

	// Set pin direction to input
	gpio_set_pin_direction(PA19, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA19,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA19, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA20

	// Set pin direction to input
	gpio_set_pin_direction(PA20, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA20,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA20, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA21

	// Set pin direction to input
	gpio_set_pin_direction(PA21, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA21,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA21, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA22

	// Set pin direction to input
	gpio_set_pin_direction(PA22, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA22,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA22, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA23

	// Set pin direction to input
	gpio_set_pin_direction(PA23, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA23,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA23, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA27

	// Set pin direction to input
	gpio_set_pin_direction(PA27, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA27,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA27, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA30

	// Set pin direction to input
	gpio_set_pin_direction(PA30, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA30,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA30, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA31

	// Set pin direction to input
	gpio_set_pin_direction(PA31, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA31,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA31, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB02

	// Set pin direction to input
	gpio_set_pin_direction(PB02, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PB02,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PB02, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB03

	// Set pin direction to input
	gpio_set_pin_direction(PB03, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PB03,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PB03, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB08

	// Set pin direction to input
	gpio_set_pin_direction(PB08, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PB08,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PB08, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB09

	// Set pin direction to input
	gpio_set_pin_direction(PB09, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PB09,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PB09, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB10

	// Set pin direction to input
	gpio_set_pin_direction(PB10, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PB10,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PB10, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB11

	// Set pin direction to input
	gpio_set_pin_direction(PB11, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PB11,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PB11, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB22

	// Set pin direction to input
	gpio_set_pin_direction(PB22, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PB22,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PB22, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB23

	// Set pin direction to input
	gpio_set_pin_direction(PB23, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PB23,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PB23, GPIO_PIN_FUNCTION_OFF);

	ADC_0_init();

	DAC_0_init();

	TIMER_0_init();

	I2C_0_init();

	SPI_0_init();

	USART_0_init();

	delay_driver_init();

	PWM_0_init();

	RAND_0_init();

	USB_0_init();
}

/******************************************************************************
* Copyright (C) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/
/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdint.h>

#define APB_BASE        0xC0000000u
#define APB_STRIDE      0x00000080u

#define GPIO_BASE       (APB_BASE + (0u * APB_STRIDE))
#define TIMER_BASE      (APB_BASE + (1u * APB_STRIDE))
#define UART_BASE       (APB_BASE + (2u * APB_STRIDE))
#define SPI_BASE        (APB_BASE + (3u * APB_STRIDE))

// GPIO
#define GPIO_LEDS_OFF   0x04u

// TIMER
#define TIMER_CONF_OFF  0x00u
#define TIMER_LO_OFF    0x04u
#define TIMER_HI_OFF    0x08u

// UART (APB_uart.sv)
#define UART_CONF_OFF   0x00u
#define UART_SPEED_OFF  0x04u
#define UART_TX_OFF     0x08u

// SPI (spi_apb_wrapper.sv)
#define SPI_CTRL_OFF    0x00u 
#define SPI_TX_OFF      0x04u
#define SPI_RX_OFF      0x08u
#define SPI_DONE_OFF    0x0Cu 

// ADXL362
#define ADXL_CMD_WRITE  0x0Au
#define ADXL_CMD_READ   0x0Bu
#define ADXL_REG_ID  0x02u
#define ADXL_REG_X      0x08u
#define ADXL_REG_Y      0x09u
#define ADXL_REG_Z      0x0Au
#define ADXL_REG_POWER  0x2Du

static inline void mmio_write32(uint32_t addr, uint32_t value)
{
	*(volatile uint32_t *)addr = value;
}

static inline uint32_t mmio_read32(uint32_t addr)
{
	return *(volatile uint32_t *)addr;
}

static void timer_init(void)
{
	mmio_write32(TIMER_BASE + TIMER_CONF_OFF, 0x00000003u);
	mmio_write32(TIMER_BASE + TIMER_CONF_OFF, 0x00000001u);
}

static uint64_t timer_read64(void)
{
	uint32_t hi1 = mmio_read32(TIMER_BASE + TIMER_HI_OFF);
	uint32_t lo = mmio_read32(TIMER_BASE + TIMER_LO_OFF);
	uint32_t hi2 = mmio_read32(TIMER_BASE + TIMER_HI_OFF);

	if (hi1 != hi2) {
		lo = mmio_read32(TIMER_BASE + TIMER_LO_OFF);
		hi1 = hi2;
	}

	return ((uint64_t)hi1 << 32) | (uint64_t)lo;
}

static void delay_cycles(uint64_t cycles)
{
	uint64_t start = timer_read64();
	while ((timer_read64() - start) < cycles) {}
}

static void uart_init(uint16_t baud_limit)
{
	mmio_write32(UART_BASE + UART_SPEED_OFF, (uint32_t)baud_limit);
	mmio_write32(UART_BASE + UART_CONF_OFF, 1u);
}

static void uart_putc(char c)
{
	mmio_write32(UART_BASE + UART_TX_OFF, (uint32_t)(uint8_t)c);
}

static void uart_puts(const char *s)
{
	while (*s) {
		char c = *s++;
		if (c == '\n') {
			uart_putc('\r');
		}
		uart_putc(c);
	}
}

static void uart_put_hex8(uint8_t v)
{
	static const char hex[] = "0123456789ABCDEF";
	uart_putc(hex[(v >> 4) & 0x0F]);
	uart_putc(hex[v & 0x0F]);
}

static uint8_t spi_xfer8(uint8_t out, uint16_t spi_limit)
{
	mmio_write32(SPI_BASE + SPI_TX_OFF, (uint32_t)out);
	mmio_write32(SPI_BASE + SPI_CTRL_OFF, ((uint32_t)spi_limit << 16) | 1u);

	while ((mmio_read32(SPI_BASE + SPI_DONE_OFF) & 1u) == 0u) {
	}

	return (uint8_t)(mmio_read32(SPI_BASE + SPI_RX_OFF) & 0xFFu);
}

static void adxl_write_reg(uint8_t reg, uint8_t value, uint16_t spi_limit)
{
	(void)spi_xfer8(ADXL_CMD_WRITE, spi_limit);
	(void)spi_xfer8(reg, spi_limit);
	(void)spi_xfer8(value, spi_limit);
}

static uint8_t adxl_read_reg(uint8_t reg, uint16_t spi_limit)
{
	(void)spi_xfer8(ADXL_CMD_READ, spi_limit);
	(void)spi_xfer8(reg, spi_limit);
	return spi_xfer8(0x00u, spi_limit);
}

typedef struct {
	uint8_t x;
	uint8_t y;
	uint8_t z;
} sample_t;

#define SAMPLE_BUF_LEN  256u
static volatile sample_t samples[SAMPLE_BUF_LEN];

int main(void)
{
	const uint16_t uart_limit = 868u;
	const uint16_t spi_limit = 50u;

	timer_init();
	uart_init(uart_limit);

	mmio_write32(GPIO_BASE + GPIO_LEDS_OFF, 0x00000000u);

	uart_puts("ADXL362 init...\n");
	adxl_write_reg(ADXL_REG_POWER, 0x02u, spi_limit);

	uint8_t id = adxl_read_reg(ADXL_REG_ID, spi_limit);
	uart_puts("ADXL362 ID=0x");
	uart_put_hex8(id);
	uart_puts("\n");

	if (id != 0xF2u) {
		uart_puts("ERROR: Unexpected ID (expected 0xF2)\n");
		mmio_write32(GPIO_BASE + GPIO_LEDS_OFF, 0x0000FFFFu);
		while (1) {
		}
	}

	uart_puts("Reading XYZ...\n");

	uint32_t w = 0;
	while (1) {
		uint8_t x = adxl_read_reg(ADXL_REG_X, spi_limit);
		uint8_t y = adxl_read_reg(ADXL_REG_Y, spi_limit);
		uint8_t z = adxl_read_reg(ADXL_REG_Z, spi_limit);

		samples[w].x = x;
		samples[w].y = y;
		samples[w].z = z;
		w = (w + 1u) & (SAMPLE_BUF_LEN - 1u);

		uart_puts("X=0x");
		uart_put_hex8(x);
		uart_puts(" Y=0x");
		uart_put_hex8(y);
		uart_puts(" Z=0x");
		uart_put_hex8(z);
		uart_puts("\n");

		delay_cycles(5000000u);
	}
}

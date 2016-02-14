/**
bus gofer - USB CDC (or serial) to I2C, SPI converter using STM32F103 (or STM32F100)

@file		main.c
@author		Matej Kogovsek (matej@hamradio.si)
@copyright	GPL v2
*/

#include <stm32f10x.h>

#include "mat/serialq.h"
#include "mat/spi.h"
#include "mat/fls_25.h"
#include "mat/ee_95.h"
#include "mat/i2c.h"
#include "mat/ee_24.h"
#include "mat/adc.h"

#include "md5.h"
#include "tmr4.h"

#include <string.h>
#include <ctype.h>

//#define AT_CMD_UART 1
/*
You can easily adapt this code for STM32F100 (or other F1 MCUs without USB) by:
1. uncommenting the above line
2. removing the USB stuff from the makefile and changing -DSTM32F10X_MD to -DSTM32F10X_MD_VL (or appropriate)
*/

#ifndef AT_CMD_UART
	#include "usb_lib.h"
	#include "usb_desc.h"
	#include "usb_endp.h"

	void usb_hwinit(void);
#else
	#define cdc_init(x,y,z,w) ser_init(AT_CMD_UART, 115200, x,y,z,w);
	#define cdc_getc(x) ser_getc(AT_CMD_UART, x)
	#define cdc_putc(x) ser_putc(AT_CMD_UART, x)
	#define cdc_puts(x) ser_puts(AT_CMD_UART, x)
	#define cdc_puti_lc(x,y,z,w) ser_puti_lc(AT_CMD_UART,x,y,z,w)
	#define cdc_puti(x,y) ser_puti_lc(AT_CMD_UART,x,y,0,' ')
#endif

#define BUFSIZE 256

typedef uint8_t(*rd_func_t)(uint8_t, uint32_t, uint8_t*, uint16_t);

//-----------------------------------------------------------------------------
//  Global variables
//-----------------------------------------------------------------------------

static uint8_t atiftxbuf[128];
static uint8_t atifrxbuf[512];

volatile uint32_t msTicks;	// counts SysTicks

static uint8_t at_echo = 0;

static const uint8_t i2c_if = 2;
static const uint8_t spi_if = 1;

static uint8_t spi_dev = 1;

static uint8_t buf1[BUFSIZE];
static uint8_t buf2[BUFSIZE];

static uint8_t* wbuf = buf1;
static uint16_t wlen = 0;

static uint8_t* rbuf = buf2;
static uint16_t rlen = 0;

static uint8_t bufdisp = 1;

static uint32_t pwmf = 1000;
static float pwmdc = 0.5;

//-----------------------------------------------------------------------------
//  newlib required functions
//-----------------------------------------------------------------------------

void _exit(int status)
{
	//ser_printf("_exit called!\r\n");
	while(1) {}
}

//-----------------------------------------------------------------------------
//  SysTick handler
//-----------------------------------------------------------------------------

void SysTick_Handler(void)
{
	msTicks++;			// increment counter necessary in _delay_ms()
}

//-----------------------------------------------------------------------------
//  delay functions
//-----------------------------------------------------------------------------

void _delay_ms(uint32_t ms)
{
	uint32_t curTicks = msTicks;
	while ((msTicks - curTicks) < ms);
}

void _delay_us(uint32_t us)
{
    us *= 8;

    asm volatile("mov r0, %[us]             \n\t"
                 "1: subs r0, #1            \n\t"
                 "bhi 1b                    \n\t"
                 :
                 : [us] "r" (us)
                 : "r0"
	);
}

//-----------------------------------------------------------------------------
//  utility functions
//-----------------------------------------------------------------------------

// unsigned decimal string to u32
uint32_t udtoi(const char* s)
{
	uint32_t x = 0;

	while( isdigit((int)*s) ) {
		x *= 10;
		x += *s - '0';
		++s;
	}

	return x;
}

// unsigned hex string to u32
uint32_t uhtoi(const char* s, uint8_t n)
{
	uint32_t x = 0;

	uint8_t c = toupper((int)*s);

	while( n-- && ( isdigit((int)c) || ( (c >= 'A') && (c <= 'F') ) ) ) {
		if( isdigit((int)c) ) {
			c -= '0';
		} else {
			c -= 'A' - 10;
		}

		x *= 16;
		x += c;

		++s;
		c = toupper((int)*s);
	}

	return x;
}

// hex print buf
void hprintbuf(uint8_t* buf, uint16_t len)
{
	uint16_t i;
	for( i = 0; i < len; ++i ) {
		cdc_puti_lc(buf[i], 16, 2, '0');
	}
	cdc_puts("\r\n");
}

uint8_t at_md5(const char* s, rd_func_t rdf, uint8_t iface)
{
	if( strlen(s) < 1 ) return 1;

	uint32_t len = udtoi(s);

	MD5_CTX md5ctx;
	MD5_Init(&md5ctx);

	uint32_t adr;
	for( adr = 0; adr < len; adr += BUFSIZE ) {
		uint32_t rlen = len - adr;
		if( rlen > BUFSIZE ) rlen = BUFSIZE;

		rdf(iface, adr, rbuf, rlen);

		MD5_Update(&md5ctx, rbuf, rlen);
	}

	unsigned char hash[16];
	MD5_Final(hash, &md5ctx);

	hprintbuf(hash, 16);

	return 0;
}

// configure PB0 as output (serving as second SPI NSS pin)
void spi_cs2_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef iotd;
	iotd.GPIO_Pin = GPIO_Pin_0;
	iotd.GPIO_Speed = GPIO_Speed_2MHz;
	iotd.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &iotd);

	GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET);
}

// spi_cs2 is spi_cs extended to two pins
// when spi_dev == 1, use original spi_cs
// when spi_dev == 2, use PB0 as NSS
void spi_cs2(uint8_t devnum, uint8_t nss)
{
	if( spi_dev == 1 ) {
		spi_cs(devnum, nss);
	} else {
		if( nss ) {
			GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET);
		} else {
			GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET);
		}
	}
}

//-----------------------------------------------------------------------------
// stm32 lib required externs
//-----------------------------------------------------------------------------

void fls25_cs(uint8_t devnum, uint8_t nss)
{
	spi_cs2(devnum, nss);
}

void ee95_cs(uint8_t devnum, uint8_t nss)
{
	spi_cs2(devnum, nss);
}

//-----------------------------------------------------------------------------
//  AT command processing
//-----------------------------------------------------------------------------

uint8_t proc_at_cmd(const char* s)
{
	if( s[0] == 0 ) { return 1; }

	// general AT commands

	if( 0 == strcmp(s, "AT") ) {
		return 0;
	}

	if( 0 == strcmp(s, "ATE0") ) {
		at_echo = 0;
		return 0;
	}

	if( 0 == strcmp(s, "ATE1") ) {
		at_echo = 1;
		return 0;
	}

	if( 0 == strcmp(s, "ATI") ) {
		cdc_puts("bus gofer v2.0\r\n");
		return 0;
	}
#ifdef AT_CMD_UART
	char atipr[] = "AT+IPR=";

	if( 0 == strncmp(s, atipr, strlen(atipr)) ) {
		s += strlen(atipr);
		uint32_t nbr = udtoi(s);
		if( nbr % 9600 ) return 1;
		ser_puts(AT_CMD_UART, "OK\r\n");
		ser_wait_txe(AT_CMD_UART);
		ser_shutdown(AT_CMD_UART);
		ser_init(AT_CMD_UART, nbr, atiftxbuf, sizeof(atiftxbuf), atifrxbuf, sizeof(atifrxbuf));
		return 2;
	}
#endif

// --- buffer commands --------------------------------------------------------

	char atbufwr[] = "AT+BUFWR=";	// dddddd...

	if( 0 == strncmp(s, atbufwr, strlen(atbufwr)) ) {
		s += strlen(atbufwr);

		uint16_t len = strlen(s);
		if( len % 2 ) return 1;
		len /= 2;
		if( len > BUFSIZE ) return 2;

		wlen = len;
		uint16_t i;
		for( i = 0; i < wlen; ++i ) {
			wbuf[i] = uhtoi(s, 2);
			s += 2;
		}

		return 0;
	}

	char atbufrd[] = "AT+BUFRD";

	if( 0 == strcmp(s, atbufrd) ) {
		if( rlen == 0 ) return 1;

		hprintbuf(rbuf, rlen);

		return 0;
	}

	char atbufrdlen[] = "AT+BUFRDLEN";

	if( 0 == strcmp(s, atbufrdlen) ) {
		cdc_puti(rlen, 10);
		cdc_puts("\r\n");

		return 0;
	}

	char atbufswap[] = "AT+BUFSWAP";

	if( 0 == strcmp(s, atbufswap) ) {
		uint8_t* b = rbuf;
		uint16_t l = rlen;

		rbuf = wbuf;
		rlen = wlen;
		wbuf = b;
		wlen = l;

		return 0;
	}

	char atbufcmp[] = "AT+BUFCMP";

	if( 0 == strcmp(s, atbufcmp) ) {
		if( rlen != wlen ) return 1;

		if( memcmp(rbuf, wbuf, rlen) ) return 1;

		return 0;
	}

	char atbufrddisp[] = "AT+BUFRDDISP=";

	if( 0 == strncmp(s, atbufrddisp, strlen(atbufrddisp)) ) {
		s += strlen(atbufrddisp);
		if( strlen(s) != 1 ) return 1;

		if( s[0] == '1' ) { bufdisp = 1; return 0; }
		if( s[0] == '0' ) { bufdisp = 0; return 0; }
		if( s[0] == '?' ) { cdc_puti(bufdisp, 10); cdc_puts("\r\n"); return 0; }

		return 1;
	}

// --- generic I2C commands ---------------------------------------------------

	char ati2cwr[] = "AT+I2CWR=";	// aa

	if( 0 == strncmp(s, ati2cwr, strlen(ati2cwr)) ) {
		s += strlen(ati2cwr);

		if( wlen == 0 ) return 1;	// nothing to write

		if( strlen(s) != 2 ) return 1;

		uint8_t adr = uhtoi(s, 2);

		i2c_wr(i2c_if, adr, wbuf, wlen);

		return 0;
	}

	char ati2crd[] = "AT+I2CRD=";	// aa,len

	if( 0 == strncmp(s, ati2crd, strlen(ati2crd)) ) {
		s += strlen(ati2crd);

		if( strlen(s) < 4 ) return 1;
		if( s[2] != ',' ) return 1;

		uint8_t adr = uhtoi(s, 2);
		s += 3;
		uint16_t len = udtoi(s);

		if( (len < 1) || (len > BUFSIZE) ) return 1;

		rlen = len;

		i2c_rd(i2c_if, adr, rbuf, rlen);

		if( bufdisp ) hprintbuf(rbuf, rlen);

		return 0;
	}

// --- I2C EEPROM commands ----------------------------------------------------

	char atee24rd[] = "AT+EE24RD="; // aaaaaa,len

	if( 0 == strncmp(s, atee24rd, strlen(atee24rd)) ) {
		s += strlen(atee24rd);

		if( strlen(s) < 8 ) return 1;
		if( s[6] != ',' ) return 1;

		uint16_t adr = uhtoi(s, 6);
		s += 7;
		uint16_t len = udtoi(s);

		if( (len < 1) || (len > BUFSIZE) ) return 1;

		rlen = len;

		ee24_rd(i2c_if, adr, rbuf, rlen);

		if( bufdisp ) hprintbuf(rbuf, rlen);

		return 0;
	}

	char atee24wr[] = "AT+EE24WR="; // aaaaaa

	if( 0 == strncmp(s, atee24wr, strlen(atee24wr)) ) {
		s += strlen(atee24wr);

		if( wlen == 0 ) return 1; // nothing to write
		//if( wlen > 64 ) return 1; // EE page write supports up to 64 bytes
		if( strlen(s) != 6 ) return 1;

		uint16_t adr = uhtoi(s, 6);

		ee24_wr(i2c_if, adr, wbuf, wlen);

		return 0;
	}

	char atee24md5[] = "AT+EE24MD5="; // len

	if( 0 == strncmp(s, atee24md5, strlen(atee24md5)) ) {
		s += strlen(atee24md5);

		return at_md5(s, ee24_rd, i2c_if);
	}

// --- generic SPI commands ---------------------------------------------------

	char atspirw[] = "AT+SPIRW";

	if( 0 == strcmp(s, atspirw) ) {
		memcpy(rbuf, wbuf, wlen);
		rlen = wlen;

		spi_cs2(spi_if, 0);
		spi_putsn(spi_if, (char*)rbuf, rlen);
		spi_cs2(spi_if, 1);

		if( bufdisp ) hprintbuf(rbuf, rlen);

		return 0;
	}

	char atspidev[] = "AT+SPIDEV=";

	if( 0 == strncmp(s, atspidev, strlen(atspidev)) ) {
		s += strlen(atspidev);
		if( strlen(s) != 1 ) return 1;

		if( s[0] == '1' ) { spi_dev = 1; return 0; }
		if( s[0] == '2' ) { spi_dev = 2; return 0; }
		if( s[0] == '?' ) { cdc_puti(spi_dev, 10); cdc_puts("\r\n"); return 0; }

		return 1;
	}

// --- SPI FLASH commands ----------------------------------------------------

	char atfls25rd[] = "AT+FLS25RD="; // aaaaaa,len

	if( 0 == strncmp(s, atfls25rd, strlen(atfls25rd)) ) {
		s += strlen(atfls25rd);

		if( strlen(s) < 8 ) return 1;
		if( s[6] != ',' ) return 1;

		uint32_t adr = uhtoi(s, 6);
		s += 7;
		uint16_t len = udtoi(s);

		if( (len < 1 ) || (len > BUFSIZE) ) return 1;

		rlen = len;

		fls25_rd(spi_if, adr, rbuf, rlen);

		if( bufdisp ) hprintbuf(rbuf, rlen);

		return 0;
	}

	char atfls25wr[] = "AT+FLS25WR="; // aaaaaa

	if( 0 == strncmp(s, atfls25wr, strlen(atfls25wr)) ) {
		s += strlen(atfls25wr);

		if( wlen == 0 ) return 1; // nothing to write
		//if( wlen > 256 ) return 1; // FLASH page program supports up to 256 bytes
		if( strlen(s) != 6 ) return 1;

		uint32_t adr = uhtoi(s, 6);

		fls25_we(spi_if);
		return fls25_wr(spi_if, adr, wbuf, wlen);
	}

	char atfls25ce[] = "AT+FLS25CE";

	if( 0 == strcmp(s, atfls25ce) ) {
		fls25_we(spi_if);
		return fls25_chiperase(spi_if);
	}

	char atfls25id[] = "AT+FLS25ID";

	if( 0 == strcmp(s, atfls25id) ) {
		uint32_t jedid = fls25_id(spi_if);

		cdc_puti_lc(jedid, 16, 6, '0');
		cdc_puts("\r\n");

		return 0;
	}

	char atfls25md5[] = "AT+FLS25MD5="; // len

	if( 0 == strncmp(s, atfls25md5, strlen(atfls25md5)) ) {
		s += strlen(atfls25md5);

		return at_md5(s, fls25_rd, spi_if);
	}

// --- SPI EEPROM commands ----------------------------------------------------

	char atee95rd[] = "AT+EE95RD="; // aaaaaa,len

	if( 0 == strncmp(s, atee95rd, strlen(atee95rd)) ) {
		s += strlen(atee95rd);

		if( strlen(s) < 8 ) return 1;
		if( s[6] != ',' ) return 1;

		uint32_t adr = uhtoi(s, 6);
		s += 7;
		uint16_t len = udtoi(s);

		if( (len < 1 ) || (len > BUFSIZE) ) return 1;

		rlen = len;

		ee95_rd(spi_if, adr, rbuf, rlen);

		if( bufdisp ) hprintbuf(rbuf, rlen);

		return 0;
	}

	char atee95wr[] = "AT+EE95WR="; // aaaaaa

	if( 0 == strncmp(s, atee95wr, strlen(atee95wr)) ) {
		s += strlen(atee95wr);

		if( wlen == 0 ) return 1; // nothing to write
		//if( wlen > 16 ) return 1; // page program supports up to 16 bytes
		if( strlen(s) != 6 ) return 1;

		uint32_t adr = uhtoi(s, 6);

		ee95_wren(spi_if);
		ee95_wr(spi_if, adr, wbuf, wlen);

		return 0;
	}

	char atee95md5[] = "AT+EE95MD5="; // len

	if( 0 == strncmp(s, atee95md5, strlen(atee95md5)) ) {
		s += strlen(atee95md5);

		return at_md5(s, ee95_rd, spi_if);
	}

// --- PWM commands -----------------------------------------------------------

	char atpwmf[] = "AT+PWMF=";

	if( 0 == strncmp(s, atpwmf, strlen(atpwmf)) ) {
		s += strlen(atpwmf);
		uint32_t fr = udtoi(s);
		if( (fr == 0) || (fr > 1000000) ) return 1;
		pwmf = fr;
		return tmr4_pwm(pwmf, pwmdc);
	}

	char atpwmdc[] = "AT+PWMDC=";

	if( 0 == strncmp(s, atpwmdc, strlen(atpwmdc)) ) {
		s += strlen(atpwmdc);
		uint32_t dc = udtoi(s);
		if( (dc == 0) || (dc >= 100) ) return 1;
		pwmdc = 1.0 * dc / 100.0;
		return tmr4_pwm(pwmf, pwmdc);
	}

// --- ADC commands -----------------------------------------------------------

	char atadcstart[] = "AT+ADCSTART="; // ench,msec

	if( 0 == strncmp(s, atadcstart, strlen(atadcstart)) ) {
		s += strlen(atadcstart);

		if( strlen(s) < 6 ) return 1;
		if( s[4] != ',' ) return 1;

		uint16_t ench = uhtoi(s, 4);
		s += 5;
		uint32_t ams = udtoi(s);

		if( ench == 0 ) return 1;
		if( (ams < 1) || (ams > 10000) ) return 1;

		adc_init(ench, 4);	// init ADC on requested channels, 4 samples averaged
		adc_startfree();

		while( 1 ) {
			_delay_ms(ams);

			uint8_t first = 1;
			for( uint8_t i = 0; i < 16; ++i ) {
				if( ench & (1 << i) ) {
					if( first ) { first = 0; } else { cdc_puts(","); }
					cdc_puti(adc_get(i), 10);
				}
			}
			cdc_puts("\r\n");

			uint8_t c;
			if( cdc_getc(&c) && (c == 'X') ) {	// stop if X received
				adc_stopfree();
				break;
			}
		}

		return 0;
	}

// --- HELP -------------------------------------------------------------------

	if( 0 == strcmp(s, "AT?") ) {
		cdc_puts("--- R/W buffer ---\r\n");
		cdc_puts(atbufwr); cdc_puts("dd...\r\n");
		cdc_puts(atbufrd); cdc_puts("\r\n");
		cdc_puts(atbufrdlen); cdc_puts("\r\n");
		cdc_puts(atbufswap); cdc_puts("\r\n");
		cdc_puts(atbufcmp); cdc_puts("\r\n");
		cdc_puts(atbufrddisp); cdc_puts("d\r\n");
		cdc_puts("--- generic I2C ---\r\n");
		cdc_puts(ati2cwr); cdc_puts("aa\r\n");
		cdc_puts(ati2crd); cdc_puts("aa,len\r\n");
		cdc_puts("--- I2C EE 24 ---\r\n");
		cdc_puts(atee24rd); cdc_puts("aaaaaa,len\r\n");
		cdc_puts(atee24wr); cdc_puts("aaaaaa\r\n");
		cdc_puts(atee24md5); cdc_puts("len\r\n");
		cdc_puts("--- generic SPI ---\r\n");
		cdc_puts(atspirw); cdc_puts("\r\n");
		cdc_puts(atspidev); cdc_puts("num\r\n");
		cdc_puts("--- SPI FLS 25 ---\r\n");
		cdc_puts(atfls25rd); cdc_puts("aaaaaa,len\r\n");
		cdc_puts(atfls25wr); cdc_puts("aaaaaa\r\n");
		cdc_puts(atfls25ce); cdc_puts("\r\n");
		cdc_puts(atfls25id); cdc_puts("\r\n");
		cdc_puts(atfls25md5); cdc_puts("len\r\n");
		cdc_puts("--- SPI EE 95 ---\r\n");
		cdc_puts(atee95rd); cdc_puts("aaaaaa,len\r\n");
		cdc_puts(atee95wr); cdc_puts("aaaaaa\r\n");
		cdc_puts(atee95md5); cdc_puts("len\r\n");
		cdc_puts("--- PWM out ---\r\n");
		cdc_puts(atpwmf); cdc_puts("freq\r\n");
		cdc_puts(atpwmdc); cdc_puts("percent\r\n");
		cdc_puts("--- ADC ---\r\n");
		cdc_puts(atadcstart); cdc_puts("cccc,msec\r\n");
		return 0;
	}

	return 1;
}

//-----------------------------------------------------------------------------
//  MAIN function
//-----------------------------------------------------------------------------

int main(void)
{
#ifndef AT_CMD_UART
	usb_hwinit();
#endif

	if( SysTick_Config(SystemCoreClock / 1000) ) { // setup SysTick Timer for 1 msec interrupts
		while( 1 );                                  // capture error
	}
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); // disable preemption

	cdc_init(atiftxbuf, sizeof(atiftxbuf), atifrxbuf, sizeof(atifrxbuf));

#ifndef AT_CMD_UART
	USB_Init(); // suspends device if no USB connected
#endif

	spi_init(spi_if, SPI_BaudRatePrescaler_16);
	spi_cs2_init(); // configure PB0 as output, this pin will serve as secondary SPI NSS pin
	i2c_init(i2c_if, 100000);

#ifdef DBG_UART
	ser_printf_n = DBG_UART;
	ser_init(DBG_UART, 115200, uart2txbuf, sizeof(uart2txbuf), uart2rxbuf, sizeof(uart2rxbuf));
#endif

	char atbuf[512+16];
	uint16_t atbuflen = 0;

	while( 1 ) {
		// at command processing
		uint8_t d;
		if( cdc_getc(&d) ) {

			// echo character
			if( at_echo ) { cdc_putc(d); }

			// buffer overflow guard
			if( atbuflen >= sizeof(atbuf) ) { atbuflen = 0; }

			// execute on enter
			if( (d == '\r') || (d == '\n') ) {
				if( atbuflen ) {
					atbuf[atbuflen] = 0;
					atbuflen = 0;
					uint8_t r = proc_at_cmd(atbuf);
					if( r == 0 ) cdc_puts("OK\r\n");
					if( r == 1 ) cdc_puts("ERR\r\n");
				}
			} else
			if( d == 0x7f ) {	// backspace
				if( atbuflen ) { --atbuflen; }
			} else {			// store character
				atbuf[atbuflen++] = toupper(d);
			}
		}
	}
}

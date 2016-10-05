#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "main.h"
#include "i2c.h"
#include "hdc1008.h"

#define ENABLE_SLEEP
#define TX_INTERVAL       45 /* TX interval in seconds */

#define BAUDRATE 1200
#define BAUDRATE_CALIBRATION -55

#define STX 0xAC

#define ADDR          0x01

#define TYPE_TRH      0x01
#define FRAMEBUFSIZE  9

/*
 * Message frame format:
 * [ STX    | ADDRSEQ  | TYPE   | DATA   | CRC    ]
 * STX = 0xACAC
 * ADDRSEQ = bit 0-3 (device (source) addr), bit 4-7 sequency no (1)
 * TYPE = Message type
 * DATA = Message contents
 * CRC = Checksum (STX through DATA)
 *
 * (1) Sequence number is used so that the receiver can keep track of 
 * missed messages. This could be used to warn if a certain node rarely
 * gets its messages through. Increased by one with each new temp/rh reading.
 */

/*
 * HW Connections:
 * TX RF Data - PD4
 * I2C SCL - PB7
 * I2C SDA - PB5
 */

static uint8_t crc(uint8_t *buf, size_t buflen);
static int frame_build(uint8_t *buf, size_t buflen, int16_t temp, uint16_t rh);
void uart_tx(uint8_t *buf, size_t len);
void uart_tx_single(uint8_t c);
void print_str(char *str, size_t len);
void print_dec(int16_t dec);
static uint8_t get_address(void);

static uint8_t g_seqno = 0;
static uint8_t g_update_flag = 0;

ISR(WDT_OVERFLOW_vect)
{
  static uint8_t cycles = TX_INTERVAL;

  if (cycles-- == 0) {
    g_update_flag = 1;
    cycles = TX_INTERVAL;
  }

#if 0
  if (cycles == 0) {
    g_update_flag = 1;
    cycles = random_numbers[cyclesidx++ % random_numbers_size]; 
  }

  --cycles;
#endif
  /* Wake up the CPU */
  //PORTB ^= (1 << PB0);

}

ISR(TIMER0_COMPA_vect)
{
  //PORTB ^= (1 << PB0);
}

ISR(TIMER0_OVF_vect)
{
  //PORTB ^= (1 << PB0);
}

static uint8_t crc(uint8_t *buf, size_t buflen)
{
  uint8_t crc = 0;
  size_t i;

  for (i = 0; i < buflen; ++i)
    crc ^= buf[i];

  return crc;
}

static int frame_build(uint8_t *buf, size_t buflen, int16_t temp, uint16_t rh)
{
  int i = 0;

  if (buflen < 9)
    return -1;

  buf[i++] = STX;
  buf[i++] = STX;
  buf[i++] = (g_seqno << 4) | (get_address() & 0x0f);
  buf[i++] = TYPE_TRH;
  buf[i++] = (temp & 0xff00) >> 8;
  buf[i++] = (temp & 0x00ff);
  buf[i++] = (rh & 0xff00) >> 8;
  buf[i++] = (rh & 0x00ff);
  buf[i] = crc(buf, i);
  ++i;

  return i;
}

void uart_tx(uint8_t *buf, size_t len)
{
  size_t i;
  for (i = 0; i < len; ++i)
    uart_tx_single(buf[i]);
}

void uart_tx_single(uint8_t c)
{
  uint32_t usecdelay = 1000000/(uint32_t)BAUDRATE+BAUDRATE_CALIBRATION;
  int i;

  /* Start bit */
  PORTD &= ~(1 << PD4);
  _delay_us(usecdelay);

  for (i = 0; i < 8; ++i) {
    if (c & (1 << i))
      PORTD |= (1 << PD4);
    else
      PORTD &= ~(1 << PD4);
  
    _delay_us(usecdelay);
  }

  /* Stop bit */
  PORTD |= (1 << PD4);
  _delay_us(usecdelay);

}

static uint8_t get_address(void)
{
  /* Read the address pins */
  uint8_t addr;
  addr = (((PIND & (1 << PIND0)) ? 1 : 0) << 3) |
         (((PIND & (1 << PIND1)) ? 1 : 0) << 2) |
         (((PINA & (1 << PINA1)) ? 1 : 0) << 1) |
         (((PINA & (1 << PINA0)) ? 1 : 0));
  return addr;
}

static void gpio_init(void)
{
  /* Test output pins */
  DDRB |= (1 << PB0);
  DDRB |= (1 << PB1);

  /* Soft uart - RF data*/
  DDRD |= (1 << PD4);

  /* Address pins:
   * PD0
   * PD1
   * PA1
   * PA0 
   */

  /* Set pins as input */
  DDRA &= ~(1 << PA1);
  DDRA &= ~(1 << PA0);
  DDRD &= ~(1 << PD0);
  DDRD &= ~(1 << PD1);
  /* Enable pull-ups */
  PORTA |= ((1 << PA0) | (1 << PA1));
  PORTD |= ((1 << PD0) | (1 << PD1));

  /* Disable all pull-ups */
  //MCUCR |= (1 << PUD);
}

static void watchdog_init(void)
{
  /*
   * The watchdog is used to wake up the system periodically in order to 
   * get the temperature and humidity from the sensor, and transmit it via the
   * RF interface.
   * The WDT oscillator runs at 128 kHz, and the longest possible timeout 
   * is approximately 8 s. This is perfectly fine for updating the temperature.
   */

  /* Enable interrupt only, no reset */
  WDTCSR = (1 << WDIE);

  /* Enable changes to prescaler */
  WDTCSR |= (1 << WDCE);

  /* 
   * Set prescaler to timeout every 1 s. When waking up wait a random number 
   * of timeouts before transmitting, to avoid colliding with other nodes
   * tranmitting data.
   */
  WDTCSR |= (1 << WDP2) | (1 << WDP1); /* 1s */
}

static void power_saving(void)
{
  /* Power down analog comparator to reduce power consumption */
  ACSR |= (1 << ACD);
}

char hex2ascii(uint8_t hexval)
{
  if (hexval < 0x0A) {
    return ('0' + hexval);
  } else {
    return ('A' + (hexval - 0x0A));
  }
}

int main(void)
{
  uint8_t frame[FRAMEBUFSIZE];
  int16_t realtemp = 0;
  uint16_t rh = 0;
  size_t len;
  int i;

  gpio_init();
  watchdog_init();
  i2c_init();
  power_saving();

  /* Enable interrupts globally */
  sei();

  /* Configure HDC1008 */
  //hdc1008_set_address(0x40);
  //hdc1008_set_resolution_temp(RES_14_BIT);
  //hdc1008_set_resolution_rh(RES_14_BIT);
  //hdc1008_heater(0);
  hdc1008_set_mode(HDC_BOTH);

  for (;;) {
#ifdef ENABLE_SLEEP
    MCUSR |= (1 << SM0) | (1 << SM1);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
#endif

    //hdc1008_measure_temp(&realtemp);
    //print_str("T:", 2);
    //print_dec(realtemp);
    //print_str("\r\n", 3);
#ifdef ENABLE_SLEEP
    if (g_update_flag) {
#endif
      hdc1008_measure_both(&realtemp, &rh);
      g_seqno = (g_seqno + 1) % 16;
#if 0
      print_str("H:", 2);
      print_dec(rh);
      print_str(",T:", 3);
      print_dec(realtemp);
      print_str("\r\n", 3);
#endif
      /* Build and transmit frame with data. Transmit 5 times to increase
       * the chance of the frame arriving at the receiver without any error.
       */
      len = frame_build(frame, FRAMEBUFSIZE, realtemp, rh);
      for (i = 0; i < 5; ++i) {
        uart_tx(frame, len);
        _delay_ms(20);
      }
#ifdef ENABLE_SLEEP
      g_update_flag = 0;
    }
#endif
#ifndef ENABLE_SLEEP
    _delay_ms(10000);
#endif
  }
}

void print_str(char *str, size_t len)
{
  uart_tx((uint8_t*)str, len);
}

void print_dec(int16_t dec)
{
  int h, d, s;
  size_t len = 0;
  char buf[4];

  h = dec / 100;
  buf[len++] = '0' + h;
  d = (dec - h * 100) / 10;
  buf[len++] = '0' + d;
  s = (dec - h * 100 - d * 10);
  buf[len++] = '0' + s;
  uart_tx((uint8_t*)buf, len);
}

void print_hex(uint16_t hex)
{
  size_t len = 0;
  char buf[4];
  buf[len++] = hex2ascii((hex & 0xf000) >> 12);
  buf[len++] = hex2ascii((hex & 0x0f00) >> 8);
  buf[len++] = hex2ascii((hex & 0x00f0) >> 4);
  buf[len++] = hex2ascii((hex & 0x000f));
  uart_tx((uint8_t*)buf, len);
}


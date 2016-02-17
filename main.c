#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define BAUDRATE 1200

#define STX 0xAC
#define ETX 0x53

#define TYPE_TMPHUM 0x01

#define ADDR        0x01
/*
 * Message frame format:
 * [ STX    | ADDR  | TYPE   | DATA   | CRC    | ETX    ]
 * STX = 0xAC
 * ADDR = bit 0-3 (device (source) addr), bit 4-7 (rfu)
 * TYPE = Message type
 * DATA = Message contents
 * CRC = Checksum (STX through DATA)
 * ETX = 0x53
 */

static uint8_t crc(uint8_t *buf, size_t buflen);
static int frame_build(uint8_t *buf, size_t buflen);
static void uart_tx(uint8_t *buf, size_t len);
static void uart_tx_single(uint8_t c);

static uint8_t crc(uint8_t *buf, size_t buflen)
{
  uint8_t crc = 0;
  size_t i;

  for (i = 0; i < buflen; ++i)
    crc ^= buf[i];

  return crc;
}

static int frame_build(uint8_t *buf, size_t buflen)
{
  int i = 0;
  buf[i++] = STX;
  buf[i++] = ADDR & 0x0f;
  buf[i++] = TYPE_TMPHUM;
  buf[i++] = 40;
  buf[i++] = 20;
  buf[i] = crc(buf, i);
  ++i;
  buf[i++] = ETX;

  return i;
}

static void uart_tx(uint8_t *buf, size_t len)
{
  size_t i;
  for (i = 0; i < len; ++i)
    uart_tx_single(buf[i]);
}

static void uart_tx_single(uint8_t c)
{
  uint32_t usecdelay = 1000000/(uint32_t)BAUDRATE;
  int i;
  /* Pin 34 = PC3 */

  /* Start bit */
  PORTC &= ~(1 << PC3);
  _delay_us(usecdelay);

  for (i = 0; i < 8; ++i) {
    if (c & (1 << i))
      PORTC |= (1 << PC3);
    else
      PORTC &= ~(1 << PC3);
  
    _delay_us(usecdelay);
  }

  /* Stop bit */
  PORTC |= (1 << PC3);
  _delay_us(usecdelay);

}

int main(void)
{
  uint8_t frame[8];
  int len;

  /* For blinking the diode */
  DDRB |= _BV(DDB7);
  DDRC |= _BV(DDC3);

  len = frame_build(frame, 8);

  for (;;) {
    _delay_ms(20);
    PORTB ^= _BV(PORTB7);
    uart_tx(frame, len);
  }
}

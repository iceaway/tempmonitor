#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdint.h>

#include "i2c.h"

#define BAUDRATE 1200

#define STX 0xAC
#define ETX 0x53

#define ADDR          0x01

#define TYPE_TMPHUM 0x01

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

/*
 * HW Connections:
 * TX RF Data - PD4
 * I2C SCL - PB7
 * I2C SDA - PB5
 */

static uint8_t crc(uint8_t *buf, size_t buflen);
static int frame_build(uint8_t *buf, size_t buflen);
static void uart_tx(uint8_t *buf, size_t len);
static void uart_tx_single(uint8_t c);

uint8_t const random_numbers[] PROGMEM = { 2,  28, 6,  1,  10, 30, 14, 21, 14,
                                           19, 16, 13, 29, 19, 26, 28, 15, 6,
                                           17, 13, 25, 22, 24, 18, 3,  15, 26,                                        
                                           25, 17, 5 };
uint8_t const random_numbers_size = sizeof(random_numbers)/sizeof(random_numbers[0]);

ISR(WDT_OVERFLOW_vect)
{
  static uint8_t cycles = 0;
  static uint8_t cyclesidx = 0;

  if (cycles == 0)
    cycles = random_numbers[cyclesidx++];
  
  --cycles;

  if (cycles == 0) {
    //update_flag();
    cycles = random_numbers[cyclesidx++ % random_numbers_size]; 
  }
  /* Wake up the CPU */
//  PORTB ^= (1 << PB0);

}

ISR(TIMER0_COMPA_vect)
{
  //PORTB ^= (1 << PB0);
}

ISR(TIMER0_OVF_vect)
{
  //PORTB ^= (1 << PB0);
}


static int16_t raw_to_temperature(uint16_t raw)
{
  /* temp = (raw / 65536) * 165 - 40; */
  /* Convert the temperature from the HDC1008 to a temperature scaled by 65536
   * in order to avoid any loss of precision. Could be scaled to a smaller
   * number but do the conversion on the host end with floating point support.
   * Or should we simply send the unconverted value as an unsigned integer to
   * the host for conversion there?
   */
  int16_t temp = (raw * 165 - 40 * 65536); 
  return temp;
}

static uint8_t read_temp_humidity(uint8_t *humidity, int16_t *temperature)
{
  /* Configure HDC1008 for 11 bit temp / 8 bit hum */
  *humidity = 50;
  *temperature = 5000;

  return 0;
  
}

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
  int16_t temperature = INT16_MAX;
  uint8_t humidity = UINT8_MAX;

  read_temp_humidity(&humidity, &temperature);

  buf[i++] = STX;
  buf[i++] = ADDR & 0x0f;
  buf[i++] = TYPE_TMPHUM;
  buf[i++] = (temperature & 0xff00) >> 8;
  buf[i++] = (temperature & 0x00ff);
  buf[i++] = humidity;
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

static void gpio_init(void)
{
  /* Test output pins */
  DDRB |= (1 << PB0);
  DDRB |= (1 << PB1);

  /* Soft uart - RF data*/
  DDRD |= (1 << PD4);

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

int main(void)
{
  uint8_t frame[8];
  uint8_t i2cbuf[8];
  int len;

  gpio_init();
  watchdog_init();
#ifdef AVR_I2C
  USI_TWI_Master_Initialise();
#else
  i2c_init();
#endif
  power_saving();

  /* Enable interrupts globally */
  sei();

  i2cbuf[0] = (0x40 << 1) | 0x01; 
  i2cbuf[1] = 0x00;

  for (;;) {
    /*
    PORTB |= (1 << PB0);
    _delay_ms(10);
    PORTB &= ~(1 << PB0);
    */
//#define OTHI2C
#ifdef AVR_I2C
    USI_TWI_Start_Transceiver_With_Data(i2cbuf, 2);
#elif defined(OTHI2C)
    USI_I2C_Master_Start_Transmission(i2cbuf, 1);
#else
    i2c_transfer(i2cbuf, 3);
#endif

#ifdef ENABLE_SLEEP
    MCUSR |= (1 << SM0) | (1 << SM1);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
#endif

    /* Build and transmit frame with data */
    len = frame_build(frame, 8);
    uart_tx(frame, len);
    _delay_ms(15);
  }
}


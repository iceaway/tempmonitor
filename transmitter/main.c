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

/* TX interval in x8 seconds, i.e. 16 => 8*16 = 128 s.
 * There is no need to update the temperature more often than once every 128s
 * and we want to reserve battery power so long delays are a plus.
 */
#define TX_INTERVAL       16 

#define ENABLE_TX_TIMER()   TCCR0B |= (1 << CS00)
#define DISABLE_TX_TIMER()  TCCR0B &= ~(1 << CS00)

#define STX             0xAC
#define PREAMBLE_1      0xAA
#define PREAMBLE_2      0xAB

#define NO_RTX        5

#define TYPE_TRH      0x01  /* Temperature and relative humitidy */
#define TYPE_T        0x02  /* Temperature only */
#define TYPE_TB       0x03  /* Temperature and battery status */
#define FRAMEBUFSIZE  10

#define RF_ON()       PORTD |= (1 << PD5)
#define RF_OFF()      PORTD &= ~(1 << PD5)

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
 *
 * Before each messaga a preamble of 0xAAAB is sent to help the receiver sync
 * the clock of the data stream. 0xAAAB is chosen so that we have a bit value
 * transition between each bit, which the receiver needs to sync the clock. 
 * The second preamble byte is different from the first so the receiver
 * knows when to sync the first byte.
 * 
 *                    |  Preamble                         | Data stream
 * i.e. Bit stream:   |  1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 1  | 1 0 ..... 1
 *                         ^ Sync (best case)
 *                                                          ^ Start of data
 * The receiver will first try to sync the clock, and then look for the bit 
 * pattern 10101011 to detect the transition from preamble to data.
 *                                     
 */

/*
 * HW Connections:
 * PD4: TX RF Data
 * PB7: I2C SCL
 * PB5: I2C SDA
 * PD3: Test mode
 * PD5: RF control (on/off)
 * PD0: Device address
 * PD1:      ''
 * PA0:      ''
 * PA1:      ''
 */

static uint8_t crc(uint8_t *buf, size_t buflen);
static int frame_build(uint8_t *buf, size_t buflen, int16_t temp, uint16_t rh);
static void rf_tx(uint8_t *buf, size_t len);
static uint8_t get_address(void);

static uint8_t g_seqno = 0;
static volatile uint8_t g_measure = 0;
static volatile uint8_t g_transmit = 0;
static volatile uint8_t g_nextbit = 0;
static volatile uint8_t g_txfinished = 0;

ISR(WDT_OVERFLOW_vect)
{
  enum state {
    WAIT,
    TRANSMIT
  };
  static enum state s = WAIT;
  static uint8_t rtx = 0;
  static uint8_t cycles = TX_INTERVAL;
  
  switch (s) {
  case WAIT:
    if (cycles-- == 0) {
      s = TRANSMIT;
      g_measure = 1;
      g_seqno = (g_seqno + 1) % 16;
      rtx = NO_RTX;
    }
    break;

  case TRANSMIT:
    if (rtx-- == 0) {
      cycles = TX_INTERVAL;
      s = WAIT;
    } else {
      g_transmit = 1;
    }
    break;

  default:
    cycles = TX_INTERVAL;
    s = WAIT;
    break;
  }
}

ISR(TIMER0_COMPA_vect)
{
  //PORTB |= (1 << PB0);

  if (g_nextbit)
    PORTD |= (1 << PD4);
  else
    PORTD &= ~(1 << PD4);

  g_txfinished = 1;

  //PORTB &= ~(1 << PB0);
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

  buf[i++] = PREAMBLE_1;
  buf[i++] = PREAMBLE_2;
  buf[i++] = STX;
  buf[i++] = (g_seqno << 4) | (get_address() & 0x0f);
  buf[i++] = TYPE_TRH;
  buf[i++] = (temp & 0xff00) >> 8;
  buf[i++] = (temp & 0x00ff);
  buf[i++] = (rh & 0xff00) >> 8;
  buf[i++] = (rh & 0x00ff);
  /* Skip CRC for the preamble */
  buf[i] = crc(buf+2, i-2);
  ++i;

  return i;
}

static void rf_tx(uint8_t *buf, size_t len)
{
  size_t i;
  enum state {
    IDLE,
    M1,
    M0,
  };
  enum state s = IDLE;
  uint8_t bitindex;
  uint8_t bitsleft;
  uint8_t txbyte;

  /* Turn on RF circuit */
  RF_ON();
  /* Let it stabilize, not sure if we need this or not */
  _delay_ms(1);

  /* Make sure output signal is high */
  PORTD |= 1 << PD4;

  /* Enable timer */
  TCNT0 = 0;
  ENABLE_TX_TIMER();

  for (i = 0; i < len; ++i) {
    bitsleft = 8;
    /* TX MSB first */
    bitindex = 7;
    txbyte = buf[i];

    /* Wait until tx finished before returning */
    while (bitsleft) {
      g_txfinished = 0;
      switch (s) {
      case IDLE:
        if (txbyte & (1 << bitindex)) {
          g_nextbit = 0;
          s = M1;
        } else {
          g_nextbit = 1;
          s = M0;
        }
        break;

      case M1:
        g_nextbit = 1;
        s = IDLE;
        --bitsleft;
        --bitindex;
        break;

      case M0:
        g_nextbit = 0;
        s = IDLE;
        --bitsleft;
        --bitindex;
        break;
      }
      while (!g_txfinished) { }
    }

  }
  /* Wait for the last bit to time out */
  g_txfinished = 0;
  while (!g_txfinished) { }

  DISABLE_TX_TIMER();
  /* Make sure output signal is high */
  PORTD |= 1 << PD4;
  
  _delay_us(100);
  /* Turn of RF unit */
  RF_OFF();
}

static uint8_t get_address(void)
{
  /* Read the address pins */
  uint8_t addr;
  addr = (((PIND & (1 << PIND0)) ? 0 : 1) << 3) |
         (((PIND & (1 << PIND1)) ? 0 : 1) << 2) |
         (((PINA & (1 << PINA1)) ? 0 : 1) << 1) |
         (((PINA & (1 << PINA0)) ? 0 : 1));
  return addr;
}

static void gpio_init(void)
{
  /* Test output pins */
  //DDRB |= (1 << PB0);
  //DDRB |= (1 << PB1);

  /* Soft uart - RF data. Set an output - high */
  DDRD |= (1 << PD4);
  PORTD |= (1 << PD4);


  /* Control RF unit on/off via transistor via PD5. High = ON */
  DDRD |= (1 << PD5);
  PORTD &= ~(1 << PD5); /* Start low */

  /* Address pins:
   * PD0
   * PD1
   * PA1
   * PA0 
   */

  /* Set pins as input */
  DDRA &= ~((1 << PA1) | (1 << PA0));
  DDRD &= ~((1 << PD0) | (1 << PD1));
  /* Enable pull-ups */
  PORTA |= ((1 << PA0) | (1 << PA1));
  PORTD |= ((1 << PD0) | (1 << PD1));
  
  /* Test mode pin - tx every 3s */
  DDRD &= ~(1 << PD3);
  PORTD |= 1 << PD3;
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
   * Set prescaler to timeout every 8 s. When waking up we check if it is
   * time to transit, otherwise we go back to sleep for another 8s.
   */
  //WDTCSR |= (1 << WDP2) | (1 << WDP1); /* 1s */
  WDTCSR |= (1 << WDP3) | (1 << WDP0); /* 8s */
}

static void power_saving(void)
{
  /* Power down analog comparator to reduce power consumption */
  ACSR |= (1 << ACD);

  /* Disable input buffers on analog pins */
  DIDR |= (1 << AIN1D) | (1 << AIN0D);
}

static uint8_t get_testmode(void)
{
  return (PIND & (1 << PIND3)) ? 0 : 1;
}

static void timer_init(void)
{
  /* Timer used for manchester encoding of data stream. Each bit frame
   * is 250 us long.
   */
  /* No prescaler. Do not do this here since it enables the timer */
  //TCCR0B = (1 << CS00);

  /* Enable the interrupt */
  TIMSK |= (1 << OCIE0A);

  /* Output compare value */
  OCR0A = 125;
}

int main(void)
{
  uint8_t frame[FRAMEBUFSIZE];
  int16_t realtemp = 0;
  uint16_t rh = 0;
  size_t len;
  uint8_t testmode = 0;

  gpio_init();
  timer_init();
  watchdog_init();
  i2c_init();
  power_saving();
  testmode = get_testmode();


  /* Enable interrupts globally */
  sei();

  /* Configure HDC1008 */
#ifdef CONFIGURE_HDC1008
  hdc1008_set_address(0x40);
  hdc1008_set_resolution_temp(RES_14_BIT);
  hdc1008_set_resolution_rh(RES_14_BIT);
  hdc1008_heater(0);
#endif
  hdc1008_set_mode(HDC_BOTH);

  for (;;) {
    if (testmode) {
      _delay_ms(1000);
      g_measure = 1;
      g_transmit = 1;
      g_seqno = (g_seqno + 1) % 16;
      frame[0] = PREAMBLE_1;
      frame[1] = PREAMBLE_2;
      frame[2] = 0x21;
      frame[3] = TYPE_TRH; 
      frame[4] = (230 & 0xff00) >> 8;
      frame[5] = (230 & 0x00ff);
      frame[6] = (400 & 0xff00) >> 8;
      frame[7] = (400 & 0x00ff);
      frame[8] = 0xfe;
    } else {
      /* 
       * When going to sleep all pins should be inputs to reduce power
       * consumption.
       * Output pins are:
       * PD4 - RF output
       * PD5 - RF circuit on/off
       */
      DDRD &= ~(1 << PD4);
      DDRD &= ~(1 << PD5);

      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      sleep_enable();
      sleep_cpu();
      sleep_disable();

      /* Set PD4 and PD5 as outputs again */
      DDRD |= (1 << PD4);
      DDRD |= (1 << PD5);
    }

    if (g_measure) {
      hdc1008_measure_both(&realtemp, &rh);
      g_measure = 0;
    }

    if (g_transmit) {
      /* Build and transmit frame with data. */
      len = frame_build(frame, FRAMEBUFSIZE, realtemp, rh);
      rf_tx(frame, len);
      g_transmit = 0;
    }
  }
}


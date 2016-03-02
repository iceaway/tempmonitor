#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
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

/*
 * HW Connections:
 * TX Data - Pin 34
 * RX Data - Pin 22
 */

#define PORT_I2C      PORTB
#define DDR_I2C       DDRB
#define PIN_I2C_SDA   PB5
#define PIN_I2C_SCL   PB7

#define SET_INPUT(ddr, pin)   ddr &= ~(1 << pin)
#define SET_OUTPUT(ddr, pin)  ddr |= (1 << pin)
#define SET_HIGH(port, pin)   port |= (1 << pin)
#define SET_LOW(port, pin)    port &= ~(1 << pin)

static uint8_t crc(uint8_t *buf, size_t buflen);
static int frame_build(uint8_t *buf, size_t buflen);
static void uart_tx(uint8_t *buf, size_t len);
static void uart_tx_single(uint8_t c);

ISR(WDT_OVERFLOW_vect)
{
  /* Wake up the CPU */
  //PORTB ^= (1 << PB0);

}

ISR(TIMER0_COMPA_vect)
{
  //PORTB ^= (1 << PB0);
}

ISR(TIMER0_OVF_vect)
{
  PORTB ^= (1 << PB0);
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

static void init_gpio(void)
{
  /* Test output pins */
  DDRB |= (1 << PB0);
  DDRB |= (1 << PB1);

  /* Soft uart */
  DDRD |= (1 << PD4);

}

static int i2c_transfer(uint8_t *buf, uint8_t bufsize)
{
  int readmode = 0;
  if (buf[0] & (1 << I2C_READ_BIT)) {
    readmode = 1;
  }

  SET_HIGH(PORT_I2C, PIN_I2C_SCL);
  /* Make sure that SCL is high */
  while (!(PIN_I2C & (1 << PIN_I2C_SCL)));
  _delay_us(3);

  /* Generate start condition */
  SET_LOW(PORT_I2C, PIN_I2C_SDA);
  _delay_us(3);
  SET_LOW(PORT_I2C, PIN_I2C_SCL);
  SET_HIGH(PORT_I2C, PIN_I2C_SDA);


  /* Transmit data */

  /* Stop condition */
  SET_LOW(PORT_I2C, PIN_I2C_SDA);
  SET_HIGH(PORT_I2C, PIN_I2C_SCL);
  /* Make sure that SCL is high */
  while (!(PIN_I2C & (1 << PIN_I2C_SCL)));
  _delay_us(3);

  /* Generate start condition */
  SET_HIGH(PORT_I2C, PIN_I2C_SDA);
  _delay_us(3);
  SET_HIGH(PORT_I2C, PIN_I2C_SCL);
  SET_HIGH(PORT_I2C, PIN_I2C_SDA);

  

}

static void i2c_init(void)
{
  /* Set both I2C pins as outputs */

  /* I2C SDA - PB5 */
  SET_OUTPUT(DDR_I2C, PIN_I2C_SDA);

  /* I2C SCL - PB7 */
  SET_OUTPUT(DDR_I2C, PIN_I2C_SCL);

  /* Enable pull-ups/set high */
  SET_HIGH(PORT_I2C, PIN_I2C_SDA);
  SET_HIGH(PORT_I2C, PIN_I2C_SCL);

  USIDR = 0xff;

  USICR = (1 << USIWM1) | /* Two wire mode */
          (1 << USICS1) | (1 << USICLK);  /* Software counter clock strobe. 
                                           * The SW counter clock strobe will 
                                           * also toggle the shift register
                                           * which is why USICS1 should be set
                                           * to external.
                                           */
  /* Clear all flags */
  USISR = (1 << USISIF) | /* Start condition */
          (1 << USIOIF) | /* Counter overflow */
          (1 << USIPF)  | /* Stop condition */
          (1 << USIDC)  | /* Data collision */
          (0x0 << USICNT0); /* Zero the counter value */

}

static void init_watchdog(void)
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
  int len;

  gpio_init();
  watchdog_init();
  i2c_init();
  power_saving();

  /* Enable interrupts globally */
  sei();

  /* len = frame_build(frame, 8); */
  for (;;) {
    //_delay_ms(40);
    //PORTB ^= (1 << PB1);
#if 0
    MCUSR |= (1 << SM0) | (1 << SM1);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
#endif

    /* Start condition */
    //USIDR = 0;
    /* First byte */
    //USIDR = 0x81;

    //PORTB ^= (1 << PB0);
    /*
    uart_tx(frame, len);
    _delay_ms(20);
    PORTB &= ~(1 << PB0);
    */
  }
}


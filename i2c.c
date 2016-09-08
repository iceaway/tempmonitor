#include <avr/io.h>
#include <util/delay.h>

//#define AVR_I2C

#ifdef AVR_I2C
#include "USI_TWI_Master.h"
#endif

#define PORT_I2C      PORTB
#define DDR_I2C       DDRB
#define PIN_I2C       PINB
#define PIN_I2C_SDA   PB5
#define PIN_I2C_SCL   PB7
#define I2C_READ_BIT  0

#define SET_INPUT(ddr, pin)   ddr &= ~(1 << pin)
#define SET_OUTPUT(ddr, pin)  ddr |= (1 << pin)
#define SET_HIGH(port, pin)   port |= (1 << pin)
#define SET_LOW(port, pin)    port &= ~(1 << pin)

/* I2C functions */
static void i2c_sda_release(void);
static void i2c_sda_output(void);
static void i2c_sda_input(void);
static void i2c_scl_release(void);
static void i2c_scl_output(void);
static void i2c_scl_input(void);

static void i2c_start(void)
{
  /* Generate start condition */
  SET_HIGH(PORT_I2C, PIN_I2C_SCL);
  SET_HIGH(PORT_I2C, PIN_I2C_SDA);
  SET_OUTPUT(DDR_I2C, PIN_I2C_SCL);
  SET_OUTPUT(DDR_I2C, PIN_I2C_SDA);

  /* Make sure that SCL is high */
  while (!(PIN_I2C & (1 << PIN_I2C_SCL)));
  _delay_us(3);

  SET_LOW(PORT_I2C, PIN_I2C_SDA);
  _delay_us(6);
  SET_LOW(PORT_I2C, PIN_I2C_SCL);
  _delay_us(2);
  SET_HIGH(PORT_I2C, PIN_I2C_SDA);
}

static void i2c_stop(void)
{
  /* Stop condition */
  SET_LOW(PORT_I2C, PIN_I2C_SDA);
  SET_HIGH(PORT_I2C, PIN_I2C_SCL);
  SET_OUTPUT(DDR_I2C, PIN_I2C_SCL);
  SET_OUTPUT(DDR_I2C, PIN_I2C_SDA);

  /* Make sure that SCL is high */
  while (!(PIN_I2C & (1 << PIN_I2C_SCL)));
  _delay_us(3);

  SET_HIGH(PORT_I2C, PIN_I2C_SDA);
  _delay_us(3);
  SET_HIGH(PORT_I2C, PIN_I2C_SCL);
  SET_HIGH(PORT_I2C, PIN_I2C_SDA);
  _delay_us(3);
  /* Release SDA and SCL */
  SET_INPUT(DDR_I2C, PIN_I2C_SCL);
  SET_LOW(PORT_I2C, PIN_I2C_SCL);
  SET_INPUT(DDR_I2C, PIN_I2C_SDA);
  SET_LOW(PORT_I2C, PIN_I2C_SDA);
}

/* Exchange a single byte over I2C */
static uint8_t i2c_exchange(uint8_t rw)
{
  uint8_t tmp = 1;
  USISR |= (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | 
           0x00;

  if (rw == 0) {
    SET_OUTPUT(DDR_I2C, PIN_I2C_SDA);
  } else {
    SET_INPUT(DDR_I2C, PIN_I2C_SDA);
  }

  do {
    _delay_us(2);
    USICR |= (1 << USITC);
    while (!(PIN_I2C & (1 << PIN_I2C_SCL)));
    _delay_us(2);
    USICR |= (1 << USITC);
  } while (!(USISR & (1 << USIOIF)));

  /* All data transferred, get or send ack */
  if (rw == 0) {
    SET_INPUT(DDR_I2C, PIN_I2C_SDA);
  } else {
    tmp = USIDR;
    SET_OUTPUT(DDR_I2C, PIN_I2C_SDA);
    USIDR = 0x00;
  }

  USISR |= (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | 
           0x0e;
  _delay_us(2);
  USICR |= (1 << USITC);
  while (!(PIN_I2C & (1 << PIN_I2C_SCL)));
  _delay_us(2);
  USICR |= (1 << USITC);

  _delay_us(4);
  USIDR = 0xff;

  if (rw == 0) {
    return tmp;
  } else {
    return USIDR;
  }
}

int i2c_read(uint8_t address, uint8_t *buffer, uint8_t bytes)
{
  /* R/W bit = 1 for read */
  uint8_t addr = (address << 1) | 0x01;

}

int i2c_write(uint8_t address, uint8_t *buffer, uint8_t bytes)
{
  uint8_t addr = (address << 1);
}

int i2c_transfer(uint8_t *buf, uint8_t len)
{
  int readmode = 0;
  uint8_t tmp;
  int ret = 0;
  uint8_t idx = 0;

  if (buf[0] & (1 << I2C_READ_BIT)) {
    readmode = 1;
  }

  i2c_start();
  /* Transmit data */

  /* Send address */
  SET_LOW(PORT_I2C, PIN_I2C_SCL);
  USIDR = buf[idx++];
  ret = i2c_exchange(0);

/*
  if (ret != 0) {
    ret = -1;
    goto out;
  }
  */

  while (idx < len) {
    if (readmode == 1) {
      buf[idx++] = i2c_exchange(readmode);
    } else {
      USIDR = buf[idx++];
      ret = i2c_exchange(readmode);
      if (ret != 0) {
        ret = -1;
        goto out;
      }
    }
  }

out:
  i2c_stop();
  return ret;
}

void i2c_init(void)
{
  /* Set both I2C pins as outputs */

  /* Release SCL and SDA (pulled up externally) */
  SET_INPUT(DDR_I2C, PIN_I2C_SDA);
  SET_LOW(PORT_I2C, PIN_I2C_SDA);

  SET_INPUT(DDR_I2C, PIN_I2C_SCL);
  SET_LOW(PORT_I2C, PIN_I2C_SCL);

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


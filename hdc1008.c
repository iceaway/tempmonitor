#include <util/delay.h>
#include "main.h"
#include "i2c.h"
#include "hdc1008.h"

#define DEFAULT_ADDR  0x40
#define BUFSIZE       8

#define TEMP_DELAY    8 /* 8ms, conversion time = 6.35 ms according to spec */
#define RH_DELAY      8 /* 8ms, conversion time = 6.50 ms according to spec */

#define REG_CONF      0x02
#define REG_SER_LOW   0xFD
#define REG_SER_MID   0xFC
#define REG_SER_HI    0xFB

#define BITPOS_MODE     12
#define BITPOS_T_RES    10
#define BITPOS_RH_RES   8
#define BITPOS_HEATER   13

static uint8_t g_addr = DEFAULT_ADDR;
static uint8_t buf[BUFSIZE];

static int conf_read(uint16_t *conf)
{
  /* Read the content of the config register and set the mode as requested */
  buf[0] = (g_addr << 1) | 0x00;
  buf[1] = REG_CONF;
  i2c_transfer(buf, 2);

  buf[0] = (g_addr << 1) | 0x01;
  buf[1] = 0;
  buf[2] = 0;
  i2c_transfer(buf, 3);

  *conf = (buf[1] << 8) | buf[2];
  return OK;
}

static int conf_write(uint16_t conf)
{
  buf[0] = (g_addr << 1) | 0x00;
  buf[1] = REG_CONF;
  buf[2] = (conf & 0xff00) >> 8;
  buf[3] = (conf & 0x00ff);
  i2c_transfer(buf, 4);
  return OK;
}

#if defined(HDC1008_MEAS_RH) || defined(HDC1008_MEAS_BOTH)
static uint16_t raw_to_rh(uint16_t raw)
{
  uint16_t tmp = ((uint32_t)raw * 100) / 6554; 
  return tmp;
}
#endif

#if defined(HDC1008_MEAS_TEMP) || defined(HDC1008_MEAS_BOTH)
static int16_t raw_to_temperature(uint16_t raw)
{
  /* temp = (raw / 65536) * 165 - 40; */
  /* Convert the temperature from the HDC1008 to a temperature scaled by 65536
   * in order to avoid any loss of precision. Could be scaled to a smaller
   * number but do the conversion on the host end with floating point support.
   * Or should we simply send the unconverted value as an unsigned integer to
   * the host for conversion there?
   * Current solution: 
   */
  int32_t tmp = ((uint32_t)raw * 165 - 40 * 65536); 
  int16_t temp = tmp / 6554;
  return temp;
}
#endif

#ifdef HDC1008_SET_RES_TEMP
int hdc1008_set_resolution_temp(enum hdc1008_resolution res)
{
  uint16_t conf = 0;
  
  conf_read(&conf);
  if (res == RES_14_BIT) {
    conf &= ~(1 << BITPOS_T_RES);
  } else if (res == RES_11_BIT) {
    conf |= (1 << BITPOS_T_RES);
  } else {
    return ERROR;
  }

  conf_write(conf);
  return OK;
}
#endif

#if HDC1008_HEATER
int hdc1008_heater(int enable)
{
  uint16_t conf = 0;

  conf_read(&conf);
  if (enable)
    conf |= (1 << BITPOS_HEATER);
  else
    conf &= ~(1 << BITPOS_HEATER);

  conf_write(conf);
  return OK;

}
#endif

#ifdef HDC1008_MEAS_BOTH
int hdc1008_measure_both(int16_t *temp, uint16_t *rh)
{
  uint16_t conf = 0;
  
  conf_read(&conf);
  if (!(conf & (1 << BITPOS_MODE))) {
    /* Mode is measure NOT both, error */
    print_str("ERR\r\n", 5);
    return ERROR;
  }

  buf[0] = (g_addr << 1) | 0x00;
  buf[1] = 0x00;
  i2c_transfer(buf, 2);
  _delay_ms(TEMP_DELAY + RH_DELAY);

  buf[0] = (g_addr << 1) | 0x01;
  i2c_transfer(buf, 5);

  *temp = raw_to_temperature((buf[1] << 8) | buf[2]);
  *rh = raw_to_rh((buf[3] << 8) | buf[4]);

  return OK;
}
#endif

#ifdef HDC1008_MEAS_RH
int hdc1008_measure_rh(uint16_t *rh)
{
  uint16_t conf = 0;
  
  conf_read(&conf);
  if (conf & (1 << BITPOS_MODE)) {
    /* Mode is measure both, error */
    print_str("ERR\r\n", 5);
    return ERROR;
  }

  buf[0] = (g_addr << 1) | 0x00;
  buf[1] = 0x01;
  i2c_transfer(buf, 2);
  _delay_ms(RH_DELAY);

  buf[0] = (g_addr << 1) | 0x01;
  i2c_transfer(buf, 3);

  *rh = raw_to_rh((buf[1] << 8) | buf[2]);

  return OK;
}
#endif

#ifdef HDC1008_MEAS_TEMP
int hdc1008_measure_temp(int16_t *temp)
{
  uint16_t conf = 0;
  
  conf_read(&conf);
  if (conf & (1 << BITPOS_MODE)) {
    /* Mode is measure both, error */
    print_str("ERR\r\n", 5);
    return ERROR;
  }

  buf[0] = (g_addr << 1) | 0x00;
  buf[1] = 0x00;
  i2c_transfer(buf, 2);
  _delay_ms(TEMP_DELAY);

  buf[0] = (g_addr << 1) | 0x01;
  i2c_transfer(buf, 3);

  *temp = raw_to_temperature((buf[1] << 8) | buf[2]);

  return OK;
}
#endif

#ifdef HDC1008_SET_RES_RH
int hdc1008_set_resolution_rh(enum hdc1008_resolution res)
{
  uint16_t conf = 0;
  
  conf_read(&conf);

  /* Clear RH resolution bits */
  conf &= ~(0x3 << BITPOS_RH_RES);
  if (res == RES_14_BIT) {
    /* Do nothing here, both bits cleared for 14 bit res */
  } else if (res == RES_11_BIT) {
    conf |= (1 << BITPOS_RH_RES);
  } else if (res == RES_8_BIT) {
    conf |= (2 << BITPOS_RH_RES);
  } else {
    return ERROR;
  }

  conf_write(conf);
  return OK;
}
#endif

#ifdef HDC1008_SET_MODE
int hdc1008_set_mode(enum hdc1008_mode mode)
{
  uint16_t conf;

  conf_read(&conf);
  if (mode == HDC_TEMP_OR_RH)
    conf &= ~(1 << BITPOS_MODE);
  else if (mode == HDC_BOTH)
    conf |= (1 << BITPOS_MODE);
  else
    return ERROR;

  conf_write(conf);
  return OK;
}
#endif

#ifdef HDC1008_SET_ADDRESS
/* Check for a valid HDC1008 address and set it */
int hdc1008_set_address(uint8_t addr)
{
  if ((addr < 0x40) || (addr > 0x43)) {
    return ERROR;
  } else {
    g_addr = addr;
    return OK;
  }
}
#endif

#ifdef HDC1008_GET_SERIALNO
int hdc1008_get_serialno(uint8_t *serialno)
{
  int i;
  uint16_t s1, s2, s3;

  for (i = 0; i < BUFSIZE; ++i) 
    buf[i] = 0;

  buf[0] = (g_addr << 1) | 0x00;
  buf[1] = 0xFD;
  i2c_transfer(buf, 2);

  buf[0] = (g_addr << 1) | 0x01;
  buf[1] = 0x00;
  i2c_transfer(buf, 3);

  s1 = (buf[1] << 8) | buf[2];
  print_str("SER:", 4);
  print_hex(s1);
  print_str("\r\n", 2);

}
#endif

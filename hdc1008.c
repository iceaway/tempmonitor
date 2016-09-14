#include "main.h"
#include "i2c.h"
#include "hdc1008.h"

#define DEFAULT_ADDR  0x40
#define BUFSIZE       8

#define REG_CONF      0x02
#define REG_SER_LOW   0xFD
#define REG_SER_MID   0xFC
#define REG_SER_HI    0xFB

#define BITPOS_MODE   12


static uint8_t g_addr = DEFAULT_ADDR;
static uint8_t buf[BUFSIZE];

int hdc1008_set_mode(enum hdc1008_mode mode)
{
  uint16_t conf;

  /* Read the content of the config register and set the mode as requested */
  buf[0] = (g_addr << 1) | 0x00;
  buf[1] = REG_CONF;
  i2c_transfer(buf, 2);

  buf[0] = (g_addr << 1) | 0x01;
  buf[1] = 0;
  buf[2] = 0;
  i2c_transfer(buf, 3);

  conf = (buf[1] << 8) | buf[2];
  print_str("CONF:", 5);
  print_hex(conf);
  print_str("\r\n", 2);
  if (mode == HDC_TEMP_OR_RH)
    conf &= ~(1 << BITPOS_MODE);
  else if (mode == HDC_BOTH)
    conf |= (1 << BITPOS_MODE);
  else
    return ERROR;

  print_str("CONF2:", 6);
  print_hex(conf);
  print_str("\r\n\r\n", 4);

  buf[0] = (g_addr << 1) | 0x00;
  buf[1] = REG_CONF;
  buf[2] = (conf & 0xff00) >> 8;
  buf[3] = (conf & 0x00ff);
  i2c_transfer(buf, 4);

  return OK;

}

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

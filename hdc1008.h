#ifndef __HDC_1008
#define __HDC_1008

#include <stdint.h>

enum hdc1008_resolution {
  RES_14_BIT,
  RES_11_BIT,
  RES_8_BIT
};

enum hdc1008_mode {
  HDC_TEMP_OR_RH,
  HDC_BOTH,
};


int hdc1008_set_resolution_temp(enum hdc1008_resolution res);
int hdc1008_set_resolution_rh(enum hdc1008_resolution res);
int hdc1008_enable_heater(void);
int hdc1008_disable_heater(void);
int hdc1008_reset(void);
int hdc1008_set_mode(enum hdc1008_mode mode);
int hdc1008_get_serialno(uint8_t *serialno);
int hdc1008_set_address(uint8_t addr);

int hdc1008_measure_temp(int16_t *temp);
int hdc1008_measure_rh(uint16_t *rh);
int hdc1008_measure_both(int16_t *temp, uint16_t *rh);

#endif

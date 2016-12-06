#ifndef __HDC_1008
#define __HDC_1008

#include <stdint.h>

#define HDC1008_SET_MODE
#define HDC1008_MEAS_RH
#define HDC1008_MEAS_BOTH

enum hdc1008_resolution {
  RES_14_BIT,
  RES_11_BIT,
  RES_8_BIT
};

enum hdc1008_mode {
  HDC_TEMP_OR_RH,
  HDC_BOTH,
};


#ifdef HDC1008_SET_RES_TEMP
int hdc1008_set_resolution_temp(enum hdc1008_resolution res);
#endif
#ifdef HDC1008_SET_RES_RH
int hdc1008_set_resolution_rh(enum hdc1008_resolution res);
#endif
#ifdef HDC1008_HEATER
int hdc1008_heater(int enable);
#endif
//int hdc1008_reset(void);
#ifdef HDC1008_SET_MODE
int hdc1008_set_mode(enum hdc1008_mode mode);
#endif
#ifdef HDC1008_GET_SERIALNO
int hdc1008_get_serialno(uint8_t *serialno);
#endif
#ifdef HDC1008_SET_ADDRESS
int hdc1008_set_address(uint8_t addr);
#endif

#ifdef HDC1008_MEAS_TEMP
int hdc1008_measure_temp(int16_t *temp);
#endif
#ifdef HDC1008_MEAS_RH
int hdc1008_measure_rh(uint16_t *rh);
#endif
#ifdef HDC1008_MEAS_BOTH
int hdc1008_measure_both(int16_t *temp, uint16_t *rh);
#endif

#endif

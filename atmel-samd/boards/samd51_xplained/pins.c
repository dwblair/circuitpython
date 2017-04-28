#include "samd51_pins.h"

// This mapping only includes functional names because pins broken
// out on connectors are labeled with their MCU name available from
// microcontroller.pin.
STATIC const mp_map_elem_t board_global_dict_table[] = {
    // SD Card connection
  { MP_OBJ_NEW_QSTR(MP_QSTR_MCDA0), (mp_obj_t)&pin_PB18 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_MCDA1), (mp_obj_t)&pin_PB19 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_MCDA2), (mp_obj_t)&pin_PB20 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_MCDA3), (mp_obj_t)&pin_PB21 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_MCCK), (mp_obj_t)&pin_PA21 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_MCCDA), (mp_obj_t)&pin_PA20 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_CD), (mp_obj_t)&pin_PD20 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_WP), (mp_obj_t)&pin_PD21 },

  // PCC Camera connections
  { MP_OBJ_NEW_QSTR(MP_QSTR_SCL), (mp_obj_t)&pin_PD09 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_SDA), (mp_obj_t)&pin_PD08 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_VSYNC), (mp_obj_t)&pin_PA12 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_HSYNC), (mp_obj_t)&pin_PA13 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_PCLK), (mp_obj_t)&pin_PA14 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_XCLK), (mp_obj_t)&pin_PA15 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_DOUT07), (mp_obj_t)&pin_PA23 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_DOUT06), (mp_obj_t)&pin_PA22 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_DOUT05), (mp_obj_t)&pin_PA21 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_DOUT04), (mp_obj_t)&pin_PA20 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_DOUT03), (mp_obj_t)&pin_PA19 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_DOUT02), (mp_obj_t)&pin_PA18 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_DOUT01), (mp_obj_t)&pin_PA17 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_DOUT00), (mp_obj_t)&pin_PA16 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_DOUT09), (mp_obj_t)&pin_PB15 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_DOUT08), (mp_obj_t)&pin_PB14 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_RESET),  (mp_obj_t)&pin_PB13 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_PWDN),   (mp_obj_t)&pin_PC11 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_DOUT11), (mp_obj_t)&pin_PC13 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_DOUT10), (mp_obj_t)&pin_PC12 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_DOUT13), (mp_obj_t)&pin_PC15 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_DOUT12), (mp_obj_t)&pin_PC14 },

  // PDEC header
  { MP_OBJ_NEW_QSTR(MP_QSTR_PHASE_A), (mp_obj_t)&pin_PC16 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_PHASE_B), (mp_obj_t)&pin_PC17 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_INDEX),   (mp_obj_t)&pin_PC18 },

  { MP_OBJ_NEW_QSTR(MP_QSTR_VBAT),   (mp_obj_t)&pin_PB03 },

  { MP_OBJ_NEW_QSTR(MP_QSTR_ADC_DAC),   (mp_obj_t)&pin_PA02 },

  { MP_OBJ_NEW_QSTR(MP_QSTR_SW0),   (mp_obj_t)&pin_PB31 },

  { MP_OBJ_NEW_QSTR(MP_QSTR_LED),   (mp_obj_t)&pin_PC18 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_D13),   (mp_obj_t)&pin_PC18 },

  { MP_OBJ_NEW_QSTR(MP_QSTR_QT_BUTTON),   (mp_obj_t)&pin_PB12 },

  { MP_OBJ_NEW_QSTR(MP_QSTR_QIO0),   (mp_obj_t)&pin_PA08 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_QIO1),   (mp_obj_t)&pin_PA09 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_QIO2),   (mp_obj_t)&pin_PA10 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_QIO3),   (mp_obj_t)&pin_PA11 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_QSCK),   (mp_obj_t)&pin_PB10 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_QCS),    (mp_obj_t)&pin_PB11 },

  { MP_OBJ_NEW_QSTR(MP_QSTR_RX),    (mp_obj_t)&pin_PB24 },
  { MP_OBJ_NEW_QSTR(MP_QSTR_TX),    (mp_obj_t)&pin_PB25 },
};
MP_DEFINE_CONST_DICT(board_module_globals, board_global_dict_table);

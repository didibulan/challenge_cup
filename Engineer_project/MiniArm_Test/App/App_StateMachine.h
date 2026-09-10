//
// Created by didib on 2026/7/6.
//

#ifndef INC_7AXIS_ARM_MC02_APP_STATEMACHINE_H
#define INC_7AXIS_ARM_MC02_APP_STATEMACHINE_H
#include "cmsis_os.h"
#include "stdbool.h"
#include "dev_referee.h"

typedef union
{
    struct {
        uint16_t w     : 1;
        uint16_t s     : 1;
        uint16_t a     : 1;
        uint16_t d     : 1;
        uint16_t shift : 1;
        uint16_t ctrl  : 1;
        uint16_t q     : 1;
        uint16_t e     : 1;
        uint16_t r     : 1;
        uint16_t f     : 1;
        uint16_t g     : 1;
        uint16_t z     : 1;
        uint16_t x     : 1;
        uint16_t c     : 1;
        uint16_t v     : 1;
        uint16_t b     : 1;
    };
    uint16_t keys;
} KeyBoard_t;

#endif //INC_7AXIS_ARM_MC02_APP_STATEMACHINE_H
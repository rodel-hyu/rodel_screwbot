/*
 * Copyright 2021 BLUESINK Co., Ltd.
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 *
 */

#ifndef __CANOPEN_HW_APPL_H_
#define __CANOPEN_HW_APPL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define CO_BUFLEN            20

extern void CANOpen_sendFrame(uint16_t cobID, uint8_t* data, uint8_t len, uint8_t rtr);
extern int CANOpen_mutexLock();
extern int CANOpen_mutexUnlock();

#ifdef __cplusplus
}
#endif

#endif
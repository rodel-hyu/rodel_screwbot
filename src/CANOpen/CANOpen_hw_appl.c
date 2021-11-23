/*
 * Copyright 2021 BLUESINK Co., Ltd.
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 *
 */

#include "CANOpen_hw_appl.h"

/* USER CODE BEGIN Includes */
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <linux/can.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
/* USER CODE END Includes */

/* Private typedef */
/* USER CODE BEGIN PD */
extern int s_can;
extern pthread_mutex_t mutex_can;
/* USER CODE END PD */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */  


void CANOpen_sendFrame(uint16_t cobID, uint8_t* data, uint8_t len, uint8_t rtr){

  /* USER CODE BEGIN 1 */
  static struct can_frame frame;

  static struct timeval timeout;
  static fd_set set;
  static int rv;

  frame.can_id = cobID;
  if(rtr == 1) frame.can_id |= CAN_RTR_FLAG;
   
  frame.can_dlc = len;
  memcpy(frame.data, data, len);

  FD_ZERO(&set);
  FD_SET(s_can, &set);

  timeout.tv_sec = 0;
  timeout.tv_usec = 200000;

  rv = select(s_can + 1, NULL, &set, NULL, &timeout);
  if(rv >= 0 && FD_ISSET(s_can, &set)){
    write(s_can, &frame, sizeof(frame));
  }
  /* USER CODE END 1 */  

}

int CANOpen_mutexLock(){

  /* USER CODE BEGIN 2 */
  return pthread_mutex_lock(&mutex_can);
  /* USER CODE END 2 */  

}

int CANOpen_mutexUnlock(){

  /* USER CODE BEGIN 3 */
  return pthread_mutex_unlock(&mutex_can);
  /* USER CODE END 3 */  

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */ 
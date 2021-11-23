/*
 * Copyright 2021 BLUESINK Co., Ltd.
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 *
 */

#include "CANOpen.h"
#include "CANOpen_list.h"

#include <string.h>

static CO_LIST general_buff[CO_BUFLEN];
static CO_LIST general_empty_head;
static CO_LIST general_filled_head;

static CO_LIST tpdo_buff[CO_BUFLEN];
static CO_LIST tpdo_empty_head;
static CO_LIST tpdo_filled_head;

static uint16_t cobID;
static uint8_t txData[8];
static volatile uint16_t tx_timeout;

CO_Status CANOpen_sendSync(){
  // Create sync frame
  cobID = 0x80;
  
  // Send frame
  CANOpen_sendFrame(cobID, txData, 0, 0);
  
  return CO_OK;
}

CO_Status CANOpen_sendTpdoRTR(uint8_t nodeId, uint8_t channel){
  // Create TPDO RTR frame
  switch(channel){
    case 1 : cobID = 0x180; break;
    case 2 : cobID = 0x280; break;
    case 3 : cobID = 0x380; break;
    case 4 : cobID = 0x480; break;
    default : return CO_ERROR;
  }
  cobID |= nodeId;
  
  // Send frame
  CANOpen_sendFrame(cobID, txData, 0, 1);
  
  return CO_OK;
}

CO_Status CANOpen_NMT(CO_NMT state, uint8_t id){

  uint8_t data[8];
  uint8_t len;

  switch(state){
    case CO_RESET :
    data[0] = 0x81; data[1] = id;
    break;

    case CO_COMMRESET :
    data[0] = 0x82; data[1] = id;
    break;

    case CO_STOPPED :
    data[0] = 0x02; data[1] = id;
    break;

    case CO_PREOP :
    data[0] = 0x80; data[1] = id;
    break;

    case CO_OP :
    data[0] = 0x01; data[1] = id;
    break;

    default :
    return CO_ERROR;
  }

  len = 2;
  CANOpen_sendFrame(0x00, data, len, 0);

  return CO_OK;
}

CO_Status CANOpen_writeOD(uint8_t nodeId,
                   uint16_t Index,
                   uint8_t subIndex,
                   uint8_t* data,
                   uint8_t len,
                   uint16_t timeout)
{
  
  if(len > 4) len = 4; // Only support expedited transfer
  if(len == 0) return CO_OK;
  
  // Create rxSDO frame
  cobID = (uint16_t)(nodeId & 0x7F);
  cobID |= 0x0600;
  txData[0] = 0x23 | ((4 - len) << 2);
  txData[1] = (uint8_t)Index;
  txData[2] = (uint8_t)(Index >> 8);
  txData[3] = subIndex;
  memset(txData + 4, 0, 4);
  memcpy(txData + 4, data, len);
  
  // Send frame
  CANOpen_sendFrame(cobID, txData, 8, 0);
  
  if(timeout == 0) return CO_OK;
  
  tx_timeout = timeout;
  while(tx_timeout != 0){
    // Find Valid response from filled list
    CO_LIST *filled_entry = &general_filled_head;

    while(filled_entry->next != &general_filled_head && filled_entry->next != &general_empty_head){
      filled_entry = filled_entry->next;
      if(filled_entry->cobID != (nodeId | 0x0580)) continue;
      if(filled_entry->data[0] != 0x60) continue;
      if(filled_entry->data[1] != txData[1]) continue;
      if(filled_entry->data[2] != txData[2]) continue;
      if(filled_entry->data[3] != txData[3]) continue;

      CANOpen_mutexLock();
      CANOpen_list_del(filled_entry);
      CANOpen_list_add_prev(filled_entry, &general_empty_head);
      CANOpen_mutexUnlock();
      return CO_OK;
    }
  }
  
  return CO_TIMEOUT;
}

CO_Status CANOpen_readOD(uint8_t nodeId,
                     uint16_t Index,
                     uint8_t subIndex,
                     uint8_t* data,
                     uint8_t* len,
                     uint16_t timeout)
{
    
  // Create rxSDO frame
  cobID = (uint16_t)(nodeId & 0x7F);
  cobID |= 0x0600;
  txData[0] = 0x40;
  txData[1] = (uint8_t)Index;
  txData[2] = (uint8_t)(Index >> 8);
  txData[3] = subIndex;
  memset(txData + 4, 0, 4);

  // Send frame
  CANOpen_sendFrame(cobID, txData, 8, 0);
  
  tx_timeout = timeout;
  while(tx_timeout != 0){
    // Find Valid response from filled list
    CO_LIST *filled_entry = &general_filled_head;

    while(filled_entry->next != &general_filled_head && filled_entry->next != &general_empty_head){
      filled_entry = filled_entry->next;
      if(filled_entry->cobID != (nodeId | 0x0580)) continue;
      if((filled_entry->data[0] & 0x43) != 0x43) continue;
      if(filled_entry->data[1] != txData[1]) continue;
      if(filled_entry->data[2] != txData[2]) continue;
      if(filled_entry->data[3] != txData[3]) continue;

      CANOpen_mutexLock();

      if(len != NULL && data != NULL){
        *len = 4 - ((filled_entry->data[0] & 0x0C) >> 2);
        memcpy(data, filled_entry->data + 4, *len);        
      }
      
      CANOpen_list_del(filled_entry);
      CANOpen_list_add_prev(filled_entry, &general_empty_head);
      CANOpen_mutexUnlock();
      return CO_OK;
    }
  }
  
  return CO_TIMEOUT;
}

CO_Status CANOpen_sendPDO(uint8_t nodeId, uint8_t channel, CO_PDOStruct* pdo_struct){
  
  if(channel == 0 || channel > 4) return CO_ERROR;
  
  if(pdo_struct->mappinglen == 0) return CO_OK;
  
  // Create rxPDO frame
  cobID = (uint16_t)nodeId;
  switch(channel){
  case 1 : cobID |= 0x200; break;
  case 2 : cobID |= 0x300; break;
  case 3 : cobID |= 0x400; break;
  case 4 : cobID |= 0x500; break;
  }
  
  uint64_t tmp64 = 0;
  uint16_t i;
  uint8_t bytelen;
  
  uint8_t total_bit = 0;
  uint8_t total_byte;
  
  i = pdo_struct->mappinglen;
  while(1){
    i --;
    total_bit += pdo_struct->bitlen[i];
    bytelen = (pdo_struct->bitlen[i] - 1) / 8 + 1;
    tmp64 = tmp64 << pdo_struct->bitlen[i];
    switch(bytelen){
    case 1 : tmp64 |= *(uint8_t*)pdo_struct->data[i]; break;
    case 2 : tmp64 |= *(uint16_t*)pdo_struct->data[i]; break;
    case 4 : tmp64 |= *(uint32_t*)pdo_struct->data[i]; break;
    }
    
    if(i == 0) break;
  }
  
  for(i = 0; i < 8; i++){
    txData[i] = (uint8_t)tmp64;
    tmp64 = tmp64 >> 8;
  }

  // Send frame
  total_byte = (total_bit - 1) /8 + 1;
  CANOpen_sendFrame(cobID, txData, total_byte, 0);
  
  return CO_OK;
}

uint16_t timeout_cnt;
CO_Status CANOpen_readPDO(uint8_t nodeId, uint8_t channel, CO_PDOStruct* pdo_struct, uint16_t timeout){
  
  if(channel == 0 || channel > 4) return CO_ERROR;
  
  uint16_t cobID_target = nodeId;
  uint64_t tmp64 = 0;
  uint16_t j;
  uint8_t bytelen;
  
  switch(channel){
  case 1 : cobID_target |= 0x180; break;
  case 2 : cobID_target |= 0x280; break;
  case 3 : cobID_target |= 0x380; break;
  case 4 : cobID_target |= 0x480; break;
  }
  
  tx_timeout = timeout;
  while(tx_timeout != 0){
    // Find Valid frame filled list
    CO_LIST *tpdo_entry = &tpdo_filled_head;

    while(tpdo_entry->next != &tpdo_filled_head && tpdo_entry->next != &tpdo_empty_head){

      tpdo_entry = tpdo_entry->next;
      if(tpdo_entry->cobID != cobID_target) continue;

      CANOpen_mutexLock();    

      memcpy(&tmp64, tpdo_entry->data, 8);
      for(j = 0; j < pdo_struct->mappinglen; j++){
        bytelen = (pdo_struct->bitlen[j] - 1) / 8 + 1;
        memcpy(pdo_struct->data[j], &tmp64, bytelen);
        tmp64 = tmp64 >> pdo_struct->bitlen[j];
      }

      CANOpen_list_del(tpdo_entry);
      CANOpen_list_add_prev(tpdo_entry, &tpdo_empty_head);
      CANOpen_mutexUnlock();
      return CO_OK;
    }
  }

  timeout_cnt++;
  return CO_TIMEOUT;
}

CO_Status CANOpen_writeOD_float(uint8_t nodeId, uint16_t Index, uint8_t subIndex, float data, uint16_t timeout){
  return CANOpen_writeOD(nodeId, Index, subIndex, (uint8_t*)&data, 4, timeout);
}

CO_Status CANOpen_writeOD_uint32(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint32_t data, uint16_t timeout){
  return CANOpen_writeOD(nodeId, Index, subIndex, (uint8_t*)&data, 4, timeout);
}

CO_Status CANOpen_writeOD_int32(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int32_t data, uint16_t timeout){
  return CANOpen_writeOD(nodeId, Index, subIndex, (uint8_t*)&data, 4, timeout);
}

CO_Status CANOpen_writeOD_uint16(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint16_t data, uint16_t timeout){
  return CANOpen_writeOD(nodeId, Index, subIndex, (uint8_t*)&data, 2, timeout);
}

CO_Status CANOpen_writeOD_int16(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int16_t data, uint16_t timeout){
  return CANOpen_writeOD(nodeId, Index, subIndex, (uint8_t*)&data, 2, timeout);
}

CO_Status CANOpen_writeOD_uint8(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint8_t data, uint16_t timeout){
  return CANOpen_writeOD(nodeId, Index, subIndex, &data, 1, timeout);
}

CO_Status CANOpen_writeOD_int8(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int8_t data, uint16_t timeout){
  return CANOpen_writeOD(nodeId, Index, subIndex, (uint8_t*)&data, 1, timeout);
}

void CANOpen_mappingPDO_init(CO_PDOStruct* pdo_struct){
  pdo_struct->mappinglen = 0;
}

void CANOpen_mappingPDO(CO_PDOStruct* pdo_struct, void* data, uint8_t bitlen){
  pdo_struct->data[pdo_struct->mappinglen] = data;
  pdo_struct->bitlen[pdo_struct->mappinglen] = bitlen;
  pdo_struct->mappinglen ++;
}

void CANOpen_mappingPDO_float(CO_PDOStruct* pdo_struct, float* data){
  CANOpen_mappingPDO(pdo_struct, data, 32);
}

void CANOpen_mappingPDO_uint32(CO_PDOStruct* pdo_struct, uint32_t* data){
  CANOpen_mappingPDO(pdo_struct, data, 32);
}

void CANOpen_mappingPDO_int32(CO_PDOStruct* pdo_struct, int32_t* data){
  CANOpen_mappingPDO(pdo_struct, data, 32);
}

void CANOpen_mappingPDO_uint16(CO_PDOStruct* pdo_struct, uint16_t* data){
  CANOpen_mappingPDO(pdo_struct, data, 16);
}

void CANOpen_mappingPDO_int16(CO_PDOStruct* pdo_struct, int16_t* data){
  CANOpen_mappingPDO(pdo_struct, data, 16);
}

void CANOpen_mappingPDO_uint8(CO_PDOStruct* pdo_struct, uint8_t* data){
  CANOpen_mappingPDO(pdo_struct, data, 8);
}

void CANOpen_mappingPDO_int8(CO_PDOStruct* pdo_struct, int8_t* data){
  CANOpen_mappingPDO(pdo_struct, data, 8);
}


void CANOpen_init(){
  CANOpen_list_init(&general_filled_head);
  CANOpen_list_init(&general_empty_head);

  CANOpen_list_init(&tpdo_filled_head);
  CANOpen_list_init(&tpdo_empty_head);

  int i;
  for(i = 0; i < CO_BUFLEN; i++){
    CANOpen_list_add_prev(&general_buff[i], &general_empty_head);
    CANOpen_list_add_prev(&tpdo_buff[i], &tpdo_empty_head);
  }
}

void CANOpen_addRxBuffer(uint16_t cobID, uint8_t* data){

  uint16_t fcode = (cobID & 0x780) >> 7;

  CANOpen_mutexLock();
  
  if(fcode > 2 && fcode < 11 && (fcode % 2) == 1){ // [[ TPDO frame ]]

    // Search from empty list
    CO_LIST *empty_entry = tpdo_empty_head.next;
    if(empty_entry == &tpdo_empty_head){
      // If no empty entry delete oldest filled entry
      empty_entry = tpdo_filled_head.prev;
    }

    CANOpen_list_del(empty_entry);

    // Copy data
    empty_entry->cobID = cobID;
    memcpy(empty_entry->data, data, 8);

    // Save only one TPDO per some channel of ID
    CO_LIST *tpdo_entry = &tpdo_filled_head;
    while(tpdo_entry->next != &tpdo_filled_head){
      tpdo_entry = tpdo_entry->next;
      if(tpdo_entry->cobID == cobID){
        CANOpen_list_del(tpdo_entry);
        CANOpen_list_add_prev(tpdo_entry, &tpdo_empty_head);
        break;
      }
    }
    CANOpen_list_add_next(empty_entry, &tpdo_filled_head); 

  }else{ // [[ Normal frame ]]

    // Search from empty list
    CO_LIST *empty_entry = general_empty_head.next;
    if(empty_entry == &general_empty_head){
      // If no empty entry delete oldest filled entry
      empty_entry = general_filled_head.prev;
    }

    CANOpen_list_del(empty_entry);

    // Copy data
    empty_entry->cobID = cobID;
    memcpy(empty_entry->data, data, 8);

    CANOpen_list_add_next(empty_entry, &general_filled_head);    
  }

  CANOpen_mutexUnlock();
  
}

void CANOpen_timerLoop(){ // Should be call every 1ms
  
  if(tx_timeout != 0) tx_timeout --;

}
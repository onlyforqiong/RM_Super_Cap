/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \BTlog\debug_BTlog.h
 * @Descripttion : 
 * @Author       : GDDG08
 * @Date         : 2021-10-31 09:17:07
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-01-18 14:20:25
 */
#ifndef DEBUG_BTLOG_H
#define DEBUG_BTLOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "uart_util.h"

typedef enum {
    BYTE = 0u,
    uInt8 = 1u,
    uInt16 = 2u,
    uInt32 = 3u,
    Float = 4u,
    Char = 5u,
    Int8 = 6u,
    Int16 = 7u,
    Int32 = 8u
} BTlog_TypeEnum;

#define BTlog_tagSize 19

typedef struct {
    void* ptr;
    uint8_t size;
    uint8_t type;
    char tag[BTlog_tagSize];
    // void (*log_func)(uint8_t buff[]);
} BTlog_TableEntry;

extern UART_HandleTypeDef* Const_BTlog_UART_HANDLER;

void BTlog_Init(void);
void BTlog_Send(void);
void BTlog_RXCallback(UART_HandleTypeDef*);
uint8_t BTLog_VerifyData(uint8_t*, uint16_t);

#ifdef __cplusplus
}
#endif

#endif

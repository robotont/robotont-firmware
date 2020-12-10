/*
 * mx.h
 *
 *  Created on: 2019/03/22
 *      Author: aruaru
 */

#ifndef SOURCE_MX_H_
#define SOURCE_MX_H_
#include "main.h"

#ifdef __cplusplus
 extern "C" {
#endif

HAL_StatusTypeDef printUart(char* buf,int num);
HAL_StatusTypeDef startRxUart(void);
int readable(void);
int getChar(char* c);
int readCndtr(void);
int readPos(void);

#ifdef __cplusplus
}
#endif
#endif /* SOURCE_MX_H_ */

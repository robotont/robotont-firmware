/* mbed Microcontroller Library
 * Copyright (c) 2018 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "stats_report.h"

/*
 * main.cpp
 *
 *  Created on: 2019/04/03
 *      Author: aruaru
 */


#include "application.h"

#include "usart.h"
#include "dma.h"
#include "mx.h"

EventQueue* hqueue;
DigitalOut* hled;
Timer utime;
int now;//1ms毎に更新
#if defined ( __GNUC__ )
int printfunc(const char *format, ...){
    char* b;
    const size_t size=300;
    int len;
    va_list arg;
    b = (char *)malloc(sizeof(char)*size);
    va_start(arg, format);
    len=vsnprintf( b , size , format, arg );
    int status;
    int retry;
    while(1)
    {
        status = printUart( b , len );
        if(status == HAL_OK)break;
        wait_ms(10);
        retry++;
        if(retry>10){
            len=0;
            break;
        }
    }
    va_end(arg);
    free(b);
    return len;
}
#else
int printfunc(const char *format, ...){
    char* b;
    int len;
    va_list arg;
    va_start(arg, format);
    len=vasprintf( &b , format, arg );
    int status;
    int retry;
    while(1)
    {
        status = printUart( b , len );
        if(status == HAL_OK)break;
        wait_ms(10);
        retry++;
        if(retry>100){
            len=0;
            break;
        }
    }
    va_end(arg);
    free(b);
    return len;
}
#endif
void interval1ms(void){
    char c;
    now=utime.read_ms();
    if(1){
        if( getChar(&c) ){
            logMessage("utime:%d,cndtr:%d,pos:%d,%02x,%c\n\r",now,readCndtr(),readPos(),c,c);
        }
    }
}
void interval1000ms(void){
    *hled = !*hled;
}
void systemInfo(){
    mbed_stats_sys_t stats;
    mbed_stats_sys_get(&stats);

    /* CPUID Register information
    [31:24]Implementer      0x41 = ARM
    [23:20]Variant          Major revision 0x0  =  Revision 0
    [19:16]Architecture     0xC  = Baseline Architecture
                            0xF  = Constant (Mainline Architecture?)
    [15:4]PartNO            0xC20 =  Cortex-M0
                            0xC60 = Cortex-M0+
                            0xC23 = Cortex-M3
                            0xC24 = Cortex-M4
                            0xC27 = Cortex-M7
                            0xD20 = Cortex-M23
                            0xD21 = Cortex-M33
    [3:0]Revision           Minor revision: 0x1 = Patch 1.
    */

    /* Compiler versions:
       ARM: PVVbbbb (P = Major; VV = Minor; bbbb = build number)
       GCC: VVRRPP  (VV = Version; RR = Revision; PP = Patch)
       IAR: VRRRPPP (V = Version; RRR = Revision; PPP = Patch)
    */
    logMessage("\n\r[%s] %s started.\n\r",__FUNCTION__,__FILE__            );
    logMessage("[%s]\tBUILD:%sT%s,\n\r",__FUNCTION__,__DATE__, __TIME__          );
    logMessage("[%s]\tmbed-OS:%d.%d.%d\n\r",__FUNCTION__,
            MBED_MAJOR_VERSION, MBED_MINOR_VERSION,MBED_PATCH_VERSION
            );
    logMessage("[%s]\tMbed OS Version: %ld \n\r",__FUNCTION__, stats.os_version);
    logMessage("[%s]\tCPU ID: 0x%x \n\r",__FUNCTION__,(unsigned int) stats.cpu_id);
    logMessage("[%s]\tCompiler ID: %d \n\r",__FUNCTION__, stats.compiler_id);
    logMessage("[%s]\tCompiler Version: %d \n\r",__FUNCTION__, (unsigned int)stats.compiler_version);
    logMessage("[%s]\tSTM32HAL_driver Version: 0x%08x \n\r",__FUNCTION__, (unsigned int)HAL_GetHalVersion());
}
int main(){
    utime.start();
    hled = new DigitalOut(LED1);
    hqueue = new EventQueue(32*EVENTS_EVENT_SIZE);

    MX_DMA_Init();
    MX_USART2_UART_Init();
    HAL_StatusTypeDef status;

    systemInfo();

    status = startRxUart();
    if( status != HAL_OK ){
        printf("error status: %d\n\r",status);
    }else{
        logMessage("uartRXstart status: %d\n\r", status );
    }
    hqueue->call_every(1000,interval1000ms);
    hqueue->call_every(1,interval1ms);
    hqueue->dispatch_forever();
}

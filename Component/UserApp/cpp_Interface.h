/********************************************************************************
  * @file           : __CPP_INTERFACE_H.h
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 12/17/2025
  *******************************************************************************/


#ifndef __CPP_INTERFACE_H
#define __CPP_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif
/*--------------------------------- c ----------------------------*/
#include "main.h"		//real main
#include "usart.h"		//
// FreeRTOS include
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

void CPP_Main();

#ifdef __cplusplus
}
/*------------------------------------ c++ ---------------------------*/
//#include "button.h"		//demo function


#endif //__cplusplus
#endif //__CPP_INTERFACE_H

/** @file CPS_common.h 
*   @brief common definitions used across the paddle shift project
*   @date ...
*   @version 0.01
*
*   This header file contains common definitions used by both processors in the paddle shift project.
*   
*   
*/

/* (c) Jonathan Thomson, Vancouver, BC */

#ifndef __CPS_COMMON_H__
#define __CPS_COMMON_H__

/* Include Files */
#include "sys_common.h"
#include "gio.h"
#include "het.h"
#include "sci.h"
#include "rti.h"
#include "adc.h"
#include "spi.h"

/* Defines */

/* Communication Definitions */
#define CPS_COMMON_SCI_SHIFTUP      0xAAu
#define CPS_COMMON_SCI_SHIFTDN      0xBFu
#define CPS_COMMON_SCI_HORNON       0xFu
#define CPS_COMMON_SCI_HORNOFF      0x48u

#endif
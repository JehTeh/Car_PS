/** @file CPS_main.h 
*   @brief ...
*   @date ...
*   @version 0.01
*
*   This header contains the main interface for the SWI processor.
*   
*/

/* (c) Legend Power Systems, Burnaby, BC. */

#ifndef __CPS_MAIN_H__
#define __CPS_MAIN_H__

/* Include Files */
#include "CPS_common.h"

/* Defines */

/* Global Types */

/* Global Function Prototypes */

void CPS_vMain(void);
void CPS_vISRADCGroup1(void);
void CPS_vISRRTICompare0(void);
void CPS_vISRRTICompare1(void);

#endif
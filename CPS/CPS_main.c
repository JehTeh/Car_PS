/** @file CPS_main.c 
*   @brief Legend startup/initilization routines
*   @date 27 SEPT 2014
*   @version 0.01
*
*   Car Paddle Shifter (CPS) code is used by the RM42 board to convert a voltage from the horn wire into a horn active or paddle up/down
*   active signal.
*/

/* (c) Jonathan Thomson, Vancouver, BC */

/* Include Files */
#include "CPS_main.h"
#include "sys_core.h"

/* Defines */
#define ADC_UPPERBOUND_SHFTUP 0x09D1u //These definitions set the limits for the ADC conversion to trigger a shift or horn signal.
#define ADC_LOWERBOUND_SHFTUP 0x078Du
#define ADC_UPPERBOUND_SHFTDN 0x0593u
#define ADC_LOWERBOUND_SHFTDN 0x044Au
#define ADC_UPPERBOUND_HORNON 0x0440u
#define ADC_LOWERBOUND_HORNON 0x0000u
#define ADC_DATABUFFERSIZE 8u

#define DEBOUNCE_PADDLES_MS 30 //Debounce time in milliseconds for the paddle shift signal (should be multiple of ten)
#define DEBOUNCE_HORN_MS    30 //Debounce time in milliseconds for the horn signal (off and on)

#define ACTIVETIME_PADDLES_MS 10 //How long to hold the paddle switch for on a valid signal

#define HOLDTIME_PADDLES_SAMPLES 3 //Number of consecutive valid samples for an "active" signal
#define HOLDTIME_HORN_SAMPLES 3 

#define IO_BYPASSRELAY_PORT hetPORT1 //idle bypass relay used to ensure horn signal works normally if module is in error state
#define IO_BYPASSRELAY_PIN 6u
#define IO_BYPASSRELAY_OPEN 1u
#define IO_BYPASSRELAY_CLOSED 0u
#define IO_SHIFTDOWN_PORT hetPORT1 //downshift output. Signal is active low.
#define IO_SHIFTDOWN_PIN 0u
#define IO_SHIFTDOWN_ON 0u
#define IO_SHIFTDOWN_OFF 1u
#define IO_SHIFTUP_PORT hetPORT1 //upshift output. Signal is active low.
#define IO_SHIFTUP_PIN 1u
#define IO_SHIFTUP_ON 0u
#define IO_SHIFTUP_OFF 1u
#define IO_HORN_PORT hetPORT1 //horn output. Signal?
#define IO_HORN_PIN 2u
#define IO_HORN_ON 0u
#define IO_HORN_OFF 1u

#define STARTUPTIME_MS 3000 //CPS "start up" time in milliseconds. All ADC signals are ignored until this time has expired.

#define COMPARETIMER_CONVERSIONFACTOR 2 //Timer period in milliseconds

/* Variable Init. */
typedef enum
{
  eCMD_ShiftUp,
  eCMD_ShiftDown,
  eCMD_HornOn,
  eCMD_HornOff,
  eCMD_Null
} xHornCommands_t;

typedef enum
{
  eIO_Horn,
  eIO_ShiftUp,
  eIO_ShiftDown
} xIOSignals_t;

/* Global Vars */

/* Internal Vars */
static bool bStartUpTimeDone;
static bool bPaddleDebounceActive;
static bool bHornDebounceActive;

static bool bShiftUpCommand;
static bool bShiftDownCommand;
static bool bHornActiveCommand;

static bool bShiftUpHoldActive;
static bool bShiftDownHoldActive;

static adcData_t xADCData[ADC_DATABUFFERSIZE];

/* Local Function Prototypes */
static void vInitCPS(void);
static xHornCommands_t ProcessADCData(uint16_t u16Data);
static void vSendCommand(xHornCommands_t xCommand);
static void vSetOutput(xIOSignals_t xOutputType, uint32_t u32OutputValue);
static void vERROR(void);

/* Global Functions */
void CPS_vMain(void)
{
  bPaddleDebounceActive = 0;
  bHornDebounceActive = 0;
  vInitCPS();
  for(;;)
  {
    if(bShiftUpCommand)
    {
      vSetOutput(eIO_ShiftUp, 1u); //activate shift up signal
      if(!bShiftUpHoldActive)
      {
        vSetOutput(eIO_ShiftUp, 0u); //turn off shift up signal
        bShiftUpCommand = 0;
      }
    }
    if(bShiftDownCommand)
    {
      vSetOutput(eIO_ShiftDown, 1u); //activate shift up signal
      if(!bShiftDownHoldActive)
      {
        vSetOutput(eIO_ShiftDown, 0u); //turn off shift up signal
        bShiftDownCommand = 0;
      }
    }
    if(bHornActiveCommand)
    {
      vSetOutput(eIO_Horn, 1u); //activate horn
    }
    else
    {
      vSetOutput(eIO_Horn, 0u); //disable horn
    }
  }
}
/* void CPS_vISRADCGroup1(void)
*   Triggered on the completion of a conversion.Should interrupt every ms with a finished conversion
*
*/
void CPS_vISRADCGroup1(void)
{
  uint32_t u32ADCDataTotal;
  u32ADCDataTotal = adcGetData(adcREG1, adcGROUP1, &xADCData[0]);
  static uint32_t u32ShiftUpSuccessiveCount;
  static uint32_t u32ShiftDownSuccessiveCount;
  static uint32_t u32HornSuccessiveCount;
  for(uint32_t u32Count = 0u; u32Count <= u32ADCDataTotal; u32Count++)
  {
    if(u32ADCDataTotal >= ADC_DATABUFFERSIZE)
    {
      vERROR();
    }
    switch(ProcessADCData(xADCData[u32Count].value))
    {
    case eCMD_ShiftUp:
      u32ShiftDownSuccessiveCount = 0u; //clear other counters
      u32HornSuccessiveCount = 0u;
      if(!bPaddleDebounceActive) //If paddle is not currently in debounce, accept signal
      {
        u32ShiftUpSuccessiveCount++; //increment sample counter
        if(u32ShiftUpSuccessiveCount >= HOLDTIME_PADDLES_SAMPLES) //if consecutive sample is valid, shifting action is considered real
        {
          u32ShiftUpSuccessiveCount = 0u;
          bPaddleDebounceActive = 1;
          vSendCommand(eCMD_ShiftUp); //Send shift up command to output handlers
        }
      }
      break;
    case eCMD_ShiftDown:
      u32ShiftUpSuccessiveCount = 0u; //clear other counters
      u32HornSuccessiveCount = 0u;
      if(!bPaddleDebounceActive) //If paddle is not currently in debounce, accept signal
      {
        u32ShiftDownSuccessiveCount++; //increment sample counter
        if(u32ShiftDownSuccessiveCount >= HOLDTIME_PADDLES_SAMPLES) //if consecutive sample is valid, shifting action is considered real
        {
          u32ShiftDownSuccessiveCount = 0u;
          bPaddleDebounceActive = 1;
          vSendCommand(eCMD_ShiftDown); //Send shift up command to output handlers
        }
      }
      break;
    case eCMD_HornOn:
      u32ShiftDownSuccessiveCount = 0u; //clear other counters
      u32ShiftUpSuccessiveCount = 0u;
      if(!bHornDebounceActive) //If horn is not currently in debounce, accept signal
      {
        u32HornSuccessiveCount++; //increment sample counter
        if(u32HornSuccessiveCount >= HOLDTIME_HORN_SAMPLES) //if consecutive sample is valid, horn is considered active
        {
          u32HornSuccessiveCount = 0u;
          bHornDebounceActive = 1;
          vSendCommand(eCMD_HornOn); //Send horn now active command to output handlers
        }
      }
      break;
    case eCMD_Null:
      u32ShiftUpSuccessiveCount = 0u;       //clear all successive counters
      u32ShiftDownSuccessiveCount = 0u;
      u32HornSuccessiveCount = 0u;
      vSendCommand(eCMD_HornOff); //If horn is not active then ensure it is off.
      break;
    default:
      u32ShiftUpSuccessiveCount = 0u;
      u32ShiftDownSuccessiveCount = 0u;
      u32HornSuccessiveCount = 0u;
      vSendCommand(eCMD_HornOff);
      break;
    }
  }
}

/* void CPS_vISRRTICompare1(void)
*   Triggered by the RTI compare0 timer. Should be 1ms time base. This is used by system counters to trigger ADC.
*
*/
void CPS_vISRRTICompare0(void)
{
  adcResetFiFo(adcREG1, adcGROUP1);
  adcStartConversion(adcREG1, adcGROUP1);
}

/* void CPS_vISRRTICompare1(void)
*   Triggered by the RTI compare1 timer. Should be 2ms time base. This is used by system counters as a timer.
*
*/
void CPS_vISRRTICompare1(void)
{
  static uint32_t u32StartTimeCounter;
  static uint32_t u32PaddleDebounceCounter;
  static uint32_t u32HornDebounceCounter;
  static uint32_t u32PaddleUpHoldCounter;
  static uint32_t u32PaddleDownHoldCounter;
  if(!bStartUpTimeDone) //Handle startup time count
  {
    u32StartTimeCounter++;
    if(u32StartTimeCounter >= STARTUPTIME_MS/COMPARETIMER_CONVERSIONFACTOR)
    {
      bStartUpTimeDone = 1; //indicate startup time finished
    }
  }
  if(bPaddleDebounceActive) //if debounce is active, start timing
  {
    u32PaddleDebounceCounter++;
    if(u32PaddleDebounceCounter >= DEBOUNCE_PADDLES_MS/COMPARETIMER_CONVERSIONFACTOR)
    {
      u32PaddleDebounceCounter = 0u; //reset counter
      bPaddleDebounceActive = 0; //indicate debounce finished
    }
  }
  if(bHornDebounceActive)
  {
    u32HornDebounceCounter++;
    if(u32HornDebounceCounter >= DEBOUNCE_HORN_MS/COMPARETIMER_CONVERSIONFACTOR)
    {
      u32HornDebounceCounter = 0u;
      bHornDebounceActive = 0;
    }
  }
  if(bShiftUpHoldActive)
  {
    u32PaddleUpHoldCounter++;
    if(u32PaddleUpHoldCounter >= ACTIVETIME_PADDLES_MS/COMPARETIMER_CONVERSIONFACTOR)
    {
      u32PaddleUpHoldCounter = 0u;
      bShiftUpHoldActive = 0;
    }
  }
  if(bShiftDownHoldActive)
  {
    u32PaddleDownHoldCounter++;
    if(u32PaddleDownHoldCounter >= ACTIVETIME_PADDLES_MS/COMPARETIMER_CONVERSIONFACTOR)
    {
      u32PaddleDownHoldCounter = 0u;
      bShiftDownHoldActive = 0;
    }
  }
}

/* Local Functions */
static void vInitCPS(void)
{
  gioInit();
  hetInit();
  adcInit();
  rtiInit();
  bStartUpTimeDone = 0;
  rtiResetCounter(0u);
  rtiStartCounter(0u);
  _enable_interrupt_();
  rtiEnableNotification(rtiNOTIFICATION_COMPARE1);
  while(!bStartUpTimeDone)
  {
    //Wait for start up time to expire
  }
  gioSetBit(IO_BYPASSRELAY_PORT, IO_BYPASSRELAY_PIN, IO_BYPASSRELAY_OPEN); //
  adcEnableNotification(adcREG1, adcGROUP1); //Enable ADC ISR routine
  adcResetFiFo(adcREG1, adcGROUP1);
  adcStartConversion(adcREG1, adcGROUP1);
  rtiEnableNotification(rtiNOTIFICATION_COMPARE0);
}

static xHornCommands_t ProcessADCData(uint16_t u16Data)
{
  if(u16Data <= ADC_UPPERBOUND_HORNON) //Is this a horn depressed signal?
  {
    if(u16Data >= ADC_LOWERBOUND_HORNON)
    {
      return(eCMD_HornOn);
    }
  }
  if(u16Data <= ADC_UPPERBOUND_SHFTUP) //Is this a shift up signal?
  {
    if(u16Data >= ADC_LOWERBOUND_SHFTUP)
    {
      return(eCMD_ShiftUp);
    }
  }
  if(u16Data <= ADC_UPPERBOUND_SHFTDN) //Is this a shift down signal?
  {
    if(u16Data >= ADC_LOWERBOUND_SHFTDN)
    {
      return(eCMD_ShiftDown);
    }
  }
  return (eCMD_Null); //Not an active signal
}
      
static void vSendCommand(xHornCommands_t xCommand)
{
  switch(xCommand)
  {
  case eCMD_ShiftUp:
    bShiftUpCommand = 1; //Send single shift up pulse
    bShiftUpHoldActive = 1;
    break;
  case eCMD_ShiftDown:
    bShiftDownCommand = 1; //Send single shift down pulse
    bShiftUpHoldActive = 1;
    break;
  case eCMD_HornOn:
    bHornActiveCommand = 1; //Switch on horn active signal
    break;
  case eCMD_HornOff:
    bHornActiveCommand = 0; //Turn off horn
    break;
  default:
    bHornActiveCommand = 0;
    break;
  }
}

static void vSetOutput(xIOSignals_t xOutputType, uint32_t u32OutputValue)
{
  switch(xOutputType)
  {
  case eIO_Horn:
    if(u32OutputValue == 1)
    {
      gioSetBit(IO_HORN_PORT, IO_HORN_PIN, IO_HORN_ON);
      gioSetBit(gioPORTA, 2u, 1u); //Debug horn both LEDs ON
      gioSetBit(hetPORT1, 8u, 1u);
    }
    else
    {
      gioSetBit(IO_HORN_PORT, IO_HORN_PIN, IO_HORN_OFF);
      gioSetBit(gioPORTA, 2u, 0u); //Debug horn both LEDs Off
      gioSetBit(hetPORT1, 8u, 0u);
    }
    break;
  case eIO_ShiftUp:
    if(u32OutputValue == 1)
    {
      gioSetBit(IO_SHIFTUP_PORT, IO_SHIFTUP_PIN, IO_SHIFTUP_ON);
      gioSetBit(gioPORTA, 2u, 1u); //Debug horn 1 led on
      gioSetBit(hetPORT1, 8u, 0u);
    }
    else
    {
      gioSetBit(IO_SHIFTUP_PORT, IO_SHIFTUP_PIN, IO_SHIFTUP_OFF);
      gioSetBit(gioPORTA, 2u, 0u); //Debug both LEDs Off
      gioSetBit(hetPORT1, 8u, 0u);
      
    }
    break;
  case eIO_ShiftDown:
    if(u32OutputValue == 1)
    {
      gioSetBit(IO_SHIFTDOWN_PORT, IO_SHIFTDOWN_PIN, IO_SHIFTDOWN_ON);
      gioSetBit(gioPORTA, 2u, 0u); //Debug horn other led ON
      gioSetBit(hetPORT1, 8u, 1u);
    }
    else
    {
      gioSetBit(IO_SHIFTDOWN_PORT, IO_SHIFTDOWN_PIN, IO_SHIFTDOWN_OFF);
      gioSetBit(gioPORTA, 2u, 0u); //Debug both LED Off
      gioSetBit(hetPORT1, 8u, 0u);
    }
    break;
  default:
    //do nothing
    break;
  }
}

static void vERROR(void)
{
  //Crash system through WDT
  dwdInit(2u); //initialize WDT
  dwdCounterEnable(); //start counting
  while(1); //Hold until crashed.
}
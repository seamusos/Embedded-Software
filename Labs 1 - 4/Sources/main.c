/* ###################################################################
**     Filename    : main.c
**     Project     : Lab4
**     Processor   : MK70FN1M0VMJ12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-07-20, 13:27, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 4.0
** @brief
**         Main module.
**         This module contains user's application code.
*/
/*!
**  @addtogroup main_module main module documentation
**  @{
*/
/* MODULE main */


// CPU module - contains low level hardware initialization routines
#include "Cpu.h"
#include "Events.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "types.h"

#include "FIFO.h"
#include "packet.h"
#include "UART.h"
#include "Flash.h"
#include "LEDs.h"
#include "FTM.h"
#include "PIT.h"
#include "RTC.h"
#include "I2C.h"
#include "accel.h"
#include "OS.h"

#define CMD_STARTUP   0x04
#define CMD_VERSION   0x09
#define CMD_NUMBER    0x0B
#define CMD_SETTIME   0x0C
#define CMD_TOWERMODE 0X0D
#define CMD_PROGBYTE  0x07
#define CMD_READBYTE  0x08
#define CMD_SETPROTOCOL 0x0A
#define CMD_ACCELVALUES 0x10

#define THREAD_STACK_SIZE 100

static const int BAUDRATE = 115200;

static void InitThread(void* pData);
static void PITThread(void* pData);
static void RTCThread(void* pData);
static void DataThread(void* pData);
static void ReadCompleteThread(void* pData);
static void PacketThread(void* pData);
static void FTMThread(void* pData);

// Function Declarations
bool StartupPacket(void);
bool VersionPacket(void);
bool NumberPacket(void);
bool ModePacket(void);
bool ReadBytePacket(void);
bool ProgBytePacket(void);
bool SetTimePacket(void);
bool SetProtocol(void);
//void PITCallback(void* arg);
//void RTCCallback(void* arg);
//void FTM0Callback(void* arg);
//void ReadCompleteCallback(void* arg);
//void DataReadyCallback(void* arg);

static volatile uint16union_t* NvtowerNumber; /*!< Static variable to store towerNumber */
static volatile uint16union_t* NvtowerMode;   /*!< Static variable to store tower Mode */

static uint16_t TowerNumber = 5953;   /*!< towerNumber of Joel Goodwin */ //
static uint16_t DefaultTowerMode = 1; /*!< default tower Mode */

volatile TAccelMode Protocol_Mode = ACCEL_POLL; /*!< Initial protocol mode selected */
static TAccelData accelerometerValues; /*!< Array to store accelerometer values */

static TFTMChannel FTM0Channel0;  /*!< Stores the parameters for Channel 0 */

TAccelSetup Accelerometer = {CPU_BUS_CLK_HZ, NULL, &Accelerometer, NULL };

static uint32_t InitThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PITThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t RTCThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t DataReadyThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t ReadCompleteThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t FTMThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PacketThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

OS_ECB *Data_Ready_Semaphore;  //defining the semaphores to be used that are declared in modules
OS_ECB *ReadComplete_Semaphore;
OS_ECB *UpdateClock_Semaphore;
OS_ECB *PIT_Semaphore;
OS_ECB *FTM_Semaphore;


/*! @brief  Sends starting values to PC
 *
 *  @return Returns True if success
 */
bool StartupPacket(void)
{
  return ((Packet_Put(CMD_STARTUP, 0, 0, 0)) &&  /* returns the packet for the startup, version and tower number and tower Mode*/
	        (Packet_Put(CMD_VERSION, 'v', 0x01, 0x00)) &&
	        (Packet_Put(CMD_NUMBER, 0x01, NvtowerNumber->s.Lo, NvtowerNumber->s.Hi)) &&
	        (Packet_Put(CMD_TOWERMODE, 0X01, NvtowerMode->s.Lo, NvtowerMode->s.Hi)) && (Packet_Put(CMD_SETPROTOCOL,1,Protocol_Mode,0)));
}

/*! @brief  Sends tower Version to PC
 *
 *  @return Returns True if success
 */
bool VersionPacket(void)
{
  return Packet_Put(CMD_VERSION, 'v', 0x01, 0x00); /* handles the packet for the tower version Parameter 1 = 'v' Parameter 2= 1 Parameter 3 = 0 from the Tower Serial Guide */
}


/*! @brief NumberPacket
 *  Returns current Tower number if requested or writes new Number into Flash 
 * 
 *  @return BOOL - true if success
 */
bool NumberPacket(void)
{
  bool success = false; /*!< stores success flag */

  if (Packet_Parameter1 == 0x01)        /* if the tower is in get mode it simply returns the current tower number */
  {
    return Packet_Put(CMD_NUMBER, 0x01, NvtowerNumber->s.Lo, NvtowerNumber->s.Hi);  // returns the towerNumber
  }
  else if (Packet_Parameter1 == 0x02) /* if the tower is in set mode it sets the tower number by setting the 1st parameter = 1 2nd parameter to low part of student number and 3 = the hi part of the student number */
  {
    success = Flash_Write16((uint16_t*)NvtowerNumber, Packet_Parameter12);
  }

  return success;  /* if the tower is not in set or get mode it will return false */
}


/*! @brief Returns current Tower mode if requested or writes new Number into Flash 
 *
 *  @return bool -  True if success
 */
bool ModePacket(void)
{
  bool success = false;                 /*!< stores success flag */
  if (Packet_Parameter1 == 0x01)       // get mode
  {
    return Packet_Put(CMD_TOWERMODE, 0x01, NvtowerMode->s.Lo, NvtowerMode->s.Hi);
  }
  else if (Packet_Parameter1 == 0x02) /* if the tower is in set mode it sets the tower mode by setting the 1st parameter = 1 2nd parameter to low part of tower number and 3 = the hi part of the tower number */
  {
    success = Flash_Write16((uint16_t*)NvtowerMode, Packet_Parameter23);
    //return Packet_Put(CMD_NUMBER, 0x01, NvtowerMode->s.Lo, NvtowerMode->s.Hi)&& success; // the code is exactly the same as above just for the parameters for the towermode
  }

  return success;  /* if the tower is not in set or get mode it will return false */

}

/*! @brief  Writes a Byte to flash
 *
 *  @return Returns True if success
 */
bool ProgBytePacket(void)
{
  if ((Packet_Parameter1 < 0) || (Packet_Parameter1 > 8)) // Checks to see if the parameter is within the limits
  {
    return false;  // if the parameters is outside of the limits it will return 0
  }

  if (Packet_Parameter1 == 0x08) // 0x08 erases the flash
  {
    return Flash_Erase(); // calls the flash erase function
  }

  return Flash_Write8((uint8_t *)(FLASH_DATA_START + Packet_Parameter1), Packet_Parameter3); // Writes the data in parameter 3 at the address starting at flash data started offsetted by pac
}

/*! @brief Reads byte from memory then returns to PC
 *
 *  @return Returns True if success
 */
bool ReadBytePacket(void)
{
  // if packet_parameter is in range then it will return the parameters
  if ((Packet_Parameter1 >= 0) || (Packet_Parameter1 <= 7))
  {
    return (Packet_Put(CMD_READBYTE, Packet_Parameter1, 0x00, _FB(FLASH_DATA_START + Packet_Parameter1))); // data is accessed via started at Flash_data_start and offsetting by PacketParameter1
  }

  return false; // if the parameter is outside of the range then the function will return false;
}


/*! @brief Sets RTC clock time
 *
 *  @return bool -  True if success
 */
bool SetTimePacket(void)
{
  if ((Packet_Parameter1 >= 0) || (Packet_Parameter1 <= 23) && (Packet_Parameter2 >= 0) || (Packet_Parameter2 <= 59) &&
      (Packet_Parameter3 >= 0) || (Packet_Parameter3 <= 59))
  {
    RTC_Set(Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
    return true;
  }
  else
  {
      return false;  // if the parameters are not within the time limits then return false
  }
}

/*! @brief Sets RTC clock time
 *
 *  @return bool -  True if success
 */
bool SetProtocol(void)
{
  bool success;

  if (Packet_Parameter1 == 1) // if packet parameter 1 then get current protocol mode
  {
    success = Packet_Put(CMD_SETPROTOCOL,1,Protocol_Mode,0); // Protocol mode
    return success;
  }

  else if (Packet_Parameter1 == 2) //if packet parameter 1 = 2 then set protocol mode
  {
    if (Packet_Parameter2 == 0) // if packet parameter 2 = 0 then set  for asynchronous mode
    {
      Protocol_Mode = ACCEL_POLL;
      success = Packet_Put(CMD_SETPROTOCOL,1,Protocol_Mode,0); // Protocol mode
      Accel_SetMode(ACCEL_POLL); // Polling Method
      return success;
    }
    else if (Packet_Parameter2 == 1) // if packet parameter 2 = 1  then set synchronous mode
    {
      Protocol_Mode = ACCEL_INT;
      success = Packet_Put(CMD_SETPROTOCOL,1,Protocol_Mode,0); // Protocol mode
      Accel_SetMode(ACCEL_INT); // Interrupt Method
      return success;
    }
  }
}

/*! @brief  Function to handle incoming packets 
 *
 */
void HandlePacket(void)
{
  bool success = false;   /*!< holds whether it was a success of failure of the packet handler */
  bool request  = false;  /*!< holds if an acknowledge request has happened */

  LEDs_On(LED_BLUE); // Turn on blue LED

  FTM0Channel0.ioType.inputDetection = TIMER_OUTPUT_HIGH; // Start FTM0 timer - channel 0
  FTM_StartTimer(&FTM0Channel0);

  if ( PACKET_ACK_MASK & Packet_Command ) /* Looks to see if the ack bit is set */
  {
    request = true; /* sets the request to true */
    Packet_Command &= 0x7F; /* takes the top bit off the command byte to ignore the ACK bit */
  }

  switch (Packet_Command)   /* checks to see which type of packet it is and handles it accordingly */
  {
    case CMD_STARTUP:
      StartupPacket();
      success = true;
      break;
    case CMD_VERSION:
      VersionPacket();
      success = true;
      break;
    case CMD_NUMBER:
      NumberPacket();
      success = true;
      break;
    case CMD_TOWERMODE:
      ModePacket();
      success = true;
      break;
    case CMD_PROGBYTE:
      ProgBytePacket();
      success = true;
      break;
    case CMD_READBYTE:
      ReadBytePacket();
      success = true;
      break;
    case CMD_SETTIME:
      SetTimePacket();
      success = true;
      break;
    case CMD_SETPROTOCOL:
      SetProtocol();
      success = true;
      break;
    default:
      success = false;
      break;
  }
  /* see if the handing of the packet was a success and an ack packet was requested if it is correct then set the ack bit to 1
  *  if the packet has failed and ack was requested set the ack to indicated a nak then return the ack packet back to the tower */
  if(request)
  {
    if (success)
      Packet_Command |= PACKET_ACK_MASK;  /* sets the ack bit if */
    else
      Packet_Command &= ~PACKET_ACK_MASK; /* clears the ack bit */

    Packet_Put(Packet_Command, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3); /* send the ack nak packet back */
  }
}


static void InitThread(void* pData)
{
  for (;;)
  {
    OS_DisableInterrupts();

    FTM0Channel0.channelNb = 0;  // Sets the parameters for the initial channel 0
    FTM0Channel0.delayCount =  CPU_MCGFF_CLK_HZ_CONFIG_0;
    FTM0Channel0.timerFunction = TIMER_FUNCTION_OUTPUT_COMPARE;
    FTM0Channel0.ioType.outputAction = TIMER_OUTPUT_HIGH;
       // FTM0Channel0.semaphore =  NULL;
        //FTM0Channel0.callbackFunction = (void* )&FTM0Callback;

    if (Packet_Init(BAUDRATE, CPU_BUS_CLK_HZ) && LEDs_Init() && Flash_Init() && FTM_Init() && FTM_Set(&FTM0Channel0) && PIT_Init(CPU_BUS_CLK_HZ) && RTC_Init() && Accel_Init(&Accelerometer))  /* check if packets initialised correctly */
    {
      LEDs_On(LED_ORANGE); // Initialising LED
    }

    if(Flash_AllocateVar((void*)&NvtowerNumber, sizeof(*NvtowerNumber)))
    {
      if (NvtowerNumber->l == 0xFFFF) //If towerNumber is not yet programmed, save it to flash// This points to address in the flash memory where the data will be, using the void pointer means that no the variable type can be changed without a cast
      {
        Flash_Write16((uint16_t*)NvtowerNumber, TowerNumber); // This function writes to the flash memory using the flash write function
      }
    }

    if(Flash_AllocateVar((void*)&NvtowerMode, sizeof(*NvtowerMode)))// same code as above except it is to set the tower mode to the flash memory// Again using teh void pointer means that no future type casting is needed.
    {
      Flash_Write16((uint16_t* )NvtowerMode, DefaultTowerMode);
    }

    PIT_Set(500, true); //Activates PIT, 500ns interrupt period
    Accel_SetMode(ACCEL_POLL);  // Set the initial Accelerometer mode

    OS_EnableInterrupts();

    StartupPacket(); // Send the startup packets

    OS_ThreadDelete(OS_PRIORITY_SELF); // Thread can't be accessed again as this is the initial thread
   }
}

/*! @brief Thread function for Packet
 *  If the PacketGet function is called then handle the packet
 */
static void PacketThread(void* pData)
{
  for (;;)
  {
    if (Packet_Get())
    {
      HandlePacket();
    }
  }
}

/*! @brief Thread function for PIT
 *  If Accelerometer is in Poll Mode, will read values and send to PC if any change
 */
static void PITThread(void* pData) //Use PIT for the one second timing
{
  for(;;)
    {
    OS_SemaphoreWait(PIT_Semaphore,0);
    static TAccelData lastAccelerometerValues; // an array of previous accelerometer values
    uint8_t axisCount; // stores the axis number

    if (Protocol_Mode == ACCEL_POLL) // if the accelerometer is in polling mode
    {

       Accel_ReadXYZ(accelerometerValues.bytes); // Read the accelerometer data
       LEDs_Toggle(LED_GREEN); // Toggle green LED

       // if the data doesnt equal the last value second every second
       if ((lastAccelerometerValues.bytes[0] != accelerometerValues.bytes[0]) ||
           (lastAccelerometerValues.bytes[1] != accelerometerValues.bytes[1]) ||
           (lastAccelerometerValues.bytes[2] != accelerometerValues.bytes[2]))
       {
  	 Packet_Put(CMD_ACCELVALUES,accelerometerValues.bytes[0],accelerometerValues.bytes[1],accelerometerValues.bytes[2]);
       }

       for (axisCount=0; axisCount < 3; axisCount++) // takes new data and places that as now the old data
       {
  	 lastAccelerometerValues.bytes[axisCount] = accelerometerValues.bytes[axisCount];
       }
     }
    }
  }

/*! @brief  User callback function for RTC
 *  Sends current programmed time to PC
 */
void RTCThread(void* pData)
{
  for(;;)
  {
    OS_SemaphoreWait(UpdateClock_Semaphore,0);
    uint8_t hours,      /*!< stores RTC hours   */
          minutes,    /*!< stores RTC minutes */
          seconds;    /*!< stores RTC seconds */

    RTC_Get(&hours,&minutes,&seconds);    // Each second it will get the current time
    Packet_Put(CMD_SETTIME,hours,minutes,seconds); // update the pc with a packet
    LEDs_Toggle(LED_YELLOW);      // Toggle yellow LED
  }
}


/*! @brief User callback function for FTM0
 *
 *  @note Assumes FTM0 interrupt has occurred
 */
void FTMThread(void* pData)
{
  for(;;)
  {
    OS_SemaphoreWait(FTM_Semaphore,0);
    FTM0Channel0.ioType.inputDetection = TIMER_OUTPUT_DISCONNECT; // stops the interupt of the timer
    FTM_StartTimer(&FTM0Channel0); // Updates the timer setttings
    LEDs_Off(LED_BLUE); // Turns of the blue Led
  }
}

void ReadCompleteThread(void* pData)
{
  // Sends the accelerometer data at 1.56Ghz
  for(;;)
  {
    OS_SemaphoreWait(ReadComplete_Semaphore,0);
    Packet_Put(CMD_ACCELVALUES,accelerometerValues.bytes[0],accelerometerValues.bytes[1],accelerometerValues.bytes[2]);
  }
}


/*! @brief User callback function for accelerometer when new data is available
 *
 *  @param arg Pointer to the user argument to use with the user callback function
 *  @note Assumes dataready interrupt has occured
 */
void DataReadyThread(void* pData)
{
  for(;;)
  {
    OS_SemaphoreWait(Data_Ready_Semaphore,0);
    Accel_ReadXYZ(accelerometerValues.bytes); // Collect the accelerometer data
    LEDs_Toggle(LED_GREEN); // Toggle the green LED
  }
}


/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error;

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */

  OS_Init(CPU_CORE_CLK_HZ, false);

  error = OS_ThreadCreate(InitThread, NULL, &InitThreadStack[THREAD_STACK_SIZE-1], 0); // Assigns the priority of each of the threads.

  // The UART

  error = OS_ThreadCreate(PITThread, NULL, &PITThreadStack[THREAD_STACK_SIZE-1], 3);

  error = OS_ThreadCreate(RTCThread, NULL, &RTCThreadStack[THREAD_STACK_SIZE-1], 4);

  error = OS_ThreadCreate(DataReadyThread, NULL, &DataReadyThreadStack[THREAD_STACK_SIZE-1], 5);

  error = OS_ThreadCreate(ReadCompleteThread, NULL, &ReadCompleteThreadStack[THREAD_STACK_SIZE-1], 6);

  error = OS_ThreadCreate(FTMThread, NULL, &FTMThreadStack[THREAD_STACK_SIZE-1], 7);

  error = OS_ThreadCreate(PacketThread, NULL, &PacketThreadStack[THREAD_STACK_SIZE-1], 8);


  OS_Start();

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/

}/*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/


/* END main */
/*!
** @}
*/
/*/
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/

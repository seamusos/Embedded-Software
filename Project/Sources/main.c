/* ###################################################################
 **     Filename    : main.c
 **     Project     : Lab6
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
 ** @version 6.0
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

// Simple OS
#include "OS.h"

// Analog functions
#include "analog.h"

#include "PIT.h"
#include "utilities.h"
#include "lookup.h"

// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100
#define NB_ANALOG_CHANNELS 3
#define SAMPLE_COUNT 16

static const int16_t ANALOG_5V = 16384;
static const int16_t ANALOG_0V = 0;

static const uint8_t TIMER_SIGNAL_CHANNEL = 1;
static const uint8_t TRIP_SIGNAL_CHANNEL = 2;



// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PITThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

extern OS_ECB *PIT_Semaphore;

static uint16_t PhaseFrequency;

typedef enum TIMER_STATUS
{
  TIMER_OFF,
  TIMER_ON,
}TIMER_STATUS;

typedef enum
{
  INVERSE,
  VERYINVERSE,
  EXTREMELYINVERSE,
}RELAY_TRAIT;


typedef enum
{
  NORMAL,
  WAITING_TRIP,
  CHANNEL_TRIPPED,
}CHANNEL_STATUS;

typedef enum
{
  TRIPPED,
  OFF,
}TRIP_STATUS;

typedef enum
{
  NoFault,
  OnePhase,
  TwoPhase,
  ThreePhase,
}FAULT_TYPE;

static TRIP_STATUS TripSignalStatus = OFF;
static FAULT_TYPE FaultType = NoFault;
static RELAY_TRAIT RelayTrait = INVERSE;
static TIMER_STATUS TimerStatus = TIMER_OFF;


// ----------------------------------------
// Thread priorities
// 0 = highest priority
// ----------------------------------------
const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {1, 2, 3};

/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
  double sampleReadings[SAMPLE_COUNT];
  uint8_t sampleCount;
  double currentRMS;
  double voltageRMS;

  double t0;
  double t1;
  uint8_t zeroCrossingCount;
  uint8_t offsetCount;

  CHANNEL_STATUS channelStatus;
  uint32_t previousTripTime;
  uint32_t tripTime;


} TAnalogThreadData;

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphore = NULL,
    .channelNb = 0,
    .sampleCount = 0,

    .t0 = 0,
    .t1 = 0,
    .zeroCrossingCount = 0
  },
  {
    .semaphore = NULL,
    .channelNb = 1,
    .sampleCount = 0,

    .t0 = 0,
    .t1 = 0,
    .zeroCrossingCount = 0
  },
  {
    .semaphore = NULL,
    .channelNb = 2,
    .sampleCount = 0,

    .t0 = 0,
    .t1 = 0,
    .zeroCrossingCount = 0
  }
};

/*! @brief Timer Signal
 *  Calculates Inverse timing and triggers Timer
 *
 *  @param TAnalogThreadData* pData pointer to data struct
 */
void TimerSignal(TAnalogThreadData* analogData)
{
  //Calculate Timer
  uint32_t tripTime = TimingLookup(analogData->currentRMS, RelayTrait);
  uint32_t timingDifference;
  PIT_Set(1000,TRUE,1); //Start PIT for 1 microsecond period

  //if timer hasn't started yet 
  if(TimerStatus == TIMER_OFF)
  {
//    (void)Analog_Put(1, 16384);
    TimerStatus = TIMER_ON;
    analogData->previousTripTime = tripTime;
    analogData->channelStatus = WAITING_TRIP; 
  }
  else if (TimerStatus == TIMER_ON)
  {
    if(tripTime > analogData->previousTripTime)
    {
      //Get difference then Set pit without Reset
      analogData->previousTripTime = (tripTime - analogData->previousTripTime);
    }
  }
}

/*! @brief FrequencyInterpolation
 *  Calculates Input wave frequency using zero crossings
 *
 *  @param AnalogThreadData structure of data from ADC
 *  @param sampleCount - index for the current storage array count
 */
void FrequencyInterpolation(TAnalogThreadData* AnalogThreadData, uint8_t sampleCount)
{
  if(AnalogThreadData->channelNb == 0)
  {
    float period = 1250000;
    float samplePeriod;
    double Frequency;
    if(sampleCount > 0)
    {

      if (AnalogThreadData->sampleReadings[sampleCount] > 0 && AnalogThreadData->sampleReadings[sampleCount - 1])
      {
        if (AnalogThreadData->zeroCrossingCount== 0)
        {
          AnalogThreadData->t0 = ((-AnalogThreadData->sampleReadings[sampleCount - 1]) / (AnalogThreadData->sampleReadings[sampleCount] - AnalogThreadData->sampleReadings[sampleCount - 1]));
          //Flag as first crossing
          AnalogThreadData->zeroCrossingCount = 1;
        }
        else if (AnalogThreadData->zeroCrossingCount == 1)
        {
          AnalogThreadData->t1 = (double)((-AnalogThreadData->sampleReadings[sampleCount - 1]) / (AnalogThreadData->sampleReadings[sampleCount] - AnalogThreadData->sampleReadings[sampleCount - 1]));

    
          period = (AnalogThreadData->offsetCount - AnalogThreadData->t0 + AnalogThreadData->t1) * ((float) period / 1000000000);
          AnalogThreadData->zeroCrossingCount = 0; //Reset Zero Crossing
//          Frequency = (double )1000000000 / period;

          samplePeriod = period / 16;
        }
      }
    }
  }
  AnalogThreadData->offsetCount++;
}

/*! @brief Initialises modules.
 *
 */
void InitModulesThread(void* pData)
{
  // Analog
  (void)Analog_Init(CPU_BUS_CLK_HZ);
  PIT_Init(CPU_BUS_CLK_HZ);

  // Generate the global analog semaphores
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    AnalogThreadData[analogNb].semaphore = OS_SemaphoreCreate(0);

  // use Pit to sample 16 times a 50hz sine wave
  PIT_Set(1250000, true, 0);


  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

/*! @brief Thread function for PIT
 *  Triggers 16 times a input cycle
 */
void PITSampleThread(void* pData) //Use PIT for the one second timing
{
  for(;;)
  {
    OS_SemaphoreWait(PIT_Semaphore,0);

    for (int i = 0; i < NB_ANALOG_CHANNELS; i++)
    {
      (void)OS_SemaphoreSignal(AnalogThreadData[i].semaphore);
    }
  }
}

/*! @brief Samples a value on an ADC channel and sends it to the corresponding DAC channel.
 *
 */
void AnalogThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

  for (;;)
  {
    int16_t analogInputValue;

    (void)OS_SemaphoreWait(analogData->semaphore, 0);
    // Get analog sample
    Analog_Get(analogData->channelNb, &analogInputValue);
    analogData->sampleReadings[analogData->sampleCount] = (double) analogInputValue;


    FrequencyInterpolation(analogData, analogData->sampleCount);


    analogData->sampleCount++;

    FrequencyInterpolation(analogData, analogData->sampleCount);

    if(analogData->sampleCount >= SAMPLE_COUNT)
    {
      analogData->voltageRMS = FloatingRMS(analogData->sampleReadings) / 3276.8;
      analogData->sampleCount = 0;
      analogData->currentRMS = analogData->voltageRMS / 0.35;
      if (analogData->currentRMS >= 1.03)
      {
        //Execute Timing Signal
        TimerSignal(analogData);
      }
      
    }

  }
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error;

  // Initialise low-level clocks etc using Processor Expert code
  PE_low_level_init();

  // Initialize the RTOS
  OS_Init(CPU_CORE_CLK_HZ, true);

  // Create module initialisation thread
  error = OS_ThreadCreate(InitModulesThread,
                          NULL,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
                          0); // Highest priority

  // Create threads for 2 analog loopback channels
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(AnalogThread,
                            &AnalogThreadData[threadNb],
                            &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            ANALOG_THREAD_PRIORITIES[threadNb]);
  }
  error = OS_ThreadCreate(PITSampleThread,
                          NULL,
                          &PITThreadStack[THREAD_STACK_SIZE - 1],
                          4);

  // Start multithreading - never returns!
  OS_Start();
}

/*!
 ** @}
 */

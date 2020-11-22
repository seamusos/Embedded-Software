/*! @file accel.c
 *
 *  @brief HAL for the accelerometer.
 *
 *  This contains the functions for interfacing to the MMA8451Q accelerometer.
 *
 *  @author Joel Goodwin & Seamus 0'Sullivan
 *  @date 2019-05-07
 */

/*!
 *  @addtogroup Accel_module Accelerometer documentation
 *  @{
 */

// Accelerometer functions
#include "accel.h"

// Inter-Integrated Circuit
#include "I2C.h"

// Median filter
#include "median.h"

// K70 module registers
#include "MK70F12.h"

// CPU and PE_types are needed for critical section variables and the defintion of NULL pointer
#include "CPU.h"
#include "PE_types.h"
#include "OS.h"

// Accelerometer registers
#define ADDRESS_OUT_X_MSB 0x01

#define ADDRESS_INT_SOURCE 0x0C

static union
{
  uint8_t byte;			/*!< The INT_SOURCE bits accessed as a byte. */
  struct
  {
    uint8_t SRC_DRDY   : 1;	/*!< Data ready interrupt status. */
    uint8_t              : 1;
    uint8_t SRC_FF_MT  : 1;	/*!< Freefall/motion interrupt status. */
    uint8_t SRC_PULSE  : 1;	/*!< Pulse detection interrupt status. */
    uint8_t SRC_LNDPRT : 1;	/*!< Orientation interrupt status. */
    uint8_t SRC_TRANS  : 1;	/*!< Transient interrupt status. */
    uint8_t SRC_FIFO   : 1;	/*!< FIFO interrupt status. */
    uint8_t SRC_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt status. */
  } bits;			/*!< The INT_SOURCE bits accessed individually. */
} INT_SOURCE_Union;

#define INT_SOURCE     		INT_SOURCE_Union.byte
#define INT_SOURCE_SRC_DRDY	INT_SOURCE_Union.bits.SRC_DRDY
#define INT_SOURCE_SRC_FF_MT	CTRL_REG4_Union.bits.SRC_FF_MT
#define INT_SOURCE_SRC_PULSE	CTRL_REG4_Union.bits.SRC_PULSE
#define INT_SOURCE_SRC_LNDPRT	CTRL_REG4_Union.bits.SRC_LNDPRT
#define INT_SOURCE_SRC_TRANS	CTRL_REG4_Union.bits.SRC_TRANS
#define INT_SOURCE_SRC_FIFO	CTRL_REG4_Union.bits.SRC_FIFO
#define INT_SOURCE_SRC_ASLP	CTRL_REG4_Union.bits.SRC_ASLP

#define ADDRESS_CTRL_REG1 0x2A

typedef enum
{
  DATE_RATE_800_HZ,
  DATE_RATE_400_HZ,
  DATE_RATE_200_HZ,
  DATE_RATE_100_HZ,
  DATE_RATE_50_HZ,
  DATE_RATE_12_5_HZ,
  DATE_RATE_6_25_HZ,
  DATE_RATE_1_56_HZ
} TOutputDataRate;

typedef enum
{
  SLEEP_MODE_RATE_50_HZ,
  SLEEP_MODE_RATE_12_5_HZ,
  SLEEP_MODE_RATE_6_25_HZ,
  SLEEP_MODE_RATE_1_56_HZ
} TSLEEPModeRate;

static union
{
  uint8_t byte;			/*!< The CTRL_REG1 bits accessed as a byte. */
  struct
  {
    uint8_t ACTIVE    : 1;	/*!< Mode selection. */
    uint8_t F_READ    : 1;	/*!< Fast read mode. */
    uint8_t LNOISE    : 1;	/*!< Reduced noise mode. */
    uint8_t DR        : 3;	/*!< Data rate selection. */
    uint8_t ASLP_RATE : 2;	/*!< Auto-WAKE sample frequency. */
  } bits;			/*!< The CTRL_REG1 bits accessed individually. */
} CTRL_REG1_Union;

#define CTRL_REG1     		  CTRL_REG1_Union.byte
#define CTRL_REG1_ACTIVE	  CTRL_REG1_Union.bits.ACTIVE
#define CTRL_REG1_F_READ  	  CTRL_REG1_Union.bits.F_READ
#define CTRL_REG1_LNOISE  	  CTRL_REG1_Union.bits.LNOISE
#define CTRL_REG1_DR	    	  CTRL_REG1_Union.bits.DR
#define CTRL_REG1_ASLP_RATE	  CTRL_REG1_Union.bits.ASLP_RATE

#define ADDRESS_CTRL_REG2 0x2B

#define ADDRESS_CTRL_REG3 0x2C

static union
{
  uint8_t byte;			/*!< The CTRL_REG3 bits accessed as a byte. */
  struct
  {
    uint8_t PP_OD       : 1;	/*!< Push-pull/open drain selection. */
    uint8_t IPOL        : 1;	/*!< Interrupt polarity. */
    uint8_t WAKE_FF_MT  : 1;	/*!< Freefall/motion function in SLEEP mode. */
    uint8_t WAKE_PULSE  : 1;	/*!< Pulse function in SLEEP mode. */
    uint8_t WAKE_LNDPRT : 1;	/*!< Orientation function in SLEEP mode. */
    uint8_t WAKE_TRANS  : 1;	/*!< Transient function in SLEEP mode. */
    uint8_t FIFO_GATE   : 1;	/*!< FIFO gate bypass. */
  } bits;			/*!< The CTRL_REG3 bits accessed individually. */
} CTRL_REG3_Union;

#define CTRL_REG3     		    CTRL_REG3_Union.byte
#define CTRL_REG3_PP_OD		    CTRL_REG3_Union.bits.PP_OD
#define CTRL_REG3_IPOL		    CTRL_REG3_Union.bits.IPOL
#define CTRL_REG3_WAKE_FF_MT	CTRL_REG3_Union.bits.WAKE_FF_MT
#define CTRL_REG3_WAKE_PULSE	CTRL_REG3_Union.bits.WAKE_PULSE
#define CTRL_REG3_WAKE_LNDPRT	CTRL_REG3_Union.bits.WAKE_LNDPRT
#define CTRL_REG3_WAKE_TRANS	CTRL_REG3_Union.bits.WAKE_TRANS
#define CTRL_REG3_FIFO_GATE	  CTRL_REG3_Union.bits.FIFO_GATE

#define ADDRESS_CTRL_REG4 0x2D

static union
{
  uint8_t byte;			/*!< The CTRL_REG4 bits accessed as a byte. */
  struct
  {
    uint8_t INT_EN_DRDY   : 1;	/*!< Data ready interrupt enable. */
    uint8_t               : 1;
    uint8_t INT_EN_FF_MT  : 1;	/*!< Freefall/motion interrupt enable. */
    uint8_t INT_EN_PULSE  : 1;	/*!< Pulse detection interrupt enable. */
    uint8_t INT_EN_LNDPRT : 1;	/*!< Orientation interrupt enable. */
    uint8_t INT_EN_TRANS  : 1;	/*!< Transient interrupt enable. */
    uint8_t INT_EN_FIFO   : 1;	/*!< FIFO interrupt enable. */
    uint8_t INT_EN_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt enable. */
  } bits;			/*!< The CTRL_REG4 bits accessed individually. */
} CTRL_REG4_Union;

#define CTRL_REG4               CTRL_REG4_Union.byte
#define CTRL_REG4_INT_EN_DRDY	CTRL_REG4_Union.bits.INT_EN_DRDY
#define CTRL_REG4_INT_EN_FF_MT	CTRL_REG4_Union.bits.INT_EN_FF_MT
#define CTRL_REG4_INT_EN_PULSE	CTRL_REG4_Union.bits.INT_EN_PULSE
#define CTRL_REG4_INT_EN_LNDPRT	CTRL_REG4_Union.bits.INT_EN_LNDPRT
#define CTRL_REG4_INT_EN_TRANS	CTRL_REG4_Union.bits.INT_EN_TRANS
#define CTRL_REG4_INT_EN_FIFO	CTRL_REG4_Union.bits.INT_EN_FIFO
#define CTRL_REG4_INT_EN_ASLP	CTRL_REG4_Union.bits.INT_EN_ASLP

#define ADDRESS_CTRL_REG5 0x2E

static union
{
  uint8_t byte;			/*!< The CTRL_REG5 bits accessed as a byte. */
  struct
  {
    uint8_t INT_CFG_DRDY   : 1;	/*!< Data ready interrupt enable. */
    uint8_t                : 1;
    uint8_t INT_CFG_FF_MT  : 1;	/*!< Freefall/motion interrupt enable. */
    uint8_t INT_CFG_PULSE  : 1;	/*!< Pulse detection interrupt enable. */
    uint8_t INT_CFG_LNDPRT : 1;	/*!< Orientation interrupt enable. */
    uint8_t INT_CFG_TRANS  : 1;	/*!< Transient interrupt enable. */
    uint8_t INT_CFG_FIFO   : 1;	/*!< FIFO interrupt enable. */
    uint8_t INT_CFG_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt enable. */
  } bits;			/*!< The CTRL_REG5 bits accessed individually. */
} CTRL_REG5_Union;

#define CTRL_REG5     		      	CTRL_REG5_Union.byte
#define CTRL_REG5_INT_CFG_DRDY		CTRL_REG5_Union.bits.INT_CFG_DRDY
#define CTRL_REG5_INT_CFG_FF_MT		CTRL_REG5_Union.bits.INT_CFG_FF_MT
#define CTRL_REG5_INT_CFG_PULSE		CTRL_REG5_Union.bits.INT_CFG_PULSE
#define CTRL_REG5_INT_CFG_LNDPRT	CTRL_REG5_Union.bits.INT_CFG_LNDPRT
#define CTRL_REG5_INT_CFG_TRANS		CTRL_REG5_Union.bits.INT_CFG_TRANS
#define CTRL_REG5_INT_CFG_FIFO		CTRL_REG5_Union.bits.INT_CFG_FIFO
#define CTRL_REG5_INT_CFG_ASLP		CTRL_REG5_Union.bits.INT_CFG_ASLP

#define ACCEL_ADDRESS 0x1D
#define I2C_BAUD 100000


extern TAccelMode Protocol_Mode; /*!< Global variable to store current protocol mode */
extern OS_ECB *Data_Ready_Semaphore;

typedef struct
{
  uint8_t xData[2]; /*!< 3 samples of x axis */
  uint8_t yData[2]; /*!< 3 samples of y axis */
  uint8_t zData[2]; /*!< 3 samples of z axis */
}TXYZData;

static TXYZData previousData;

bool Accel_Init(const TAccelSetup* const accelSetup)
{
  //Initalise I2C
  TI2CModule i2cInitModule;
  i2cInitModule.baudRate = I2C_BAUD;
  i2cInitModule.primarySlaveAddress = ACCEL_ADDRESS;

  if (!I2C_Init(&i2cInitModule, i2cInitModule.baudRate))
  {
    return false; //I2C Failed to Initalise
  }
  I2C_SelectSlaveDevice(ACCEL_ADDRESS);

  //Initalise Accelerometer
  CTRL_REG1_F_READ = 1;             //Set to 8Bit Mode
  CTRL_REG1_DR = DATE_RATE_1_56_HZ; //Set to 0b111
  CTRL_REG1_ACTIVE = 1;             //Set Active Mode
  CTRL_REG5_INT_CFG_DRDY = 1;       //Set Data Ready Interrupt to Pin INT1
  CTRL_REG3_IPOL = 1;               //Set interrupt polarity to Active High

  I2C_Write(ADDRESS_CTRL_REG1, CTRL_REG1); //Write to Control Register 1
  I2C_Write(ADDRESS_CTRL_REG3, CTRL_REG3); //Write to Control Register 3
  I2C_Write(ADDRESS_CTRL_REG5, CTRL_REG5); //Write to Control Register 5

  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; // Enable clock gate for PortB
  PORTB_PCR4 = PORT_PCR_MUX(1) | PORT_PCR_ISF_MASK;

  //Configure NVIC
  NVICICPR2 |= NVIC_ICPR_CLRPEND(1 << 24); // Clear pending interrupts
  NVICISER2 |= NVIC_ISER_SETENA(1 << 24); // Enable interrupts

  Data_Ready_Semaphore = OS_SemaphoreCreate(0);

  return true;
}





void Accel_ReadXYZ(uint8_t data[3])
{
  uint8_t xyzData[3]; // array to store the median values

  if (Protocol_Mode == ACCEL_POLL)
  {
    I2C_PollRead(ADDRESS_OUT_X_MSB, data, 3); // Read accelerometer data using polling method
  }
  else
  {
    I2C_IntRead(ADDRESS_OUT_X_MSB, data, 3); // Read accelerometer data using interrupt method
  }

  // Find median of the 3 samples
  xyzData[0]= Median_Filter3(data[0],previousData.xData[0], previousData.xData[1]); // X after passing through median
  xyzData[1]= Median_Filter3(data[1], previousData.yData[0], previousData.yData[1]); // Y after passing through median
  xyzData[2]= Median_Filter3(data[2], previousData.zData[0], previousData.zData[1]); // Z after passing through median

  //Copy new data to old
  previousData.xData[1] = previousData.xData[0];
  previousData.xData[0] = data[0];

  previousData.yData[1] = previousData.yData[0];
  previousData.yData[0] = data[1];

  previousData.zData[1] = previousData.zData[0];
  previousData.zData[0] = data[2];

  data = xyzData;
}


void Accel_SetMode(const TAccelMode mode)
{
  EnterCritical();

  CTRL_REG4_INT_EN_DRDY = mode; // Interrupt or polling
  CTRL_REG1_ACTIVE = 0; 	// Deactivate accelerometer

  I2C_Write(ADDRESS_CTRL_REG1,CTRL_REG1); 	// Deactivate accelerometer

  I2C_Write(ADDRESS_CTRL_REG4,CTRL_REG4); 	// Data ready interrupt enable
  CTRL_REG1_ACTIVE = 1; 			// Activate accelerometer
  I2C_Write(ADDRESS_CTRL_REG1,CTRL_REG1); 	// 8-bits data select, 1.56Hz select, and activate

  if (mode == ACCEL_INT)
    PORTB_PCR4 |= PORT_PCR_IRQC(0x09); // Trigger interrupt on rising edge
  else
    PORTB_PCR4 &= ~PORT_PCR_IRQC_MASK; // Disable PORTB interrupt

  ExitCritical();
}


void __attribute__ ((interrupt)) AccelDataReady_ISR(void)
{
  OS_ISREnter();

  PORTB_PCR4 |= PORT_PCR_ISF_MASK; //Clear PORTB ISF Flag
  OS_SemaphoreSignal(Data_Ready_Semaphore);

  OS_ISRExit();
}


/*!
 * @}
*/

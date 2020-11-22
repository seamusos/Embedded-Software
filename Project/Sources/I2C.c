/*! @file
 *
 *  @brief I/O routines for the K70 I2C interface.
 *
 *  This contains the functions for operating the I2C (inter-integrated circuit) module.
 *
 *  @author Joel Goodwin & Seamus O'Sullivan
 *  @date 2019-05-15
 */

/*!
 *  @addtogroup i2c_module I2C module documentation
 *  @{
 */

// Included header files
#include "Cpu.h"
#include "I2C.h"
#include "MK70F12.h"
#include "PE_Types.h"
#include <stdlib.h>
#include "OS.h"

// Enum that allows to change the device address from read to write
typedef enum
{
  WRITE = 0,
  READ = 1
} RW;

// Function Prototypes
static void Start(void);
static void Wait(void);
static void Stop(void);

static const uint16_t SCL_MAX = 0x3F;  // max constant values for size of scl and mult matrix
static const uint8_t MULT_MAX = 3;

static char SlaveAddress;         // Private Global to Store Address of Slave
static uint8_t IntReadBytes[3];

static uint8_t Position;   // Position during the interupt reading
static uint8_t* DataPtr;          // Pointer to store the bytes
static uint8_t NumBytes;          // the number of bytes to read
static uint8_t DataCounter = 0;    // This counts the number of reads and writes

//static void* ReadCompleteUserArgumentsGlobal;      /*!< Private global pointer to the user arguments to use with the user callback function */
//static void (*ReadCompleteCallbackGlobal)(void *); /*!< Private global pointer to data ready user callback function */

static OS_ECB *I2CWriteSemaphore;
OS_ECB *ReadComplete_Semaphore;



static void Reset()
{

  GPIOE_PDDR &= ~(1 << 18);
  GPIOE_PDDR &= ~(1 << 19);  // Changes the Pins to Inputs


  GPIOE_PCOR = (1 << 19); // Makes the SCL Line become an output of 0


  PORTE_PCR18 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;  // Changes the configuration of the pins to have internal pull up
  PORTE_PCR19 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;


  while ((GPIOE_PDIR & (1 << 18)) == 0) // If the SDA line is low, we clock the SCL line to free it
  {
    // Clear SCL line to a 0
    GPIOE_PDDR |= (1 << 19);   // This clear the SCL Line to 0
    //Delay(100000);
    for (uint16_t i = 0; i < 1000; i++); // for loop which acts as a delay
    GPIOE_PDDR &= ~(1 << 19);  // Configure SCL line as an input
    //Delay(100000);
    for (uint16_t i = 0; i < 1000; i++);  // for loop which acts as a delay
  }


  PORTE_PCR18 = PORT_PCR_MUX(0x04) | PORT_PCR_ODE_MASK; // Pins returned to have normal I2C functionality
  PORTE_PCR19 = PORT_PCR_MUX(0x04) | PORT_PCR_ODE_MASK;
}


bool I2C_Init(const TI2CModule* const aI2CModule, const uint32_t moduleClk)
{
  //ReadCompleteUserArgumentsGlobal = aI2CModule->readCompleteCallbackArguments; // userArguments made globally(private) accessible
  //ReadCompleteCallbackGlobal = aI2CModule->readCompleteCallbackFunction; // userFunction made globally(private) accessible

  // Multiplier and Scl Array Values from Manual
  uint8_t mult[] = {1,2,4};
  uint16_t scl[] = {20,22,24,26,28,30,34,40,28,32,36,40,44,48,56,68,
                    48,56,64,72,80,88,104,128,80,96,112,128,144,160,192,240,
  		    160,192,224,256,288,320,384,480,320,384,448,512,576,640,
  		    768,960,640,768,896,1024,1152,1280,1536,1920,1280,1536,
  		    1792,2048,2304,2560,3072,3840};

  uint8_t sclDivCount;             // this is a countrer for going through the SCL array
  uint8_t mulCount;                   // this is a countrer for going through the multplier array
  uint8_t multiplier;               // current value of the multiplier
  uint8_t sclDivider;                 // current value of the scl
  uint32_t baudRateError = 0xFFFFFFFF;   // Sets the Initial Maximum Baud Rate Error
  uint32_t newBaudRate;          /*!< Baud rate of current calculation */

  SIM_SCGC4 |= SIM_SCGC4_IIC0_MASK ; // Enable clock gate for I2C module

  PORTE_PCR18 |= PORT_PCR_MUX(4) | PORT_PCR_ODE_MASK; // Configures the Pins to be the SDA and the SDL
  PORTE_PCR19 |= PORT_PCR_MUX(4) | PORT_PCR_ODE_MASK; //  Open drain

  // Loop to find the baud rate
  for (sclDivCount=0; sclDivCount < SCL_MAX; sclDivCount++)  // for loop to cycle through the SCL values
  {
    for (mulCount=0; mulCount < MULT_MAX; mulCount++) // for loop to cycle through the multplier values
    {
      if (abs(newBaudRate - aI2CModule->baudRate) < baudRateError) // Check if baud rate is closer to required baud rate, than before
      {
	newBaudRate = moduleClk / (mult[mulCount] * scl[sclDivCount]); // Calculate baud rate
        baudRateError = abs(newBaudRate - aI2CModule->baudRate); // Subtracts the new Baud Rate from Required
        multiplier = mulCount; // Increases the multiplier value
        sclDivider = sclDivCount; // Increases the SCL Value
      }
    }
  }

  I2C0_F = I2C_F_MULT(mult[multiplier]) | I2C_F_ICR(sclDivider); // Formula to set the Baud Rate

  I2C0_C1 |= I2C_C1_IICEN_MASK; // I2C enable

  NVICICPR0 = NVIC_ICPR_CLRPEND(1 << 24); // Clear any pending interrupts and enable
  NVICISER0 = NVIC_ISER_SETENA(1 << 24); //

  if ((I2C0_S & I2C_S_BUSY_MASK) == I2C_S_BUSY_MASK) // if the bus is bus carry out the reset function
  {
    Reset();
  }

  I2CWriteSemaphore = OS_SemaphoreCreate(1); // This initialises the semaphores to 1

  ReadComplete_Semaphore = OS_SemaphoreCreate(0); // This initialises the semaphore to 0

  return true;
}


void I2C_SelectSlaveDevice(const uint8_t slaveAddress)
{
  SlaveAddress = slaveAddress; // Store the slave address globally(private)
}

/*! @brief Select master mode and transmit mode to start communication
 */
static void Start(void)
{
  I2C0_C1 |= I2C_C1_MST_MASK; // Enables the Master Mode
  I2C0_C1 |= I2C_C1_TX_MASK;  // Enables the Transfer Mode
}

/*! @brief Wait until interrupt flag is set and clear it.
 */
static void Wait(void)
{
  while (!((I2C0_S & I2C_S_IICIF_MASK) == I2C_S_IICIF_MASK)){}   // This waits for the interrupt flag to set
  I2C0_S |= I2C_S_IICIF_MASK;  // Once the flag is set than clear it
}

/*! @brief Stop communication by clearing master mode
 */
static void Stop(void)
{
  I2C0_C1 &= ~I2C_C1_MST_MASK;  // the clears the master mode and sends stop signal
}

void I2C_Write(const uint8_t registerAddress, const uint8_t data)
{
  OS_SemaphoreWait(I2CWriteSemaphore,0);  // Wait for the previous write to be complete

  while ((I2C0_S & I2C_S_BUSY_MASK) == I2C_S_BUSY_MASK){} // Waits until the bus is idle and ready to use

  Start(); // START signal, master-transmit mode

  I2C0_D = (SlaveAddress << 1)  | WRITE; // Send slave address with write bit
  Wait(); // Wait for ACK

  I2C0_D = registerAddress; // Send slave register address
  Wait(); // calls the wait function to wait for the ack

  I2C0_D = data; // Writes the data
  Wait(); // calls the wait function to wait for the ack

  Stop(); // Calls the Stop Funciton

  OS_SemaphoreSignal(I2CWriteSemaphore); // This signal thats the completion of the writing to slave device
}


void I2C_PollRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes)
{

  uint8_t dataCount; // counts the data bytes that have been read

  while ((I2C0_S & I2C_S_BUSY_MASK) == I2C_S_BUSY_MASK) // Waits until the bus is idle and ready to use
  {}

  Start(); // Calls the start function

  I2C0_D = (SlaveAddress << 1) | WRITE; // Send the slave address along with the write bit
  Wait(); // Calls wait function to wait for the ack

  I2C0_D = registerAddress; // Send slave register address
  Wait(); // Wait for ACK

  I2C0_C1 |= I2C_C1_RSTA_MASK; // Repeat start

  I2C0_D = (SlaveAddress << 1) | READ;// this sends the slave address with the read bit
  Wait(); // Wait for ACK

  I2C0_C1 &= ~I2C_C1_TX_MASK; // Receive mode
  I2C0_C1 &= ~I2C_C1_TXAK_MASK; // Turn on ACK from master

  data[0] = I2C0_D; // Dummy Read to begin the communication
  Wait(); // Wait for ACK

  for (dataCount =0; dataCount < nbBytes -1; dataCount ++)
  {
    data[dataCount] = I2C0_D; // Read all of the bytes until the second last byte
    Wait(); // Wait Funciton
  }

  I2C0_C1 |= I2C_C1_TXAK_MASK; // Gets a NACK from Master

  data[dataCount++] = I2C0_D; // Reads the second last byte
  Wait();

  Stop(); // Stop Function

  data[dataCount++] = I2C0_D; // Now reads the last byte
}


void I2C_IntRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes)
{
  while ((I2C0_S & I2C_S_BUSY_MASK) == I2C_S_BUSY_MASK) // Waits until the bus is idle and ready to use
  {}
  I2C0_S |= I2C_S_IICIF_MASK; // Clears the interrupt flag

  I2C0_C1 |= I2C_C1_IICIE_MASK; // Enables the I2C iNTERUPT
  DataPtr = data; // Array to store values into, is made globally accessible
  NumBytes = nbBytes; // Number of bytes to read
  IntReadBytes[0] = (SlaveAddress << 1) | WRITE; // Send the slave address with the write bit
  IntReadBytes[1] = registerAddress; // Register Address
  IntReadBytes[2] = (SlaveAddress << 1) | READ;// Send the slave address with the read bit
  Position = 0; // Sets the position to 0

  Start(); // Start funciton
  I2C0_D = IntReadBytes[0]; // Sends teh slave address with write bit
}


void __attribute__ ((interrupt)) I2C_ISR(void)
{
  OS_ISREnter(); // Start of the Routine

  I2C0_S |= I2C_S_IICIF_MASK; // This clears interrupt flag

  if (I2C0_S & I2C_S_BUSY_MASK) // Bus is busy
  {
    if (I2C0_C1 & I2C_C1_TX_MASK) // In transmit mode
    {
      if (!(I2C0_S & I2C_S_RXAK_MASK)) // Checks to see if an ACK has been recieved
      {
        Position++; // Increases the position in the position
        if (Position == 2)
        {
          I2C0_C1 |=  I2C_C1_RSTA_MASK; // Initiates signal again
          I2C0_D = IntReadBytes[Position]; // From last function sends the slave address with the read bit
        }
        else if (Position == 3)
        {
          I2C0_C1 &= ~I2C_C1_TX_MASK; // Turns on Recieve Mode
    	  I2C0_C1 &= ~I2C_C1_TXAK_MASK; // Get the ACK from Master
    	  DataPtr[DataCounter] = I2C0_D; // Reads the dummy
        }
    	else
      	  I2C0_D = IntReadBytes[Position]; // Sends the slave address\

        OS_ISRExit();
        return;
      }
      else // If no ack is received than it goes to the stop function
      {
        Stop(); // Stop signal
        OS_ISRExit();
    	return;
      }
    }

    else if (!(I2C0_C1 & I2C_C1_TX_MASK) && (DataCounter <NumBytes)) // If it is in recieve mode
    {
      if (DataCounter == (NumBytes -1)) // See if last byte need to be read
      {
    	Stop();
        DataPtr[DataCounter] = I2C0_D; // Read the last byte if it is available
    	DataCounter = 0; // Resets the counter
    	Position = 0; // Reset the position

    	I2C0_C1 &= ~I2C_C1_IICIE_MASK; // disable interrupts and clear the interrupt flag
        I2C0_S |= I2C_S_IICIF_MASK; //

        //(*ReadCompleteCallbackGlobal)(ReadCompleteUserArgumentsGlobal); // Callback Function
        OS_SemaphoreSignal(ReadComplete_Semaphore);
        OS_ISRExit();
        return;
      }
      else if (DataCounter == (NumBytes -2)) // If the second last byte to be read
        I2C0_C1 |= I2C_C1_TXAK_MASK; // get a nack from the master

        DataPtr[DataCounter] = I2C0_D; // Read the data from second last byte
        DataCounter++; // Increase the Data Counter
    }
  }

  OS_ISRExit(); // End of the Interupt
}


/*!
 * @}
*/

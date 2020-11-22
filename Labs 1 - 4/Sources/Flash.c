/*! @file
 *
 *  @brief Routines for erasing and writing to the Flash.
 *
 *  This contains the functions needed for accessing the internal Flash.
 *
 *  @author Seamus O'Sullivan & Joel Goodwin
 *  @date 2019-04-18
 */

/*!
 *  @addtogroup Flash_module Flash documentation
 *  @{
*/


  // new types
  #include "types.h"
  #include "PE_Types.h"
  #include "Flash.h"
  #include "IO_MAP.h"


#define CHECKER(x,n) ((x >> (n)) & 1) // It compares each bit with LSB
#define ACCERR_FPVIOL_ERROR (FTFE_FSTAT & 0x30) // Bits showing ACCER Error or FPVIOL Error


typedef struct
{

        uint8_t Command;  /*!< Stores FTFE command */
        uint8_t FlashAddress1; /*!< Stores starting address bits [23:16] */
        uint8_t FlashAddress2; /*!< Stores starting address bits [15:8]  */
        uint8_t FlashAddress3; /*!< Stores starting address bits [7:0]   */
        uint8_t DataByte0; /*!< Stores phrase bits [63:56] */
        uint8_t DataByte1; /*!< Stores phrase bits [55:48] */
        uint8_t DataByte2; /*!< Stores phrase bits [47:40] */
        uint8_t DataByte3; /*!< Stores phrase bits [39:32] */
        uint8_t DataByte4; /*!< Stores phrase bits [31:24] */
        uint8_t DataByte5; /*!< Stores phrase bits [23:16] */
        uint8_t DataByte6; /*!< Stores phrase bits [15:8]  */
        uint8_t DataByte7; /*!< Stores phrase bits [7:0]   */

}TFCCOB;

static bool AllocateVar(volatile void **data, const uint8_t end,const uint8_t size);
static bool LaunchCommand(TFCCOB* commonCommandObject);
static bool WritePhrase(const uint32_t address, const uint64_t phrase);
static bool EraseSector(const uint32_t address);

static TFCCOB Fccob;
static uint8_t Occupied = 0x00; // Occupied space in flash memory where each bit represents a byte in a phrase

/*! @brief Writes the 64 bit phrase and the 32 bit address to the structure
 *
 *  @return bool - TRUE if the Flash "data" sector was erased successfully
 *  @param address is the starting address of the data to be written at
 *  @param data is the 64 bit phrase
 */

static bool WritePhrase(const uint32_t address, const uint64_t data)
{

  Flash_Erase(); //Erase flash memory content before write

  Fccob.Command = FTFE_FCCOB0_CCOBn(0x07); // tHIS is the command that allows the command to program the phrase

  Fccob.FlashAddress1 = address >> 16; // 23-16 bits of the address
  Fccob.FlashAddress2 = address >> 8; // 15-8 bits of the address
  Fccob.FlashAddress3 = address; // 7-0 bits of the address

  Fccob.DataByte0 = data >> 56; // this stores the phrase seperating the bits like the structure above
  Fccob.DataByte1 = data >> 48;
  Fccob.DataByte2 = data >> 40;
  Fccob.DataByte3 = data >> 32;
  Fccob.DataByte4 = data >> 24;
  Fccob.DataByte5 = data >> 16;
  Fccob.DataByte6 = data >> 8;
  Fccob.DataByte7 = data;

  return (LaunchCommand(&Fccob));  // returns the launch command

}

/*! @brief Erases the entire Flash sector.
 *
 *  @return bool - TRUE if the Flash "data" sector was erased successfully.
 */
bool Flash_Erase(void)
{

  return EraseSector(FLASH_DATA_START); // Erase flash

}

/*! @brief Enables the Flash module.
 *
 *  @return bool - TRUE if the Flash was setup successfully.
 */
bool Flash_Init(void)
{

        return true;  // Do nothing
}



/*! @brief Allocates space for a non-volatile variable in the Flash memory.
 *
 *  @param variable is the address of a pointer to a variable that is to be allocated space in Flash memory.
 *         The pointer will be allocated to a relevant address:
 *         If the variable is a byte, then any address.
 *         If the variable is a half-word, then an even address.
 *         If the variable is a word, then an address divisible by 4.
 *         This allows the resulting variable to be used with the relevant Flash_Write function which assumes a certain memory address.
 *         e.g. a 16-bit variable will be on an even address
 *  @param size The size, in bytes, of the variable that is to be allocated space in the Flash memory. Valid values are 1, 2 and 4.
 *  @return bool - TRUE if the variable was allocated space in the Flash memory.
 */
bool Flash_AllocateVar(volatile void** variable, const uint8_t size)
{

  uint8_t state = 0; //Current position
  uint8_t free = 0; //Number of available positions
  uint8_t vacant = 0; //Empty positions found

  while(state<8) //Loop through sector locations to find free space
  {
    if(CHECKER(Occupied,state++) == 0) //Free position
    {
      free++; //this will increase the amount of available positions
      vacant++;// increase the amount of empty positions
    }
    else
    {
    vacant = 0; //no free positions
    continue; //Return to loop begining
    }

    if((free % size == 0) && (vacant == size)) //Enough available in a row spaces for data
    {
    if(AllocateVar(variable, state-1, size)) //Allocate variable into the found free spaces
    {
      break;
    }
    else{
      return false;
        }
    }
    }
   return true;
}

/*! @brief Allocates space for a non-volatile variable in Flash
 *
 *  @param variable is the address of a pointer to a variable that is to be allocated space in flash memory
 *  @param end is the position of the free starting location in the sector (position-1)
 *  @param size is the size, in bytes, of the variable that is to be allocated space in the flash memory (1, 2 or 4)
 *
 *  @return bool - TRUE if the variable was allocated space in the Flash memory.
 */

bool AllocateVar(volatile void **variable, const uint8_t end, const uint8_t size)
{

  switch (size)
  {
    case 1: //Data is 1 Byte size

    *(volatile uint8_t**) variable = &(_FB(FLASH_DATA_START+end)); //Write Byte into memory
    Occupied |= 0x1 << end; //Here we say that one Byte has been occupied
    break;

    case 2: //Data is 2 Bytes size

    *(volatile uint16_t**) variable = &(_FH(FLASH_DATA_START+(end-1))); //Write half-word into memory
    Occupied |= 0x3 << end-1; //Here we say that two Bytes has been occupied
    break;

    case 4: //Data is 4 Bytes size

    *(volatile uint32_t**) variable = &(_FW(FLASH_DATA_START+(end-3))); //Write word into memory
    Occupied |= 0xF << end-3; //Here we say that four Bytes has been occupied
    break;

    default: return false;
  }

  return true; //Data successfully allocated
}



/*! @brief Writes a 32-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 32-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 4-byte boundary or if there is a programming error.
 */
bool Flash_Write32(volatile uint32_t* const address, const uint32_t data)
{

  uint64union_t phrase; //Stores the hi and lo components of the phrase
  uint32_t flashAddress = (uint32_t)address; //Stores the starting address to store data

  if ((flashAddress/4) % 2 == 0) // Setting up a phrase
  {

    phrase.s.Lo = data; //Combine data in address+4 with current word
    phrase.s.Hi = _FW(flashAddress+4);

    return WritePhrase(flashAddress,phrase.l); // Return 64 bit phrase
  }
  else
  {
    phrase.s.Lo = _FW(flashAddress-4); //Combine data in address-4 with current word
    phrase.s.Hi = data;
    return WritePhrase(flashAddress-4,phrase.l); // Return 64 bit phrase
  }

}


/*! @brief Writes a 16-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 16-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 2-byte boundary or if there is a programming error.
 */
bool Flash_Write16(volatile uint16_t* const address, const uint16_t data)
{
  uint32union_t word; //Stores the hi and lo components of the word
  uint32_t flashAddress = (uint32_t)address; //Stores the starting address to store data

  if(flashAddress % 4 ==0)
  {
    word.s.Lo = data; //Combine data in address+2 with current half word
    word.s.Hi = _FH(flashAddress+2);
    return Flash_Write32(&(_FW(flashAddress)),word.l); // Return 32 bit word
  }
  else
  {
    word.s.Lo = _FH(flashAddress-2); //Combine data in address-2 with current half word
    word.s.Hi = data;
    return Flash_Write32(&(_FW(flashAddress-2)),word.l); // Return 32 bit word
  }

}


/*! @brief Writes an 8-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 8-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if there is a programming error.
 */
bool Flash_Write8(volatile uint8_t* const address, const uint8_t data)
{

  uint16union_t halfword; //Stores the hi and lo components of the halfword
  uint32_t flashAddress = (uint32_t)address; //Stores the starting address to store data

  if(flashAddress % 2 ==0)
  {
    halfword.s.Lo = data; //this combines the data with the current byte
    halfword.s.Hi = _FB(flashAddress+1);
    return Flash_Write16(&(_FH(flashAddress)),halfword.l); // this will return the 16bit half word
  }
  else
  {
    halfword.s.Lo = _FB(flashAddress-1); //combines the data in address -1 with current byte
    halfword.s.Hi = data;
    return Flash_Write16(&(_FH(flashAddress-1)),halfword.l); // this will return the 16bit half word
  }

}

/*! @brief Erases the entire Flash sector
 *
 *  @return bool - TRUE if the Flash "data" sector was erased successfully
 *  @param address is the starting address of the sector
 */
static bool EraseSector(const uint32_t address)
{

  Fccob.Command = FTFE_FCCOB0_CCOBn(0x09); // Command to erase sector
  Fccob.FlashAddress1 = address >> 16; // this is 23-16 of starting address
  Fccob.FlashAddress2 = address >> 8; // this is 15-8 of starting address
  Fccob.FlashAddress3 = address; // this is 7-0 of starting address
  return LaunchCommand(&Fccob);

}


/*! @brief Updates FCCOB registers to write to flash
 *
 *  @return bool - TRUE if the the writing was completed successfully
 *  @param commonCommandObject is the structure which contains the stored values
 */
static bool LaunchCommand(TFCCOB* commonCommandObject)
{

  if(ACCERR_FPVIOL_ERROR) // Check to see for ACCER AND FPVIOL FLAG
  {
    FTFE_FSTAT = FTFE_FSTAT_ACCERR_MASK | FTFE_FSTAT_FPVIOL_MASK; // This will clear any erros in the past
  }

  FTFE_FCCOB0 = commonCommandObject->Command; // THIS will place the structure in the FCCOB Register
  FTFE_FCCOB1 = commonCommandObject->FlashAddress1;
  FTFE_FCCOB2 = commonCommandObject->FlashAddress2;
  FTFE_FCCOB3 = commonCommandObject->FlashAddress3;

  FTFE_FCCOB8 = commonCommandObject->DataByte0;
  FTFE_FCCOB9 = commonCommandObject->DataByte1;
  FTFE_FCCOBB = commonCommandObject->DataByte2;
  FTFE_FCCOBA = commonCommandObject->DataByte3;
  FTFE_FCCOB4 = commonCommandObject->DataByte4;
  FTFE_FCCOB5 = commonCommandObject->DataByte5;
  FTFE_FCCOB6 = commonCommandObject->DataByte6;
  FTFE_FCCOB7 = commonCommandObject->DataByte7;

  FTFE_FSTAT = FTFE_FSTAT_CCIF_MASK; // Launch command sequence

  while(!(FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK))
  {

  }

  return true;
}

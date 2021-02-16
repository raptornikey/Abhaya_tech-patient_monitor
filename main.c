

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"




#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define TWI_SCL_M           22         //I2C SCL Pin
#define TWI_SDA_M           23         //I2C SDA Pin




#define MAX30105_WHO_AM_I     0x57     //MAX30105 ID
 
#define MAX30105_INTSTAT1 0x00
#define MAX30105_INTSTAT2 0x01
#define MAX30105_INTENABLE1 0x02
#define MAX30105_INTENABLE2 0x03

// FIFO Registers
#define MAX30105_FIFOWRITEPTR 0x04
#define MAX30105_FIFOOVERFLOW 0x05
#define MAX30105_FIFOREADPTR 0x06
#define MAX30105_FIFODATA 0x07

// Configuration Registers
#define MAX30105_FIFOCONFIG 0x08
#define MAX30105_MODECONFIG 0x09
#define MAX30105_PARTICLECONFIG 0x0A    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
#define MAX30105_LED1_PULSEAMP 0x0C
#define MAX30105_LED2_PULSEAMP 0x0D
#define MAX30105_LED3_PULSEAMP 0x0E
#define MAX30105_LED_PROX_AMP 0x10
#define MAX30105_MULTILEDCONFIG1  0x11
#define MAX30105_MULTILEDCONFIG2  0x12

// Die Temperature Registers
#define MAX30105_DIETEMPINT 0x1F
#define MAX30105_DIETEMPFRAC 	0x20
#define MAX30105_DIETEMPCONFIG 0x21

// Proximity Function Registers
#define MAX30105_PROXINTTHRESH 0x30

// Part ID Registers
#define MAX30105_REVISIONID 0xFE
#define MAX30105_PARTID 0xFF    // Should always be 0x15. Identical to MAX30102.

// MAX30105 Commands
// Interrupt configuration (pg 13, 14)
#define MAX30105_INT_A_FULL_MASK ~0b10000000
#define MAX30105_INT_A_FULL_ENABLE 0x80
#define MAX30105_INT_A_FULL_DISABLE 0x00

#define MAX30105_INT_DATA_RDY_MASK ~0b01000000
//byte MAX30105_INT_DATA_RDY_MASK = ~0b01000000
#define MAX30105_INT_DATA_RDY_ENABLE 	0x40
#define MAX30105_INT_DATA_RDY_DISABLE 0x00

#define MAX30105_INT_ALC_OVF_MASK  ~0b00100000
#define MAX30105_INT_ALC_OVF_ENABLE 	0x20
#define MAX30105_INT_ALC_OVF_DISABLE  0x00

#define MAX30105_INT_PROX_INT_MASK  ~0b00010000
#define MAX30105_INT_PROX_INT_ENABLE  0x10
#define MAX30105_INT_PROX_INT_DISABLE  0x00

#define MAX30105_INT_DIE_TEMP_RDY_MASK  ~0b00000010
#define MAX30105_INT_DIE_TEMP_RDY_ENABLE  0x02
#define MAX30105_INT_DIE_TEMP_RDY_DISABLE  0x00

#define MAX30105_SAMPLEAVG_MASK ~0b11100000
#define MAX30105_SAMPLEAVG_1  	0x00
#define MAX30105_SAMPLEAVG_2  	0x20
#define MAX30105_SAMPLEAVG_4  	0x40
#define MAX30105_SAMPLEAVG_8  	0x60
#define MAX30105_SAMPLEAVG_16 	0x80
#define MAX30105_SAMPLEAVG_32 	0xA0

#define MAX30105_ROLLOVER_MASK 0xEF
#define MAX30105_ROLLOVER_ENABLE  0x10
#define MAX30105_ROLLOVER_DISABLE 0x00

#define MAX30105_A_FULL_MASK  	0xF0

// Mode configuration commands (page 19)
#define MAX30105_SHUTDOWN_MASK 0x7F
#define MAX30105_SHUTDOWN 0x80
#define MAX30105_WAKEUP 0x00

#define MAX30105_RESET_MASK 0xBF
#define MAX30105_RESET 0x40

#define MAX30105_MODE_MASK 0xF8
#define MAX30105_MODE_REDONLY 0x02
#define MAX30105_MODE_REDIRONLY 0x03
#define MAX30105_MODE_MULTILED 0x07

// Particle sensing configuration commands (pgs 19-20)
#define MAX30105_ADCRANGE_MASK 0x9F
#define MAX30105_ADCRANGE_2048 0x00
#define MAX30105_ADCRANGE_4096 0x20
#define MAX30105_ADCRANGE_8192 0x40
#define MAX30105_ADCRANGE_16384 0x60

#define MAX30105_SAMPLERATE_MASK  0xE3
#define MAX30105_SAMPLERATE_50 0x00
#define MAX30105_SAMPLERATE_100 0x04
#define MAX30105_SAMPLERATE_200 0x08
#define MAX30105_SAMPLERATE_400 0x0C
#define MAX30105_SAMPLERATE_800 0x10
#define MAX30105_SAMPLERATE_1000  0x14
#define MAX30105_SAMPLERATE_1600  0x18
#define MAX30105_SAMPLERATE_3200  0x1C

#define MAX30105_PULSEWIDTH_MASK  0xFC
#define MAX30105_PULSEWIDTH_69 0x00
#define MAX30105_PULSEWIDTH_118 0x01
#define MAX30105_PULSEWIDTH_215 0x02
#define MAX30105_PULSEWIDTH_411 0x03

//Multi-LED Mode configuration (pg 22)
#define MAX30105_SLOT1_MASK 0xF8
#define MAX30105_SLOT2_MASK 0x8F
#define MAX30105_SLOT3_MASK 0xF8
#define MAX30105_SLOT4_MASK 0x8F

#define SLOT_NONE 0x00
#define SLOT_RED_LED 0x01
#define SLOT_IR_LED 0x02
#define SLOT_GREEN_LED 	0x03
#define SLOT_NONE_PILOT 0x04
#define SLOT_RED_PILOT 0x05
#define SLOT_IR_PILOT 0x06
#define SLOT_GREEN_PILOT 0x07
#define I2C_BUFFER_LENGTH 64
#define MAX_30105_EXPECTEDPARTID 0x15	

#define FreqS 25    //sampling frequency
#define BUFFER_SIZE (FreqS * 4) 
#define MA4_SIZE 4 // DONOT CHANGE


#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif



static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/*
uint32_t getRed(void); //Returns immediate red value
uint32_t getIR(void); //Returns immediate IR value
uint32_t getGreen(void); //Returns immediate green value
bool safeCheck(uint8_t maxTimeToCheck); //Given a max amount of time, check for new data


void twi_master_init(void); // initialize the twi communication
bool max30105_init(void);    // initialize the max30105

void softReset();
void shutDown(); 
void wakeUp(); 

void setLEDMode(uint8_t mode);

void setADCRange(uint8_t adcRange);
void setSampleRate(uint8_t sampleRate);
void setPulseWidth(uint8_t pulseWidth);
void setPulseAmplitudeRed(uint8_t value);
void setPulseAmplitudeIR(uint8_t value);
void setPulseAmplitudeGreen(uint8_t value);
void setPulseAmplitudeProximity(uint8_t value);

void setProximityThreshold(uint8_t threshMSB);

  //Multi-led configuration mode (page 22)
void enableSlot(uint8_t slotNumber, uint8_t device); //Given slot number, assign a device to slot
void disableSlots(void);
  
  // Data Collection

  //Interrupts (page 13, 14)
uint8_t getINT1(void); //Returns the main interrupt group
uint8_t getINT2(void); //Returns the temp ready interrupt
void enableAFULL(void); //Enable/disable individual interrupts
void disableAFULL(void);
void enableDATARDY(void);
void disableDATARDY(void);
void enableALCOVF(void);
void disableALCOVF(void);
void enablePROXINT(void);
void disablePROXINT(void);
void enableDIETEMPRDY(void);
void disableDIETEMPRDY(void);

  //FIFO Configuration (page 18)
void setFIFOAverage(uint8_t samples);
void enableFIFORollover();
void disableFIFORollover();
void setFIFOAlmostFull(uint8_t samples);
  
  //FIFO Reading
uint16_t check(void); //Checks for new data and fills FIFO
uint8_t available(void); //Tells caller how many new samples are available (head - tail)
void nextSample(void); //Advances the tail of the sense array
uint32_t getFIFORed(void); //Returns the FIFO sample pointed to by tail
uint32_t getFIFOIR(void); //Returns the FIFO sample pointed to by tail
uint32_t getFIFOGreen(void); //Returns the FIFO sample pointed to by tail

uint8_t getWritePointer(void);
uint8_t getReadPointer(void);
void clearFIFO(void); //Sets the read/write pointers to zero

  //Proximity Mode Interrupt Threshold
void setPROXINTTHRESH(uint8_t val);

  // Die Temperature
float readTemperature();
float readTemperatureF();

  // Detecting ID/Revision
uint8_t getRevisionID();
uint8_t readPartID();  

  // Setup the IC with user selectable settings
 
uint8_t readRegister8(uint8_t address, uint8_t reg);
void writeRegister8(uint8_t address, uint8_t reg, uint8_t value);

*/

#define MAX30105_ADDRESS          (0x57 >> 1)
#define MY_TIMER            NRF_TIMER1
#define MY_TIMER_IRQn       TIMER1_IRQn
#define MY_TIMER_IRQHandler TIMER1_IRQHandler
static volatile bool m_xfer_done = false;


uint8_t *_i2cPort; //The generic connection to user's chosen I2C hardware
const uint8_t  _i2caddr;

  //activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
int activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO
  
uint8_t revisionID;

//void readRevisionID();

//void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
 
#define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro
typedef struct Record
  {
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    uint32_t green[STORAGE_SIZE];
    int head;
    int tail;
  } sense_struct; //This is our circular buffer of readings from the sensor

sense_struct sense;
/*
const uint8_t uch_spo2_table[184]={ 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99, 
              99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
              100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 
              97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 
              90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81, 
              80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67, 
              66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50, 
              49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29, 
              28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 
              3, 2, 1 } ;
static  int32_t an_x[ BUFFER_SIZE]; //ir
static  int32_t an_y[ BUFFER_SIZE]; //red
void maxim_heart_rate_and_oxygen_saturation(uint16_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid);
void maxim_find_peaks(int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num);
void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height);
void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance);
void maxim_sort_ascend(int32_t  *pn_x, int32_t n_size);
void maxim_sort_indices_descend(int32_t  *pn_x, int32_t *pn_indx, int32_t n_size);
void wFE(void);
static uint32_t millis(void);
static void my_timer_start(void);
void MY_TIMER_IRQHandler(void);

*/

//Initialize the TWI as Master device
void twi_master_init(void)
{
    ret_code_t err_code;

    // Configure the settings for twi communication
    const nrf_drv_twi_config_t twi_config = {
       .scl                = TWI_SCL_M,  //SCL Pin
       .sda                = TWI_SDA_M,  //SDA Pin
       .frequency          = NRF_DRV_TWI_FREQ_400K, //Communication Speed
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH, //Interrupt Priority(Note: if using Bluetooth then select priority carefully)
       .clear_bus_init     = false //automatically clear bus
    };


    //A function to initialize the twi communication
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    
    //Enable the TWI Communication
    nrf_drv_twi_enable(&m_twi);
}

uint8_t readRegister8(uint8_t address, uint8_t reg) {
   ret_code_t err_code;
   //uint8_t re_read[1] = {reg};
   err_code = nrf_drv_twi_rx(&m_twi, address, reg, sizeof(reg));
   APP_ERROR_CHECK(err_code);
   return(err_code);

  //_i2cPort->requestFrom((uint8_t)address, (uint8_t)1);
   
}

void writeRegister8(uint8_t address, uint8_t reg, uint8_t value) {
   
    ret_code_t err_code;
    uint8_t tx_buf[address+1];
    tx_buf[0] = reg;
    tx_buf[1] = value;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, address, tx_buf, address+1, false);
    APP_ERROR_CHECK(err_code);
}

static uint32_t millis(void)
{

    uint32_t my_timer_seconds = 0;
    // Store the current value of the timer in the CC[1] register, by triggering the capture task
    MY_TIMER->TASKS_CAPTURE[1] = 1;
    
    // Combine the state of the second variable with the current timer state, and return the result
    return (my_timer_seconds * 1000) + (MY_TIMER->CC[1] / 1000);
}



void softReset(void) {
  bitMask(MAX30105_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  unsigned long startTime = millis();
  while (millis() - startTime < 100)
  {
    uint8_t response = readRegister8(_i2caddr, MAX30105_MODECONFIG);
    if ((response & MAX30105_RESET) == 0) break;
    nrf_delay_ms(1); 
  }
}

void bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents = readRegister8(_i2caddr, reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeRegister8(_i2caddr, reg, originalContents | thing);
}

void setFIFOAverage(uint8_t numberOfSamples) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, numberOfSamples);
}

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
void clearFIFO(void) {
  writeRegister8(_i2caddr, MAX30105_FIFOWRITEPTR, 0);
  writeRegister8(_i2caddr, MAX30105_FIFOOVERFLOW, 0);
  writeRegister8(_i2caddr, MAX30105_FIFOREADPTR, 0);
}

//Enable roll over if FIFO over flows
void enableFIFORollover(void) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}


void setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
  bitMask(MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode);
}

void setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX30105_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange);
}

void setSampleRate(uint8_t sampleRate) {
  // sampleRate: one of MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sampleRate);
}

void setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX30105_PULSEWIDTH_69, _188, _215, _411
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulseWidth);
}

//  Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void setPulseAmplitudeRed(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX30105_LED1_PULSEAMP, amplitude);
}

void setPulseAmplitudeIR(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX30105_LED2_PULSEAMP, amplitude);
}

void setPulseAmplitudeGreen(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX30105_LED3_PULSEAMP, amplitude);
}

void setPulseAmplitudeProximity(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX30105_LED_PROX_AMP, amplitude);
}

//Given a slot number assign a thing to it
//Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
//Assigning a SLOT_RED_LED will pulse LED
//Assigning a SLOT_RED_PILOT will ??
void enableSlot(uint8_t slotNumber, uint8_t device) {

  uint8_t originalContents;

  switch (slotNumber) {
    case (1):
      bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
      break;
    case (2):
      bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
      break;
    case (3):
      bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device);
      break;
    case (4):
      bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4);
      break;
    default:
      
      break;
  }
}


//Read the FIFO Write Pointer
uint8_t getWritePointer(void) {
  return (readRegister8(_i2caddr, MAX30105_FIFOWRITEPTR));
}

//Read the FIFO Read Pointer
uint8_t getReadPointer(void) {
  return (readRegister8(_i2caddr, MAX30105_FIFOREADPTR));
}



uint16_t check(void)
{
  //Read register FIDO_DATA in (3-byte * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR
  uint32_t readPointer;
  uint32_t writePointer;
  readPointer = getReadPointer();
  writePointer = getWritePointer();

  int numberOfSamples = 0;

  //Do we have new data?
  if (readPointer != writePointer)
  {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //Get ready to read a burst of data from the FIFO register
    ret_code_t err_code;
    uint8_t reg_write[1] = {MAX30105_FIFODATA};
    err_code = nrf_drv_twi_tx(&m_twi, MAX30105_ADDRESS, reg_write, sizeof(reg_write), true);
    APP_ERROR_CHECK(err_code);

    //We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
    //I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
    //Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
    while (bytesLeftToRead > 0)
    {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH)
      {
        //If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
        //32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
      }

      bytesLeftToRead -= toGet;

      //Request toGet number of bytes from sensor
      ret_code_t err_code1;
      uint8_t m_sample[sizeof(toGet)];
      
      while (toGet > 0)
      {
        sense.head++; //Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; //Wrap condition

        uint32_t temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
        uint32_t templong;

        //Burst read three bytes - RED
        err_code1 = nrf_drv_twi_rx(&m_twi, MAX30105_ADDRESS, temp, 3);                
        APP_ERROR_CHECK(err_code1);
        templong = ((uint32_t)temp[0] << 16) | ((uint32_t)temp[1]<<8) | temp[2];

        //Convert array to long
        memcpy(&templong, temp, sizeof(templong));
		
		templong &= 0x3FFFF; //Zero out all but 18 bits

        sense.red[sense.head] = templong; //Store this reading into the sense array

        if (activeLEDs > 1)
        {
          //Burst read three more bytes - IR
          err_code1 = nrf_drv_twi_rx(&m_twi, MAX30105_ADDRESS, temp, 3);                
          APP_ERROR_CHECK(err_code1);
          templong = ((uint32_t)temp[0] << 16) | ((uint32_t)temp[1]<<8) | temp[2];

          //Convert array to long
          memcpy(&templong, temp, sizeof(templong));

		  templong &= 0x3FFFF; //Zero out all but 18 bits
          
		  sense.IR[sense.head] = templong;
        }

        if (activeLEDs > 2)
        {
          //Burst read three more bytes - Green
          err_code1 = nrf_drv_twi_rx(&m_twi, MAX30105_ADDRESS, temp, 3);                
          APP_ERROR_CHECK(err_code1);
          templong = ((uint32_t)temp[0] << 16) | ((uint32_t)temp[1]<<8) | temp[2];

          //Convert array to long
          memcpy(&templong, temp, sizeof(templong));

		  templong &= 0x3FFFF; //Zero out all but 18 bits

          sense.green[sense.head] = templong;
        }

        toGet -= activeLEDs * 3;
      }

    } //End while (bytesLeftToRead > 0)

  } //End readPtr != writePtr

  return (numberOfSamples); //Let the world know how much new data we found
}



void setup( float  powerLevel,int sampleAverage, int  ledMode, int sampleRate, int pulseWidth, int adcRange) {
  softReset(); //Reset all configuration, threshold, and data registers to POR values

  //FIFO Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //The chip will average multiple samples of same type together if you wish
  if (sampleAverage == 1) setFIFOAverage(MAX30105_SAMPLEAVG_1); //No averaging per FIFO record
  else if (sampleAverage == 2) setFIFOAverage(MAX30105_SAMPLEAVG_2);
  else if (sampleAverage == 4) setFIFOAverage(MAX30105_SAMPLEAVG_4);
  else if (sampleAverage == 8) setFIFOAverage(MAX30105_SAMPLEAVG_8);
  else if (sampleAverage == 16) setFIFOAverage(MAX30105_SAMPLEAVG_16);
  else if (sampleAverage == 32) setFIFOAverage(MAX30105_SAMPLEAVG_32);
  else setFIFOAverage(MAX30105_SAMPLEAVG_4);

  //setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
  enableFIFORollover(); //Allow FIFO to wrap/roll over
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Mode Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (ledMode == 3) setLEDMode(MAX30105_MODE_MULTILED); //Watch all three LED channels
  else if (ledMode == 2) setLEDMode(MAX30105_MODE_REDIRONLY); //Red and IR
  else setLEDMode(MAX30105_MODE_REDONLY); //Red only
  activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Particle Sensing Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if(adcRange < 4096) setADCRange(MAX30105_ADCRANGE_2048); //7.81pA per LSB
  else if(adcRange < 8192) setADCRange(MAX30105_ADCRANGE_4096); //15.63pA per LSB
  else if(adcRange < 16384) setADCRange(MAX30105_ADCRANGE_8192); //31.25pA per LSB
  else if(adcRange == 16384) setADCRange(MAX30105_ADCRANGE_16384); //62.5pA per LSB
  else setADCRange(MAX30105_ADCRANGE_2048);

  if (sampleRate < 100) setSampleRate(MAX30105_SAMPLERATE_50); //Take 50 samples per second
  else if (sampleRate < 200) setSampleRate(MAX30105_SAMPLERATE_100);
  else if (sampleRate < 400) setSampleRate(MAX30105_SAMPLERATE_200);
  else if (sampleRate < 800) setSampleRate(MAX30105_SAMPLERATE_400);
  else if (sampleRate < 1000) setSampleRate(MAX30105_SAMPLERATE_800);
  else if (sampleRate < 1600) setSampleRate(MAX30105_SAMPLERATE_1000);
  else if (sampleRate < 3200) setSampleRate(MAX30105_SAMPLERATE_1600);
  else if (sampleRate == 3200) setSampleRate(MAX30105_SAMPLERATE_3200);
  else setSampleRate(MAX30105_SAMPLERATE_50);

  //The longer the pulse width the longer range of detection you'll have
  //At 69us and 0.4mA it's about 2 inches
  //At 411us and 0.4mA it's about 6 inches
  if (pulseWidth < 118) setPulseWidth(MAX30105_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution
  else if (pulseWidth < 215) setPulseWidth(MAX30105_PULSEWIDTH_118); //16 bit resolution
  else if (pulseWidth < 411) setPulseWidth(MAX30105_PULSEWIDTH_215); //17 bit resolution
  else if (pulseWidth == 411) setPulseWidth(MAX30105_PULSEWIDTH_411); //18 bit resolution
  else setPulseWidth(MAX30105_PULSEWIDTH_69);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //LED Pulse Amplitude Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Default is 0x1F which gets us 6.4mA
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

  setPulseAmplitudeRed(powerLevel);
  setPulseAmplitudeIR(powerLevel);
  setPulseAmplitudeGreen(powerLevel);
  setPulseAmplitudeProximity(powerLevel);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Multi-LED Mode Configuration, Enable the reading of the three LEDs
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  enableSlot(1, SLOT_RED_LED);
  if (ledMode > 1) enableSlot(2, SLOT_IR_LED);
  if (ledMode > 2) enableSlot(3, SLOT_GREEN_LED);
  //enableSlot(1, SLOT_RED_PILOT);
  //enableSlot(2, SLOT_IR_PILOT);
  //enableSlot(3, SLOT_GREEN_PILOT);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  clearFIFO(); //Reset the FIFO before we begin checking the sensor
}




bool max30105_verify_product_id(void)
{
    uint8_t who_am_i; // create a variable to hold the who am i value


    // Note: All the register addresses including WHO_AM_I are declared in 
    // max30105.h file, you can check these addresses and values from the
    // datasheet of your slave device.
    if (readRegister8(MAX30105_ADDRESS, &who_am_i))
    {
        if (who_am_i != MAX30105_WHO_AM_I)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}


/*
  Function to initialize the max30105
*/ 
bool max30105_init(void)
{   
  bool transfer_succeeded = true;
	
  //Check the id to confirm that we are communicating with the right device
  transfer_succeeded &= max30105_verify_product_id();
	
  if(max30105_verify_product_id() == false)
    {
	return false;
      }

  // Set the registers with the required values, see the datasheet to get a good idea of these values
  /*(void)max30105_register_write(MPU_PWR_MGMT1_REG , 0x00); 
  (void)max30105_register_write(MPU_SAMPLE_RATE_REG , 0x07); 
  (void)max30105_register_write(MPU_CFG_REG , 0x06); 						
  (void)max30105_register_write(MPU_INT_EN_REG, 0x00); 
  (void)max30105_register_write(MPU_GYRO_CFG_REG , 0x18); 
  (void)max30105_register_write(MPU_ACCEL_CFG_REG,0x00);   		
*/
  return transfer_succeeded;
}


bool safeCheck(uint8_t maxTimeToCheck)
{
  uint32_t markTime = millis();
  
  while(1)
  {
	if(millis() - markTime > maxTimeToCheck) return(false);

	if(check() == true) //We found new data!
	  return(true);

	nrf_delay_ms(1);
  }
}


//Report the most recent red value
uint32_t getRed(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.red[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent IR value
uint32_t getIR(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.IR[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent Green value
uint32_t getGreen(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.green[sense.head]);
  else
    return(0); //Sensor failed to find new data
}


// main code

float  powerLevel=0x1F; int sampleAverage=4; int  ledMode=3; int sampleRate=400; int pulseWidth=411; int adcRange=4096;

int main(void)
{

// initialize the logger
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
	
	
// create arrays which will hold x,y & z co-ordinates values of acc and gyro
    //static int16_t AccValue[3], GyroValue[3];

    bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS); // initialize the leds and buttons

    twi_master_init(); // initialize the twi 
    nrf_delay_ms(1000); // give some delay

    while(max30105_init() == false) // wait until max30105 sensor is successfully initialized
    {
      NRF_LOG_INFO("MPU_6050 initialization failed!!!"); // if it failed to initialize then print a message
      nrf_delay_ms(1000);
      
    }

   NRF_LOG_INFO("max30105 Init Successfully!!!"); 
   setup(powerLevel,sampleAverage,ledMode,sampleRate,pulseWidth,adcRange);
   NRF_LOG_INFO("...."); // display a message to let the user know that the device is starting to read the values
   nrf_delay_ms(2000);


  
    
    while (true)
    {
        //if(max30105_ReadAcc(&AccValue[0], &AccValue[1], &AccValue[2]) == true) // Read acc value from max30105 internal registers and save them in the array
        //{
          NRF_LOG_INFO(" R[ %d", getRed()); // display the read values
        //}
        //else
        //{
        //  NRF_LOG_INFO("Reading ACC values Failed!!!"); // if reading was unsuccessful then let the user know about it
        //}


        //if(max30105_ReadGyro(&GyroValue[0], &GyroValue[1], &GyroValue[2]) == true) // read the gyro values from max30105's internal registers and save them in another array
        //{
          NRF_LOG_INFO("] IR[ %d",getIR()); // display then values
        //}

        //else
        //{
          NRF_LOG_INFO("] G[ %d",getGreen());
        //}

       nrf_delay_ms(100); // give some delay 


    }
}

/** @} */

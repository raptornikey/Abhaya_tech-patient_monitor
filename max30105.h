#ifndef AT24C02_H__
#define AT24C02_H__
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//I2C Pins Settings, you change them to any other pins
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


uint8_t *_i2cPort; //The generic connection to user's chosen I2C hardware
const uint8_t  _i2caddr;

  //activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
int activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO
  
uint8_t revisionID;

void readRevisionID();

void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
 
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


#endif




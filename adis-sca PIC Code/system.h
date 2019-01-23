/*!
 * \file  system.h
 *
 *
 * \date January 20, 2015
 * \version Senslope Alpha 3.0
 *
 */


#ifndef SYSTEM_H
#define	SYSTEM_H

#ifdef	__cplusplus
extern "C" {
#endif

/******************************************************************************/
/* Definitions						                                          */
/******************************************************************************/

#define TIMEOUT 0x2E84                                  // Sampling rate, 5 seconds1E84
#define WAITTIME 0x0
#define SOMSSAMPLES 50

// accel related definitions

#define RESOLUTION 							1024    // Accelerometer calibrated value resolution
#define AXEL_WAITTIME                       60      // Accel updating time
#define axel_sample                         10		// Number of accel samples for averaging routine
#define MULTIPLIER1 						10000	// Multiplier for 3x3 matrix calibration parameters
#define MULTIPLIER2 						1		// Multiplier for 3x1 matrix calibration parameters

/******************************************************************************/
/* List of commands						                                      */
/******************************************************************************/

//Broadcast
#define BROAD_AXEL1_RAW_INIT                2
#define BROAD_AXEL2_RAW_INIT                3
#define BROAD_SOMS_RAW_INIT                 4

#define BROAD_SOMS_CALIB_INIT               7

#define BROAD_AXEL1_RAW_NEW                 8
#define BROAD_AXEL2_RAW_NEW                 9
#define BROAD_SOMS_RAW_NEW                  10

#define BROAD_AXEL1_CALIB_NEW               11
#define BROAD_AXEL2_CALIB_NEW               12
#define BROAD_SOMS_CALIB_NEW                13

#define BROAD_SELF_TEST_ONECOMMAND          16
#define BROAD_AXEL1_SELFTEST_OUTPUT_CHANGE  17
#define BROAD_AXEL2_SELFTEST_OUTPUT_CHANGE  19
#define BROAD_SELF_TEST_ROUTINE             20

#define BROAD_CALIBRATE_REFVOLTAGE          21
#define BROAD_GET_DIAGNOSTICS               22

#define BROAD_SOMS_AIR                      23
#define BROAD_SOMS_WATER                    24
#define	BROAD_SOMS_CAP						27		//for functionality testing

#define BROAD_EEPROM_ACCESS		            34
#define BROAD_CHANGE_DEBUG_MODE             35

#define BROAD_AXEL2_INCL_NEW                50      //for INCL data of ADIS16210

//Node specific

#define EEPROM_MATRIX_AXEL1                 25
#define EEPROM_MATRIX_AXEL2                 26

//This is a list of some of the registers available on the SCA3100.
#define REVID   0x01  //  ASIC revision ID number
#define CTRL    0x04  //  Control
#define STATUS   0x08  //  Status
#define RESET   0x03  //  Reset Component
#define X_LSB   0x10  //  Output, x-axis LSB frame
#define X_MSB   0x15  //  Output, x-axis MSB frame
#define Y_LSB   0x19  //  Output, y-axis LSB frame
#define Y_MSB   0x1C  //  Output, y-axis LSB frame
#define Z_LSB   0x20  //  Output, z-axis MSB frame
#define Z_MSB   0x25  //  Output, z-axis LSB frame
#define TEMP_LSB  0x49 //  Temperature LSB frame
#define TEMP_MSB  0x4C //  Temperature MSB frame

#define INT_STATUS    0x58 //  Interrupt status register in multi_axis components
#define ID    0x9D  //  Component ID

//
//This is a list of some of the registers available on the ADIS16210.
#define BIT4            0x0010
#define FLASH_CNT   0x00  //  Diagnostics, flash write counter (16-bit binary)
#define SUPPLY_OUT    0x02  //  Output, power supply
#define XACCL_OUT   0x04  //  Output, x-axis acceleration
#define YACCL_OUT   0x06  //  Output, y-axis acceleration
#define ZACCL_OUT   0x08  //  Output, z-axis acceleration
#define TEMP_OUT    0x0A  //  Output, internal temperature
#define XINCL_OUT   0x0C  //  Output, ±180° x-axis inclination
#define YINCL_OUT   0x0E  //  Output, ±180° y-axis inclination
#define ZINCL_OUT   0x10  //  Output, ±180° z-axis inclination
#define XACCL_NULL    0x12  //  Calibration, x-axis acceleration offset null
#define YACCL_NULL    0x14  //  Calibration, y-axis acceleration offset null
#define ZACCL_NULL    0x16  //  Calibration, z-axis acceleration offset null
#define ALM_MAG_X   0x20  //  Alarm, x-axis amplitude threshold
#define ALM_MAG_Y   0x22  //  Alarm, y-axis amplitude threshold
#define ALM_MAG_Z   0x24  //  Alarm, z-axis amplitude threshold
#define ALM_MAG_S   0x26  //  Alarm, system alarm threshold
#define ALM_SMPL_X    0x28  //  Alarm, x-axis sample period
#define ALM_SMPL_Y    0x2A  //  Alarm, y-axis sample period
#define ALM_SMPL_Z    0x2C  //  Alarm, z-axis sample period
#define ALM_CTRL    0x2E  //  Operation, alarm control
#define GPIO_CTRL   0x32  //  Operation, general I/O configuration and data
#define MSC_CTRL    0x34  //  Operation, orientation mode
#define DIO_CTRL    0x36  //  Operation, digital I/O configuration and data
#define AVG_CNT     0x38  //  Operation, decimation filter configuration
#define SLP_CNT     0x3A  //  Operation, sleep count
#define DIAG_STAT   0x3C  //  Diagnostics, system status register
#define GLOB_CMD    0x3E  //  Operation, system command register
#define LOT_ID1     0x52  //  Lot identification, Code 1
#define LOT_ID2     0x54  //  Lot identification, Code 2
#define PROD_ID     0x56  //  Production identification number
#define SERIAL_NUM    0x58  //  Serial Number


/*
#define POLL_AXEL1_RAW_INIT                 102
#define POLL_AXEL2_RAW_INIT                 103
#define POLL_SOMS_RAW_INIT                  104

#define POLL_SOMS_CALIB_INIT                107

#define POLL_AXEL1_RAW_NEW                  108
#define POLL_AXEL2_RAW_NEW                  109
#define POLL_SOMS_RAW_NEW                   110

#define POLL_AXEL1_CALIB_NEW                111
#define POLL_AXEL2_CALIB_NEW                112
#define POLL_SOMS_CALIB_NEW                 113

#define POLL_SELF_TEST_ONECOMMAND 	        116
#define POLL_AXEL1_SELFTEST_OUTPUT_CHANGE   117
#define POLL_AXEL2_SELFTEST_OUTPUT_CHANGE   119
#define POLL_SELF_TEST_ROUTINE              120

#define POLL_CALIBRATE_REFVOLTAGE           121
#define POLL_GET_DIAGNOSTICS	            122

#define POLL_SOMS_AIR                       123
#define POLL_SOMS_WATER                     124

#define POLL_CHANGE_DEBUG_MODE             	135

*/

/******************************************************************************/
/* Function prototypes					                                      */
/******************************************************************************/

void get_axel1raw (void);
void get_axel2raw (void);
void get_axel2incl (void);

void get_axelcal_m(int xdata, int ydata, int zdata, int axel);
//void get_axel2cal_m(void);

void axel_initialize (void);
void axel_initialize_self (void);
void axel_writebyte (	unsigned char address,
						unsigned char data);
unsigned char axel_readbyte (unsigned char address);

void axel2_initialize (void);
void axel2_initialize_self (void);
void axel2_writebyte (	unsigned char address,
						unsigned char data);
int axel2_readbyte (unsigned char address);

void axel_selftest (void);

unsigned char EEPROM_write(	unsigned char addressh,
							unsigned char address,
							unsigned char data);
unsigned char EEPROM_read(	unsigned char addressh,
							unsigned char address);

void cal_refvoltage (long ref);
int get_refvoltage (void);
int get_temperature (void);

void soms_adc_initialize (void);
void get_somsraw (int func_test);
void get_somscal (void);
int get_median(unsigned int *analogValues, int numReadings);

#ifdef	__cplusplus
}
#endif

#endif	/* SYSTEM_H */


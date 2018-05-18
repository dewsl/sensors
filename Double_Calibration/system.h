/*!
 * \file  system.h
 *
 *
 * \date July 2014
 * \version 1.0
 *
 * Project: Senslope \n
 * Processor:      pic18f2585 \n
 * Compiler:       c18 \n
 *\n
 * Version 1.0\n
 * System definitions and node commands written here
 *
 *
 */


#ifndef SYSTEM_H
#define	SYSTEM_H

#ifdef	__cplusplus
extern "C" {
#endif


/*Definitions*/
#define TIMEOUT 							0x2E84  // Sampling period
#define DEBUG 								1       // Debugging mode
#define PULSECOUNT 							2048    // SoMS oscillator timeout
#define RESOLUTION 							1024    // Accelerometer calibrated value resolution
#define AXEL_WAITTIME                       60      // Accel updating time
#define axel_sample                         10		// Number of accel samples for averaging routine
#define MULTIPLIER1 						10000	// Multiplier for 3x3 matrix calibration parameters
#define MULTIPLIER2 						1		// Multiplier for 3x1 matrix calibration parameters

/*List of commands*/
 
    /*NODE SPECIFIC COMMANDS*/
#define EEPROM_AXEL1_XYZ                    101		// Saving calibration factors
#define EEPROM_AXEL1_OFFSET                 102
#define EEPROM_AXEL2_XYZ                    103
#define EEPROM_AXEL2_OFFSET                 104

#define SOMS_AIR_OVER_TEN_20                105
#define SOMS_WATER_OVER_TEN_20              106
#define SOMS_AIR_OVER_TEN_8                 107
#define SOMS_WATER_OVER_TEN_8               108

#define EEPROM_MATRIX_AXEL1                 109
#define EEPROM_MATRIX_AXEL2                 110

    /*BROADCAST COMMANDS*/
#define EEPROM_TESTING                      1		// Testing EEPROM

#define PASS_AXEL1_RAW                      2  		// Pass initial raw data
#define PASS_AXEL2_RAW                      3
#define PASS_SOMS_RAW                       4

#define PASS_AXEL1_CALIB                    5 		// Pass initial calibrated data
#define PASS_AXEL2_CALIB                    6
#define PASS_SOMS_CALIB                     7

#define PASS_AXEL1_RAW_NEW                  8 		// Pass updated raw data
#define PASS_AXEL2_RAW_NEW                  9
#define PASS_SOMS_RAW_NEW                   10

#define PASS_AXEL1_CALIB_MINMAX             11 		// Pass updated MINMAX_calibrated data
#define PASS_AXEL2_CALIB_MINMAX             12
#define PASS_SOMS_CALIB_NEW                 13

#define PASS_AXEL1_RAW_AVE                  14 		// Pass average accelerometer data
#define PASS_AXEL2_RAW_AVE                  15

#define PASS_AXEL1_AXIS_STAT                16		// Pass self-test data
#define PASS_AXEL1_SELFTEST_OUTPUT_CHANGE   17
#define PASS_AXEL2_AXIS_STAT                18
#define PASS_AXEL2_SELFTEST_OUTPUT_CHANGE   19
#define SELF_TEST_ROUTINE                   20

#define PASS_AXEL1_CALIB_MATRIX             22 		// Pass updated MATRIX_calibrated data
#define PASS_AXEL2_CALIB_MATRIX             23

#define PASS_SOMS_RAW_NEW_8                 21 		// Pass updated SoMS 8MHz data
#define PASS_SOMS_CALIB_NEW_8               26

#define PASS_AXEL1_RAW_ADC_NEW              30		// Pass updated accel and ADC data
#define PASS_AXEL2_RAW_ADC_NEW              31
#define PASS_AXEL1_ADC_CALIB_MINMAX         32
#define PASS_AXEL2_ADC_CALIB_MINMAX         33

/*Function prototypes*/

void get_axel1raw (void);
void get_axel2raw (void);
void average_axel1(int x);
void average_axel2(int x);

void get_axelcal (int axel);
void get_axel1cal_m(void);
void get_axel2cal_m(void);

void soil_initialize (void);
void get_somsraw (unsigned char clk);
void resample_soms(unsigned char clock_select);
void get_somscal (void);

void axel_initialize (void);
void axel_initialize_self (void);
void axel_writebyte (	unsigned char address,
						unsigned char data);
unsigned char axel_readbyte (unsigned char address);
void axel2_initialize (void);
void axel2_initialize_self (void);
void axel2_writebyte (	unsigned char address,
						unsigned char data);
unsigned char axel2_readbyte (unsigned char address);

unsigned char EEPROM_write(	unsigned char addressh,
							unsigned char address,
							unsigned char data);
unsigned char EEPROM_read(	unsigned char addressh,
							unsigned char address);

void get_adcvoltage (void);


#ifdef	__cplusplus
}
#endif

#endif	/* SYSTEM_H */


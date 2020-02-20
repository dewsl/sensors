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

// accel related definitions

#define RESOLUTION 							1024    // Accelerometer calibrated value resolution
#define AXEL_WAITTIME                       60      // Accel updating time
#define MULTIPLIER1 						10000	// Multiplier for 3x3 matrix calibration parameters
#define MULTIPLIER2 						1		// Multiplier for 3x1 matrix calibration parameters

/******************************************************************************/
/* List of commands						                                      */
/******************************************************************************/

//Broadcast
#define BROAD_AXEL1_RAW_INIT                2
#define BROAD_AXEL2_RAW_INIT                3


#define BROAD_AXEL1_RAW_NEW                 8
#define BROAD_AXEL2_RAW_NEW                 9


#define BROAD_AXEL1_CALIB_NEW               41
#define BROAD_AXEL2_CALIB_NEW               42


#define BROAD_SELF_TEST_ONECOMMAND          16
#define BROAD_AXEL1_SELFTEST_OUTPUT_CHANGE  17
#define BROAD_AXEL2_SELFTEST_OUTPUT_CHANGE  19
#define BROAD_SELF_TEST_ROUTINE             20

#define BROAD_CALIBRATE_REFVOLTAGE          21
#define BROAD_GET_DIAGNOSTICS               22
#define GET_TEMPERATURE_ACCEL1              23
#define GET_TEMPERATURE_ACCEL2              24
#define GET_STATUS_ACCEL                    27
//#define ACCEL_STATUS                        28
    
#define BROAD_EEPROM_ACCESS		            34
#define BROAD_CHANGE_DEBUG_MODE             35
#define BROAD_CHANGE_AVERAGING_SAMPLE       36


//Node specific

#define EEPROM_MATRIX_AXEL1                 25
#define EEPROM_MATRIX_AXEL2                 26
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

#define POLL_CHANGE_DEBUG_MODE             135

*/

/******************************************************************************/
/* Function prototypes					                                      */
/******************************************************************************/

void get_axel1raw (void);
void get_axel2raw (void);

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
unsigned char axel2_readbyte (unsigned char address);

void axel_selftest (void);
int iis_selftest (void);

unsigned char EEPROM_write(	unsigned char addressh,
							unsigned char address,
							unsigned char data);
unsigned char EEPROM_read(	unsigned char addressh,
							unsigned char address);

void cal_refvoltage (long ref);
int get_refvoltage (void);
int get_temperature (void);
int get_temperature_accel (int accel);
//int get_status_accel (void);

void average_axel1(int x);
void average_axel2(int x);

#ifdef	__cplusplus
}
#endif

#endif	/* SYSTEM_H */


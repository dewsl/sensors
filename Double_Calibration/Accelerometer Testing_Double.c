/*!
 * \file  Senslope_MainSensorSoMS.c
 *
 *
 * \date September 17, 2014
 * \version Senslope Alpha 2.0
 *
 * Processor:      pic18f2585 \n
 * Compiler:       c18         \n
 *
 * Resources used:
 * 		Timers:		Timer0 - CAN timeout and SoMS timeout
 *					Timer1 - SoMS frequency capture
 *					Timer2 - not used
 *					Timer3 - Edge-triggered interrupt
 * 
 * 
 *
 *
 *
 *
 *
 *
 *
 */
#include <p18f2585.h>
#include <stdio.h>
#include <usart.h>
#include <timers.h>
#include <spi.h>
#include <delays.h>
#include <capture.h>
#include "system.h"
#include "can.h"


/*******************************************************************************
 *  Microcontroller configuration bits
 ******************************************************************************/

#pragma config OSC = IRCIO67	// Senslope_MainSensorOnly.hex OSC=IRCIO67	Internal Clock
								// Senslope_MainSensorSoMS.hex OSC=HS		Crystal Clock
#pragma config WDT = OFF		// Watchdog Timer disabled
#pragma config PBADEN = OFF		// PORTB Pins configured as Digital I/O
#pragma config LPT1OSC = OFF	// High power operation in Timer1
#pragma config XINST = OFF		// Extended instruction set disabled
#pragma config BBSIZ = 1024		// Boot block size = 1K words
#pragma config IESO = ON		// Oscillator switchover mode enabled
#pragma config FCMEN = ON		// Fail-safe clock monitor enabled

#pragma config CP1 = ON			// Code-protected
#pragma config CP0 = ON
#pragma config CP2 = ON

#pragma config CPD = OFF   		// Data EEPROM not code-protected
#pragma config CPB = OFF		// Boot block not code-protected

#pragma config WRT0 = OFF		// Not write-protected
#pragma config WRT1 = OFF
#pragma config WRT2 = OFF

#pragma config WRTC = OFF		// Configuration registers not write protected
#pragma config WRTB = OFF		// Boot block not write protected
#pragma config WRTD = OFF		// Data EEPROM not wite-protected

#pragma config EBTR0 = OFF		// Table not protected
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTRB = OFF


/*******************************************************************************
 *  Global variables
 ******************************************************************************/
#pragma udata data1
int i,timeout_counter,timestart,timeend,timeelapsed,nopulseindicator,waittime1,waittime2;				// Timeout variables
int x1_self,y1_self,z1_self,x2_self,y2_self,z2_self,x1stat,y1stat,z1stat,x2stat,y2stat,z2stat;			// Accelerometer self-test data
int x1data,y1data,z1data,x2data,y2data,z2data;
int x_range1,y_range1,z_range1,x_off1,y_off1,z_off1,x_range2,y_range2,z_range2,x_off2,y_off2,z_off2;	// Accelerometer EEPROM values
float xsum1,xsum2,ysum1,ysum2,zsum1,zsum2;																// Accelerometer averaging values
double x_cal,y_cal,z_cal;																				// Accelerometer calibrated data
#pragma udata

#pragma idata data2
int unique_nodeid;																						// Manufacturing node id
unsigned char xh1,xl1,yh1,yl1,zh1,zl1,xh2,xl2,yh2,yl2,zh2,zl2;											// Accelerometer raw data
double ax1_m1,ax1_m2,ax1_m3,ax1_m4,ax1_m5,ax1_m6,ax1_m7,ax1_m8,ax1_m9,ax1_m10,ax1_m11,ax1_m12,mult;		// Accelerometer matrix calibration parameters
double ax2_m1,ax2_m2,ax2_m3,ax2_m4,ax2_m5,ax2_m6,ax2_m7,ax2_m8,ax2_m9,ax2_m10,ax2_m11,ax2_m12;
unsigned int tot0,tot1,fshift,fshiftair,fshiftwater;													// Soil moisture raw data
unsigned char fshiftairl,fshiftairh,fshiftwaterl,fshiftwaterh;											// Soil moisture EEPROM values
int fcal,clk_stat = 20;																					// Soil moisture calibrated data and clock status
long adc_result, adcin;																					// ADC data
char string[64];																						// Print buffer
#pragma idata

CANDATA_EXTENDED canBuffer;


void main (void)
{

    unsigned char clock;
    unsigned char idh, idl;
    unsigned int status,command,canTransIden;
	int j = 0;

    /* OSCCON = 0X70 if not using crystal oscillator */
	OSCCON = 0x70;	// Set PIC oscillator to 8MHz

	OpenUSART(USART_TX_INT_OFF & USART_RX_INT_OFF & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH, 8);	// 8(internal) or 20(crystal) for 57600 baudrate
	TXREG=0;
	OpenTimer0(TIMER_INT_OFF & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);															// Initialize timer0 for CAN and SoMS timeout
	OpenTimer1(TIMER_INT_OFF & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_8 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF);						// Initialize timer1 for frequency capture in SoMS
	OpenSPI (SPI_FOSC_64, MODE_11, SMPEND);																						// Initialize SPI to access accelerometer registers	

	TRISB &= ~0x02;	// Set output pins to control enable of level shifters
	TRISB &= ~0x01;

	sprintf(string,(const far rom char *)"Starting...\n\r"); putsUSART(string);

	//soil_initialize();
	//if (DEBUG) {Delay10KTCYx(0); sprintf(string,(const far rom char *)"Soil moisture initialized\n\r"); putsUSART(string);}

	axel_initialize();
    if (DEBUG) {Delay10KTCYx(0); sprintf(string,(const far rom char *)"Accel1 initialized\n\r"); putsUSART(string);}
	axel2_initialize();
    if (DEBUG) {Delay10KTCYx(0); sprintf(string,(const far rom char *)"Accel2 initialized\n\r"); putsUSART(string);}
	
    can_initialize_extended(&canBuffer);
    if (DEBUG) {Delay10KTCYx(0); sprintf(string,(const far rom char *)"CAN initialized\n\r"); putsUSART(string);}   
       
	/* Get sensor initial values */

    idh = EEPROM_read(0x01,0x00);	// Load uniqued ID EEPROM values
	idl = EEPROM_read(0x01,0x01);
    unique_nodeid = (idh*256) + idl;
	if (DEBUG) { sprintf(string,(const far rom char *)"Node: %d\n\r", unique_nodeid); putsUSART(string); }

	get_axel1raw();
	if (DEBUG) { sprintf(string,(const far rom char *)"Accel1 Initial Raw: xh=%d\txl=%d\tyh=%d\tyl=%d\tzh=%d\tzl=%d\n\r", xh1, xl1, yh1, yl1, zh1, zl1); putsUSART(string); }
	get_axel2raw();
	if (DEBUG) { sprintf(string,(const far rom char *)"Accel2 Initial Raw: xh=%d\txl=%d\tyh=%d\tyl=%d\tzh=%d\tzl=%d\n\r", xh2, xl2, yh2, yl2, zh2, zl2); putsUSART(string); }

    /* Comment this out if Senslope_MainSensorOnly.hex */
    //clock = 0x03;					 // Enable external clock oscillator as primary oscillator
	//get_somsraw(clock);
	//if (tot1 < tot0)	resample_soms(clock);
   	//OSCCON = OSCCON | 0x02;
	//if (DEBUG) { sprintf(string,(const far rom char *)"SoMS Initial Raw (20MHz): tot0=%d\ttotl=%d\n\r", tot0, tot1); putsUSART(string); }
	//get_somscal();
	//if (DEBUG) { sprintf(string,(const far rom char *)"SoMS Initial Cal: fcal=%d\n\r", fcal); putsUSART(string); }

    while(1) {

            command = 0;
            status = 0;

            can_check_errors();
            status = can_check_for_datain_extended(&canBuffer);

            if(status)	// Command received
                command = can_process_commands_extended(&canBuffer,unique_nodeid);
			 
			/* If command < 99 -> broadcast command
			 * If command > 99 -> node-specific command
			 *
			 * Depending on the command, node will send back CAN frame
			 * Master-to-Node format:
			 * Node-specific	[msg identifier][idH][idL][data1][data2][data3][data4][data5]
			 * Broadcast		[msg identifier][data1][data2][data3][data4][data5][data6][data7]
			 */

             switch(command) {


				 /* CAN testing commands */

                 case 0:	// Command not for this node
                    	 break;

                 case 99:	// Test case for fastest return what was sent
                         //canBuffer.data1 = 12;
                         //canBuffer.data2 = 11;
                         //canBuffer.data3 = 13;
                         //canBuffer.data4 = 14;
                         //canBuffer.data5 = 15;
                         //canBuffer.data6 = 16;
                         //canBuffer.data7 = 16;
                         //canBuffer.data8 = 16;
                         canBuffer.dlc = 8;
                  		 break;

                 case 98:	// 1 second delay test case
                         Delay10KTCYx(200); //delay for 1 second
                         Delay10KTCYx(200); //delay for 1 second
                         Delay10KTCYx(200); //delay for 1 second
                         Delay10KTCYx(200); //delay for 1 second
                         Delay10KTCYx(200); //delay for 1 second

                         //canBuffer.data1 = 12;
                         //canBuffer.data2 = 11;
                         //canBuffer.data3 = 13;
                         //canBuffer.data4 = 14;
                         //canBuffer.data5 = 15;
                         //canBuffer.data6 = 16;
                         //canBuffer.data7 = 16;
                         //canBuffer.data8 = 16;
                         canBuffer.dlc = 8;
                  		 break;

                 /* Node-specific commands*/

                 case EEPROM_AXEL1_XYZ:
                     if (DEBUG) {sprintf(string,(const far rom char *)"EEPROM_AXEL1_XYZ\n\r"); putsUSART(string);}
                     
                     canTransIden = canBuffer.data4;
                     canBuffer.data2 = canTransIden;    // Transacation identifier
                     if(canTransIden == 1){
                         canBuffer.data3 = EEPROM_write(0x03,0x10,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x03,0x11,canBuffer.data6);
                         canBuffer.data5 = EEPROM_write(0x03,0x12,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x03,0x13,canBuffer.data8);
                         canBuffer.dlc = 6;
                     }else if(canTransIden == 2){
                         canBuffer.data3 = EEPROM_write(0x03,0x14,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x03,0x15,canBuffer.data6);
                         canBuffer.dlc = 4;
                     }else{ // Error case
                         status = 0;//dont respond
                         sprintf(string,(const far rom char *)"\n\rError in CAN transaction number\n\r"); putsUSART(string);
                     }
                     break;

                 case EEPROM_AXEL1_OFFSET:
                     if (DEBUG) {sprintf(string,(const far rom char *)"EEPROM_AXEL1_OFFSET\n\r"); putsUSART(string);}

                     canTransIden = canBuffer.data4;
                     canBuffer.data2 = canTransIden;    // Transaction identifier
                     if(canTransIden == 1){
                         canBuffer.data3 = EEPROM_write(0x03,0x20,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x03,0x21,canBuffer.data6);
                         canBuffer.data5 = EEPROM_write(0x03,0x22,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x03,0x23,canBuffer.data8);
                         canBuffer.dlc = 6;
                     }else if(canTransIden == 2){
                         canBuffer.data3 = EEPROM_write(0x03,0x24,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x03,0x25,canBuffer.data6);
                         canBuffer.dlc = 4;
                     }else{ //error case
                         status = 0;//dont respond
                         sprintf(string,(const far rom char *)"\n\rError in CAN transaction number\n\r"); putsUSART(string);
                     }
                     break;

                 case EEPROM_AXEL2_XYZ:
                     if (DEBUG) {sprintf(string,(const far rom char *)"EEPROM_AXEL2_XYZ\n\r"); putsUSART(string);}

                     canTransIden = canBuffer.data4;
                     canBuffer.data2 = canTransIden;    //Transaction identifier
                     if(canTransIden == 1){
                         canBuffer.data3 = EEPROM_write(0x03,0x30,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x03,0x31,canBuffer.data6);
                         canBuffer.data5 = EEPROM_write(0x03,0x32,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x03,0x33,canBuffer.data8);
                         canBuffer.dlc = 6;
                     }else if(canTransIden == 2){
                         canBuffer.data3 = EEPROM_write(0x03,0x34,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x03,0x35,canBuffer.data6);
                         canBuffer.dlc = 4;
                     }else{ //error case
                         status = 0;//dont respond
                         sprintf(string,(const far rom char *)"\n\rError in CAN transaction number\n\r"); putsUSART(string);
                     }
                     break;

                 case EEPROM_AXEL2_OFFSET:
                     if (DEBUG) {sprintf(string,(const far rom char *)"EEPROM_AXEL2_OFFSET\n\r"); putsUSART(string);}

                     canTransIden = canBuffer.data4;
                     canBuffer.data2 = canTransIden;    //Transaction identifier
                     if(canTransIden == 1){
                         canBuffer.data3 = EEPROM_write(0x03,0x40,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x03,0x41,canBuffer.data6);
                         canBuffer.data5 = EEPROM_write(0x03,0x42,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x03,0x43,canBuffer.data8);
                         canBuffer.dlc = 6;
                     }else if(canTransIden == 2){
                         canBuffer.data3 = EEPROM_write(0x03,0x44,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x03,0x45,canBuffer.data6);
                         canBuffer.dlc = 4;
                     }else{ //error case
                         status = 0;//dont respond
                         sprintf(string,(const far rom char *)"\n\rError in CAN transaction number\n\r"); putsUSART(string);
                     }
					break;

				case EEPROM_MATRIX_AXEL1:
					if (DEBUG) {sprintf(string,(const far rom char *)"EEPROM_MATRIX_AXEL1\n\r"); putsUSART(string);}
					canTransIden = canBuffer.data4;
                    canBuffer.data2 = canTransIden;    //Transaction identifier
					 if(canTransIden == 1){
                         canBuffer.data3 = EEPROM_write(0x02,0x10,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x02,0x11,canBuffer.data6);
                         canBuffer.data5 = EEPROM_write(0x02,0x12,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x02,0x13,canBuffer.data8);
                         canBuffer.dlc = 6;
                     }else if(canTransIden == 2){
                         canBuffer.data3 = EEPROM_write(0x02,0x14,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x02,0x15,canBuffer.data6);
						 canBuffer.data5 = EEPROM_write(0x02,0x20,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x02,0x21,canBuffer.data8);
						 canBuffer.dlc = 6;
					}else if(canTransIden == 3){
                         canBuffer.data3 = EEPROM_write(0x02,0x22,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x02,0x23,canBuffer.data6);
						 canBuffer.data5 = EEPROM_write(0x02,0x24,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x02,0x25,canBuffer.data8);
						 canBuffer.dlc = 6;
					}else if(canTransIden == 4){
                         canBuffer.data3 = EEPROM_write(0x02,0x30,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x02,0x31,canBuffer.data6);
						 canBuffer.data5 = EEPROM_write(0x02,0x32,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x02,0x33,canBuffer.data8);
						 canBuffer.dlc = 6;
					}else if(canTransIden == 5){
                         canBuffer.data3 = EEPROM_write(0x02,0x34,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x02,0x35,canBuffer.data6);
						 canBuffer.data5 = EEPROM_write(0x02,0x40,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x02,0x41,canBuffer.data8);
						 canBuffer.dlc = 6;
					}else if(canTransIden == 6){
                         canBuffer.data3 = EEPROM_write(0x02,0x42,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x02,0x43,canBuffer.data6);
						 canBuffer.data5 = EEPROM_write(0x02,0x44,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x02,0x45,canBuffer.data8);
						 canBuffer.dlc = 6;
                     }else{ //error case
                         status = 0;//dont respond
                         sprintf(string,(const far rom char *)"\n\rError in CAN transaction number\n\r"); putsUSART(string);
                     }
                     break;

				case EEPROM_MATRIX_AXEL2:
					if (DEBUG) {sprintf(string,(const far rom char *)"EEPROM_MATRIX_AXEL2\n\r"); putsUSART(string);}
					canTransIden = canBuffer.data4;
                    canBuffer.data2 = canTransIden;    //Transaction identifier
					 if(canTransIden == 1){
                         canBuffer.data3 = EEPROM_write(0x02,0x70,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x02,0x71,canBuffer.data6);
                         canBuffer.data5 = EEPROM_write(0x02,0x72,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x02,0x73,canBuffer.data8);
                         canBuffer.dlc = 6;
                     }else if(canTransIden == 2){
                         canBuffer.data3 = EEPROM_write(0x02,0x74,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x02,0x75,canBuffer.data6);
						 canBuffer.data5 = EEPROM_write(0x02,0x80,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x02,0x81,canBuffer.data8);
						 canBuffer.dlc = 6;
					}else if(canTransIden == 3){
                         canBuffer.data3 = EEPROM_write(0x02,0x82,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x02,0x83,canBuffer.data6);
						 canBuffer.data5 = EEPROM_write(0x02,0x84,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x02,0x85,canBuffer.data8);
						 canBuffer.dlc = 6;
					}else if(canTransIden == 4){
                         canBuffer.data3 = EEPROM_write(0x02,0x90,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x02,0x91,canBuffer.data6);
						 canBuffer.data5 = EEPROM_write(0x02,0x92,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x02,0x93,canBuffer.data8);
						 canBuffer.dlc = 6;
					}else if(canTransIden == 5){
                         canBuffer.data3 = EEPROM_write(0x02,0x94,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x02,0x95,canBuffer.data6);
						 canBuffer.data5 = EEPROM_write(0x02,0xA0,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x02,0xA1,canBuffer.data8);
						 canBuffer.dlc = 6;
					}else if(canTransIden == 6){
                         canBuffer.data3 = EEPROM_write(0x02,0xA2,canBuffer.data5);
                         canBuffer.data4 = EEPROM_write(0x02,0xA3,canBuffer.data6);
						 canBuffer.data5 = EEPROM_write(0x02,0xA4,canBuffer.data7);
                         canBuffer.data6 = EEPROM_write(0x02,0xA5,canBuffer.data8);
						 canBuffer.dlc = 6;
                     }else{ //error case
                         status = 0;//dont respond
                         sprintf(string,(const far rom char *)"\n\rError in CAN transaction number\n\r"); putsUSART(string);
                     }
                     break;
					 
                 case SOMS_AIR_OVER_TEN_20:
                     if (DEBUG) {sprintf(string,(const far rom char *)"Frequency Shift Air\n\r"); putsUSART(string);}

                     canBuffer.data2 = EEPROM_write(0x03,0x50,canBuffer.data4);
                     canBuffer.data3 = EEPROM_write(0x03,0x51,canBuffer.data5);
                     canBuffer.dlc = 3;
                     break;

                 case SOMS_WATER_OVER_TEN_20:
                     if (DEBUG) {sprintf(string,(const far rom char *)"Frequency Shift Air\n\r"); putsUSART(string);}

                     canBuffer.data2 = EEPROM_write(0x03,0x52,canBuffer.data4);
                     canBuffer.data3 = EEPROM_write(0x03,0x53,canBuffer.data5);
                     canBuffer.dlc = 3;
                     break;

                 case SOMS_AIR_OVER_TEN_8:
                     if (DEBUG) {sprintf(string,(const far rom char *)"Frequency Shift Air\n\r"); putsUSART(string);}

                     canBuffer.data2 = EEPROM_write(0x03,0x54,canBuffer.data4);
                     canBuffer.data3 = EEPROM_write(0x03,0x55,canBuffer.data5);
                     canBuffer.dlc = 3;
                     break;

                 case SOMS_WATER_OVER_TEN_8:
                     if (DEBUG) {sprintf(string,(const far rom char *)"Frequency Shift Air\n\r"); putsUSART(string);}

                     canBuffer.data2 = EEPROM_write(0x03,0x56,canBuffer.data4);
                     canBuffer.data3 = EEPROM_write(0x03,0x57,canBuffer.data5);
                     canBuffer.dlc = 3;
                     break;

                 /* Broadcast commands*/

                 case EEPROM_TESTING:	// Testing EEPROM
                     if (DEBUG) {sprintf(string,(const far rom char *)"EEPROM_TESTING\n\r"); putsUSART(string);}

                      canBuffer.data2 = EEPROM_write(0x03,0x60,canBuffer.data2);
                      canBuffer.data3 = EEPROM_write(0x03,0x61,canBuffer.data3);
                      canBuffer.data4 = EEPROM_write(0x03,0x62,canBuffer.data4);
                      canBuffer.data5 = EEPROM_write(0x03,0x63,canBuffer.data5);
                      canBuffer.data6 = EEPROM_write(0x03,0x64,canBuffer.data6);
                      canBuffer.data7 = EEPROM_write(0x03,0x65,canBuffer.data7);
                      canBuffer.dlc = 7;
                      break;

                 case PASS_AXEL1_RAW:	// Initial accel1 raw data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL1_RAW\n\r"); putsUSART(string);}

                     canBuffer.data2= xl1;
                     canBuffer.data3= xh1;
                     canBuffer.data4= yl1;
                     canBuffer.data5= yh1;
                     canBuffer.data6= zl1;
                     canBuffer.data7= zh1;
                     canBuffer.dlc = 7;
                     break;

                 case PASS_AXEL2_RAW: // Initial accel2 raw data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL2_RAW\n\r"); putsUSART(string);}

                     canBuffer.data2= xl2;
                     canBuffer.data3= xh2;
                     canBuffer.data4= yl2;
                     canBuffer.data5= yh2;
                     canBuffer.data6= zl2;
                     canBuffer.data7= zh2;
                     canBuffer.dlc = 7;
                     break;

                 case PASS_SOMS_RAW:	// Initial SoMS 20MHZ data	
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_SOMS_RAW\n\r"); putsUSART(string);}

                     canBuffer.data2 = (unsigned char)(tot0 & 0xFF);
                     canBuffer.data3 = (unsigned char)(tot0 >> 8);
                     canBuffer.data4 = (unsigned char)(tot1 & 0xFF);
                     canBuffer.data5 = (unsigned char)(tot1 >> 8);
                     canBuffer.dlc = 5;
                     break;

                 case PASS_AXEL1_CALIB: // Initial accel1 MINMAX_calibrated data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL1_CALIB\n\r"); putsUSART(string);}

                     get_axelcal(1);
                     canBuffer.data2= (unsigned char)((int)x_cal & 0xFF);
                     canBuffer.data3= (unsigned char)((int)x_cal >> 8);
                     canBuffer.data4= (unsigned char)((int)y_cal & 0xFF);
                     canBuffer.data5= (unsigned char)((int)y_cal >> 8);
                     canBuffer.data6= (unsigned char)((int)z_cal & 0xFF);
                     canBuffer.data7= (unsigned char)((int)z_cal >> 8);
                     canBuffer.dlc = 7;
                     break;

                 case PASS_AXEL2_CALIB: // Initial accel2 MINMAX_calibrated data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL2_CALIB\n\r"); putsUSART(string);}

                     get_axelcal(2);
                     canBuffer.data2= (unsigned char)((int)x_cal & 0xFF);
                     canBuffer.data3= (unsigned char)((int)x_cal >> 8);
                     canBuffer.data4= (unsigned char)((int)y_cal & 0xFF);
                     canBuffer.data5= (unsigned char)((int)y_cal >> 8);
                     canBuffer.data6= (unsigned char)((int)z_cal & 0xFF);
                     canBuffer.data7= (unsigned char)((int)z_cal >> 8);
                     canBuffer.dlc = 7;
                     break;

                 case PASS_SOMS_CALIB: // Initial SoMS 20MHZ calibrated data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_SOMS_CALIB\n\r"); putsUSART(string);}

                     canBuffer.data2= (unsigned char)(fcal & 0xFF);
                     canBuffer.data3= (unsigned char)(fcal >> 8);			
                     canBuffer.dlc = 3;
                     break;
                 
                 case PASS_AXEL1_RAW_NEW: // Updated accel1 raw data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL1_RAW_NEW\n\r"); putsUSART(string);}

                     get_axel1raw();
                     canBuffer.data2= xl1;
                     canBuffer.data3= xh1;
                     canBuffer.data4= yl1;
                     canBuffer.data5= yh1;
                     canBuffer.data6= zl1;
                     canBuffer.data7= zh1;
                     canBuffer.dlc = 7;
                     break;

                 case PASS_AXEL2_RAW_NEW: // Updated accel2 raw data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL2_RAW_NEW\n\r"); putsUSART(string);}

                     get_axel2raw();
                     canBuffer.data2= xl2;
                     canBuffer.data3= xh2;
                     canBuffer.data4= yl2;
                     canBuffer.data5= yh2;
                     canBuffer.data6= zl2;
                     canBuffer.data7= zh2;
                     canBuffer.dlc = 7;
                     break;
                     
                 case PASS_SOMS_RAW_NEW: // Updated SoMS 20MHz raw data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_SOMS_RAW_NEW\n\r"); putsUSART(string);}

                     clock = 0x03;
                     get_somsraw(clock);
                     if (tot1 < tot0){
                              resample_soms(clock);
							  sprintf(string,(const far rom char *)"Resample SoMS\n\r"); putsUSART(string);
                     }
                     OSCCON = OSCCON | 0x02;
                     canBuffer.data2 = (unsigned char)(tot0 & 0xFF);
                     canBuffer.data3 = (unsigned char)(tot0 >> 8);
                     canBuffer.data4 = (unsigned char)(tot1 & 0xFF);
                     canBuffer.data5 = (unsigned char)(tot1 >> 8);
                     canBuffer.dlc = 5;
                     break;

                 case PASS_AXEL1_CALIB_MATRIX: // Updated accel1 MATRIX_calibrated data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL1_CALIB_MATRIX\n\r"); putsUSART(string);}

                     get_axel1raw();
                     get_axel1cal_m();
                     if (DEBUG) {sprintf(string,(const far rom char *)"Axel1 data: %d\t%d\t%d\n\r", (int)x_cal,(int)y_cal,(int)z_cal); putsUSART(string);}

                     canBuffer.data2 = (unsigned char)((int)x_cal & 0xFF);
                     canBuffer.data3 = (unsigned char)((int)x_cal >> 8);
                     canBuffer.data4 = (unsigned char)((int)y_cal & 0xFF);
                     canBuffer.data5 = (unsigned char)((int)y_cal >> 8);
                     canBuffer.data6 = (unsigned char)((int)z_cal & 0xFF);
                     canBuffer.data7 = (unsigned char)((int)z_cal >> 8);
                     canBuffer.dlc = 7;
                     break;

                 case PASS_AXEL2_CALIB_MATRIX: // Updated accel2 MATRIX_calibrated data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL2_CALIB_MATRIX\n\r"); putsUSART(string);}

                     get_axel2raw();
                     get_axel2cal_m();
                     if (DEBUG) {sprintf(string,(const far rom char *)"Axel2 data: %d\t%d\t%d\n\r", (int)x_cal,(int)y_cal,(int)z_cal); putsUSART(string);}

                     canBuffer.data2 = (unsigned char)((int)x_cal & 0xFF);
                     canBuffer.data3 = (unsigned char)((int)x_cal >> 8);
                     canBuffer.data4 = (unsigned char)((int)y_cal & 0xFF);
                     canBuffer.data5 = (unsigned char)((int)y_cal >> 8);
                     canBuffer.data6 = (unsigned char)((int)z_cal & 0xFF);
                     canBuffer.data7 = (unsigned char)((int)z_cal >> 8);
                     canBuffer.dlc = 7;
                     break;

                 case PASS_AXEL1_CALIB_MINMAX:  // Updated accel1 MINMAX_calibrated data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL1_CALIB_MINMAX\n\r"); putsUSART(string);}

					 //get_axel1raw();
                     get_axelcal(1);
                     if (DEBUG) {sprintf(string,(const far rom char *)"Axel1 data: %d\t%d\t%d\n\r", (int)x_cal,(int)y_cal,(int)z_cal); putsUSART(string);}
                   
                     canBuffer.data2 = (unsigned char)((int)x_cal & 0xFF);
                     canBuffer.data3 = (unsigned char)((int)x_cal >> 8);
                     canBuffer.data4 = (unsigned char)((int)y_cal & 0xFF);
                     canBuffer.data5 = (unsigned char)((int)y_cal >> 8);
                     canBuffer.data6 = (unsigned char)((int)z_cal & 0xFF);
                     canBuffer.data7 = (unsigned char)((int)z_cal >> 8);
                     canBuffer.dlc = 7;
                     break;

                case PASS_AXEL2_CALIB_MINMAX: // Updated accel2 MINMAX_calibrated data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL2_CALIB_MINMAX\n\r"); putsUSART(string);}

					 //get_axel2raw();
                     get_axelcal(2);
                     if (DEBUG) {sprintf(string,(const far rom char *)"Axel2 data: %d\t%d\t%d\n\r", (int)x_cal,(int)y_cal,(int)z_cal); putsUSART(string);}
                    
                     canBuffer.data2 = (unsigned char)((int)x_cal & 0xFF);
                     canBuffer.data3 = (unsigned char)((int)x_cal >> 8);
                     canBuffer.data4 = (unsigned char)((int)y_cal & 0xFF);
                     canBuffer.data5 = (unsigned char)((int)y_cal >> 8);
                     canBuffer.data6 = (unsigned char)((int)z_cal & 0xFF);
                     canBuffer.data7 = (unsigned char)((int)z_cal >> 8);
                     canBuffer.dlc = 7;
                     break;

                 case PASS_SOMS_CALIB_NEW: // Updated SoMS 20MHz calibrated data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_SOMS_CALIB_NEW\n\r"); putsUSART(string);}

                     clock = 0x03;
                     get_somsraw(clock);
                     if (tot1 < tot0)		resample_soms(clock);
                     OSCCON = OSCCON | 0x02;
                     get_somscal();
                     canBuffer.data2= (unsigned char)(fcal & 0xFF);
                     canBuffer.data3= (unsigned char)(fcal >> 8);
                     canBuffer.dlc = 3;
                     break;
                     
                  case  PASS_SOMS_CALIB_NEW_8: // Updated SoMS 8MHz calibrated data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_SOMS_CALIB_NEW_8\n\r"); putsUSART(string);}

                     clock = 0x02;
                     get_somsraw(clock);
                     if (tot1 < tot0)		resample_soms(clock);
                     get_somscal();
                     canBuffer.data2= (unsigned char)(fcal & 0xFF);
                     canBuffer.data3= (unsigned char)(fcal >> 8);
                     canBuffer.dlc = 3;
                     break;

                 case PASS_AXEL1_RAW_AVE: // Average accel1 raw data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL1_RAW_AVE\n\r"); putsUSART(string);}

		     		 average_axel1(axel_sample);
                     canBuffer.data2= xl1;
                     canBuffer.data3= xh1;
                     canBuffer.data4= yl1;
                     canBuffer.data5= yh1;
                     canBuffer.data6= zl1;
                     canBuffer.data7= zh1;
                     canBuffer.dlc = 7;
                     break;

                 case PASS_AXEL2_RAW_AVE: // Average accel2 raw data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL2_RAW_AVE\n\r"); putsUSART(string);}

                     average_axel2(axel_sample);
                     canBuffer.data2= xl2;
                     canBuffer.data3= xh2;
                     canBuffer.data4= yl2;
                     canBuffer.data5= yh2;
                     canBuffer.data6= zl2;
                     canBuffer.data7= zh2;
                     canBuffer.dlc = 7;
                     break;

                 case  PASS_AXEL1_AXIS_STAT: // Self-test accel1 axis data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL1_AXIS_STAT\n\r"); putsUSART(string);}

                     if ((x1_self > 250) && (x1_self < 900))          x1stat = 1;
                     if ((y1_self > 250) && (y1_self < 900))          y1stat = 1;
                     if ((z1_self > 100) && (z1_self < 600))          z1stat = 1;

                     canBuffer.data2= x1stat;
                     canBuffer.data3= 0;
                     canBuffer.data4= y1stat;
                     canBuffer.data5= 0;
                     canBuffer.data6= z1stat;
                     canBuffer.dlc = 6;
                     break;

                 case PASS_AXEL1_SELFTEST_OUTPUT_CHANGE: // Self-test accel1 raw data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL1_SELFTEST_OUTPUT_CHANGE\n\r"); putsUSART(string);}

                     canBuffer.data2= (unsigned char)((int)x1_self & 0xFF);
                     canBuffer.data3= (unsigned char)((int)x1_self >> 8);
                     canBuffer.data4= (unsigned char)((int)y1_self & 0xFF);
                     canBuffer.data5= (unsigned char)((int)y1_self >> 8);
                     canBuffer.data6= (unsigned char)((int)z1_self & 0xFF);
                     canBuffer.data7= (unsigned char)((int)z1_self >> 8);
                     canBuffer.dlc = 7;
                     break;

                 case  PASS_AXEL2_AXIS_STAT: // Self-test accel2 axis data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL2_AXIS_STAT\n\r"); putsUSART(string);}

                     if ((x2_self > 250) && (x2_self < 900))        x2stat = 1;
                     if ((y2_self > 250) && (y2_self < 900))        y2stat = 1;
                     if ((z2_self > 100) && (z2_self < 600))        z2stat = 1;

                     canBuffer.data2= x2stat;
                     canBuffer.data3= 0;
                     canBuffer.data4= y2stat;
                     canBuffer.data5= 0;
                     canBuffer.data6= z2stat;
                     canBuffer.dlc = 6;
                     break;

                 case PASS_AXEL2_SELFTEST_OUTPUT_CHANGE: // Self-test accel2 raw data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL2_SELFTEST_OUTPUT_CHANGE\n\r"); putsUSART(string);}

                     canBuffer.data2= (unsigned char)((int)x2_self & 0xFF);
                     canBuffer.data3= (unsigned char)((int)x2_self >> 8);
                     canBuffer.data4= (unsigned char)((int)y2_self & 0xFF);
                     canBuffer.data5= (unsigned char)((int)y2_self >> 8);
                     canBuffer.data6= (unsigned char)((int)z2_self & 0xFF);
                     canBuffer.data7= (unsigned char)((int)z2_self >> 8);
                     canBuffer.dlc = 7;
                     break;

                 case SELF_TEST_ROUTINE: // Self-test operation
                     if (DEBUG) {sprintf(string,(const far rom char *)"SELF_TEST_ROUTINE\n\r"); putsUSART(string);}
                     /*case would respond with the same message as acknowledgement*/
                     x1stat = 0;
                     y1stat = 0;
                     z1stat = 0;

                     axel_initialize();				// Get accelerometer raw data
                     Delay1KTCYx(0);
                     get_axel1raw();
                     x1data = (xh1*256) + xl1;
                     y1data = (yh1*256) + yl1;
                     z1data = (zh1*256) + zl1;
                     axel_initialize_self();		// Get accelerometer self-test data
                     Delay1KTCYx(0);
                     get_axel1raw();
                     x2data = (xh1*256) + xl1;
                     y2data = (yh1*256) + yl1;
                     z2data = (zh1*256) + zl1;
                     x1_self = x2data - x1data;		// Get difference
                     y1_self = y2data - y1data;
                     z1_self = z1data - z2data;
                     axel_initialize();

                     x2stat = 0;
                     y2stat = 0;
                     z2stat = 0;

                     axel2_initialize();
                     Delay1KTCYx(0);
                     get_axel2raw();
                     x1data = (xh2*256) + xl2;
                     y1data = (yh2*256) + yl2;
                     z1data = (zh2*256) + zl2;
                     axel2_initialize_self();
                     Delay1KTCYx(0);
                     get_axel2raw();
                     x2data = (xh2*256) + xl2;
                     y2data = (yh2*256) + yl2;
                     z2data = (zh2*256) + zl2;
                     x2_self = x2data - x1data;
                     y2_self = y2data - y1data;
                     z2_self = z1data - z2data;
                     axel2_initialize();

                     canBuffer.dlc = 8;

                     if (DEBUG) {sprintf(string,(const far rom char *)"X1=%d\tY1=%d\tZ1=%d\tX2=%d\tY2=%d\tZ2=%d\t\n\r",x1_self,y1_self,z1_self,x2_self,y2_self,z2_self); putsUSART(string);}
                     break;

                 case PASS_SOMS_RAW_NEW_8: // Updated SoMS 8MHz raw data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_SOMS_RAW_NEW_8\n\r"); putsUSART(string);}

					 clock = 0x02;
                     get_somsraw(clock);
	                 if (tot1 < tot0)		resample_soms(clock);
                     canBuffer.data2 = (unsigned char)(tot0 & 0xFF);
                     canBuffer.data3 = (unsigned char)(tot0 >> 8);
                     canBuffer.data4 = (unsigned char)(tot1 & 0xFF);
                     canBuffer.data5 = (unsigned char)(tot1 >> 8);
                     canBuffer.dlc = 5;
                     break;
  
                 case PASS_AXEL1_RAW_ADC_NEW: // Updated accel1 raw and ADC data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL1_RAW_ADC_NEW\n\r"); putsUSART(string);}

                     get_axel1raw();
					 get_adcvoltage();	
                     canBuffer.data2= xl1;
                     canBuffer.data3= xh1;
                     canBuffer.data4= yl1;
                     canBuffer.data5= yh1;
                     canBuffer.data6= zl1;
                     canBuffer.data7= zh1;
					 canBuffer.data8= (unsigned char)(adcin-200);
                     canBuffer.dlc = 8;
                     break;

                 case PASS_AXEL2_RAW_ADC_NEW: // Updated accel2 raw and ADC data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL2_ADC_RAW_NEW\n\r"); putsUSART(string);}

                     get_axel2raw();
					 get_adcvoltage();	
                     canBuffer.data2= xl2;
                     canBuffer.data3= xh2;
                     canBuffer.data4= yl2;
                     canBuffer.data5= yh2;
                     canBuffer.data6= zl2;
                     canBuffer.data7= zh2;
					 canBuffer.data8= (unsigned char)(adcin-200);
                     canBuffer.dlc = 8;
                     break;
         
                 case PASS_AXEL1_ADC_CALIB_MINMAX:  // Updated accel1 MINMAX_calibrated and ADC data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL1_ADC_CALIB_MINMAX\n\r"); putsUSART(string);}

					 get_axel1raw();
                     get_axelcal(1);
					 get_adcvoltage();	
                     if (DEBUG) {sprintf(string,(const far rom char *)"Axel1 data: %d\t%d\t%d\n\r", (int)x_cal,(int)y_cal,(int)z_cal); putsUSART(string);}
                   
                     canBuffer.data2 = (unsigned char)((int)x_cal & 0xFF);
                     canBuffer.data3 = (unsigned char)((int)x_cal >> 8);
                     canBuffer.data4 = (unsigned char)((int)y_cal & 0xFF);
                     canBuffer.data5 = (unsigned char)((int)y_cal >> 8);
                     canBuffer.data6 = (unsigned char)((int)z_cal & 0xFF);
                     canBuffer.data7 = (unsigned char)((int)z_cal >> 8);
					 canBuffer.data8= (unsigned char)(adcin-200);
                     canBuffer.dlc = 8;
                     break;

                case PASS_AXEL2_ADC_CALIB_MINMAX: // Updated accel2 MINMAX_calibrated and ADC data
                     if (DEBUG) {sprintf(string,(const far rom char *)"PASS_AXEL2_ADC_CALIB_MINMAX\n\r"); putsUSART(string);}

					 get_axel2raw();
                     get_axelcal(2);
					 get_adcvoltage();	
                     if (DEBUG) {sprintf(string,(const far rom char *)"Axel2 data: %d\t%d\t%d\n\r", (int)x_cal,(int)y_cal,(int)z_cal); putsUSART(string);}
                    
                     canBuffer.data2 = (unsigned char)((int)x_cal & 0xFF);
                     canBuffer.data3 = (unsigned char)((int)x_cal >> 8);
                     canBuffer.data4 = (unsigned char)((int)y_cal & 0xFF);
                     canBuffer.data5 = (unsigned char)((int)y_cal >> 8);
                     canBuffer.data6 = (unsigned char)((int)z_cal & 0xFF);
                     canBuffer.data7 = (unsigned char)((int)z_cal >> 8);
					 canBuffer.data8= (unsigned char)(adcin-200);
                     canBuffer.dlc = 8;
                     break;
            
                 default:
                     sprintf(string,(const far rom char *)"\n\rCommand NOT FOUND\n\r"); putsUSART(string);
                 	 break;

             }

			/*
			 * Node-to-Master format:
			 * CAN frame ID = Unique ID
			 * Other data = depends on the command received by node
			 */

             if((status !=0) && (command != 0)) {  //check if received something
               
                 canBuffer.id = unique_nodeid;

                 if (DEBUG) {sprintf(string,(const far rom char *)"Transmitting..."); putsUSART(string);}
                 if (DEBUG) {sprintf(string,(const far rom char *)"Node %d: %d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n\r",
                         unique_nodeid, canBuffer.data1, canBuffer.data2, canBuffer.data3, canBuffer.data4,
                         canBuffer.data5, canBuffer.data6, canBuffer.data7,canBuffer.data8); putsUSART(string);}
				for ( j = 0; j<3; j++){
                 can_send_data_with_arb_repeat_extended(&canBuffer,TIMEOUT);
				}
             }
        }
}


//RAW DATA

/**
 * @brief       This function gathers accelerometer 1 data.
 *
 * This function collects accelerometer data by accessing its registers as indicated LIS3LV02DL datasheet.
 *
 */

void get_axel1raw (void)
{	

	PORTC |= 0x01;	// chip select axel1
	PORTB &= ~0x02;	// disable level shifter for axel2
	PORTB |= 0x01;	// able level shifter for axel1

	xl1 = axel_readbyte(0x28);	// read x
	xh1 = axel_readbyte(0x29);
	yl1 = axel_readbyte(0x2A);	// read y
	yh1 = axel_readbyte(0x2B);
	zl1 = axel_readbyte(0x2C);	// read z
	zh1 = axel_readbyte(0x2D);

}

/**
 * @brief       This function gathers accelerometer 2 data.
 *
 * This function collects accelerometer data by accessing its registers as indicated LIS3LV02DL datasheet.
 *
 */

void get_axel2raw (void)
{	

	PORTA |= 0x20;	// chip select axel2
	PORTB &= ~0x01;	// disable level shifter for axel1
	PORTB |= 0x02;	// able level shifter for axel2

	xl2 = axel2_readbyte(0x28);	// read x2
	xh2 = axel2_readbyte(0x29);
	yl2 = axel2_readbyte(0x2A);	// read y2
	yh2 = axel2_readbyte(0x2B);
	zl2 = axel2_readbyte(0x2C);	// read z2
	zh2 = axel2_readbyte(0x2D);

}


//AVERAGE DATA

/**
 * @brief       This function gathers accelerometer 1 data and compute their average
 *
 * This function collects accelerometer data with given number of samples. 
 * Sampling period is set to assure accel updated its registers as indicated by output data rate.
 * Then, it computes average value.
 * 
 * @param [in] x  Number of accelerometer samples
 *
 */

void average_axel1(int x)
{	xsum1 = 0;
	ysum1 = 0;
	zsum1 = 0;
	for ( i=0; i<x;)
	{
            Delay1KTCYx(AXEL_WAITTIME);
            PORTC |= 0x01;	// chip select axel1
            PORTB &= ~0x02;	// disable level shifter for axel2
            PORTB |= 0x01;	// able level shifter for axel1

            xl1 = axel_readbyte(0x28);	// read x
            xh1 = axel_readbyte(0x29);
            yl1 = axel_readbyte(0x2A);	// read y
            yh1 = axel_readbyte(0x2B);
            zl1 = axel_readbyte(0x2C);	// read z
            zh1 = axel_readbyte(0x2D);

            xsum1 = xl1+(xh1*256) + xsum1;
            ysum1 = yl1+(yh1*256) + ysum1;
            zsum1 = zl1+(zh1*256) + zsum1;
            i++;

	}
	xsum1 = xsum1/x;
	ysum1 = ysum1/x;
	zsum1 = zsum1/x;

	xl1 = ((int)xsum1 & 0xFF);
	xh1 = ((int)xsum1 >> 8);
	yl1 = ((int)ysum1 & 0xFF);
	yh1 = ((int)ysum1 >> 8);
	zl1 = ((int)zsum1 & 0xFF);
	zh1 = ((int)zsum1 >> 8);
	

}

/**
 * @brief       This function gathers accelerometer 2 data and compute their average
 *
 * This function collects accelerometer data with given number of samples. 
 * Sampling period is set to assure accel updated its registers as indicated by output data rate.
 * Then, it computes average value.
 * 
 * @param [in] x  Number of accelerometer samples
 *
 */

void average_axel2(int x)
{	xsum2 = 0;
	ysum2 = 0;
	zsum2 = 0;
	for ( i=0; i<x;)
	{
            Delay1KTCYx(AXEL_WAITTIME);
            PORTA |= 0x20;	// chip select axel2
            PORTB &= ~0x01;	// disable level shifter for axel1
            PORTB |= 0x02;	// able level shifter for axel2

            xl2 = axel2_readbyte(0x28);	// read x2
            xh2 = axel2_readbyte(0x29);
            yl2 = axel2_readbyte(0x2A);	// read y2
            yh2 = axel2_readbyte(0x2B);
            zl2 = axel2_readbyte(0x2C);	// read z2
            zh2 = axel2_readbyte(0x2D);

            xsum2 = xl2+(xh2*256) + xsum2;
            ysum2 = yl2+(yh2*256) + ysum2;
            zsum2 = zl2+(zh2*256) + zsum2;
            i++;
	}
	xsum2 = xsum2/x;
	ysum2 = ysum2/x;
	zsum2 = zsum2/x;

	xl2 = ((int)xsum2 & 0xFF);
	xh2 = ((int)xsum2 >> 8);
	yl2 = ((int)ysum2 & 0xFF);
	yh2 = ((int)ysum2 >> 8);
	zl2 = ((int)zsum2 & 0xFF);
	zh2 = ((int)zsum2 >> 8);
	

}


//CALIBRATED DATA

/**
 * @brief       This function calculates calibrated accelerometer data using MINMAX calibration
 *
 * This function should be called after calling get_axel1raw, get_axel2raw, average_axel1 or average_axel2.
 * The function assumes xl2,yl2,zl2,xh2,yh2,zh2 or xl1,yl1,zl1,xh1,yh1 and zh1 already have values. 
 *
 * @param [in]  axel  Accelerometer to be calibrated
 *
 * Ex:
 * get_axel2raw();
 * get_axelcal(2); // This will replace the values of x_cal,y_cal,z_cal to the calibrated values for accelerometer 2
 *
 */

void get_axelcal (int axel)
{
	unsigned char xh_mem,xl_mem,yh_mem,yl_mem,zh_mem,zl_mem;		// Accelerometer EEPROM data
	unsigned char xh_off,xl_off,yh_off,yl_off,zh_off,zl_off;
	int x_range,y_range,z_range,x_off,y_off,z_off;					// Accelerometer EEPROM values
	int	xdata,ydata,zdata;
	float xrange_fl,yrange_fl,zrange_fl,xoff_fl,yoff_fl,zoff_fl;	// Calibrated accelerometer computation variables
	float xdata_ten,ydata_ten,zdata_ten;

	/* Initialize all temporary variables to zero */
	xh_mem = 0; xl_mem = 0; yh_mem = 0; yl_mem = 0; zh_mem = 0; zl_mem = 0;
	xh_off = 0; xl_off = 0; yh_off = 0; yl_off = 0; zh_off = 0; zl_off = 0;
	xrange_fl = 0; yrange_fl = 0; zrange_fl = 0; xoff_fl = 0; yoff_fl = 0; zoff_fl = 0;
	x_range = 0; y_range = 0; z_range = 0; x_off = 0; y_off = 0; z_off = 0;
	xdata = 0; ydata = 0; zdata = 0;
	xdata_ten = 0; ydata_ten = 0; zdata_ten = 0;

	if (axel == 1)
	{
		xl_mem = EEPROM_read(0x03,0x10);	// Load EEPROM values
		xh_mem = EEPROM_read(0x03,0x11);
		yl_mem = EEPROM_read(0x03,0x12);
		yh_mem = EEPROM_read(0x03,0x13);
		zl_mem = EEPROM_read(0x03,0x14);
		zh_mem = EEPROM_read(0x03,0x15);
		xl_off = EEPROM_read(0x03,0x20);
		xh_off = EEPROM_read(0x03,0x21);
		yl_off = EEPROM_read(0x03,0x22);
		yh_off = EEPROM_read(0x03,0x23);
		zl_off = EEPROM_read(0x03,0x24);
		zh_off = EEPROM_read(0x03,0x25);

		xdata = (xh1*256) + xl1;			// Reconstruct data
		ydata = (yh1*256) + yl1;
		zdata = (zh1*256) + zl1;
	}
	else if (axel == 2)
	{
		xl_mem = EEPROM_read(0x03,0x30);	// Load EEPROM values
		xh_mem = EEPROM_read(0x03,0x31);
		yl_mem = EEPROM_read(0x03,0x32);
		yh_mem = EEPROM_read(0x03,0x33);
		zl_mem = EEPROM_read(0x03,0x34);
		zh_mem = EEPROM_read(0x03,0x35);
		xl_off = EEPROM_read(0x03,0x40);
		xh_off = EEPROM_read(0x03,0x41);
		yl_off = EEPROM_read(0x03,0x42);
		yh_off = EEPROM_read(0x03,0x43);
		zl_off = EEPROM_read(0x03,0x44);
		zh_off = EEPROM_read(0x03,0x45);

		xdata = (xh2*256) + xl2;			// Reconstruct data
		ydata = (yh2*256) + yl2;
		zdata = (zh2*256) + zl2;
	}
	x_range = (xh_mem*256) + xl_mem;		// Reconstruct range values
	y_range = (yh_mem*256) + yl_mem;
	z_range = (zh_mem*256) + zl_mem;

	xrange_fl = x_range;					// Range converted to float
	yrange_fl = y_range;
	zrange_fl = z_range;

	x_off = (xh_off*256) + xl_off;			// Reconstuct offset values
	y_off = (yh_off*256) + yl_off;
	z_off = (zh_off*256) + zl_off;

	xoff_fl = x_off*10.00;					// Offset*10 converted to float
	yoff_fl = y_off*10.00;
	zoff_fl = z_off*10.00;

	xdata_ten = xdata*10.00;				// Data*10
	ydata_ten = ydata*10.00;
	zdata_ten = zdata*10.00;

	x_cal = ( ( (xdata_ten - xoff_fl) /xrange_fl )*RESOLUTION );	// Calibration equation
	y_cal = ( ( (ydata_ten - yoff_fl) /yrange_fl )*RESOLUTION );
	z_cal = ( ( (zdata_ten - zoff_fl) /zrange_fl )*RESOLUTION );
}

/**
 * @brief       This function calculates calibrated accelerometer 1 data using MATRIX calibration
 *
 * This function should be called after calling get_axel2raw or average_axel2.
 * The function assumes xl1,yl1,zl1,xh1,yh1,and zh1 already have values.
 * Implementation of calibration as suggested by App Note 3182 by ST Electronics.
 *
 * Currently NOT used due to undesirable output.
 */


/*
void get_axel1cal_m()
{

	int var1,var2,var3,var4,var5,var6,var7,var8,mult1,mult2,xdata,ydata,zdata;  // declaration of temporary variables
	float xdata_ten,ydata_ten,zdata_ten;

	xdata = 0; ydata = 0; zdata = 0;											// initialize variables to 0
	xdata_ten = 0.0; ydata_ten = 0.0; zdata_ten = 0.0;
	x_cal = 0; y_cal = 0; z_cal = 0;

	mult1 = MULTIPLIER1;														
	mult2 = MULTIPLIER2;

	xdata = (xh1*256) + xl1;													// reconstruct accel data
	ydata = (yh1*256) + yl1;
	zdata = (zh1*256) + zl1;

	xdata_ten = xdata*10.00;													// multiply by 10 and convert to float
	ydata_ten = ydata*10.00;
	zdata_ten = zdata*10.00;

	var1 = 0; var2 = 0; var3 = 0; var4 = 0; var5 = 0; var6 = 0; var7 = 0; var8 = 0;	//initialize all temporary variables to 0 before usage

	var1 = EEPROM_read(0x02,0x10); // ax1_m1 low byte
	var2 = EEPROM_read(0x02,0x11); // ax1_m1 high byte
	var3 = EEPROM_read(0x02,0x12); // ax1_m2 low byte
	var4 = EEPROM_read(0x02,0x13); // ax1_m2 high byte
	var5 = EEPROM_read(0x02,0x14); // ax1_m3 low byte
	var6 = EEPROM_read(0x02,0x15); // ax1_m3 high byte
	var7 = EEPROM_read(0x02,0x40); // ax1_m10 low byte
	var8 = EEPROM_read(0x02,0x41); // ax1_m10 high byte

	ax1_m1 = ((((var2*256)+var1)*1.00)/(mult1*1.00));							// compute for the multipliers for x y and z
	ax1_m2 = ((((var4*256)+var3)*1.00)/(mult1*1.00));
	ax1_m3 = ((((var6*256)+var5)*1.00)/(mult1*1.00));
	ax1_m10 = ((((var8*256)+var7)*1.00)/(mult2*1.00));
																				
																				// if DEBUG mode is enabled print computed values
	if (DEBUG) {sprintf(string,(const far rom char *)"x1data y1data z1data %d\t%d\t%d\t\n\r", x1data,y1data,z1data); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"float x1data y1data z1data%d\t%d\t%d\t\n\r", (int)xdata_ten,(int)ydata_ten,(int)zdata_ten); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"ax1_m1 ax1_m2 ax1_m3 %d\t%d\t%d\t\n\r",(int)ax1_m1,(int)ax1_m2,(int)ax1_m3); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v1 v2 v3 %d\t%d\t%d\t\n\r", var1,var2,var3); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v4 v5 v6 %d\t%d\t%d\t\n\r", var4,var5,var6); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v7 v8    %d\t%d\t\n\r", var7,var8); putsUSART(string);}

	x_cal =( ( (ax1_m1*xdata_ten)+(ax1_m2*ydata_ten)+(ax1_m3*zdata_ten)+(ax1_m10*10.00))/ 10.00);	//compute the calibrated x value of accel 1

	var1 = 0; var2 = 0; var3 = 0; var4 = 0; var5 = 0; var6 = 0; var7 = 0; var8 = 0; //initialize all temporary variables to 0 before usage

	var1 = EEPROM_read(0x02,0x20); // ax1_m1 low byte
	var2 = EEPROM_read(0x02,0x21); // ax1_m1 high byte
	var3 = EEPROM_read(0x02,0x22); // ax1_m2 low byte
	var4 = EEPROM_read(0x02,0x23); // ax1_m2 high byte
	var5 = EEPROM_read(0x02,0x24); // ax1_m3 low byte
	var6 = EEPROM_read(0x02,0x25); // ax1_m3 high byte
	var7 = EEPROM_read(0x02,0x42); // ax1_m10 low byte
	var8 = EEPROM_read(0x02,0x43); // ax1_m10 high byte

	ax1_m4 = ((((var2*256)+var1)*1.00)/(mult1*1.00));
	ax1_m5 = ((((var4*256)+var3)*1.00)/(mult1*1.00));
	ax1_m6 = ((((var6*256)+var5)*1.00)/(mult1*1.00));
	ax1_m11 = ((((var8*256)+var7)*1.00)/(mult2*1.00));

	if (DEBUG) {sprintf(string,(const far rom char *)"v1 v2 v3 %d\t%d\t%d\t\n\r", var1,var2,var3); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v4 v5 v6 %d\t%d\t%d\t\n\r", var4,var5,var6); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v7 v8    %d\t%d\t\n\r", var7,var8); putsUSART(string);}

	y_cal =(((ax1_m4*xdata_ten) + (ax1_m5*ydata_ten) + (ax1_m6*zdata_ten) + (ax1_m11*10.00))/10.00);	//compute the calibrated y value of accel 1

	var1 = 0; var2 = 0; var3 = 0; var4 = 0; var5 = 0; var6 = 0; var7 = 0; var8 = 0;

	var1 = EEPROM_read(0x02,0x30); // ax1_m1 low byte
	var2 = EEPROM_read(0x02,0x31); // ax1_m1 high byte
	var3 = EEPROM_read(0x02,0x32); // ax1_m2 low byte
	var4 = EEPROM_read(0x02,0x33); // ax1_m2 high byte
	var5 = EEPROM_read(0x02,0x34); // ax1_m3 low byte
	var6 = EEPROM_read(0x02,0x35); // ax1_m3 high byte
	var7 = EEPROM_read(0x02,0x44); // ax1_m10 low byte
	var8 = EEPROM_read(0x02,0x45); // ax1_m10 high byte

	ax1_m7 = ((((var2*256)+var1)*1.00)/(mult1*1.00));
	ax1_m8 = ((((var4*256)+var3)*1.00)/(mult1*1.00));
	ax1_m9 = ((((var6*256)+var5)*1.00)/(mult1*1.00));
	ax1_m12 = ((((var8*256)+var7)*1.00)/(mult2*1.00));

	if (DEBUG) {sprintf(string,(const far rom char *)"v1 v2 v3 %d\t%d\t%d\t\n\r", var1,var2,var3); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v4 v5 v6 %d\t%d\t%d\t\n\r", var4,var5,var6); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v7 v8    %d\t%d\t\n\r", var7,var8); putsUSART(string);}

	z_cal = (((ax1_m7 * xdata_ten)+(ax1_m8 * ydata_ten)+(ax1_m9 * zdata_ten) +(ax1_m12 * 10.00))/10.00);	//compute the calibrated z value of accel 1

	xdata = 0; ydata = 0; zdata = 0;
	xdata_ten = 0.0; ydata_ten = 0.0; zdata_ten = 0.0;

}
*/
void get_axel1cal_m()
{

//	int var1,var2,var3,var4,var5,var6,mult1,mult2,xdata,ydata,zdata;  // declaration of temporary variables
	int var1,var2,var3,var4,var5,var6,xdata,ydata,zdata;  // declaration of temporary variables
	double xdata_ten,ydata_ten,zdata_ten;
	long int mult1,mult2;

	xdata = 0; ydata = 0; zdata = 0;											// initialize variables to 0
	xdata_ten = 0.0; ydata_ten = 0.0; zdata_ten = 0.0;
	x_cal = 0; y_cal = 0; z_cal = 0;

	mult1 = MULTIPLIER1;														
	mult2 = MULTIPLIER2;

	xdata = (xh1*256) + xl1;													// reconstruct accel data
	ydata = (yh1*256) + yl1;
	zdata = (zh1*256) + zl1;

	xdata_ten = xdata*1.00;														// convert to float
	ydata_ten = ydata*1.00;
	zdata_ten = zdata*1.00;

	var1 = 0; var2 = 0; var3 = 0; var4 = 0; var5 = 0; var6 = 0;	//initialize all temporary variables to 0 before usage

	var1 = EEPROM_read(0x02,0x40); // ax1_m10 low byte
	var2 = EEPROM_read(0x02,0x41); // ax1_m10 high byte
	var3 = EEPROM_read(0x02,0x42); // ax1_m11 low byte
	var4 = EEPROM_read(0x02,0x43); // ax1_m11 high byte
	var5 = EEPROM_read(0x02,0x44); // ax1_m12 low byte
	var6 = EEPROM_read(0x02,0x45); // ax1_m12 high byte

	ax1_m10 = ((((var2*256)+var1)*1.00)/(mult2*1.00));							// recover offset values and convert to float
	ax1_m11 = ((((var4*256)+var3)*1.00)/(mult2*1.00));
	ax1_m12 = ((((var6*256)+var5)*1.00)/(mult2*1.00));

	xdata_ten = xdata_ten - ax1_m10;											// subtract offsets from x,y,and z
	ydata_ten = ydata_ten - ax1_m11;
	zdata_ten = zdata_ten - ax1_m12;

	var1 = 0; var2 = 0; var3 = 0; var4 = 0; var5 = 0; var6 = 0;

	var1 = EEPROM_read(0x02,0x10); // ax1_m1 low byte
	var2 = EEPROM_read(0x02,0x11); // ax1_m1 high byte
	var3 = EEPROM_read(0x02,0x12); // ax1_m2 low byte
	var4 = EEPROM_read(0x02,0x13); // ax1_m2 high byte
	var5 = EEPROM_read(0x02,0x14); // ax1_m3 low byte
	var6 = EEPROM_read(0x02,0x15); // ax1_m3 high byte

	ax1_m1 = ((((var2*256)+var1)*1.00)/(mult1*1.00));							// compute for the multipliers for x y and z
	ax1_m2 = ((((var4*256)+var3)*1.00)/(mult1*1.00));
	ax1_m3 = ((((var6*256)+var5)*1.00)/(mult1*1.00));
																		
																				// if DEBUG mode is enabled print computed values
	if (DEBUG) {sprintf(string,(const far rom char *)"x1data y1data z1data %d\t%d\t%d\t\n\r", x1data,y1data,z1data); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"float x1data y1data z1data%d\t%d\t%d\t\n\r", (int)xdata_ten,(int)ydata_ten,(int)zdata_ten); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"ax1_m1 ax1_m2 ax1_m3 %d\t%d\t%d\t\n\r",(int)ax1_m1,(int)ax1_m2,(int)ax1_m3); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v1 v2 v3 %d\t%d\t%d\t\n\r", var1,var2,var3); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v4 v5 v6 %d\t%d\t%d\t\n\r", var4,var5,var6); putsUSART(string);}
	//if (DEBUG) {sprintf(string,(const far rom char *)"v7 v8    %d\t%d\t\n\r", var7,var8); putsUSART(string);}

	//x_cal =(((ax1_m1*xdata_ten)+(ax1_m2*ydata_ten)+(ax1_m3*zdata_ten))*RESOLUTION);	//compute the calibrated x value of accel 1 
	x_cal =(((ax1_m1*xdata_ten)+(ax1_m2*ydata_ten)+(ax1_m3*zdata_ten)));

	var1 = 0; var2 = 0; var3 = 0; var4 = 0; var5 = 0; var6 = 0;  //initialize all temporary variables to 0 before usage

	var1 = EEPROM_read(0x02,0x20); // ax1_m1 low byte
	var2 = EEPROM_read(0x02,0x21); // ax1_m1 high byte
	var3 = EEPROM_read(0x02,0x22); // ax1_m2 low byte
	var4 = EEPROM_read(0x02,0x23); // ax1_m2 high byte
	var5 = EEPROM_read(0x02,0x24); // ax1_m3 low byte
	var6 = EEPROM_read(0x02,0x25); // ax1_m3 high byte

	ax1_m4 = ((((var2*256)+var1)*1.00)/(mult1*1.00));
	ax1_m5 = ((((var4*256)+var3)*1.00)/(mult1*1.00));
	ax1_m6 = ((((var6*256)+var5)*1.00)/(mult1*1.00));


	if (DEBUG) {sprintf(string,(const far rom char *)"v1 v2 v3 %d\t%d\t%d\t\n\r", var1,var2,var3); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v4 v5 v6 %d\t%d\t%d\t\n\r", var4,var5,var6); putsUSART(string);}

	//y_cal =(((ax1_m4*xdata_ten) + (ax1_m5*ydata_ten) + (ax1_m6*zdata_ten))*RESOLUTION);	//compute the calibrated y value of accel 1
	y_cal =(((ax1_m4*xdata_ten) + (ax1_m5*ydata_ten) + (ax1_m6*zdata_ten)));
	var1 = 0; var2 = 0; var3 = 0; var4 = 0; var5 = 0; var6 = 0;

	var1 = EEPROM_read(0x02,0x30); // ax1_m1 low byte
	var2 = EEPROM_read(0x02,0x31); // ax1_m1 high byte
	var3 = EEPROM_read(0x02,0x32); // ax1_m2 low byte
	var4 = EEPROM_read(0x02,0x33); // ax1_m2 high byte
	var5 = EEPROM_read(0x02,0x34); // ax1_m3 low byte
	var6 = EEPROM_read(0x02,0x35); // ax1_m3 high byte

	ax1_m7 = ((((var2*256)+var1)*1.00)/(mult1*1.00));
	ax1_m8 = ((((var4*256)+var3)*1.00)/(mult1*1.00));
	ax1_m9 = ((((var6*256)+var5)*1.00)/(mult1*1.00));


	if (DEBUG) {sprintf(string,(const far rom char *)"v1 v2 v3 %d\t%d\t%d\t\n\r", var1,var2,var3); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v4 v5 v6 %d\t%d\t%d\t\n\r", var4,var5,var6); putsUSART(string);}

	//z_cal = (((ax1_m7 * xdata_ten)+(ax1_m8 * ydata_ten)+(ax1_m9 * zdata_ten))*RESOLUTION);	//compute the calibrated z value of accel 1
	z_cal = ((ax1_m7 * xdata_ten)+(ax1_m8 * ydata_ten)+(ax1_m9 *zdata_ten));

	xdata = 0; ydata = 0; zdata = 0;
	xdata_ten = 0.0; ydata_ten = 0.0; zdata_ten = 0.0;

}

/**
 * @brief       This function calculates calibrated accelerometer 2 data using MATRIX calibration
 *
 * This function should be called after calling get_axel2raw or average_axel2.
 * The function assumes xl2,yl2,zl2,xh2,yh2,and zh2 already have values.
 * Implementation of calibration as suggested by App Note 3182 by ST Electronics.
 *
 * Currently NOT used due to undesirable output.
 */

void get_axel2cal_m()
{
	/*
	int var1,var2,var3,var4,var5,var6,var7,var8,mult1,mult2,xdata,ydata,zdata;		// declaration of temporary variables
	float xdata_ten,ydata_ten,zdata_ten;
	*/
	int var1,var2,var3,var4,var5,var6,xdata,ydata,zdata;  // declaration of temporary variables
	double xdata_ten,ydata_ten,zdata_ten;
	long int mult1,mult2;

	xdata = 0; ydata = 0; zdata = 0;												// initialize variables to 0
	xdata_ten = 0.0; ydata_ten = 0.0; zdata_ten = 0.0;
	x_cal = 0; y_cal = 0; z_cal = 0;

	mult1 = MULTIPLIER1;														
	mult2 = MULTIPLIER2;

	xdata = (xh2*256) + xl2;														// reconstruct accel data
	ydata = (yh2*256) + yl2;
	zdata = (zh2*256) + zl2;

	xdata_ten = xdata*1.00;															//convert to float
	ydata_ten = ydata*1.00;
	zdata_ten = zdata*1.00;

	var1 = 0; var2 = 0; var3 = 0; var4 = 0; var5 = 0; var6 = 0;						//initialize all temporary variables to 0 before usage

	var1 = EEPROM_read(0x02,0xA0); // ax2_m10 low byte
	var2 = EEPROM_read(0x02,0xA1); // ax2_m10 high byte
	var3 = EEPROM_read(0x02,0xA2); // ax2_m11 low byte
	var4 = EEPROM_read(0x02,0xA3); // ax2_m11 high byte
	var5 = EEPROM_read(0x02,0xA4); // ax1_m12 low byte
	var6 = EEPROM_read(0x02,0xA5); // ax1_m12 high byte

	ax2_m10 = ((((var2*256)+var1)*1.00)/(mult2*1.00));								// recover offset values and convert to float
	ax2_m11 = ((((var4*256)+var3)*1.00)/(mult2*1.00));
	ax2_m12 = ((((var6*256)+var5)*1.00)/(mult2*1.00));

	xdata_ten = xdata_ten - ax2_m10;												// subtract offsets from x,y,and z
	ydata_ten = ydata_ten - ax2_m11;
	zdata_ten = zdata_ten - ax2_m12;

	var1 = 0; var2 = 0; var3 = 0; var4 = 0; var5 = 0; var6 = 0;						// clear variables

	var1 = EEPROM_read(0x02,0x70); // ax2_m1 low byte
	var2 = EEPROM_read(0x02,0x71); // ax2_m1 high byte
	var3 = EEPROM_read(0x02,0x72); // ax2_m2 low byte
	var4 = EEPROM_read(0x02,0x73); // ax2_m2 high byte
	var5 = EEPROM_read(0x02,0x74); // ax2_m3 low byte
	var6 = EEPROM_read(0x02,0x75); // ax2_m3 high byte

	ax2_m1 = ((((var2*256)+var1)*1.00)/(mult1*1.00));								// compute for the multipliers for x y and z
	ax2_m2 = ((((var4*256)+var3)*1.00)/(mult1*1.00));
	ax2_m3 = ((((var6*256)+var5)*1.00)/(mult1*1.00));

	if (DEBUG) {sprintf(string,(const far rom char *)"x2data y2data z2data %d\t%d\t%d\t\n\r", x2data,y2data,z2data); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"float x2data y2data z2data %d\t%d\t%d\t\n\r", (int)xdata_ten,(int)ydata_ten,(int)zdata_ten); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"ax2_m1 ax2_m2 ax2_m3 %d\t%d\t%d\t\n\r",(int)ax2_m1,(int)ax2_m2,(int)ax2_m3); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v1 v2 v3 %d\t%d\t%d\t\n\r", var1,var2,var3); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v4 v5 v6 %d\t%d\t%d\t\n\r", var4,var5,var6); putsUSART(string);}

	x_cal =((ax2_m1*xdata_ten)+(ax2_m2*ydata_ten)+(ax2_m3*zdata_ten));	//compute the calibrated x value of accel 2

	var1 = 0; var2 = 0; var3 = 0; var4 = 0; var5 = 0; var6 = 0; 

	var1 = EEPROM_read(0x02,0x80); // ax2_m4 low byte
	var2 = EEPROM_read(0x02,0x81); // ax2_m4 high byte
	var3 = EEPROM_read(0x02,0x82); // ax2_m5 low byte
	var4 = EEPROM_read(0x02,0x83); // ax2_m5 high byte
	var5 = EEPROM_read(0x02,0x84); // ax2_m6 low byte
	var6 = EEPROM_read(0x02,0x85); // ax2_m6 high byte

	ax2_m4 = ((((var2*256)+var1)*1.00)/(mult1*1.00));
	ax2_m5 = ((((var4*256)+var3)*1.00)/(mult1*1.00));
	ax2_m6 = ((((var6*256)+var5)*1.00)/(mult1*1.00));

	if (DEBUG) {sprintf(string,(const far rom char *)"v1 v2 v3 %d\t%d\t%d\t\n\r", var1,var2,var3); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v4 v5 v6 %d\t%d\t%d\t\n\r", var4,var5,var6); putsUSART(string);}

	y_cal =((ax2_m4*xdata_ten) + (ax2_m5*ydata_ten) + (ax2_m6*zdata_ten));	//compute the calibrated y value of accel 2

	var1 = 0; var2 = 0; var3 = 0; var4 = 0; var5 = 0; var6 = 0;

	var1 = EEPROM_read(0x02,0x90); // ax1_m1 low byte
	var2 = EEPROM_read(0x02,0x91); // ax1_m1 high byte
	var3 = EEPROM_read(0x02,0x92); // ax1_m2 low byte
	var4 = EEPROM_read(0x02,0x93); // ax1_m2 high byte
	var5 = EEPROM_read(0x02,0x94); // ax1_m3 low byte
	var6 = EEPROM_read(0x02,0x95); // ax1_m3 high byte

	ax2_m7 = ((((var2*256)+var1)*1.00)/(mult1*1.00));
	ax2_m8 = ((((var4*256)+var3)*1.00)/(mult1*1.00));
	ax2_m9 = ((((var6*256)+var5)*1.00)/(mult1*1.00));

	if (DEBUG) {sprintf(string,(const far rom char *)"v1 v2 v3 %d\t%d\t%d\t\n\r", var1,var2,var3); putsUSART(string);}
	if (DEBUG) {sprintf(string,(const far rom char *)"v4 v5 v6 %d\t%d\t%d\t\n\r", var4,var5,var6); putsUSART(string);}

	z_cal = ((ax2_m7*xdata_ten) + (ax2_m8*ydata_ten) + (ax2_m9*zdata_ten));	//compute the calibrated z value of accel 2

	xdata = 0; ydata = 0; zdata = 0;
	xdata_ten = 0.0; ydata_ten = 0.0; zdata_ten = 0.0;

}

/**
 * @brief       This function computes calibrated SoMS data
 *
 * This function gathers all calibration values saved in EEPROM given the clock status.
 * It computes calibrated data using normalization equation that assumes
 * fshiftair as minimum value and fshiftwater as maximum value.
 *
 */

void get_somscal (void)
{	
    if (clk_stat == 20) {
	   fshiftairh = EEPROM_read(0x03,0x50);
	   fshiftairl = EEPROM_read(0x03,0x51);
	   fshiftwaterh = EEPROM_read(0x03,0x52);
	   fshiftwaterl = EEPROM_read(0x03,0x53);
    }
    if (clk_stat == 8) {
       fshiftairh = EEPROM_read(0x03,0x54);	
	   fshiftairl = EEPROM_read(0x03,0x55);
	   fshiftwaterh = EEPROM_read(0x03,0x56);
	   fshiftwaterl = EEPROM_read(0x03,0x57);
    }

	fshiftair = (fshiftairh*256) + fshiftairl;
	fshiftwater = (fshiftwaterh*256) + fshiftwaterl;

	fcal = 1000*(((int)(fshift-fshiftair))/((float)(fshiftwater-fshiftair)));	// Normalization equation
}


//SOIL MOISTURE

/**
 * @brief       This function initializes GPIO and interrupts used in SoMS routine.
 *
 * This function sets PORT RC2 as input for signal fed by the SoMS board and PORT RC1
 * as output to control the reed relay for connecting/disconnecting sensor.
 * It also sets Timer3 for edge-triggered interrupt
 *
 */

void soil_initialize (void)
{
	TRISC |= 0x04;			// set port RC2 to input
	TRISC &= ~0x02;			// set RC1 as output
	PIE2bits.TMR3IE = 1;	// enable overflow interrupt of Timer3
}

/**
 * @brief       This function computes raw pulse counts and frequency shift.
 *
 * This function computes the frequency shift of the square signals fed by the SoMS board.
 * It measures two frequencies, one with disconnected sensor and the other with connected electrodes.
 * 
 * Steps:
 * Determine which oscillator to use from clock status, either 8MHz internal or 20MHz crystal.
 * Use PORT C1 to disconnect sensor.
 * Capture signal using Timer1 and use Timer3 for edge-triggered interrupt.
 * For 30 times, measure the total pulse counts captured. Get the average.
 * Do this again with PORT C1 utilized to connect sensor.
 * If no signal is detected, end function and return zero pulse counts and frequency shift.
 * Compute frequency shift depending on clock status.
 * 
 * @param [in]  clk  Clock status
 *
 */

void get_somsraw (unsigned char clk)
{
	unsigned int result=0,res0=0,res1=0,sw_delay=0;
	int i=0,k=0;

	if (clk == 0x03) {
       sw_delay = 25;
	   OSCCON = OSCCON & ~clk;
	}
    if (clk == 0x02) {
       sw_delay = 10;   
	   OSCCON = OSCCON | clk;
    }
	Delay10KTCYx(1);
	        
	PIR2bits.TMR3IF = 0;	// Set overflow flag bit to zero
	PORTC &= ~0x02;			// Set RC1 to '0' to set switch off, get freq at base operation
	Delay10KTCYx(sw_delay);		// Required delay for switch release time (2msec)

	tot0 = 0;
	i = 0;

	while(i<30)	{
		// Configure Capture1
		OpenCapture1(C1_EVERY_16_RISE_EDGE & CAPTURE_INT_ON);	// Set capture every 16th rising edge of signal
		T3CON = 0xC1;

		nopulseindicator = 0;
		WriteTimer0(0);
		timestart = ReadTimer0();								// Read timer value to counter;
		
		while(!PIR1bits.CCP1IF) {								// Wait for event   
					timeend = ReadTimer0();						// Read timer value to counter;
					timeelapsed = timeend - timestart;
					if (timeelapsed > PULSECOUNT)	{nopulseindicator = 1;  break;}
		}

		if (nopulseindicator == 1) { tot0 = 0; break; }

		res0 = ReadCapture1();									// Read result
		PIR1bits.CCP1IF = 0;									// Reset CCP interrupt flag
		Delay1TCY();

		nopulseindicator = 0;
		WriteTimer0(0);
		timestart = ReadTimer0();								// Read timer value to counter;

		while(!PIR1bits.CCP1IF)  {								// Wait for event   
					timeend = ReadTimer0();						// Read timer value to counter;
					timeelapsed = timeend - timestart;
					if (timeelapsed > PULSECOUNT)	{nopulseindicator = 1;  break;}
		}

		if (nopulseindicator == 1) { tot0 = 0; break; }

		res1 = ReadCapture1();									 // Read result

		CloseTimer3();
		CloseCapture1();

		result = res1 - res0;									// Get difference
		
  		if(!PIR2bits.TMR3IF)	{								// Get the sum of accumulated measurements if an overflow condition has not occurred
			tot0 = ((int)((long)result + (long)tot0));
			i++;
  		} PIR2bits.TMR3IF = 0;									// Set overflow flag bit to zero
	}

	PIR2bits.TMR3IF = 0;										// Set overflow flag bit to zero
	PORTC |= 0x02;												// Set RC1 to '1' to toggle switch
//QUESTION:
	Delay10KTCYx(25);											// Required delay for switch release time (2msec)

	tot1 = 0;
	i = 0;

	while(i<30)	{
		OpenCapture1(C1_EVERY_16_RISE_EDGE & CAPTURE_INT_ON);
		T3CON = 0xC1; 

		nopulseindicator = 0;
		WriteTimer0(0);
		timestart = ReadTimer0();
		
		while(!PIR1bits.CCP1IF) { 
					timeend = ReadTimer0();
					timeelapsed = timeend - timestart;
					if (timeelapsed > PULSECOUNT)	{nopulseindicator = 1;  break;}
		}

		if (nopulseindicator == 1) { tot1 = 0; break; }

		res0 = ReadCapture1();
		PIR1bits.CCP1IF = 0;
		Delay1TCY();

		nopulseindicator = 0;
		WriteTimer0(0);	
		timestart = ReadTimer0();
		
		while(!PIR1bits.CCP1IF)  {
					timeend = ReadTimer0();
					timeelapsed = timeend - timestart;
					if (timeelapsed > PULSECOUNT)	{nopulseindicator = 1;  break;}
		}

		if (nopulseindicator == 1) { tot1 = 0; break; }

		res1 = ReadCapture1(); 	

		CloseTimer3();
		CloseCapture1();

		result = res1 - res0;

  		if(!PIR2bits.TMR3IF)	{
			tot1 = ((int)((long)result + (long)tot1));
			i++;
  		} PIR2bits.TMR3IF = 0;
	}

	PIR2bits.TMR3IF = 0;


    if (OSCCONbits.OSTS){
           clk_stat = 20;
    }
    
    if (OSCCONbits.IOFS){
           clk_stat = 8;
    }  

	tot0 = ((int)(((unsigned int)tot0)/30));				// Get average pulse counts
	tot1 = ((int)(((unsigned int)tot1)/30));


	if ((tot0 == 0)||(tot1 == 0))	fshift = 0;				// 0 output pulse count if timeout

	else {
         if (clk_stat == 20)   
            fshift = 20000000/tot0 - 20000000/tot1;
         else if (clk_stat == 8)
            fshift = 8000000/tot0 - 8000000/tot1;
         else
             fshift = 0;    
     }
		
	Delay10KTCYx(1);

}

/**
 * @brief       This function gets another SoMS samples
 *
 * This function configures PORT A5 as accel 1 chip select
 * It controls accel 1 chip select and its level shifter enable
 * It initializes accelerometer with power-on, decimate by 512, normal mode and enable tri-axis
 */

void resample_soms(unsigned char clock_select)
{  
     int i = 0;
     
     do {
        get_somsraw(clock_select);
        i++;
		sprintf(string,(const far rom char *)"Resample i = %d\n\r",i); putsUSART(string);
     	if (tot1 > tot0)		break;
     } while ((i < 4));
//QUESTION:	Why 4? Is there an indicator to know if sample is verified acceptable?
}    
     

//ACCELEROMETER

/**
 * @brief       This function intializes normal operation of accelerometer 1
 *
 * This function configures PORT A5 as accel 1 chip select
 * It controls accel 1 chip select and its level shifter enable
 * It initializes accelerometer with power-on, decimate by 512, normal mode and enable tri-axis
 */

void axel_initialize (void)
{
	TRISA &= ~0x20;				// Set pin C0 (chip select) to output

	PORTA |= 0x20;				// Chip select is high on default
	PORTB &= ~0x02;
	PORTB |= 0x01;
	axel_writebyte(0x20, 0xC7);	// Accel initialization, ctrl_reg1
}

/**
 * @brief       This function intializes self-test operation of accelerometer 1
 *
 * This function configures PORT A5 as accel 1 chip select
 * It controls accel 1 chip select and its level shifter enable
 * It initializes accelerometer with power-on, decimate by 512, self-test mode and enable tri-axis
 */

void axel_initialize_self (void)
{
	TRISA &= ~0x20;				// Set pin C0 (chip select) to output

	PORTA |= 0x20;				// Chip select is high on default
	PORTB &= ~0x02;
	PORTB |= 0x01;
	axel_writebyte(0x20, 0xCF);	// Accel initialization, ctrl_reg1
}

/**
 * @brief       This function writes data to accelerometer 1 register
 *
 * This function follows the SPI writing routine as discussed in LIS3LV02DL datasheet
 *
 * @param [in]  address   Accelerometer register address
 * @param [in]  data	  Data to be written
 *
 */

void axel_writebyte (unsigned char address, unsigned char data)
{
	unsigned char temp;
	
	address &= ~0x80;			// Set to 0 the MSB for write
	PORTA &= ~0x20;				// Assert chip select
	temp = putcSPI(address);	// Send low byte of address
	temp = putcSPI(data);		// Send data byte
	PORTA |= 0x20;				// Negate chip select
}

/**
 * @brief       This function reads data of accelerometer 1 register
 *
 * This function follows the SPI reading routine as discussed in LIS3LV02DL datasheet
 *
 * @param [in]  address		Accelerometer register address
 *
 * @retval	Read data
 */

unsigned char axel_readbyte (unsigned char address)
{
	unsigned char temp;
	
	address |= 0x80;			// Set to 1 the MSB for read
	PORTA &= ~0x20;				// Assert chip select
	temp = putcSPI(address);	// Send low byte of address
	temp = getcSPI();			// Read single byte
	PORTA |= 0x20;				// Negate chip select
	return (temp);				// Return read data
}

/**
 * @brief       This function intializes normal operation of accelerometer 2
 *
 * This function configures PORT C0 as accel 2 chip select
 * It controls accel 2 chip select and its level shifter enable
 * It initializes accelerometer with power-on, decimate by 512, normal mode and enable tri-axis
 */

void axel2_initialize (void)
{
	TRISC &= ~0x01;					// Set pin C0 (chip select) to output
	PORTC |= 0x01;					// Chip select is high on default
	PORTB &= ~0x01;
	PORTB |= 0x02;
	axel2_writebyte(0x20, 0xC7);	// Accel initialization, ctrl_reg1
}

/**
 * @brief       This function intializes self-test operation of accelerometer 2
 *
 * This function configures PORT C0 as accel 2 chip select
 * It controls accel 2 chip select and its level shifter enable
 * It initializes accelerometer with power-on, decimate by 512, self-test mode and enable tri-axis
 */

void axel2_initialize_self (void)
{
	TRISC &= ~0x01;					// Set pin C0 (chip select) to output
	PORTC |= 0x01;					// Chip select is high on default
	PORTB &= ~0x01;
	PORTB |= 0x02;
	axel2_writebyte(0x20, 0xCF);	// Accel initialization, ctrl_reg1
}

/**
 * @brief       This function writes data to accelerometer 2 register
 *
 * This function follows the SPI writing routine as discussed in LIS3LV02DL datasheet
 *
 * @param [in]  address   Accelerometer register address
 * @param [in]  data	  Data to be written
 *
 */

void axel2_writebyte (unsigned char address, unsigned char data)
{
	unsigned char temp;
	
	address &= ~0x80;			// Set to 0 the MSB for write
	PORTC &= ~0x01;				// Assert chip select
	temp = putcSPI(address);	// Send low byte of address
	temp = putcSPI(data); 		// Send data byte
	PORTC |= 0x01;				// Negate chip select
}

/**
 * @brief       This function reads data of accelerometer 2 register
 *
 * This function follows the SPI reading routine as discussed in LIS3LV02DL datasheet
 *
 * @param [in]  address		Accelerometer register address
 *
 * @retval	Read data
 */

unsigned char axel2_readbyte (unsigned char address)
{
	unsigned char temp;
	
	address |= 0x80;			// Set to 1 the MSB for read
	PORTC &= ~0x01;				// Assert chip select
	temp = putcSPI(address);	// Send low byte of address
	temp = getcSPI();			// Read single byte
	PORTC |= 0x01;				// Negate chip select
	return (temp);				// Return read data
}

	
//EEPROM

/**
 * @brief       This function writes data to specific address of EEPROM
 *
 * This function follows the EEPROM writing routine as discussed in pic18f2585 datasheet
 * At end, it checks if the read data is the same as the data to be written
 * It writes again to memory if previous writing failed
 * Limited up to ten tries only, returns pass/fail writing
 *
 * @param [in]  addressh  High byte EEPROM address
 * @param [in]  address   Low byte EEPROM address
 * @param [in]  data	  Data to be written
 *
 * @retval 1    Fail writing
 * @retval 2    Success writing
 */

unsigned char EEPROM_write (unsigned char addressh, unsigned char address, unsigned char data)
{
    unsigned char INTCON_SAVE;
	unsigned char var;
	unsigned char write_status = 0;
	i = 0;
					
	do {
	    EEADRH  = addressh;
	    EEADR  = address;
	    EEDATA = data;
	    EECON1bits.EEPGD= 0;		// 0 = Access data EEPROM memory
	    EECON1bits.CFGS = 0;		// 0 = Access Flash program or DATA EEPROM memory
	    EECON1bits.WREN = 1;		// Enable writes to internal EEPROM
	    INTCON_SAVE=INTCON;			// Save INTCON register contants
	    INTCON=0;					// Disable interrupts, Next two lines SHOULD run without interrupts
	    
	    EECON2=0x55;				// Required sequence for write to internal EEPROM
	    EECON2=0xaa;				// Required sequence for write to internal EEPROM
	    EECON1bits.WR=1;			// Begin write to internal EEPROM
	    INTCON=INTCON_SAVE;			// Now we can safely enable interrupts if previously used
	    
	    Nop();
	    while (PIR2bits.EEIF==0)	// Wait until write operation complete
	    {
	        Nop();
	    }
	    EECON1bits.WREN=0;			// Disable writes to EEPROM on write complete (EEIF flag on set PIR2 )
	    PIR2bits.EEIF=0;			// Clear EEPROM write complete flag.

		var = EEPROM_read(addressh, address);
		i++;

	}	while ((data != var) && (i<10));	// Up to ten trials only

	if (i==10)	write_status = 1;	// Fail EEPROM writing
	else		write_status = 2;	// Success EEPROM writing

	return write_status;

}

/**
 * @brief       This function reads data of specific address of EEPROM
 *
 * This function follows the EEPROM reading routine as discussed in pic18f2585 datasheet
 *
 * @param [in]  addressh  High byte EEPROM address
 * @param [in]  address   Low byte EEPROM address
 *
 * @retval	Read data
 */
	
unsigned char EEPROM_read (unsigned char addressh, unsigned char address)
{
    unsigned char data;

    EEADRH  = addressh;
    EEADR  = address;
    EECON1bits.EEPGD= 0;	// 0 = Access data EEPROM memory
    EECON1bits.CFGS = 0;	// 0 = Access Flash program or DATA EEPROM memory
    EECON1bits.RD = 1;		// Enable reading internal EEPROM
    data = EEDATA;
   
    Nop();
    return data;
}


//ADC

/**
 * @brief       This function reads analog voltage fed to accelerometer
 *
 * This function reads analog voltage reading to Channel 2 and converts it to digital reading.
 * It returns with global variable adcin which is the actual voltage multiplied by 100
 *
 */

void get_adcvoltage (void)
{	
	ADCON0 = 0x09;				// Channel to AN2
	ADCON1 = 0x0C;
	ADCON2 = 0x8A;				// Right justified ADC result format

	ADCON0bits.GO = 1;			// Start conversion
	while(ADCON0bits.GO);
	adc_result = ADRES;	

	adcin = (adc_result*500)/1023;	// reading*5/1023 (multiplied by 100 to prevent avoid representation)
}

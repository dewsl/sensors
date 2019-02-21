/*!
 * \file  Senslope.c
 *
 *
 * \date January 20, 2015
 * \version Senslope Alpha 3.0
 *
 * Processor:      pic18f25k80 \n
 * Compiler:       c18         \n
 *
 */


/******************************************************************************/
/* Included Files	                                                          */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>    		    // XC8 General Include File
    #include <plib.h>
#elif defined(HI_TECH_C)
    #include <htc.h>       		// HiTech General Include File
#elif defined(__18CXX)
    #include <p18cxxx.h>		// C18 General Include File 
#endif

#if defined(__XC) || defined(HI_TECH_C)
	#include <stdint.h>        	// For uint8_t definition
	#include <stdbool.h>       	// For true/false definition
#endif
	
#include "system.h"        /* System funct/params, like osc/peripheral config */
//#include "user.h"          /* User funct/params, such as InitApp */
#include "can.h"
#include <usart.h>
#include <stdio.h>
#include <delays.h>
#include <spi.h>
#include <adc.h>
#include <string.h>
#include <math.h>




/******************************************************************************/
/* Configuration Bits				                                          */
/******************************************************************************/

#include <p18F25K80.h>

// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Ultra low-power regulator is Disabled (Controlled by REGSLP bit))
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = DIG    // SOSC Power Selection and mode Configuration bits (Digital (SCLKI) mode)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config FOSC = INTIO2    // Oscillator (Internal RC oscillator)
#pragma config PLLCFG = OFF     // PLL x4 Enable bit (Disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = OFF       // Internal External Oscillator Switch Over Mode (Disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power Up Timer (Disabled)
#pragma config BOREN = SBORDIS  // Brown Out Detect (Enabled in hardware, SBOREN disabled)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (1.8V)
#pragma config BORPWR = ZPBORMV // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WDTPS = 1048576  // Watchdog Postscaler (1:1048576)

// CONFIG3H
#pragma config CANMX = PORTB    // ECAN Mux bit (ECAN TX and RX pins are located on RB2 and RB3, respectively)
#pragma config MSSPMSK = MSK7   // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = ON       // Master Clear Enable (MCLR Enabled, RE3 Disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size (2K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF         // Code Protect 00800-01FFF (Enabled)
#pragma config CP1 = OFF         // Code Protect 02000-03FFF (Enabled)
#pragma config CP2 = OFF         // Code Protect 04000-05FFF (Enabled)
#pragma config CP3 = OFF         // Code Protect 06000-07FFF (Enabled)

// CONFIG5H
#pragma config CPB = OFF        // Code Protect Boot (Disabled)
#pragma config CPD = OFF        // Data EE Read Protect (Disabled)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protect 00800-01FFF (Disabled)
#pragma config WRT1 = OFF       // Table Write Protect 02000-03FFF (Disabled)
#pragma config WRT2 = OFF       // Table Write Protect 04000-05FFF (Disabled)
#pragma config WRT3 = OFF       // Table Write Protect 06000-07FFF (Disabled)

// CONFIG6H
#pragma config WRTC = OFF       // Config. Write Protect (Disabled)
#pragma config WRTB = OFF       // Table Write Protect Boot (Disabled)
#pragma config WRTD = OFF       // Data EE Write Protect (Disabled)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protect 00800-01FFF (Disabled)
#pragma config EBTR1 = OFF      // Table Read Protect 02000-03FFF (Disabled)
#pragma config EBTR2 = OFF      // Table Read Protect 04000-05FFF (Disabled)
#pragma config EBTR3 = OFF      // Table Read Protect 06000-07FFF (Disabled)

// CONFIG7H
#pragma config EBTRB = OFF      // Table Read Protect Boot (Disabled)


/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/
volatile int i=0;
volatile int unique_nodeid=0, bandgap=0, reference=0, temperature=0, DEBUG=0, counter=0;
volatile unsigned char xh1=0,xl1=0,yh1=0,yl1=0,zh1=0,zl1=0,xh2=0,xl2=0,yh2=0,yl2=0,zh2=0,zl2=0;	// Accelerometer raw variables
volatile int x1data=0,y1data=0,z1data=0,x2data=0,y2data=0,z2data=0,x1_self=0,y1_self=0,z1_self=0,x2_self=0,y2_self=0,z2_self=0,acc_stat=0;	// Accelerometer self-test variables
volatile unsigned int soms_raw=0,somsair=0,somswater=0, somsadc=0;												// Soil moisture raw data
volatile unsigned char somsairl=0,somsairh=0,somswaterl=0,somswaterh=0;									// Soil moisture EEPROM values
volatile int soms_cal=0;                                                                           // Soil moisture normalized value
volatile long bg=0;                                                                              // Bandgap voltage for computing internal 2V ADC reference used by SOMS        
CANDATA_EXTENDED canBuffer;
char string[64];

/*****************************************/
/* Accel Calibration Global Variables
/*****************************************/
volatile double x_cal=0,y_cal=0,z_cal=0;
volatile int xdata=0,ydata=0,zdata=0;



/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

void main(void)
{
    volatile unsigned char idhigh=0, idlow=0;
    volatile unsigned int status,command,canTransIden;

	OSCCON = 0x60;	// Set PIC oscillator to 8MHz

    idhigh = EEPROM_read(0x01,0x00);	// load EEPROM values
	idlow = EEPROM_read(0x01,0x01);
    unique_nodeid = (idhigh*256) + idlow;

	DEBUG = EEPROM_read(0x01, 0x04);
	counter = 0;

    /* Initialize I/O and Peripherals for application */
    RCSTA2bits.SPEN = 1;
    
	OpenSPI (SPI_FOSC_64, MODE_11, SMPEND);
    Open1USART(	USART_TX_INT_OFF &				// initialize uart
				USART_RX_INT_OFF  &
				USART_ASYNCH_MODE &
				USART_EIGHT_BIT &
				USART_CONT_RX &
				USART_BRGH_LOW,
				12 ); 	//9600

	/* Initialize accelerometer chip select pins */
	TRISC &= ~0x01;				// Set pin C0 (chip select 1) to output
	TRISB &= ~0x01;				// Set pin B0 (chip select 2) to output
	
	TRISC &= ~0x04;             //set pin C2 to output for SOMS
    PORTC &= ~0x04;				//set SOMS switch enable to low (switch is active high)

	axel_initialize();
	axel2_initialize();
	can_initialize_extended(&canBuffer);

    Delay10KTCYx(0);

	get_axel1raw();
	if (DEBUG) {sprintf(string,(const char *)"A1 IRaw: xh=%d xl=%d yh=%d yl=%d zh=%d zl=%d\n\r", xh1, xl1, yh1, yl1, zh1, zl1); puts1USART(string); }
	get_axel2raw();
	if (DEBUG) {sprintf(string,(const char *)"A2 IRaw: xh=%d xl=%d yh=%d yl=%d zh=%d zl=%d\n\r", xh2, xl2, yh2, yl2, zh2, zl2); puts1USART(string); }


    while(1)
    {

         command = 0;
         status = 0;

         if (DEBUG) { can_check_errors(); }

         status = can_check_for_datain_extended(&canBuffer);

         if(status)	command = can_process_commands_extended(&canBuffer,(unsigned int)unique_nodeid);
		 
         switch(command) {

             case 0:
                 break;

             case 99:

   				if (DEBUG) {sprintf(string,(const char *)"COMMAND 99\n\r"); puts1USART(string); }

                 canBuffer.data2 = 2;
                 canBuffer.data3 = 2;
                 canBuffer.data4 = 2;
                 canBuffer.data5 = 2;
                 canBuffer.data6 = 2;
                 canBuffer.dlc = 6;
                 break;

             case 98:
                 canBuffer.data2 = 3;
                 canBuffer.data3 = 3;
                 canBuffer.data4 = 3;
                 canBuffer.data5 = 3;
                 canBuffer.data6 = 3;
                 canBuffer.dlc = 6;
                 Delay10KTCYx(200); //delay for 1 second
                 Delay10KTCYx(200); //delay for 1 second
                 break;
                 
             case 97:
                 canBuffer.data2 = (unsigned int)counter++;
                 canBuffer.dlc = 2;
                 break;

            case BROAD_AXEL1_RAW_INIT: // Initial accel1 raw data

                 canBuffer.data2= xl1;
                 canBuffer.data3= xh1;
                 canBuffer.data4= yl1;
                 canBuffer.data5= yh1;
                 canBuffer.data6= zl1;
                 canBuffer.data7= zh1;
                 canBuffer.data8= (unsigned int)(get_refvoltage()-200);
                 canBuffer.dlc = 8;
                 break;

            case BROAD_AXEL2_RAW_INIT: // Initial accel2 raw data

                 canBuffer.data2= xl2;
                 canBuffer.data3= xh2;
                 canBuffer.data4= yl2;
                 canBuffer.data5= yh2;
                 canBuffer.data6= zl2;
                 canBuffer.data7= zh2;
                 canBuffer.data8= (unsigned int)(get_refvoltage()-200);
                 canBuffer.dlc = 8;
                 break;
                 
            case BROAD_AXEL1_RAW_NEW: // Updated accel1 raw data
                 axel_writebyte(0x22, 0x00);
                 get_axel1raw();
                 //average_axel1(EEPROM_read(0x01,0x05));
                 canBuffer.data2= xl1;
                 canBuffer.data3= xh1;
                 canBuffer.data4= yl1;
                 canBuffer.data5= yh1;
                 canBuffer.data6= zl1;
                 canBuffer.data7= zh1;
                 canBuffer.data8= (unsigned int)(get_refvoltage()-200);
                 canBuffer.dlc = 8;
                 break;

            case BROAD_AXEL2_RAW_NEW: // Updated accel2 raw data

                 //get_axel2raw();
                 axel_writebyte(0x22, 0x00);	// Accel initialization, ctrl_reg3
                 average_axel1(EEPROM_read(0x01,0x05));
                 canBuffer.data2= xl1;
                 canBuffer.data3= xh1;
                 canBuffer.data4= yl1;
                 canBuffer.data5= yh1;
                 canBuffer.data6= zl1;
                 canBuffer.data7= zh1;
                 canBuffer.data8= (unsigned int)(get_refvoltage()-200);
                 canBuffer.dlc = 8;
                 /*average_axel2(EEPROM_read(0x01,0x05));
                 canBuffer.data2= xl2;
                 canBuffer.data3= xh2;
                 canBuffer.data4= yl2;
                 canBuffer.data5= yh2;
                 canBuffer.data6= zl2;
                 canBuffer.data7= zh2;
                 canBuffer.data8= get_refvoltage()-200;
                 canBuffer.dlc = 8;
                 */
                break;

			case BROAD_AXEL1_CALIB_NEW:
				 axel_writebyte(0x22, 0x40);	// Accel initialization, ctrl_reg3
                average_axel1(EEPROM_read(0x01,0x05));
                 canBuffer.data2= xl1;
                 canBuffer.data3= xh1;
                 canBuffer.data4= yl1;
                 canBuffer.data5= yh1;
                 canBuffer.data6= zl1;
                 canBuffer.data7= zh1;
                 canBuffer.data8= (unsigned int)(get_refvoltage()-200);
                 canBuffer.dlc = 8;
                 
				 //get_axel1raw();
                 /*average_axel1(EEPROM_read(0x01,0x05));
				 
                 xdata = (xh1*256) + xl1;													// reconstruct accel data
				 ydata = (yh1*256) + yl1;
			 	 zdata = (zh1*256) + zl1;
                 get_axelcal_m(xdata,ydata,zdata,1);
                  
                 canBuffer.data2= (unsigned char)((int)x_cal & 0xFF);
                 canBuffer.data3= (unsigned char)((int)x_cal >> 8);
                 canBuffer.data4= (unsigned char)((int)y_cal & 0xFF);
                 canBuffer.data5= (unsigned char)((int)y_cal >> 8);
                 canBuffer.data6= (unsigned char)((int)z_cal & 0xFF);
                 canBuffer.data7= (unsigned char)((int)z_cal >> 8);
                 canBuffer.data8= get_refvoltage()-200;
                 canBuffer.dlc = 8;
                 */
                 break;

			case BROAD_AXEL2_CALIB_NEW:
				axel_writebyte(0x22, 0x60);	// Accel initialization, ctrl_reg3
                average_axel1(EEPROM_read(0x01,0x05));
                 canBuffer.data2= xl1;
                 canBuffer.data3= xh1;
                 canBuffer.data4= yl1;
                 canBuffer.data5= yh1;
                 canBuffer.data6= zl1;
                 canBuffer.data7= zh1;
                 canBuffer.data8= (unsigned int)(get_refvoltage()-200);
                 canBuffer.dlc = 8; 
                 
                //average_axel2(EEPROM_read(0x01,0x05));
                 /*canBuffer.data2= xl2;
                 canBuffer.data3= xh2;
                 canBuffer.data4= yl2;
                 canBuffer.data5= yh2;
                 canBuffer.data6= zl2;
                 canBuffer.data7= zh2;
                 canBuffer.data8= get_refvoltage()-200;
                 canBuffer.dlc = 8;
				 */
                 //get_axel2raw();
                   
                /*
				 xdata = (xh2*256) + xl2;													// reconstruct accel data
				 ydata = (yh2*256) + yl2;
			 	 zdata = (zh2*256) + zl2;
                 get_axelcal_m(xdata,ydata,zdata,2);
                  
                 canBuffer.data2= (unsigned char)((int)x_cal & 0xFF);
                 canBuffer.data3= (unsigned char)((int)x_cal >> 8);
                 canBuffer.data4= (unsigned char)((int)y_cal & 0xFF);
                 canBuffer.data5= (unsigned char)((int)y_cal >> 8);
                 canBuffer.data6= (unsigned char)((int)z_cal & 0xFF);
                 canBuffer.data7= (unsigned char)((int)z_cal >> 8);
                 canBuffer.data8= get_refvoltage()-200;
                 canBuffer.dlc = 8;
                 */
                 break;

			case EEPROM_MATRIX_AXEL1 :

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
                    }
                    break;

			case EEPROM_MATRIX_AXEL2 :

				canTransIden = canBuffer.data4;
                canBuffer.data2 = canTransIden;    //Transaction identifier
				 if(canTransIden == 1){
                        canBuffer.data3 = EEPROM_write(0x02,0x70,(unsigned char)canBuffer.data5);
                        canBuffer.data4 = EEPROM_write(0x02,0x71,(unsigned char)canBuffer.data6);
                        canBuffer.data5 = EEPROM_write(0x02,0x72,(unsigned char)canBuffer.data7);
                        canBuffer.data6 = EEPROM_write(0x02,0x73,(unsigned char)canBuffer.data8);
                        canBuffer.dlc = 6;
                    }else if(canTransIden == 2){
                        canBuffer.data3 = EEPROM_write(0x02,0x74,(unsigned char)canBuffer.data5);
                        canBuffer.data4 = EEPROM_write(0x02,0x75,(unsigned char)canBuffer.data6);
					 	canBuffer.data5 = EEPROM_write(0x02,0x80,(unsigned char)canBuffer.data7);
                        canBuffer.data6 = EEPROM_write(0x02,0x81,(unsigned char)canBuffer.data8);
					 	canBuffer.dlc = 6;
					}else if(canTransIden == 3){
                        canBuffer.data3 = EEPROM_write(0x02,0x82,(unsigned char)canBuffer.data5);
                        canBuffer.data4 = EEPROM_write(0x02,0x83,(unsigned char)canBuffer.data6);
					 	canBuffer.data5 = EEPROM_write(0x02,0x84,(unsigned char)canBuffer.data7);
                        canBuffer.data6 = EEPROM_write(0x02,0x85,(unsigned char)canBuffer.data8);
					 	canBuffer.dlc = 6;
					}else if(canTransIden == 4){
                        canBuffer.data3 = EEPROM_write(0x02,0x90,(unsigned char)canBuffer.data5);
                        canBuffer.data4 = EEPROM_write(0x02,0x91,(unsigned char)canBuffer.data6);
					 	canBuffer.data5 = EEPROM_write(0x02,0x92,(unsigned char)canBuffer.data7);
                        canBuffer.data6 = EEPROM_write(0x02,0x93,(unsigned char)canBuffer.data8);
					 	canBuffer.dlc = 6;
					}else if(canTransIden == 5){
                        canBuffer.data3 = EEPROM_write(0x02,0x94,(unsigned char)canBuffer.data5);
                        canBuffer.data4 = EEPROM_write(0x02,0x95,(unsigned char)canBuffer.data6);
					 	canBuffer.data5 = EEPROM_write(0x02,0xA0,(unsigned char)canBuffer.data7);
                        canBuffer.data6 = EEPROM_write(0x02,0xA1,(unsigned char)canBuffer.data8);
					 	canBuffer.dlc = 6;
					}else if(canTransIden == 6){
                        canBuffer.data3 = EEPROM_write(0x02,0xA2,(unsigned char)canBuffer.data5);
                        canBuffer.data4 = EEPROM_write(0x02,0xA3,(unsigned char)canBuffer.data6);
					 	canBuffer.data5 = EEPROM_write(0x02,0xA4,(unsigned char)canBuffer.data7);
                        canBuffer.data6 = EEPROM_write(0x02,0xA5,(unsigned char)canBuffer.data8);
					 	canBuffer.dlc = 6;
                    }//else{ //error case
                        //status = 0;//dont respond
                        //sprintf(string,(const char *)"\n\rError in CAN transaction number\n\r"); puts1USART(string);
                    //}
                    break;				

            case BROAD_AXEL1_SELFTEST_OUTPUT_CHANGE: // Self-test accel1 raw data

                 canBuffer.data2= (unsigned char)((int)x1_self & 0xFF);
                 canBuffer.data3= (unsigned char)((int)x1_self >> 8);
                 canBuffer.data4= (unsigned char)((int)y1_self & 0xFF);
                 canBuffer.data5= (unsigned char)((int)y1_self >> 8);
                 canBuffer.data6= (unsigned char)((int)z1_self & 0xFF);
                 canBuffer.data7= (unsigned char)((int)z1_self >> 8);
                 canBuffer.data8= (unsigned int)(get_refvoltage()-200);
                 canBuffer.dlc = 8;
                 break;

            case BROAD_AXEL2_SELFTEST_OUTPUT_CHANGE: // Self-test accel2 raw data

                 canBuffer.data2= (unsigned char)((int)x2_self & 0xFF);
                 canBuffer.data3= (unsigned char)((int)x2_self >> 8);
                 canBuffer.data4= (unsigned char)((int)y2_self & 0xFF);
                 canBuffer.data5= (unsigned char)((int)y2_self >> 8);
                 canBuffer.data6= (unsigned char)((int)z2_self & 0xFF);
                 canBuffer.data7= (unsigned char)((int)z2_self >> 8);
                 canBuffer.data8= (unsigned int)(get_refvoltage()-200);
                 canBuffer.dlc = 8;
                 break;

            case BROAD_SELF_TEST_ROUTINE: // Self-test operation
                 /*case would respond with the same message as acknowledgement*/

				 axel_selftest();

                 canBuffer.dlc = 1;

                 //if (DEBUG) {sprintf(string,(const char *)"X1=%d\tY1=%d\tZ1=%d\tX2=%d\tY2=%d\tZ2=%d\t\n\r",x1_self,y1_self,z1_self,x2_self,y2_self,z2_self); puts1USART(string);}

                 break;

			case BROAD_CALIBRATE_REFVOLTAGE:	// Get internal bandgap voltage
					
				 canTransIden = canBuffer.data4;
                 cal_refvoltage((long)(canTransIden)+(long)(200));
					
                 canBuffer.data2= EEPROM_write(0x01,0x02,(unsigned char)(bandgap>>8));
                 canBuffer.data3= EEPROM_write(0x01,0x03,(unsigned char)(bandgap&0xFF));
                 canBuffer.data4= (unsigned char)(bandgap>>8);
                 canBuffer.data5= (unsigned char)(bandgap&0xFF);
                 canBuffer.dlc = 5;
                 break;

            case BROAD_SELF_TEST_ONECOMMAND:

				 axel_selftest();
				 reference = get_refvoltage();
				 temperature = get_temperature();

                 acc_stat = 0;

                 if ((x1_self > 250) && (x1_self < 900))        acc_stat = acc_stat+32;
                 if ((y1_self > 250) && (y1_self < 900))        acc_stat = acc_stat+16;       
                 if ((z1_self > 100) && (z1_self < 600))        acc_stat = acc_stat+8;        
                 if ((x2_self > 250) && (x2_self < 900))        acc_stat = acc_stat+4;        
                 if ((y2_self > 250) && (y2_self < 900))        acc_stat = acc_stat+2;        
                 if ((z2_self > 100) && (z2_self < 600))        acc_stat = acc_stat+1;        

                 canBuffer.data2= (unsigned char)(reference>>8);
                 canBuffer.data3= (unsigned char)(reference&0xFF);
                 canBuffer.data4= (unsigned char)(temperature>>8);
                 canBuffer.data5= (unsigned char)(temperature&0xFF);
                 canBuffer.data6= (unsigned char)(acc_stat);
                 canBuffer.dlc = 6;

                 //if (DEBUG) {sprintf(string,(const char *)"X1=%d\tY1=%d\tZ1=%d\tX2=%d\tY2=%d\tZ2=%d\t\n\r",x1_self,y1_self,z1_self,x2_self,y2_self,z2_self); puts1USART(string);}

                 break;

			case BROAD_GET_DIAGNOSTICS:	// Get node health
             		
				 reference = get_refvoltage();
				 temperature = get_temperature();

                 acc_stat = 0;

                 if ((x1_self > 250) && (x1_self < 900))        acc_stat = acc_stat+32;
                 if ((y1_self > 250) && (y1_self < 900))        acc_stat = acc_stat+16;       
                 if ((z1_self > 100) && (z1_self < 600))        acc_stat = acc_stat+8;        
                 if ((x2_self > 250) && (x2_self < 900))        acc_stat = acc_stat+4;        
                 if ((y2_self > 250) && (y2_self < 900))        acc_stat = acc_stat+2;        
                 if ((z2_self > 100) && (z2_self < 600))        acc_stat = acc_stat+1;        

                 canBuffer.data2= (unsigned char)(reference>>8);
                 canBuffer.data3= (unsigned char)(reference&0xFF);
                 canBuffer.data4= (unsigned char)(temperature>>8);
                 canBuffer.data5= (unsigned char)(temperature&0xFF);
                 canBuffer.data6= (unsigned char)(acc_stat);
                 canBuffer.dlc = 6;
                 break;
                 
            case BROAD_EEPROM_ACCESS:

				 canBuffer.data2 = canBuffer.data4;
				 canBuffer.data3 = canBuffer.data5;
                 canBuffer.data4 = EEPROM_read(canBuffer.data4,canBuffer.data5);
                 canBuffer.dlc = 4;
                 break;

            case BROAD_CHANGE_DEBUG_MODE:

                 canBuffer.data2 = EEPROM_write(0x01,0x04,canBuffer.data4);
                 canBuffer.data3 = canBuffer.data4;
                 canBuffer.dlc = 3;

				 DEBUG = (int)canBuffer.data4;
                 break;
                 
            case BROAD_CHANGE_AVERAGING_SAMPLE:

                 canBuffer.data2 = EEPROM_write(0x01,0x05,canBuffer.data4);
                 canBuffer.data3 = canBuffer.data4;
                 canBuffer.dlc = 3;
                 break;     

             default:
                 break;
                 
                 
         }


         if((status !=0) && (command != 0))	{ //check if received something
                 canBuffer.id = (unsigned int)unique_nodeid;

                 if (DEBUG) { sprintf(string,(const char *)"  TX..."); puts1USART(string); }
                 if (DEBUG) { sprintf(string,(const char *)"  %d: %d_%d_%d_%d_%d_%d_%d_%d\n\r",
                         unique_nodeid, canBuffer.data1, canBuffer.data2, canBuffer.data3, canBuffer.data4,
                         canBuffer.data5, canBuffer.data6, canBuffer.data7,canBuffer.data8); puts1USART(string); }

                 can_send_data_with_arb_repeat_extended(&canBuffer,TIMEOUT);
     	 }

    }

}


/******************************************************************************/
/* Function		                                                              */
/******************************************************************************/


//RAW DATA

/**
 * @brief       This function gathers accelerometer 1 data.
 *
 * This function collects accelerometer data by accessing its registers as indicated LIS3LV02DL datasheet.
 *
 */

void get_axel1raw (void)
{	
	axel_initialize();

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
	axel2_initialize();

	xl2 = axel2_readbyte(0x28);	// read x2
	xh2 = axel2_readbyte(0x29);
	yl2 = axel2_readbyte(0x2A);	// read y2
	yh2 = axel2_readbyte(0x2B);
	zl2 = axel2_readbyte(0x2C);	// read z2
	zh2 = axel2_readbyte(0x2D);

}


//ACCELEROMETER

/**
 * @brief       This function intializes normal operation of accelerometer 1
 *
 * This function configures PORT C0 as accel 1 chip select and PORT B0 as accel 2 chip select
 * It controls accel 1 chip select
 * It initializes accelerometer with power-on, decimate by 512, normal mode and enable tri-axis
 */

void axel_initialize (void)
{
	PORTC |= 0x01;
	axel_writebyte(0x20, 0xC7);	// Accel initialization, ctrl_reg1
}

/**
 * @brief       This function intializes self-test operation of accelerometer 1
 *
 * This function configures PORT C0 as accel 1 chip select and PORT B0 as accel 2 chip select
 * It controls accel 1 chip select
 * It initializes accelerometer with power-on, decimate by 512, self-test mode and enable tri-axis
 */

void axel_initialize_self (void)
{
	PORTC |= 0x01;
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
	PORTC &= ~0x01;				// Assert chip select
	temp = (unsigned char)putcSPI(address);	// Send low byte of address
	temp = (unsigned char)putcSPI(data);		// Send data byte
	PORTC |= 0x01;				// Negate chip select
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
	PORTC &= ~0x01;				// Assert chip select
	temp = (unsigned char)putcSPI(address);	// Send low byte of address
	temp = getcSPI();			// Read single byte
	PORTC |= 0x01;				// Negate chip select
	return (temp);				// Return read data
}

/**
 * @brief       This function intializes normal operation of accelerometer 2
 *
 * This function configures PORT C0 as accel 1 chip select and PORT B0 as accel 2 chip select
 * It controls accel 2 chip select
 * It initializes accelerometer with power-on, decimate by 512, normal mode and enable tri-axis
 */

void axel2_initialize (void)
{
	PORTB |= 0x01;
	axel2_writebyte(0x20, 0xC7);	// Accel initialization, ctrl_reg1
}

/**
 * @brief       This function intializes self-test operation of accelerometer 2
 *
 * This function configures PORT C0 as accel 1 chip select and PORT B0 as accel 2 chip select
 * It controls accel 2 chip select
 * It initializes accelerometer with power-on, decimate by 512, self-test mode and enable tri-axis
 */

void axel2_initialize_self (void)
{
	PORTB |= 0x01;
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
	PORTB &= ~0x01;				// Assert chip select
	temp = (unsigned char)putcSPI(address);	// Send low byte of address
	temp = (unsigned char)putcSPI(data); 		// Send data byte
	PORTB |= 0x01;				// Negate chip select
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
	PORTB &= ~0x01;				// Assert chip select
	temp = (unsigned char)putcSPI(address);	// Send low byte of address
	temp = getcSPI();			// Read single byte
	PORTB |= 0x01;				// Negate chip select
	return (temp);				// Return read data
}

//ACCELEROMETER SELF-TEST

/**
 * @brief       This function computes self-test values of accelerometers 1 and 2.
 *
 * This function collects raw accelerometer data and self-test mode accelerometer data.
 * Then, take the difference of each axis to ompute for self-test raw values.
 * Appropriate initialization needed for each accelerometer mode.
 *
 */

void axel_selftest (void)
{

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

}

//AVERAGE DATA


void average_axel1(int x)
{	
	float xsum1,ysum1,zsum1;
	xsum1 = 0;
	ysum1 = 0;
	zsum1 = 0;
    axel_initialize();
    //axel_writebyte(0x22, 0x40);	// Accel initialization, ctrl_reg3
	for ( i=0; i<x;i++)
	{
            //Delay1KTCYx(AXEL_WAITTIME);
            //PORTC |= 0x01;	// chip select axel1
            //PORTB &= ~0x02;	// disable level shifter for axel2
            //PORTB |= 0x01;	// able level shifter for axel1
            
            xl1 = axel_readbyte(0x28);	// read x
            xh1 = axel_readbyte(0x29);
            yl1 = axel_readbyte(0x2A);	// read y
            yh1 = axel_readbyte(0x2B);
            zl1 = axel_readbyte(0x2C);	// read z
            zh1 = axel_readbyte(0x2D);

            xsum1 = xl1+(xh1*256) + xsum1;
            ysum1 = yl1+(yh1*256) + ysum1;
            zsum1 = zl1+(zh1*256) + zsum1;

	}
	xsum1 = round(xsum1/x);
	ysum1 = round(ysum1/x);
	zsum1 = round(zsum1/x);

	xl1 = (unsigned char)((int)xsum1 & 0xFF);
	xh1 = (unsigned char)((int)xsum1 >> 8);
	yl1 = (unsigned char)((int)ysum1 & 0xFF);
	yh1 = (unsigned char)((int)ysum1 >> 8);
	zl1 = (unsigned char)((int)zsum1 & 0xFF);
	zh1 = (unsigned char)((int)zsum1 >> 8);
	

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
{
	float xsum2,ysum2,zsum2;
	xsum2 = 0;
	ysum2 = 0;
	zsum2 = 0;
    axel2_initialize();
    axel2_writebyte(0x22, 0x40);	// Accel initialization, ctrl_reg3
	for ( i=0; i<x;i++)
        
	{
            /*Delay1KTCYx(AXEL_WAITTIME);
            PORTA |= 0x20;	// chip select axel2
            PORTB &= ~0x01;	// disable level shifter for axel1
            PORTB |= 0x02;	// able level shifter for axel2
             */
            xl2 = axel2_readbyte(0x28);	// read x2
            xh2 = axel2_readbyte(0x29);
            yl2 = axel2_readbyte(0x2A);	// read y2
            yh2 = axel2_readbyte(0x2B);
            zl2 = axel2_readbyte(0x2C);	// read z2
            zh2 = axel2_readbyte(0x2D);

            xsum2 = xl2+(xh2*256) + xsum2;
            ysum2 = yl2+(yh2*256) + ysum2;
            zsum2 = zl2+(zh2*256) + zsum2;
            
	}
	xsum2 = round(xsum2/x);
	ysum2 = round(ysum2/x);
	zsum2 = round(zsum2/x);

	xl2 = (unsigned char)((int)xsum2 & 0xFF);
	xh2 = (unsigned char)((int)xsum2 >> 8);
	yl2 = (unsigned char)((int)ysum2 & 0xFF);
	yh2 = (unsigned char)((int)ysum2 >> 8);
	zl2 = (unsigned char)((int)zsum2 & 0xFF);
	zh2 = (unsigned char)((int)zsum2 >> 8);
	

}

/**
 * @brief       This function calculates calibrated accelerometer 1 data using MATRIX calibration
 *
 * This function should be called after calling get_axel2raw or average_axel2.
 * The function assumes xl1,yl1,zl1,xh1,yh1,and zh1 already have values.
 * Implementation of calibration as suggested by App Note 3182 by ST Electronics.
 *
 */

void get_axelcal_m( int xdata, int ydata, int zdata, int axel )
{
	double m1,m2,m3,m4,m5,m6,m7,m8,m9,m10,m11,m12;

	x_cal = 0; y_cal = 0; z_cal = 0;

	if ( axel == 1 )
	{
		m10 = ((((EEPROM_read(0x02,0x41)*256)+EEPROM_read(0x02,0x40))*1.00)/(MULTIPLIER2*1.00));
		m11 = ((((EEPROM_read(0x02,0x43)*256)+EEPROM_read(0x02,0x42))*1.00)/(MULTIPLIER2*1.00));
		m12 = ((((EEPROM_read(0x02,0x45)*256)+EEPROM_read(0x02,0x44))*1.00)/(MULTIPLIER2*1.00));
	
		m1 = ((((EEPROM_read(0x02,0x11)*256)+EEPROM_read(0x02,0x10))*1.00)/(MULTIPLIER1*1.00));							// compute for the multipliers for x y and z
		m2 = ((((EEPROM_read(0x02,0x13)*256)+EEPROM_read(0x02,0x12))*1.00)/(MULTIPLIER1*1.00));
		m3 = ((((EEPROM_read(0x02,0x15)*256)+EEPROM_read(0x02,0x14))*1.00)/(MULTIPLIER1*1.00));
	
		m4 = ((((EEPROM_read(0x02,0x21)*256)+EEPROM_read(0x02,0x20))*1.00)/(MULTIPLIER1*1.00));
		m5 = ((((EEPROM_read(0x02,0x23)*256)+EEPROM_read(0x02,0x22))*1.00)/(MULTIPLIER1*1.00));
		m6 = ((((EEPROM_read(0x02,0x25)*256)+EEPROM_read(0x02,0x24))*1.00)/(MULTIPLIER1*1.00));
	
		m7 = ((((EEPROM_read(0x02,0x31)*256)+EEPROM_read(0x02,0x30))*1.00)/(MULTIPLIER1*1.00));
		m8 = ((((EEPROM_read(0x02,0x33)*256)+EEPROM_read(0x02,0x32))*1.00)/(MULTIPLIER1*1.00));
		m9 = ((((EEPROM_read(0x02,0x35)*256)+EEPROM_read(0x02,0x34))*1.00)/(MULTIPLIER1*1.00));
	
		x_cal =( (m1*((xdata*1.00)-m10)) + (m2*((ydata*1.00)-m11)) + (m3*((zdata*1.00)-m12)));
		y_cal =( (m4*((xdata*1.00)-m10)) + (m5*((ydata*1.00)-m11)) + (m6*((zdata*1.00)-m12)));
		z_cal =( (m7*((xdata*1.00)-m10)) + (m8*((ydata*1.00)-m11)) + (m9*((zdata*1.00)-m12)));
	} 
	else if ( axel == 2)
	{

		m10 = ((((EEPROM_read(0x02,0xA1)*256)+EEPROM_read(0x02,0xA0))*1.00)/(MULTIPLIER2*1.00));
		m11 = ((((EEPROM_read(0x02,0xA3)*256)+EEPROM_read(0x02,0xA2))*1.00)/(MULTIPLIER2*1.00));
		m12 = ((((EEPROM_read(0x02,0xA5)*256)+EEPROM_read(0x02,0xA4))*1.00)/(MULTIPLIER2*1.00));
	
		m1 = ((((EEPROM_read(0x02,0x71)*256)+EEPROM_read(0x02,0x70))*1.00)/(MULTIPLIER1*1.00));
		m2 = ((((EEPROM_read(0x02,0x73)*256)+EEPROM_read(0x02,0x72))*1.00)/(MULTIPLIER1*1.00));
		m3 = ((((EEPROM_read(0x02,0x75)*256)+EEPROM_read(0x02,0x74))*1.00)/(MULTIPLIER1*1.00));
	
		m4 = ((((EEPROM_read(0x02,0x81)*256)+EEPROM_read(0x02,0x80))*1.00)/(MULTIPLIER1*1.00));
		m5 = ((((EEPROM_read(0x02,0x83)*256)+EEPROM_read(0x02,0x82))*1.00)/(MULTIPLIER1*1.00));
		m6 = ((((EEPROM_read(0x02,0x85)*256)+EEPROM_read(0x02,0x84))*1.00)/(MULTIPLIER1*1.00));
	
		m7 = ((((EEPROM_read(0x02,0x91)*256)+EEPROM_read(0x02,0x90))*1.00)/(MULTIPLIER1*1.00));
		m8 = ((((EEPROM_read(0x02,0x93)*256)+EEPROM_read(0x02,0x92))*1.00)/(MULTIPLIER1*1.00));
		m9 = ((((EEPROM_read(0x02,0x95)*256)+EEPROM_read(0x02,0x94))*1.00)/(MULTIPLIER1*1.00));
	
		x_cal =( (m1*((xdata*1.00)-m10)) + (m2*((ydata*1.00)-m11)) + (m3*((zdata*1.00)-m12)));
		y_cal =( (m4*((xdata*1.00)-m10)) + (m5*((ydata*1.00)-m11)) + (m6*((zdata*1.00)-m12)));
		z_cal =( (m7*((xdata*1.00)-m10)) + (m8*((ydata*1.00)-m11)) + (m9*((zdata*1.00)-m12)));

	}

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
	    while (PIR4bits.EEIF==0)	// Wait until write operation complete
	    {
	        Nop();
	    }
	    EECON1bits.WREN=0;			// Disable writes to EEPROM on write complete (EEIF flag on set PIR2 )
	    PIR4bits.EEIF=0;			// Clear EEPROM write complete flag.

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
 * @brief       This function calibrates reference voltage readings
 *
 * This function gets the actual internal bandgap voltage of PIC18F25K80
 * Bangap voltage may vary for different PIC, typical is 1.024V.
 * Bandgap*1000 is saved in EEPROM.
 *
 * @param [in]  ref  Reference voltage measured by external microcontroller
 *
 */

void cal_refvoltage (long ref)
{	
	int rh, rl;
	long adc_result;

	ANCON1 = 0x00;				// No analog channels

	ADCON2bits.ADFM = 1;		// Right justified ADC result format
	ADCON2bits.ACQT = 1;		// Acquisition time = 2TAD
	ADCON2bits.ADCS = 2;		// Clock conversion = Fosc/32

	ADCON1bits.VCFG0 = 0; 		// Vref+ = AVdd
	ADCON1bits.VCFG1 = 0;		// Vref+ = AVdd
	ADCON1bits.VNCFG = 0;		// Vref- = AVss (GND)
	ADCON0bits.CHS = 31;		// ADC input = constant 1.024V band gap

	ADCON0bits.ADON = 1;		// Turn on ADC
	Delay1KTCYx(0);
	ADCON0bits.GO = 1;			// Start conversion

	while (ADCON0bits.GO == 0)	// Wait conversion finish
	ADCON0bits.ADON = 0;		// Turn off ADC


	Delay1KTCYx(1);	

	rh = ADRESH & 0x0F;			// Mask unnecessary data
	rl = ADRESL;

	adc_result = rh*256 + rl;	// Get ADC value

	bandgap = (int)(((long)(10)*(long)(ref)*adc_result)/((long)4095));

}

/**
 * @brief       This function reads reference voltage of the microcontroller
 *
 * This function reads reference voltage through Channel 31 and converts it to digital reading.
 * It loads the bandgap voltage in EEPROM for actual measurement of reference voltage.
 * It returns with integer Vref which is the actual voltage multiplied by 100
 *
 * @retval		Vref
 *
 */

int get_refvoltage (void)
{	
	int Vref, rh, rl;
    unsigned char bgh, bgl;
	long adc_result;

	ANCON1 = 0x00;				// No analog channels

	ADCON2bits.ADFM = 1;		// Right justified ADC result format
	ADCON2bits.ACQT = 1;		// Acquisition time = 2TAD
	ADCON2bits.ADCS = 2;		// Clock conversion = Fosc/32

	ADCON1bits.VCFG0 = 0; 		// Vref+ = AVdd
	ADCON1bits.VCFG1 = 0;		// Vref+ = AVdd
	ADCON1bits.VNCFG = 0;		// Vref- = AVss (GND)
	ADCON0bits.CHS = 31;		// ADC input = constant 1.024V band gap

	ADCON0bits.ADON = 1;		// Turn on ADC
	Delay1KTCYx(0);
	ADCON0bits.GO = 1;			// Start conversion

	while (ADCON0bits.GO == 0)	// Wait conversion finish
	ADCON0bits.ADON = 0;		// Turn off ADC

	Delay1KTCYx(1);	

	rh = ADRESH & 0x0F;			// Mask unnecessary data
	rl = ADRESL;

	adc_result = rh*256 + rl;	// Get ADC value

    bgh = EEPROM_read(0x01,0x02);	// load EEPROM values
	bgl = EEPROM_read(0x01,0x03);
    bg = (bgh*256) + bgl;

	Vref = (int)((bg*(long)4095)/((long)10*adc_result));	// Bandgap*(2^12-1)/adc_result

	return Vref;

}

/**
 * @brief       This function reads analog voltage of PIC internal diode responsive to temperature
 *
 * This function reads reference voltage through Channel 29 and converts it to digital reading.
 * It uses Charge Time Measurement Unit module of PIC18F25K80.
 *
 * @retval		temp
 *
 */

int get_temperature (void)
{	
	int temp;

	CTMUICON = 0x03;			// 100*Ib current source of temp diode
	CTMUCONHbits.CTMUEN = 1;	// Enable charge time measurement unit
	CTMUCONLbits.EDG1STAT = 1;	// Edge 1 occurence

	ADCON0 = 0x75;				// Channel to internal diode
	ADCON1 = 0x00;
	ADCON2 = 0x8A;				// Right justified ADC result format

	ADCON0bits.GO = 1;			// Start conversion
	while(ADCON0bits.GO);
	temp = ADRES;				// Get ADC value, inversely proportional to temperature (needs to be calibrated first)

	CTMUCONHbits.CTMUEN = 0;	// Disable charge time measurement unit

	return temp;

}


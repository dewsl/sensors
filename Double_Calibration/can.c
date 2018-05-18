/*! 
 * \file  can.c
 * 
 * \author Raphael Victor L. Canseco
 * \date July 2014
 * \version 1.2
 *  
 * Project: Senslope \n
 * Processor:      pic18f2585 \n
 * Compiler:       c18 \n
 *
 * Version 1.2 \n
 * This verison functions are added for extended ids
 * \n
 * Version 1.1
 * In this version the target improvement is the arbitration technique being done
 * The node would wait till it could send its message. The error warning for
 * the CAN bus would also be implemented.
 * 
 *
 */

#include <p18cxxx.h>
//#include "system.h"
#include "can.h"
#include <timers.h>
#include <stdio.h>
#include <usart.h>


extern char string[64];

//****************************** CAN source codes *******************

/*old source*/
/**
 * 
 * @brief      configures the can module of the microcontroller
 *
 * This function configures the can to run in 32kbps. It configures to run
 * in standard mode using the 11bit identifier. The mode of the module is
 * configured as mode0 or legacy CAN.This function also initializes the CAN
 * buffer to be used which is the dedicated RXB0. Filter0 and mask0 are also
 * pointed to the buffer RXB0.
 *
 * @param [out]     buffer        location for messages
 *
 *
 * @retval 0    successful
 */
void can_initialize (CANDATA *buffer)
{
	CANCON |= 0b10000000;							// Put module in config mode
	while((CANSTAT & 0b10000000) != 0b10000000 );	// Wait until in confg mode

	//BRGCON1 = 0b01001001;							// set baud rate values (crystal 01001001, internal 01000011)
	//internal
        BRGCON1= 0b01000011;                                                    //sjw 2TQ ,BRP =4,

        BRGCON2 = 0b11111111;                                                   // phase seg1 = 8Tq, prop seg = 8Tq,
	BRGCON3 = 0b00000111;                                                   //phase seg2 = 8Tq


	CIOCON |= 0b00100000;							// Set TXDRIVE and CAN Capture modes
	ECANCON &= ~0b11000000;							// Set ECAN functional mode
	RXB0CON &= ~0b01100100;							// Set RXB0 buffer modes


        can_set_filter_id_standard(1,0);  //set filter 0 to accept id 1 (just make sure bits corresponding to > 32 must be zero
        can_set_filter_id_standard(1,1);  //set filter 1 to accept id 1 from master

        can_set_mask_standard(0x07E0,0);    //set mask0 to look at the bits corressponding to > 32 so that all ids below that could pass freely
        can_set_mask_standard(0x07E0,1);    //set mask1 to look into each bit

         MSEL0bits.FIL0_0 =0;               //filter 0 uses acceptance mask 0
         MSEL0bits.FIL0_1 =0;
         MSEL0bits.FIL1_0 =1;               //filter 1 uses acceptance mask 1
         MSEL0bits.FIL1_1 =0;


        RXFBCON0 = 0x00;                    //filter 1 is associated in buffer0
         /*initialize buffer*/
         buffer->id = 0;
         buffer->data1 = 0;
         buffer->data2 = 0;
         buffer->data3 = 0;
         buffer->data4 = 0;
         buffer->data5 = 0;
         buffer->data6 = 0;
         buffer->data7 = 0;
         buffer->data8 = 0;
         buffer->dlc = 0;

         PIR3bits.RXB0IF = 0; //set flag to zero
         /*enable interrupt*/
         PIE3bits.RXB0IE = 1;




	CANCON &= ~0b11100000;							// Put module in normal mode
	while((CANSTAT & 0b11100000) != 0b00000000 );	// Wait until in normal mode
        return;
}



/**
 * 
 * @brief       This function checks CAN errors
 *
 * This function looks at the flags of the can module                  \n
 * Error Warning bit       -> rxwarn or txwarn                         \n
 * Receiver Warning        -> 127> rcv error >95                       \n
 * Receiver Bus Passive    -> rcv error > 127                          \n
 * Transmitter Bus Passive -> tx error > 127                           \n
 * Transmitter Bus-Off     -> tx > 255 (would turn off the bus )       \n
 * Receiver Bus overflow   -> buffer overflow                          \n
 *
 *
 *
 * @retval 0    no error
 * @retval 1    rx warning only
 * @retval 2    tx warning only
 * @retval 4    rx passive only
 * @retval 8    tx passive only
 * @retval 16   tx buss off only
 * @retval 32   buffer 0 overflow only
 * @retval 64   buffer 1 overflow only
 * @retval others combination of errors
 *
 */
char can_check_errors(void){
    char error;
    error = 0;
    if(COMSTATbits.RXWARN){
        sprintf(string,(const far rom char *)"\n\rRx Warning\n\r");
        putsUSART(string);
        error |= 0x01;
    }
    if(COMSTATbits.TXWARN){
        sprintf(string,(const far rom char *)"\n\rTx Warning\n\r");
        putsUSART(string);
        error |= 0x02;
    }
    if(COMSTATbits.RXBP){
        sprintf(string,(const far rom char *)"\n\rRx Bus Passive\n\r");
        putsUSART(string);
        error |= 0x04;
    }
     if(COMSTATbits.TXBP){
        sprintf(string,(const far rom char *)"\n\rTx Bus Passive\n\r");
        putsUSART(string);
        error |= 0x08;
    }
    if(COMSTATbits.TXBO){
        sprintf(string,(const far rom char *)"\n\rTx Bus off\n\r");
        putsUSART(string);
        error |= 0x10;
    }
    if(COMSTATbits.RXB0OVFL){
        sprintf(string,(const far rom char *)"\n\rRxBuffer 0 overflow\n\r");
        putsUSART(string);
        error |= 0x20;
    }
    if(COMSTATbits.RXB1OVFL){
        sprintf(string,(const far rom char *)"\n\rRxBuffer 1 overflow\n\r");
        putsUSART(string);
        error |= 0x40;
    }

    return error;



}

/**
 * 
 * @brief       This function sets the filterid for a specific filter
 *
 * This function assumes standard length identifier and accepts
 * only upto 6 filters. This could set id upto 11 bits
 *
 *
 * @param [in]  id          11bit CAN id
 * @param [in]  filternum   filter to be modified from 0-5
 *
 * @retval 1    id not within range of 11bits
 * @retval 2    filter not within range of standard 6 filters
 * @retval 0    successfull
 */
char can_set_filter_id_standard(unsigned int id,unsigned char filternum)
{
    if(id > 2048 || id < 1) //id not within range of 11 bit identifier
        return 1;

    if(filternum > 6 || filternum < 0) //filter not within range
        return 2;                      //to get more use mode 1 or 2
                                       // then disable this check

    switch(filternum)
    {
        case 0:
            RXF0SIDH = id >> 3;   //get upper 8 bit from 11 bits
            RXF0SIDL = ((id & 0x0007) << 5)& 0xE0; //place the lower 3 bits of id in the upper 3 bits of this register
            RXF0SIDL &= ~0x08;              //clear bit 3 for standar ids
            break;
        case 1:
            RXF1SIDH = id >> 3;   //get upper 8 bit from 11 bits
            RXF1SIDL = ((id & 0x0007) << 5)& 0xE0; //place the lower 3 bits of id in the upper 3 bits of this register
            RXF1SIDL &= ~0x08;              //clear bit 3 for standar ids
            break;
            break;
        case 2:
            RXF2SIDH = id >> 3;   //get upper 8 bit from 11 bits
            RXF2SIDL = ((id & 0x0007) << 5)& 0xE0; //place the lower 3 bits of id in the upper 3 bits of this register
            RXF2SIDL &= ~0x08;              //clear bit 3 for standar ids
            break;
            break;
        case 3:
            RXF3SIDH = id >> 3;   //get upper 8 bit from 11 bits
            RXF3SIDL = ((id & 0x0007) << 5)& 0xE0; //place the lower 3 bits of id in the upper 3 bits of this register
            RXF3SIDL &= ~0x08;              //clear bit 3 for standar ids
            break;
            break;
        case 4:
            RXF4SIDH = id >> 3;   //get upper 8 bit from 11 bits
            RXF4SIDL = ((id & 0x0007) << 5)& 0xE0; //place the lower 3 bits of id in the upper 3 bits of this register
            RXF4SIDL &= ~0x08;              //clear bit 3 for standar ids
            break;
            break;
        case 5:
            RXF5SIDH = id >> 3;   //get upper 8 bit from 11 bits
            RXF5SIDL = ((id & 0x0007) << 5)& 0xE0; //place the lower 3 bits of id in the upper 3 bits of this register
            RXF5SIDL &= ~0x08;              //clear bit 3 for standar ids
            break;
            break;
        default:
            break;

    }
    return 0;

}


/**
 * 
 * @brief       This function sets the mask used in CAN
 *
 * This function assumes standard length mask and accepts
 * only upto 2 masks,mask0 and mask1
 *
 *
 * @param [in]  mask          11bit CAN mask
 * @param [in]  masknum   mask to be modified from 0-1
 *
 * @retval 1    mask not within range of 11bits
 * @retval 2    masknum more than 1
 * @retval 0    successful
 */
char can_set_mask_standard(unsigned int mask,unsigned char masknum)
{
    if(mask > 2048 || mask < 0)
        return 1;           //mask not in range of 11 bit

    if(masknum > 1)
        return 2;           //mask availabe is only 2 can't give more

    switch(masknum)
    {
        case 0:
            RXM0SIDH = mask >> 3;   //get upper 8 bit from 11 bits
            RXM0SIDL = (mask & 0x0007) << 5; //place the lower 3 bits of mask in the upper 3 bits of this register
            RXM0SIDL &= ~0x08;              //clear bit 3 for mask to accept both
            break;
        case 1:
            RXM1SIDH = mask >> 3;   //get upper 8 bit from 11 bits
            RXM1SIDL = (mask & 0x0007) << 5; //place the lower 3 bits of mask in the upper 3 bits of this register
            RXM1SIDL &= ~0x08;              //clear bit 3 for mask to accept both
            break;
        default:
            break;


    }

    return 0;
}

/**
 * 
 * @brief       checks for data in can bus
 *
 * This function checks if data is available in the CAN bus. The message is placed
 * in the buffer pointer that is passed through this function.
 *
 *
 * @param [out]  buffer  datatype where all of the data is placed
 *
 *
 * @retval 0    nothing received
 * @retval !0   message received
 */
unsigned char can_check_for_datain(CANDATA *buffer)
{
    unsigned int temp;

    if(!PIR3bits.RXB0IF)          //is there message
        return 0;                 // no return

    /* filtered area
     * Messages would go here only if the messages got into the buffer by
     * passing through the filters established. The CAN filters for the node
     * should be configured to let messages with msgid of 1 to pass through
     * because these messages are assigned as master requests to node
     */

    //there is a message
    PIR3bits.RXB0IF = 0;              //clear flags
    RXB0CON &= ~0x80;			// Clear flag

    buffer->dlc = RXB0DLC;	     // Retrieve message length
    temp = RXB0SIDH;                 //since RXB0SIDH is only a byte it should be placed in an it for data movement
    buffer->id = (temp << 3) & 0x07F8;
    buffer->id |= (RXB0SIDL >> 5) & 0x0007;

    buffer->data1 = RXB0D0;			// Retrieve data
    buffer->data2 = RXB0D1;
    buffer->data3 = RXB0D2;
    buffer->data4 = RXB0D3;
    buffer->data5 = RXB0D4;
    buffer->data6 = RXB0D5;
    buffer->data7 = RXB0D6;
    buffer->data8 = RXB0D7;


    return 1;


}
/**
 * 
 * @brief       sends the data in the CAN buffer with repeat for arbitration loss
 *
 * This function sends data to the CAN bus. It waits but if message was not received
 * till the specified time in the timeout it will then exit. If a send fail due
 * to arbitration was experienced a resend would be also done assuming it is
 * also within the specified timeout.
 * This function assumes an standard identifier
 *
 * Requires:
 *  Timer0
 *
 *
 * @param [in]  buffer    buffer of what to send
 * @param [in]  timeout   number of instruction cycle before skipping
 *
 *
 *
 * @retval 0    successful
 * @retval 1    timed out
 */
unsigned int can_send_data_with_arb_repeat(CANDATA *buffer,unsigned int timeout)
{
    unsigned int temp,status,timeout_counter;
    status = 0;

    CANCON &= ~0b00001110;                              // Clear WIN bits
    CANCON |=  0b00001000;                              // Access TXB0 instead of RXB0
                                                        // TXB0 is not in access bank. Access it through RXB0
    RXB0CON |= 0x03;                                    // Set transmit priority
    RXB0DLC = buffer->dlc;                              // Save DLC value

    /*modification for 11 bit transmission*/
    temp = buffer->id;
    RXB0SIDH = temp >> 3;
    RXB0SIDL &= 0x1F;                                   //zero out upper bits
    /*place the lower 3 bits of id to upper 3 bits of register */
    RXB0SIDL = (((temp & 0x0007) << 5) & 0xE0);
    RXB0SIDL &= ~0x08;                                  //zero out exide for standard id transmission


    RXB0D0 = buffer->data1;                             // Put data
    RXB0D1 = buffer->data2;
    RXB0D2 = buffer->data3;
    RXB0D3 = buffer->data4;
    RXB0D4 = buffer->data5;
    RXB0D5 = buffer->data6;
    RXB0D6 = buffer->data7;
    RXB0D7 = buffer->data8;

    RXB0CON |= 0x08;                                    // Request transmision
    WriteTimer0(0);                                     // Restart timer;
    timeout_counter = ReadTimer0();
    while ((RXB0CON & 0x08) != 0x00){
                                                        // Wait until sent and not yet time out
        if(RXB0CON & 0x20)                              //lost to arbitration
            RXB0CON |= 0x08;                            // Request transmision again

        timeout_counter = ReadTimer0();
        if(timeout_counter > timeout){                  //timedout
            status = 1;
            break;
        }
    }

    CANCON &= ~0b00001110;		        // Return access  to RXB0
    return status;

}

/******************************************************************************
 * Extended identifier commands
 ******************************************************************************/
/**
 * 
 * @brief      configures the can module of the microcontroller
 *
 * This function configures the can to run in 32kbps. It configures to run
 * in extended mode using the 29bit identifier. The mode of the module is
 * configured as mode0 or legacy CAN.This function also initializes the CAN
 * buffer to be used which is the dedicated RXB0. Filter0 and mask0 are also
 * pointed to the buffer RXB0.
 *
 * @param [out]         buffer        location for messages
 *
 *
 * @retval 0    successful
 */
void can_initialize_extended (CANDATA_EXTENDED *buffer)
{
	CANCON |= 0b10000000;				// Put module in config mode
	while((CANSTAT & 0b10000000) != 0b10000000 );	// Wait until in confg mode

        /*Sets the CAN bitrate to 32kbps*/
	//BRGCON1 = 0b01001001;		// set baud rate values (crystal 01001001, internal 01000011)
	//internal
        BRGCON1= 0b01000011;            //sjw 2TQ ,BRP =4,

        BRGCON2 = 0b11111111;           // phase seg1 = 8Tq, prop seg = 8Tq,
	BRGCON3 = 0b00000111;           //phase seg2 = 8Tq


	CIOCON |= 0b00100000;		// Set TXDRIVE and CAN Capture modes
	ECANCON &= ~0b11000000;	        // Set ECAN functional mode
	RXB0CON &= ~0b01100100;	        // Set RXB0 buffer modes
        RXB0CON &= 0b10011111;          //zero out bit 5 and bit 6 to accept extended and standard


        //only accept from master with message id of 1
        can_set_filter_id_extended(1,0);  //set filter 0 to accept id 1 
        can_set_filter_id_extended(1,1);  //set filter 1 to accept id 1 

        //check each bit for masking
        can_set_mask_extended(0x1FFFFFFF,0);    //set mask0 to look at everybit
        can_set_mask_extended(0x1FFFFFFF,1);    //set mask1 to look into each bit

         MSEL0bits.FIL0_0 =0;               //filter 0 uses acceptance mask 0
         MSEL0bits.FIL0_1 =0;
         MSEL0bits.FIL1_0 =1;               //filter 1 uses acceptance mask 1
         MSEL0bits.FIL1_1 =0;


         RXFBCON0 = 0x00;                    //filter 1 is associated in buffer0
         /*initialize buffer*/
         buffer->id = 0;
         buffer->data1 = 0;
         buffer->data2 = 0;
         buffer->data3 = 0;
         buffer->data4 = 0;
         buffer->data5 = 0;
         buffer->data6 = 0;
         buffer->data7 = 0;
         buffer->data8 = 0;
         buffer->dlc = 0;

         PIR3bits.RXB0IF = 0; //set flag to zero
         /*enable interrupt*/
         PIE3bits.RXB0IE = 1;




	CANCON &= ~0b11100000;                          // Put module in normal mode
	while((CANSTAT & 0b11100000) != 0b00000000 );	// Wait until in normal mode
        return;
}



/**
 * 
 * @brief       This function sets the filterid for a specific filter
 *
 * This function assumes extended identifier and accepts
 * only upto 6 filters. The id could be upto 29bits.
 *
 *
 * @param [in]  id          29bit CAN id
 * @param [in]  filternum   filter to be modified from 0-5
 *
 * 
 * @retval 2    filter not within range of standard 6 filters
 * @retval 0    successfull
 */
char can_set_filter_id_extended(unsigned long id,unsigned char filternum)
{


    if(filternum > 6 || filternum < 0)                      //filter not within range
        return 2;                                           //to get more use mode 1 or 2
                                                            // then disable this check

    switch(filternum)
    {
        case 0:
            RXF0EIDL = id & 0x000000FF;                     //bits 0->7 of the 29bit id
            RXF0EIDH = (id >> 8) &  0x000000FF;             //bits 8->15
            // place in this format 20/19/18/sRR/exid/x/17/16
            RXF0SIDL = (id >> 16) &  0x00000003;            //place 17/16
            RXF0SIDL |= (id >> 13) & 0x000000E0;            //place 20/19/18
            RXF0SIDL |= 0x08;                               //accept extended
            RXF0SIDH = (id >> 21)& 0x000000FF ;             //place 21->28
            break;
        case 1:
            RXF1EIDL = id & 0x000000FF;                     //bits 0->7 of the 29bit id
            RXF1EIDH = (id >> 8) &  0x000000FF;             //bits 8->15
            // place in this format 20/19/18/sRR/exid/x/17/16
            RXF1SIDL = (id >> 16) &  0x00000003;            //place 17/16
            RXF1SIDL |= (id >> 13) & 0x000000E0;            //place 20/19/18
            RXF1SIDL |= 0x08;                               //accept extended
            RXF1SIDH = (id >> 21)& 0x000000FF ;             //place 21->28
            break;
        case 2:
            RXF2EIDL = id & 0x000000FF;                     //bits 0->7 of the 29bit id
            RXF2EIDH = (id >> 8) &  0x000000FF;             //bits 8->15
            // place in this format 20/19/18/sRR/exid/x/17/16
            RXF2SIDL = (id >> 16) &  0x00000003;            //place 17/16
            RXF2SIDL |= (id >> 13) & 0x000000E0;            //place 20/19/18
            RXF2SIDL |= 0x08;                               //accept extended
            RXF2SIDH = (id >> 21)& 0x000000FF ;             //place 21->28
            break;
        case 3:
            RXF3EIDL = id & 0x000000FF;                     //bits 0->7 of the 29bit id
            RXF3EIDH = (id >> 8) &  0x000000FF;             //bits 8->15
            // place in this format 20/19/18/sRR/exid/x/17/16
            RXF3SIDL = (id >> 16) &  0x00000003;            //place 17/16
            RXF3SIDL |= (id >> 13) & 0x000000E0;            //place 20/19/18
            RXF3SIDL |= 0x08;                               //accept extended
            RXF3SIDH = (id >> 21)& 0x000000FF ;             //place 21->28
            break;
        case 4:
            RXF4EIDL = id & 0x000000FF;                     //bits 0->7 of the 29bit id
            RXF4EIDH = (id >> 8) &  0x000000FF;             //bits 8->15
            // place in this format 20/19/18/sRR/exid/x/17/16
            RXF4SIDL = (id >> 16) &  0x00000003;            //place 17/16
            RXF4SIDL |= (id >> 13) & 0x000000E0;            //place 20/19/18
            RXF4SIDL |= 0x08;                               //accept extended
            RXF4SIDH = (id >> 21)& 0x000000FF ;             //place 21->28
            break;
        case 5:
            RXF5EIDL = id & 0x000000FF;                     //bits 0->7 of the 29bit id
            RXF5EIDH = (id >> 8) &  0x000000FF;             //bits 8->15
            // place in this format 20/19/18/sRR/exid/x/17/16
            RXF5SIDL = (id >> 16) &  0x00000003;           //place 17/16
            RXF5SIDL |= (id >> 13) & 0x000000E0;           //place 20/19/18
            RXF5SIDL |= 0x08;                              //accept extended
            RXF5SIDH = (id >> 21)& 0x000000FF ;            //place 21->28
            break;
        default:
            break;

    }
    return 0;

}


/**
 * 
 * @brief       This function sets the mask for the masknumber
 *
 * This function assumes extended length mask and accepts
 * only upto 2 masks. It can configure mask0 and mask1
 *
 *
 * @param [in]  mask          11bit CAN mask
 * @param [in]  masknum   mask to be modified from 0-1
 *
 * @retval 2    masknum more than 1,
 * @retval 0    successful
 */
char can_set_mask_extended(unsigned long mask,unsigned char masknum)
{


    if(masknum > 1)
        return 2;           //mask availabe is only 2 can't give more

    switch(masknum)
    {
        case 0:
            RXM0EIDL = mask & 0x000000FF;        //bits 0->7 of the 29bit id
            RXM0EIDH = (mask >> 8) &  0x000000FF; //bits 8->15
            // place in this format 20/19/18/sRR/exid/x/17/16
            RXM0SIDL = (mask >> 16) &  0x00000003;                //place 17/16
            RXM0SIDL |= (mask >> 13) & 0x000000E0;                //place 20/19/18
            RXM0SIDL |= 0x08;                                   //accept extended
            RXM0SIDH = (mask >> 21)& 0x000000FF ;                 //place 21->28
            break;
        case 1:
            RXM1EIDL = mask & 0x000000FF;        //bits 0->7 of the 29bit id
            RXM1EIDH = (mask >> 8) &  0x000000FF; //bits 8->15
            // place in this format 20/19/18/sRR/exid/x/17/16
            RXM1SIDL = (mask >> 16) &  0x00000003;                //place 17/16
            RXM1SIDL |= (mask >> 13) & 0x000000E0;                //place 20/19/18
            RXM1SIDL |= 0x08;                                   //accept extended
            RXM1SIDH = (mask >> 21)& 0x000000FF ;                 //place 21->28
            break;
        default:
            break;


    }

    return 0;
}

/**
 * 
 * @brief       sends the data in the CAN buffer with repeat for arbitration loss
 *
 * This function sends data to the CAN bus. It waits but if message was not received
 * till the specified time in the timeout it will then exit. If a send fail due
 * to arbitration was experienced a resend would be also done assuming it is
 * also within the specified timeout.
 * This function assumes an extended identifier
 *
 * Requires:
 *  Timer0
 *  
 *
 * @param [in]  buffer    buffer of what to send
 * @param [in]  timeout   contains how many instruction cycles before skipping
 *
 *
 * @retval 0    successful
 * @retval 1    timed out
 */
unsigned int can_send_data_with_arb_repeat_extended(CANDATA_EXTENDED *buffer,unsigned int timeout)
{
    unsigned int timeout_counter,status;
    status = 0;

    /*use RXBO as pointer to TXBO*/
    CANCON &= ~0b00001110;                              // Clear WIN bits
    CANCON |=  0b00001000;                              // Access TXB0 instead of RXB0
                                                        // TXB0 is not in access bank. Access it through RXB0
    RXB0CON |= 0x03;                                    // Set transmit priority
    RXB0DLC = buffer->dlc;                              // Save DLC value


    /*29 bit transmission*/
    RXB0EIDL = buffer->id & 0x000000FF;                 //bits 0->7 of the 29bit id
    RXB0EIDH = (buffer->id >> 8) &  0x000000FF;         //bits 8->15
    // place in this format 20/19/18/sRR/exid/x/17/16
    RXB0SIDL = (buffer->id >> 16) &  0x00000003;        //place 17/16
    RXB0SIDL |= (buffer->id >> 13) & 0x000000E0;        //place 20/19/18
    RXB0SIDL |= 0x08;                                   // extended id marker
    RXB0SIDH = (buffer->id >> 21)& 0x000000FF ;         //place 21->28


    RXB0D0 = buffer->data1;                             // Put data
    RXB0D1 = buffer->data2;
    RXB0D2 = buffer->data3;
    RXB0D3 = buffer->data4;
    RXB0D4 = buffer->data5;
    RXB0D5 = buffer->data6;
    RXB0D6 = buffer->data7;
    RXB0D7 = buffer->data8;

    RXB0CON |= 0x08;                                    // Request transmision
    WriteTimer0(0);                                     // Restart timer;
    timeout_counter = ReadTimer0();
    while ((RXB0CON & 0x08) != 0x00 ){
                                                        // Wait until sent and not yet time out
        if(RXB0CON & 0x20)                              //lost to arbitration
            RXB0CON |= 0x08;                            // Request transmision again

        timeout_counter = ReadTimer0();
        if(timeout_counter > timeout){                  //timedout
            status = 1;
            break;
        }
    }

    /*Put back pointer to RXBO*/
    CANCON &= ~0b00001110;                              // Return access  to RXB0
    return status;

}


/**
 * 
 * @brief       checks for data in can bus
 *
 * This function checks if data is available in the CAN bus. The message is placed
 * in the buffer pointer that is passed through this function.
 *
 *
 * @param [out]  buffer  datatype where all of the data is placed
 *
 *
 * @retval 0    nothing received
 * @retval !0   message received
 */
unsigned char can_check_for_datain_extended(CANDATA_EXTENDED *buffer)
{
    unsigned long temp;

    if(!PIR3bits.RXB0IF)                                    //is there message
        return 0;                                           // no return

    /* filtered area
     * Messages would go here only if the messages got into the buffer by
     * passing through the filters established. The CAN filters for the node
     * should be configured to let messages with msgid of 1 to pass through
     * because these messages are assigned as master requests to node
     */

    //there is a message
    PIR3bits.RXB0IF = 0;                                    // clear interrupt flag
    RXB0CON &= ~0x80;                                       // Clear flag

    buffer->dlc = RXB0DLC;                                  // Retrieve message length

    buffer->id =  RXB0EIDL    & 0x000000FF;                 //bits 0->7 of the 29bit id
    temp = RXB0EIDH;
    buffer->id |= (temp << 8) &  0x0000FF00;                //bits 8->15
    // place in this format 20/19/18/sRR/exid/x/17/16
    temp = RXB0SIDL;
    buffer->id |= (temp << 16) & 0x00030000;                //place 17/16
    buffer->id |= (temp << 13) & 0x001C0000;                //20/19/18
    temp = RXB0SIDH;
    buffer->id |= (temp << 21) & 0x1FE00000;                //place 21->28

    buffer->data1 = RXB0D0;                                 // Retrieve data
    buffer->data2 = RXB0D1;
    buffer->data3 = RXB0D2;
    buffer->data4 = RXB0D3;
    buffer->data5 = RXB0D4;
    buffer->data6 = RXB0D5;
    buffer->data7 = RXB0D6;
    buffer->data8 = RXB0D7;

    
    return 1;
}

/**
 *
 * @brief       sets the baud to 800kbps
 *
 * This function sets the baud to 800kbps assumming that the clock used is
 * 8Megahertz.
 *
 *
 *
 *
 *
 * @retval 0    successful
 */
void can_set_baud_to_800kbps(void) {

        BRGCON1bits.SJW0 = 0;       //sync jump set to 0b00 -> 1TQ
        BRGCON1bits.SJW1 = 0;
        BRGCON1 &= 0xC0;           //zero out BRP bits
        BRGCON1 |= 0x00;          //0b00000 = 1, 0b11111 = 63

        BRGCON2bits.SEG2PHTS = 0; //max of pheg 1 is information processing time
        BRGCON2bits.SAM = 1;      // 1-> sample 3 times
        /*phase segment 2*/
        BRGCON2bits.SEG1PH2 = 0; //0b000 = 1Tq 0b111 = 8Tq for phase seg1
        BRGCON2bits.SEG1PH1 = 0;
        BRGCON2bits.SEG1PH0 = 0;

        /*propagation time*/
        BRGCON2bits.PRSEG2 = 0; //0b000 = 1Tq 0b111 = 8Tq for prop seg
        BRGCON2bits.PRSEG1 = 0;
        BRGCON2bits.PRSEG0 = 0;

        /*wakdis not implemented*/

        /*phase segment 2 only useful if seg2phts = 1*/
        BRGCON3bits.SEG2PH2 =0;
        BRGCON3bits.SEG2PH1 =0;
        BRGCON3bits.SEG2PH0 =0;

}

/**
 * 
 * @brief       decides which command is given by the master
 *
 * It checks the msgid and verifies if it is a broadcast or node specific.
 * It lets pass all of the broadcast command. If a node specific command was
 * seen, a check would be done to see if it is for the specific node. If it is
 * not it blocks it and just outputs a zero. Requires that a message is in the
 * buffer.
 *
 * @param [in]  buffer      contains the data that was received
 * @param [in]  nodeid      unique id of the specific CAN node
 *
 *
 * @retval 0    command is not for the node
 * @retval !0   command given by the master
 */
unsigned char can_process_commands_extended(CANDATA_EXTENDED *buffer,unsigned long nodeid)
{
    unsigned long idH;
    unsigned long idL;
    unsigned int command;
    
    command = 0;
    if(buffer->data1 > MAXCANBROADCAST){
        //node specific
        idH = buffer->data2;
        idL = buffer->data3;
        if(( idH<<8 | idL) != nodeid) //id did not match
            command = 0;
        else
            command = buffer->data1;
    }else // broadcast
        command = buffer->data1;

    return command;

}

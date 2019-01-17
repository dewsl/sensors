/* 
 * File:   can.h
 * Author: rap
 *
 * Created on July 3, 2014, 10:14 AM
 */
/*!
 * \file  can.h
 *
 * \author Raphael Victor L. Canseco
 * \date July 2014
 * \version 1.3
 *
 * Project: Senslope
 * Processor:      pic18fk80
 * Compiler:       c18
 *
 * this file contains the definitions used and the prototype for the can.c
 * functions
 * Version 1.3
 * Converted for pic18fk80
 *
 * Version 1.2
 * This verison functions are added for extended ids
 *
 * Version 1.1
 * In this version the target improvement is the arbitration technique being done
 * The node would wait till it could send its message. The error warning for
 * the CAN bus would also be implemented.
 *
 *
 */

#ifndef CAN_H
#define	CAN_H

#ifdef	__cplusplus
extern "C" {
#endif


#define MAXCANBROADCAST                    99 //!< defines the limit for broadcast commands



/*Obsolete functions*/
void can_send (	unsigned int id, unsigned char data1,unsigned char data2,
		unsigned char data3,unsigned char data4,unsigned char data5,
		unsigned char data6,unsigned char data7,unsigned char data8);

void can_receive (unsigned int *id,unsigned char *data1,unsigned char *data2,
		  unsigned char *data3,	unsigned char *data4,unsigned char *data5,
		  unsigned char *data6,unsigned char *data7,unsigned char *data8);



/*general CAN functions*/
char can_check_errors(void);


/**
 * @brief struct for the CAN data used CAN communication
 *
 * This struct contains the data for a standard CAN communication data
 * where the id is 11 bit long.
 */
typedef struct {
    unsigned int id;        //!< message identifier 11 bit
    unsigned char dlc;      //!< number of bytes passed in the transaction
    unsigned char data1;    //!< first data or databyte0 in other documents
    unsigned char data2;    //!< 2nd data or databyte1 in other documents
    unsigned char data3;    //!< 3rd data or databyte2 in other documents
    unsigned char data4;    //!< 4rth data or databyte3 in other documents
    unsigned char data5;    //!< 5th data or databyte4 in other documents
    unsigned char data6;    //!< 6th data or databyte5 in other documents
    unsigned char data7;    //!< 7th data or databyte6 in other documents
    unsigned char data8;    //!< 8th data or databyte7 in other documents
}CANDATA;

/*Functions for standard CAN*/
void can_initialize (CANDATA *buffer);
char can_set_filter_id_standard(unsigned int id,unsigned char filternum);
char can_set_mask_standard(unsigned int mask,unsigned char masknum);
unsigned char can_check_for_datain(CANDATA *buffer);
unsigned int can_send_data_with_arb_repeat(CANDATA *buffer,unsigned int timeout);


/**
 * @brief struct for the extended CAN data used CAN communication
 *
 * This struct contains the data for a extended CAN communication data
 * where the id is 29 bit long.
 */
typedef struct {
    unsigned long id;       //!< message identifier 29 bit
    unsigned char dlc;      //!< number of bytes passed in the transaction
    unsigned char data1;    //!< first data or databyte0 in other documents
    unsigned char data2;    //!< 2nd data or databyte1 in other documents
    unsigned char data3;    //!< 3rd data or databyte2 in other documents
    unsigned char data4;    //!< 4rth data or databyte3 in other documents
    unsigned char data5;    //!< 5th data or databyte4 in other documents
    unsigned char data6;    //!< 6th data or databyte5 in other documents
    unsigned char data7;    //!< 7th data or databyte6 in other documents
    unsigned char data8;    //!< 8th data or databyte7 in other documents
}CANDATA_EXTENDED;

/*functions for extended CAN*/
void can_initialize_extended (CANDATA_EXTENDED *buffer);
char can_set_filter_id_extended(unsigned long id,unsigned char filternum);
char can_set_mask_extended(unsigned long mask,unsigned char masknum);
unsigned int can_send_data_with_arb_repeat_extended(CANDATA_EXTENDED *buffer,unsigned int timeout);
unsigned char can_check_for_datain_extended(CANDATA_EXTENDED *buffer);
unsigned char can_process_commands_extended(CANDATA_EXTENDED *buffer,unsigned long nodeid);


#ifdef	__cplusplus
}
#endif

#endif	/* CAN_H */


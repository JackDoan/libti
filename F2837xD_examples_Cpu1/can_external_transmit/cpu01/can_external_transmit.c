//###########################################################################
//
// FILE:   can_external_transmit.c
//
// TITLE:  Example to demonstrate CAN external transmission
//
//! \addtogroup cpu01_example_list
//! <h1>CAN-A to CAN-B External Transmit (can_external_transmit)</h1>
//!
//! This example initializes CAN module A and CAN module B for external
//! communication. CAN-A module is setup to transmit incrementing data for "n"
//! number of times to the CAN-B module, where "n" is the value of TXCOUNT.
//! CAN-B module is setup to trigger an interrupt service routine (ISR) when
//! data is received. An error flag will be set if the transmitted data doesn't
//! match the received data.
//!
//!


/*
 * Yikes! Here there be dragons!
 * spruhm8f, page 2183. 22.3.1.note: use 32-bit accesses whenever possible when touching
 * CAN registers. But, if the compiler splits the access, set the LSB, then MSB.
 */


//
//###########################################################################
// $TI Release: F2837xD Support Library v210 $
// $Release Date: Tue Nov  1 14:46:15 CDT 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//
// Included Files
//
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_can.h"
#include "driverlib/can.h"
#include "elmoCANObjs.h"

//
// Defines
//
#define TXCOUNT  100
#define MSG_DATA_LENGTH    4
#define TX_MSG_OBJ_ID    1
#define RX_MSG_OBJ_ID    1

//
// Globals
//
volatile unsigned long i;
volatile uint32_t txMsgCount = 0;
volatile uint32_t rxMsgCount = 0;
volatile uint32_t errorFlag = 0;
unsigned char elmoRequestData[8];
unsigned char elmoResponseData[8];
tCANMsgObject sTXCANMessage;
tCANMsgObject elmoResponse;
unsigned char canOpenTest[8] =          {0x40, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char getControlMode[8] =          {0x40, 0x61, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char setTorqueModeTraj[8] =  {0x22, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00};
unsigned char getStatusWord[8] =        {0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char getTorqueWord[8] =        {0x40, 0x77, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char setTorqueWord50percent[8] =        {0x22, 0x71, 0x60, 0x00, 50, 0x00, 0x00, 0x00};
unsigned char setControlWordShutdown[8] =       {0x22, 0x40, 0x60, 0x00, 0b00000110, 0x00, 0x00};
unsigned char setControlWordSwitchOn[8] =       {0x22, 0x40, 0x60, 0x00, 0b00001111, 0x00, 0x00};
unsigned char setControlWord[8] =       {0x22, 0x40, 0x60, 0x00, 0b10001011, 0x00, 0x00};

unsigned char messageBuilderBuffer[8];




void sdoMessageBuilder(unsigned char* buf, sdo_ccs_t ccs, elmo_obj_t obj,
                       unsigned int subindex, unsigned long data) {
    buf[0] = (unsigned char)ccs;
    buf[1] = (unsigned char)(obj & 0x00FF);
    buf[2] = (unsigned char)(obj >> 16);
    buf[3] = (unsigned char)(subindex);
    buf[4] = (unsigned char)( (data      ) & 0x000000FF);
    buf[5] = (unsigned char)( (data >>  8) & 0x000000FF);
    buf[6] = (unsigned char)( (data >> 16) & 0x000000FF);
    buf[7] = (unsigned char)( (data >> 24) & 0x000000FF);
}

//
// Function Prototypes
//
tCANMsgObject elmoRequest;
int sendMsg = 1;
int getMsg = 0;
    uint32_t Base = CANB_BASE;
    uint32_t ObjID = RX_MSG_OBJ_ID;
    tCANMsgObject msg;
    tMsgObjType type = MSG_OBJ_TYPE_TX;


__interrupt void canbISR(void);

void main(void) {

    InitSysCtrl();
    InitGpio();
    GPIO_SetupPinMux(17, GPIO_MUX_CPU1, 2); //GPIO10 -  CANRXB
    GPIO_SetupPinOptions(17, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 2);  //GPIO8 - CANTXB
    GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_PUSHPULL);

    // Initialize the CAN controllers
    //CANInit(CANA_BASE);
    CANInit(CANB_BASE);

    // Setup CAN to be clocked off the PLL output clock
    //CANClkSourceSelect(CANA_BASE, 0);   // 500kHz CAN-Clock
    CANClkSourceSelect(CANB_BASE, 0);   // 500kHz CAN-Clock

    //
    // Set up the CAN bus bit rate to 500kHz for each module
    // This function sets up the CAN bus timing for a nominal configuration.
    // You can achieve more control over the CAN bus timing by using the
    // function CANBitTimingSet() instead of this one, if needed.
    // Additionally, consult the device data sheet for more information about
    // the CAN module clocking.
    //
    //CANBitRateSet(CANA_BASE, 200000000, 500000);
    CANBitRateSet(CANB_BASE, 200000000, 500000);

    // Enable interrupts on the CAN B peripheral.
    CANIntEnable(CANB_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    DINT; // Disable CPU interrupts

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    InitPieCtrl();
    IER = 0x0000; // Disable CPU interrupts
    IFR = 0x0000; //and clear all CPU interrupt flags

    // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
    InitPieVectTable();

    // This registers the interrupt handler in PIE vector table.
    EALLOW;
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 1; //STO as output
    PieVectTable.CANB0_INT = canbISR;
    EDIS;

    // Enable the CAN-B interrupt on the processor (PIE).
    PieCtrlRegs.PIEIER9.bit.INTx7 = 1;
    IER |= M_INT9;
    EINT;

    // Enable the CAN-B interrupt signal
    CANGlobalIntEnable(CANB_BASE, CAN_GLB_INT_CANINT0);

    //we're going to build a packet that makes an SDO request for


    //const int NMT_function_code = 0x0;
    //const int SDO_function code = 0x0; //TODO

    int elmo_SDO_TX_begin = 0x580;
    int elmo_SDO_TX_end   = 0x5FF;
    int elmo_SDO_RX_begin = 0x600;
    int elmo_SDO_RX_end   = 0x67F;
    uint32_t elmoID = 127;
    uint32_t elmoGID = 128;
    int elmo_bus_voltage_obj = 0x6079;


    GpioDataRegs.GPASET.bit.GPIO25 = 1; //set STO high



    uint32_t cob_id = (elmoID+elmo_SDO_RX_begin) & 0x7FF; //this is the COB-ID, and is always 11 bits
    elmoRequest.MsgID = cob_id;
    elmoRequest.MsgIDMask = 0;
    elmoRequest.Flags = MSG_OBJ_RX_INT_ENABLE;
    elmoRequest.MsgLen = 8;
    elmoRequest.Data = canOpenTest;
    elmoResponse.MsgID = elmoID+elmo_SDO_TX_begin;
    elmoResponse.MsgIDMask = 0;
    elmoResponse.Flags = 0;
    elmoResponse.MsgLen = 8;
    elmoResponse.Data = elmoResponseData;

    // Start CAN module A and B operations
    //CANEnable(CANA_BASE);
    CANEnable(CANB_BASE);
    CANMessageSet(Base, 2, &elmoResponse, MSG_OBJ_TYPE_RX);
    //msg = &canopen;
    while(1) {
        if(sendMsg == 1) {
            CANMessageSet(Base, ObjID, &elmoRequest, type);
            sendMsg = 0;
        }
        if(getMsg == 1) {
            CANMessageGet(CANB_BASE, 2, &elmoResponse, true);
            getMsg = 0;
        }
    }
}

// CAN B ISR - The interrupt service routine called when a CAN interrupt is
//             triggered on CAN module B.
__interrupt void canbISR(void) {
    uint32_t status;

    // Read the CAN-B interrupt status to find the cause of the interrupt
    status = CANIntStatus(CANB_BASE, CAN_INT_STS_CAUSE);

    // If the cause is a controller status interrupt, then get the status
    if(status == CAN_INT_INT0ID_STATUS) {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        status = CANStatusGet(CANB_BASE, CAN_STS_CONTROL);

        // Check to see if an error occurred.
        if(!(( (status  & ~(CAN_ES_RXOK) ) == 7) || /* status can be wither 7 or 0, depending on if it's been checked recently.*/
            (  (status  & ~(CAN_ES_RXOK) ) == 0))) {
            // Set a flag to indicate some errors may have occurred.
            errorFlag = status;
        }
    }
        // Check if the cause is the CAN-B receive message object 1
    else if(status == RX_MSG_OBJ_ID) {
        // Get the received message
        CANMessageGet(CANB_BASE, RX_MSG_OBJ_ID, &elmoResponse, true);

        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.  Clear the
        // message object interrupt.
        CANIntClear(CANB_BASE, RX_MSG_OBJ_ID);

        // Increment a counter to keep track of how many messages have been
        // received. In a real application this could be used to set flags to
        // indicate when a message is received.
        rxMsgCount++;

        // Since the message was received, clear any error flags.
        errorFlag = 0;
    }
    // If something unexpected caused the interrupt, this would handle it.
    else {
        //
        // Spurious interrupt handling can go here.
        //
    }

    CANGlobalIntClear(CANB_BASE, CAN_GLB_INT_CANINT0); // Clear the global interrupt flag for the CAN interrupt line
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;  // Acknowledge this interrupt located in group 9
}



void mapPDO() {

    //during NMT states OPERATIONAL and PRE-OPERATIONAL:
    //destroy the old PDO   => (PDO_OBJ.1) = 0b1
    //disable mapping       => (PDO_OBJ.0) = 0b0
    //modify PDO_OBJ as appropriate?
        //if there is a problem, drive responds with 0x06020000 or 0x06040041
    //enable mapping        => (PDO_OBJ.0) = (#mapped objs)
        //if there is a problem, drive responds with 0x06020000 or 0x06040042
    //enable the new PDO    => (PDO_OBJ.1) = 0b0

    //if the drive gets a RXPDO with more data than it has mapped, it will use the bytes
    //that it can, and then respond with an EMCY write
    //if the drive gets less data than it has mapped, it will send EMCY code 0x8210 0x21

    //this function maps statusword (0x6041), 16 bits, and "actual position" (0x6064), 32 bits, with type "synchronous" onto TPDO1:
    unsigned char txBuf[8];
    unsigned char rxBuf[8];

    sdoMessageBuilder(txBuf, OBJ_WRITE, TPDO_COMM_PARAM_1, 1, 0x81010080); //disable TPDO1, COBID0x181?
    sdoMessageBuilder(rxBuf, OBJ_READ, TPDO_COMM_PARAM_1, 1, 0); //drive responds

    sdoMessageBuilder(txBuf, OBJ_WRITE, TPDO_MAP_1, 0, 0); //clear tpdo map 1
    sdoMessageBuilder(rxBuf, OBJ_READ, TPDO_MAP_1, 0, 0); //drive responds

    int objToMap = (int)STATUS_WORD;
    sdoMessageBuilder(txBuf, OBJ_WRITE, TPDO_MAP_1, 1,
                      ( 0x10000000 | (__byte(&objToMap,0) << 8) | (__byte(&objToMap,1)) )); //map statusword onto index 1 with 16 bits
    sdoMessageBuilder(rxBuf, OBJ_READ, TPDO_MAP_1, 1, 0);

    objToMap = (int)ACTUAL_POSITION;
    sdoMessageBuilder(txBuf, OBJ_WRITE, TPDO_MAP_1, 2,
                          ( 0x20000000 | (__byte(&objToMap,0) << 8) | (__byte(&objToMap,1)) )); //map actual_position onto index 1 with 32 bits
    sdoMessageBuilder(rxBuf, OBJ_READ, TPDO_MAP_1, 2, 0); //drive responds

    sdoMessageBuilder(txBuf, OBJ_WRITE, TPDO_MAP_1, 0, 0x02000000); //set two objs mapped
    sdoMessageBuilder(rxBuf, OBJ_READ, TPDO_MAP_1, 0, 0); //drive responds

    sdoMessageBuilder(txBuf, OBJ_WRITE, TPDO_COMM_PARAM_1, 2, 0x01000000); //transmission type SYNC, on every SYNC.
    sdoMessageBuilder(rxBuf, OBJ_READ, TPDO_COMM_PARAM_1, 2, 0); //drive responds

    sdoMessageBuilder(txBuf, OBJ_WRITE, TPDO_COMM_PARAM_1, 1, 0x81010000); //enable TPDO1, COBID0x181?
    sdoMessageBuilder(rxBuf, OBJ_READ, TPDO_COMM_PARAM_1, 1, 0); //drive responds


}

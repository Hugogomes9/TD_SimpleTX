/*! ----------------------------------------------------------------------------
 *  @file    dw_main.c
 *  @brief   main loop for the DecaRanging application
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
/* Includes */
#include "compiler.h"
#include "port.h"

#include "instance.h"

#include "deca_types.h"

#include "deca_spi.h"
#include "stdio.h"

int dr_mode = 0;
int instance_mode = ANCHOR;

uint8 s1switch = 0;
int chan, tagaddr, ancaddr, prf;

int ranging = 0;

//Configuration for DecaRanging Modes (8 default use cases selectable by the switch S1 on EVK)
instanceConfig_t chConfig[8] ={
                    //mode 1 - S1: 7 off, 6 off, 5 off
                    {
                        2,              // channel
						3,              // preambleCode
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 2
                    {
                        2,              // channel
                        3,             // preambleCode
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 3
                    {
                        2,              // channel
                        9,             // preambleCode
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 4
                    {
                        2,              // channel
                        9,             // preambleCode
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 5
                    {
                        5,              // channel
                        3,             // preambleCode
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 6
                    {
                        5,              // channel
                        3,             // preambleCode
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 7
                    {
                        5,              // channel
                        9,             // preambleCode
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 8
                    {
                        5,              // channel
                        9,             // preambleCode
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    }
};


uint32 inittestapplication();

uint32 inittestapplication()
{
    uint32 devID ;
    int result;

    port_set_dw1000_slowrate();  //max SPI before PLLs configured is ~4M

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = instancereaddeviceid() ;
    if(DWT_DEVICE_ID != devID) //if the read of device ID fails, the DW1000 could be asleep
    {
        port_wakeup_dw1000();

        devID = instancereaddeviceid() ;
        // SPI not working or Unsupported Device ID
        if(DWT_DEVICE_ID != devID)
            return(-1) ;
        //clear the sleep bit - so that after the hard reset below the DW does not go into sleep
        dwt_softreset();
    }

    //reset the DW1000 by driving the RSTn line low
    reset_DW1000();

    result = instance_init() ;
    if (0 > result) return(-1) ; // Some failure has occurred

    port_set_dw1000_fastrate();
    devID = instancereaddeviceid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT DW1000 device
    {
        // SPI not working or Unsupported Device ID
        return(-1) ;
    }
    instance_mode = ANCHOR;

    instance_mode = TAG;

    instance_init_s(instance_mode);
    //dr_mode = decarangingmode(s1switch);

    chan = chConfig[dr_mode].channelNumber ;
    prf = (chConfig[dr_mode].pulseRepFreq == DWT_PRF_16M)? 16 : 64 ;

    instance_config(&chConfig[dr_mode]) ;                  // Set operating channel etc

    instancesettagsleepdelay(POLL_SLEEP_DELAY, BLINK_SLEEP_DELAY); //set the Tag sleep time

    instance_init_timings();

    return devID;
}

/*
 * @fn      main()
 * @brief   main entry point
**/
int dw_main(void)
{
    int i = 0;
    int toggle = 1;
    double range_result = 0;
    double avg_result = 0;
    int canSleep;

    peripherals_init();

    spi_peripheral_init();

    Sleep(1000);

    port_DisableEXT_IRQ(); 	//disable DW1000 IRQ until we configure the application

    instance_mode = ANCHOR;
    instance_mode = TAG;
	instance_get_addr();
    port_EnableEXT_IRQ();
    // main loop
    while(1)
    {
		instance_data_t* inst = instance_get_local_structure_ptr(0);
		canSleep = instance_run();

        //if delayed TX scheduled but did not happen after expected time then it has failed... (has to be < slot period)
        //if anchor just go into RX and wait for next message from tags/anchors
        //if tag handle as a timeout
        if((inst->monitor == 1) && ((portGetTickCnt() - inst->timeofTx) > inst->finalReplyDelay_ms))
        {
			inst->wait4ack = 0;

			if(instance_mode == TAG)
			{
				inst_processrxtimeout(inst);
			}
			else //if(instance_mode == ANCHOR)
			{
				dwt_forcetrxoff();	//this will clear all events
				//enable the RX
				inst->testAppState = TA_RXE_WAIT ;
			}
			inst->monitor = 0;
        }

        if(instancenewrange())
        {
        	int n, l = 0, /*txl = 0, rxl = 0,*/ aaddr, taddr, txa, rxa, rng, rng_raw;
            ranging = 1;
            //send the new range information to LCD and/or USB
            range_result = instance_get_idist();
            avg_result = instance_get_adist();
            //set_rangeresult(range_result);

            l = instance_get_lcount();
            //txl = instance_get_txl();
            //rxl = instance_get_rxl();
            aaddr = instancenewrangeancadd();
            taddr = instancenewrangetagadd();
            txa =  instancetxantdly();
            rxa =  instancerxantdly();
            rng = (int) (range_result*1000);
            rng_raw = (int) (instance_get_idistraw()*1000);
        }
        if(canSleep)__WFI();
    }


    return 0;
}




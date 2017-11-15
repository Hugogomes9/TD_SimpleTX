/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 8.0.0   2017-10-16

The MIT License (MIT)
Copyright (c) 2009-2017 Atollic AB

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

******************************************************************************
*/

/* Includes */
#include <stddef.h>
#include "stm32f10x.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "sleep.h"
#include "port.h"
//static uint8 tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0};
#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000

static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Buffer to store received frame. See NOTE 1 below. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;

/**
 * Application entry point.
 */

int main(void)
{
	/* Start with board specific hardware init. */
	    peripherals_init();

	    /* Reset and initialise DW1000. See NOTE 2 below.
	     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
	     * performance. */
	    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
	    spi_set_rate_low();
	    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
	    {
	        while (1)
	        { };
	    }
	    spi_set_rate_high();

	    /* Configure DW1000. */
	    dwt_configure(&config);

	    /* Loop forever receiving frames. */
	    while (1)
	    {
	        int i;

	        /* TESTING BREAKPOINT LOCATION #1 */

	        /* Clear local RX buffer to avoid having leftovers from previous receptions  This is not necessary but is included here to aid reading
	         * the RX buffer.
	         * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
	         * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. */
	        for (i = 0 ; i < FRAME_LEN_MAX; i++ )
	        {
	            rx_buffer[i] = 0;
	        }

	        /* Activate reception immediately. See NOTE 3 below. */
	        dwt_rxenable(DWT_START_RX_IMMEDIATE);

	        /* Poll until a frame is properly received or an error/timeout occurs. See NOTE 4 below.
	         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
	         * function to access it. */
	        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
	        { };

	        if (status_reg & SYS_STATUS_RXFCG)
	        {
	            /* A frame has been received, copy it to our local buffer. */
	            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
	            if (frame_len <= FRAME_LEN_MAX)
	            {
	                dwt_readrxdata(rx_buffer, frame_len, 0);
	            }

	            /* Clear good RX frame event in the DW1000 status register. */
	            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
	        }
	        else
	        {
	            /* Clear RX error events in the DW1000 status register. */
	            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
	        }
	    }

}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*
 * Minimal __assert_func used by the assert() macro
 * */
void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}

#ifdef USE_SEE
#ifndef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval sEE_FAIL.
  */
uint32_t sEE_TIMEOUT_UserCallback(void)
{
  /* Return with error code */
  return sEE_FAIL;
}
#endif
#endif /* USE_SEE */

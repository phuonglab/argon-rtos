/* ---------------------------------------------------------------------------------------*/
/*  @file:    startup_MKL43Z4.s                                                           */
/*  @purpose: CMSIS Cortex-M0P Core Device Startup File                                   */
/*            MKL43Z4                                                                     */
/*  @version: 1.3                                                                         */
/*  @date:    2014-8-21                                                                   */
/*  @build:   b140821                                                                     */
/* ---------------------------------------------------------------------------------------*/
/*                                                                                        */
/* Copyright (c) 1997 - 2014 , Freescale Semiconductor, Inc.                              */
/* All rights reserved.                                                                   */
/*                                                                                        */
/* Redistribution and use in source and binary forms, with or without modification,       */
/* are permitted provided that the following conditions are met:                          */
/*                                                                                        */
/* o Redistributions of source code must retain the above copyright notice, this list     */
/*   of conditions and the following disclaimer.                                          */
/*                                                                                        */
/* o Redistributions in binary form must reproduce the above copyright notice, this       */
/*   list of conditions and the following disclaimer in the documentation and/or          */
/*   other materials provided with the distribution.                                      */
/*                                                                                        */
/* o Neither the name of Freescale Semiconductor, Inc. nor the names of its               */
/*   contributors may be used to endorse or promote products derived from this            */
/*   software without specific prior written permission.                                  */
/*                                                                                        */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND        */
/* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED          */
/* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                 */
/* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR       */
/* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES         */
/* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;           */
/* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON         */
/* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                */
/* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS          */
/* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/*****************************************************************************/
/* Version: CodeSourcery Sourcery G++ Lite (with CS3)                        */
/*****************************************************************************/


/* Vector Table */

    .section ".cs3.interrupt_vector"
    .globl  __cs3_interrupt_vector_cortex_m
    .type   __cs3_interrupt_vector_cortex_m, %object

__cs3_interrupt_vector_cortex_m:
    .long   __cs3_stack                 /* Top of Stack                 */
    .long   __cs3_reset                 /* Reset Handler                */
    .long   NMI_Handler                                     /* NMI Handler*/
    .long   HardFault_Handler                               /* Hard Fault Handler*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   SVC_Handler                                     /* SVCall Handler*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   PendSV_Handler                                  /* PendSV Handler*/
    .long   SysTick_Handler                                 /* SysTick Handler*/

                                                            /* External Interrupts*/
    .long   DMA0_IRQHandler                                 /* DMA channel 0 transfer complete*/
    .long   DMA1_IRQHandler                                 /* DMA channel 1 transfer complete*/
    .long   DMA2_IRQHandler                                 /* DMA channel 2 transfer complete*/
    .long   DMA3_IRQHandler                                 /* DMA channel 3 transfer complete*/
    .long   Reserved20_IRQHandler                           /* Reserved interrupt*/
    .long   FTFA_IRQHandler                                 /* Command complete and read collision*/
    .long   PMC_IRQHandler                                  /* Low-voltage detect, low-voltage warning*/
    .long   LLWU_IRQHandler                                 /* Low leakage wakeup*/
    .long   I2C0_IRQHandler                                 /* I2C0 interrupt*/
    .long   I2C1_IRQHandler                                 /* I2C1 interrupt*/
    .long   SPI0_IRQHandler                                 /* SPI0 single interrupt vector for all sources*/
    .long   SPI1_IRQHandler                                 /* SPI1 single interrupt vector for all sources*/
    .long   LPUART0_IRQHandler                              /* LPUART0 status and error*/
    .long   LPUART1_IRQHandler                              /* LPUART1 status and error*/
    .long   UART2_FLEXIO_IRQHandler                         /* UART2 or FLEXIO*/
    .long   ADC0_IRQHandler                                 /* ADC0 interrupt*/
    .long   CMP0_IRQHandler                                 /* CMP0 interrupt*/
    .long   TPM0_IRQHandler                                 /* TPM0 single interrupt vector for all sources*/
    .long   TPM1_IRQHandler                                 /* TPM1 single interrupt vector for all sources*/
    .long   TPM2_IRQHandler                                 /* TPM2 single interrupt vector for all sources*/
    .long   RTC_IRQHandler                                  /* RTC alarm*/
    .long   RTC_Seconds_IRQHandler                          /* RTC seconds*/
    .long   PIT_IRQHandler                                  /* PIT interrupt*/
    .long   I2S0_IRQHandler                                 /* I2S0 interrupt*/
    .long   USB0_IRQHandler                                 /* USB0 interrupt*/
    .long   DAC0_IRQHandler                                 /* DAC0 interrupt*/
    .long   Reserved42_IRQHandler                           /* Reserved interrupt*/
    .long   Reserved43_IRQHandler                           /* Reserved interrupt*/
    .long   LPTMR0_IRQHandler                               /* LPTMR0 interrupt*/
    .long   LCD_IRQHandler                                  /* LCD interrupt*/
    .long   PORTA_IRQHandler                                /* PORTA Pin detect*/
    .long   PORTCD_IRQHandler                               /* Single interrupt vector for PORTC; PORTD Pin detect*/

    .size   __cs3_interrupt_vector_cortex_m, . - __cs3_interrupt_vector_cortex_m

/* Flash Configuration */
    .section .FlashConfig

    .long 0xFFFFFFFF
    .long 0xFFFFFFFF
    .long 0xFFFFFFFF
    .long 0xFFFF3FFE

    .thumb

/* Reset Handler */

    .section .cs3.reset,"x",%progbits
    .thumb_func
    .globl  __cs3_reset_cortex_m
    .type   __cs3_reset_cortex_m, %function
__cs3_reset_cortex_m:
    .fnstart
    LDR     R0, =SystemInit
    BLX     R0
    LDR     R0, =init_data_bss
    BLX     R0
    LDR     R0,=_start
    BX      R0
    .pool
    .cantunwind
    .fnend
    .size   __cs3_reset_cortex_m,.-__cs3_reset_cortex_m

    .section ".text"
    .weak  Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    B       .
    .size   Default_Handler, . - Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
  .macro	def_irq_handler	handler_name
  .weak	\handler_name
  .set	\handler_name, Default_Handler
  .endm

/* Exception Handlers */
    def_irq_handler    NMI_Handler
    def_irq_handler    HardFault_Handler
    def_irq_handler    SVC_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler
    def_irq_handler    DMA0_IRQHandler
    def_irq_handler    DMA1_IRQHandler
    def_irq_handler    DMA2_IRQHandler
    def_irq_handler    DMA3_IRQHandler
    def_irq_handler    Reserved20_IRQHandler
    def_irq_handler    FTFA_IRQHandler
    def_irq_handler    PMC_IRQHandler
    def_irq_handler    LLWU_IRQHandler
    def_irq_handler    I2C0_IRQHandler
    def_irq_handler    I2C1_IRQHandler
    def_irq_handler    SPI0_IRQHandler
    def_irq_handler    SPI1_IRQHandler
    def_irq_handler    LPUART0_IRQHandler
    def_irq_handler    LPUART1_IRQHandler
    def_irq_handler    UART2_FLEXIO_IRQHandler
    def_irq_handler    ADC0_IRQHandler
    def_irq_handler    CMP0_IRQHandler
    def_irq_handler    TPM0_IRQHandler
    def_irq_handler    TPM1_IRQHandler
    def_irq_handler    TPM2_IRQHandler
    def_irq_handler    RTC_IRQHandler
    def_irq_handler    RTC_Seconds_IRQHandler
    def_irq_handler    PIT_IRQHandler
    def_irq_handler    I2S0_IRQHandler
    def_irq_handler    USB0_IRQHandler
    def_irq_handler    DAC0_IRQHandler
    def_irq_handler    Reserved42_IRQHandler
    def_irq_handler    Reserved43_IRQHandler
    def_irq_handler    LPTMR0_IRQHandler
    def_irq_handler    LCD_IRQHandler
    def_irq_handler    PORTA_IRQHandler
    def_irq_handler    PORTCD_IRQHandler
    def_irq_handler    DefaultISR

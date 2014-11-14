/*
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== MSP_EXP430FR5969LP.c ========
 *  This file is responsible for setting up the board specific items for the
 *  MSP_EXP430FR5969LP board.
 *
 *  The following defines are used to determine which TI-RTOS peripheral drivers
 *  to include:
 *     TI_DRIVERS_GPIO_INCLUDED
 *     TI_DRIVERS_I2C_INCLUDED
 *     TI_DRIVERS_SDSPI_INCLUDED
 *     TI_DRIVERS_SPI_INCLUDED
 *     TI_DRIVERS_UART_INCLUDED
 *     TI_DRIVERS_WATCHDOG_INCLUDED
 *     TI_DRIVERS_WIFI_INCLUDED
 *  These defines are created when a useModule is done on the driver in the
 *  application's .cfg file. The actual #define is in the application
 *  generated header file that is brought in via the xdc/cfg/global.h.
 *  For example the following in the .cfg file
 *     var GPIO = xdc.useModule('ti.drivers.GPIO');
 *  Generates the following
 *     #define TI_DRIVERS_GPIO_INCLUDED 1
 *  If there is no useModule of ti.drivers.GPIO, the constant is set to 0.
 *
 *  Note: a useModule is generated in the .cfg file via the graphical
 *  configuration tool when the "Add xxx to my configuration" is checked
 *  or "Use xxx" is selected.
 */

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Timestamp.h>

#include <msp430.h>
#include <inc/hw_memmap.h>
#include <dma.h>
#include <eusci_a_uart.h>
#include <gpio.h>
#include <pmm.h>

#include "MSP_EXP430FR5969LP.h"

#include <ti/drivers/SPI.h>

const SPI_Config SPI_config[];

/*
 *  ======== MSP_EXP430FR5969LP_initGeneral ========
 */
void MSP_EXP430FR5969LP_initGeneral(void) {
    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();
}

/*
 *  ======== MSP_EXP430FR5969LP_isrDMA ========
 *  This is a application defined DMA ISR. This ISR must map and call the
 *  appropriate Driver_event(handle) API to indicate completed DMA transfers.
 */
Void MSP_EXP430FR5969LP_isrDMA(UArg arg)
{
#if TI_DRIVERS_WIFI_INCLUDED || TI_DRIVERS_SPI_INCLUDED
    /* Call the SPI DMA function, passing the SPI handle used for WiFi */
    SPI_serviceISR((SPI_Handle) &(SPI_config[0]));
#endif
}

#if TI_DRIVERS_GPIO_INCLUDED
#include <ti/drivers/GPIO.h>

/* GPIO configuration structure */
const GPIO_HWAttrs gpioHWAttrs[MSP_EXP430FR5969LP_GPIOCOUNT] = {
    {GPIO_PORT_P4, GPIO_PIN6, GPIO_OUTPUT}, /* MSP_EXP430FR5969LPLP_LED1 */
    {GPIO_PORT_P1, GPIO_PIN0, GPIO_OUTPUT}, /* MSP_EXP430FR5969LPLP_LED2 */
    {GPIO_PORT_P4, GPIO_PIN5, GPIO_INPUT},  /* MSP_EXP430FR5969LPLP_S1 (S2) */
    {GPIO_PORT_P1, GPIO_PIN1, GPIO_INPUT},  /* MSP_EXP430FR5969LPLP_S2 (S3) */
};

const GPIO_Config GPIO_config[] = {
    {&gpioHWAttrs[0]},
    {&gpioHWAttrs[1]},
    {&gpioHWAttrs[2]},
    {&gpioHWAttrs[3]},
    {NULL},
};

/*
 *  ======== MSP_EXP430FR5969LPLP_initGPIO ========
 */
void MSP_EXP430FR5969LP_initGPIO(void)
{
    /* Buttons are active low with pullup resistor */
    GPIO_setAsInputPinWithPullUpresistor(GPIO_PORT_P4, GPIO_PIN5);
    GPIO_setAsInputPinWithPullUpresistor(GPIO_PORT_P1, GPIO_PIN1);

    /* LEDs */
    /* LED1 */
    GPIO_setAsOutputPin  (GPIO_PORT_P4, GPIO_PIN6);

    /* LED2 */
    GPIO_setAsOutputPin  (GPIO_PORT_P1, GPIO_PIN0);

    /* Once GPIO_init is called, GPIO_config cannot be changed */
    GPIO_init();
}
#endif /* TI_DRIVERS_GPIO_INCLUDED */

#if TI_DRIVERS_I2C_INCLUDED
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CEUSCIB.h>

/* I2C objects */
I2CEUSCIB_Object i2cEUSCIBObjects[MSP_EXP430FR5969LP_I2CCOUNT];

/* I2C configuration structure */
const I2CEUSCIB_HWAttrs i2cEUSCIBHWAttrs[MSP_EXP430FR5969LP_I2CCOUNT] = {
    {
        EUSCI_B0_BASE,
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK
    },

};

const I2C_Config I2C_config[] = {
    {
        &I2CEUSCIB_fxnTable,
        &i2cEUSCIBObjects[0],
        &i2cEUSCIBHWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== MSP_EXP430FR5969LP_initI2C ========
 */
void MSP_EXP430FR5969LP_initI2C(void)
{
    /* EUSCIB0 */
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P1,
        GPIO_PIN6 | GPIO_PIN7, GPIO_SECONDARY_MODULE_FUNCTION);

    I2C_init();
}
#endif /* TI_DRIVERS_I2C_INCLUDED */

#if TI_DRIVERS_SDSPI_INCLUDED
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/sdspi/SDSPIEUSCIB.h>

/* SDSPI objects */
SDSPIEUSCIB_Object sdspiEUSCIBobjects[MSP_EXP430FR5969LP_SDSPICOUNT];

/* SDSPI configuration structure, describing which pins are to be used */
const SDSPIEUSCIB_HWAttrs sdspiEUSCIBHWAttrs[MSP_EXP430FR5969LP_SDSPICOUNT] = {
    {

        EUSCI_B0_BASE,
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,

        GPIO_PORT_P2,    /* SCK PORT */
        GPIO_PIN2,       /* SCK PIN */

        GPIO_PORT_P1,    /* MISO PORT */
        GPIO_PIN7,       /* MIS0 PIN */

        GPIO_PORT_P1,    /* MOSI PORT */
        GPIO_PIN6,       /* MOSI PIN */

        GPIO_PORT_P3,    /* Chip select port */
        GPIO_PIN4,       /* Chip select pin */
    }
};

const SDSPI_Config SDSPI_config[] = {
    {
        &SDSPIEUSCIB_fxnTable,
        &sdspiEUSCIBobjects[0],
        &sdspiEUSCIBHWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== MSP_EXP430FR5969LP_initSDSPI ========
 */
void MSP_EXP430FR5969LP_initSDSPI(void)
{
    SDSPI_init();
}
#endif /* TI_DRIVERS_SDSPI_INCLUDED */

#if TI_DRIVERS_SPI_INCLUDED
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPIEUSCIBDMA.h>

/* SPI objects */
SPIEUSCIBDMA_Object spiEUSCIBDMAobjects[MSP_EXP430FR5969LP_SPICOUNT];
uint8_t spiEUSCIBDMAscratchBuf[MSP_EXP430FR5969LP_SPICOUNT];

/* SPI configuration structure, describing which pins are to be used */
const SPIEUSCIBDMA_HWAttrs spiEUSCIBDMAHWAttrs[MSP_EXP430FR5969LP_SPICOUNT] = {
    {
        EUSCI_B0_BASE,
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,
        EUSCI_B_SPI_MSB_FIRST,
        &spiEUSCIBDMAscratchBuf[0],
        0,

        /* DMA */
        DMA_BASE,
        /* Rx Channel */
        DMA_CHANNEL_0,
        DMA_TRIGGERSOURCE_18,
        /* Tx Channel */
        DMA_CHANNEL_1,
        DMA_TRIGGERSOURCE_19
    }
};

const SPI_Config SPI_config[] = {
    {
        &SPIEUSCIBDMA_fxnTable,
        &spiEUSCIBDMAobjects[0],
        &spiEUSCIBDMAHWAttrs[0]
    },
    {NULL, NULL, NULL},
};

/*
 *  ======== MSP_EXP430FR5969LP_initSPI ========
 */
void MSP_EXP430FR5969LP_initSPI(void)
{
    /* EUSCIB0 */
    /* SOMI/MISO */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN7, GPIO_SECONDARY_MODULE_FUNCTION);

    /* SIMO/MOSI */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,
            GPIO_PIN6, GPIO_SECONDARY_MODULE_FUNCTION);

    /* CLK */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
            GPIO_PIN2, GPIO_SECONDARY_MODULE_FUNCTION);

    SPI_init();
}
#endif /* TI_DRIVERS_SPI_INCLUDED */

#if TI_DRIVERS_UART_INCLUDED
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTEUSCIA.h>

/* UART objects */
UARTEUSCIA_Object uartEUSCIAObjects[MSP_EXP430FR5969LP_UARTCOUNT];

/*
 * The baudrate dividers were determined by using the MSP430 baudrate
 * calculator
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const UARTEUSCIA_BaudrateConfig uartEUSCIABaudrates[] = {
    /* baudrate, input clock, prescalar, UCBRFx, UCBRSx, oversampling */
    {115200, 8000000, 4,  5, 85, 1},
    {9600,   8000000, 52, 1, 0,  1},
    {9600,   32768,   3,  0, 3,  0},
};

/* UART configuration structure */
const UARTEUSCIA_HWAttrs uartEUSCIAHWAttrs[MSP_EXP430FR5969LP_UARTCOUNT] = {
    {
        EUSCI_A0_BASE,
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        EUSCI_A_UART_LSB_FIRST,
        sizeof(uartEUSCIABaudrates)/sizeof(UARTEUSCIA_BaudrateConfig),
        uartEUSCIABaudrates
    },
    {
        EUSCI_A0_BASE,
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        EUSCI_A_UART_LSB_FIRST,
        sizeof(uartEUSCIABaudrates)/sizeof(UARTEUSCIA_BaudrateConfig),
        uartEUSCIABaudrates
    },
};

const UART_Config UART_config[] = {
    {
        &UARTEUSCIA_fxnTable,
        &uartEUSCIAObjects[0],
        &uartEUSCIAHWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== MSP_EXP430FR5969LP_initUART ========
 */
void MSP_EXP430FR5969LP_initUART(void)
{
    /* P4.4,5 = USCI_A1 TXD/RXD */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
            GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,
            GPIO_PIN1, GPIO_SECONDARY_MODULE_FUNCTION);

    /* Initialize the UART driver */
    UART_init();
}
#endif /* TI_DRIVERS_UART_INCLUDED */

#if TI_DRIVERS_WATCHDOG_INCLUDED
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogMSP430.h>

/* Watchdog objects */
WatchdogMSP430_Object watchdogMSP430Objects[MSP_EXP430FR5969LP_WATCHDOGCOUNT];

/* Watchdog configuration structure */
const WatchdogMSP430_HWAttrs watchdogMSP430HWAttrs[MSP_EXP430FR5969LP_WATCHDOGCOUNT] = {
    {
        WDT_A_BASE,
        SFR_BASE,
        WATCHDOG_CLOCKSOURCE_SMCLK,
        WATCHDOG_CLOCKDIVIDER_8192K
    }, /* MSP430F5529_WATCHDOG */
};

const Watchdog_Config Watchdog_config[] = {
    {
        &WatchdogMSP430_fxnTable,
        &watchdogMSP430Objects[0],
        &watchdogMSP430HWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== MSP_EXP430FR5969LP_initWatchdog ========
 */
void MSP_EXP430FR5969LP_initWatchdog(void)
{
    /* Initialize the Watchdog driver */
    Watchdog_init();
}
#endif /* TI_DRIVERS_WATCHDOG_INCLUDED */

#if TI_DRIVERS_WIFI_INCLUDED
#include <ti/drivers/WiFi.h>

#if TI_DRIVERS_WIFI_CC3000
#include <ti/drivers/wifi/WiFiMSP430CC3000.h>

/* WiFi objects */
WiFiMSP430CC3000_Object wiFiMSP430CC3000Objects[MSP_EXP430FR5969LP_WIFICOUNT];

/* WiFi configuration structure */
const WiFiMSP430CC3000_HWAttrs wiFiMSP430CC3000HWAttrs[MSP_EXP430FR5969LP_WIFICOUNT] = {
    {
        GPIO_PORT_P1, /* IRQ port */
        GPIO_PIN2,    /* IRQ pin */

        GPIO_PORT_P3, /* CS port */
        GPIO_PIN0,    /* CS pin */

        GPIO_PORT_P4, /* WLAN EN port */
        GPIO_PIN2     /* WLAN EN pin */
    }
};

const WiFi_Config WiFi_config[] = {
    {
        &WiFiMSP430CC3000_fxnTable,
        &wiFiMSP430CC3000Objects[0],
        &wiFiMSP430CC3000HWAttrs[0]
    },
    {NULL, NULL, NULL},
};

/*
 *  ======== MSP_EXP430FR5969LP_initWiFi ========
 */
void MSP_EXP430FR5969LP_initWiFi(void)
{
    /* Configure SPI */

    /* SOMI/MISO */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN7, GPIO_SECONDARY_MODULE_FUNCTION);

    /* CLK and SIMO/MOSI */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,
            GPIO_PIN6, GPIO_SECONDARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
            GPIO_PIN2, GPIO_SECONDARY_MODULE_FUNCTION);

    /* Configure IRQ pin */
    GPIO_setAsInputPinWithPullUpresistor(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN2,
                             GPIO_HIGH_TO_LOW_TRANSITION);

    /* Configure WLAN EN pin */
    GPIO_setAsOutputPin  (GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);

    /* Configure CS pin */
    GPIO_setAsOutputPin  (GPIO_PORT_P3, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0);

    /* Initialize SPI and WiFi drivers */
    SPI_init();
    WiFi_init();
}
#endif /* TI_DRIVERS_WIFI_CC3000 */

#if TI_DRIVERS_WIFI_CC3100
#include <ti/drivers/wifi/WiFiCC3100.h>

/* WiFi objects */
WiFiCC3100_Object wiFiCC3100Objects[MSP_EXP430FR5969LP_WIFICOUNT];

/* WiFi configuration structure */
const WiFiCC3100_HWAttrs wiFiCC3100HWAttrs[MSP_EXP430FR5969LP_WIFICOUNT] = {
    {
        GPIO_PORT_P1, /* IRQ port */
        GPIO_PIN2,    /* IRQ pin */
        NULL,         /* IRQ port interrupt not used */

        GPIO_PORT_P3, /* CS port */
        GPIO_PIN0,    /* CS pin */

        GPIO_PORT_P4, /* WLAN EN port */
        GPIO_PIN3     /* WLAN EN pin */
    }
};

const WiFi_Config WiFi_config[] = {
    {
        &WiFiCC3100_fxnTable,
        &wiFiCC3100Objects[0],
        &wiFiCC3100HWAttrs[0]
    },
    {NULL, NULL, NULL},
};

/*
 *  ======== MSP_EXP430FR5969LP_initWiFi ========
 */
void MSP_EXP430FR5969LP_initWiFi(void)
{
    /* Configure EN & CS pins to disable CC3100 */
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN3);

    /* Configure SPI */
    /* SPI CLK */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN2,
                                                GPIO_SECONDARY_MODULE_FUNCTION);
    /* MOSI/SIMO */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN6,
                                                GPIO_SECONDARY_MODULE_FUNCTION);
    /* MISO/SOMI */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN7,
                                               GPIO_SECONDARY_MODULE_FUNCTION);

    /* Configure IRQ pin */
    GPIO_setAsInputPinWithPullDownresistor(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN2,
                             GPIO_LOW_TO_HIGH_TRANSITION);

    /* Initialize SPI and WiFi drivers */
    SPI_init();
    WiFi_init();
}
#endif /* TI_DRIVERS_WIFI_CC3100 */

#endif /* TI_DRIVERS_WIFI_INCLUDED */

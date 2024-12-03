/*******************************************************************************
* File Name:   main.c
*
* Description: This example project demonstrates the basic operation of SPI
* resource as Slave. The SPI slave receives command packets from the
* SPI master to control an user LED.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/*******************************************************************************
* Global Variables
*******************************************************************************/
cy_stc_scb_spi_context_t context;
/*******************************************************************************
* Function Prototypes
*******************************************************************************/
/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  uint32_t status - status indicates success or failure
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(uint32_t status)
{
    if (status != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* The main function.
*   1. Initializes the board, retarget-io and led
*   2. Configures the SPI slave to receive packet from the master and execute
*   the instructions
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    handle_error(result);

    /* Initialize the User LED */
    result = Cy_GPIO_Pin_Init(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, &CYBSP_USER_LED_config);
    /* User LED init failed. Stop program execution */
    handle_error(result);

    /* Initialize retarget-io to use the debug UART port */
     Cy_SCB_UART_Init(UART_HW, &UART_config, NULL);
     Cy_SCB_UART_Enable(UART_HW);
     cy_retarget_io_init(UART_HW);

    /* Initialize retarget-io for uart logs */
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("*************** "
           "PDL: SPI Slave "
           "*************** \r\n\n");

    printf("Configuring SPI slave...\r\n");

    result = Cy_SCB_SPI_Init(SPI_S_HW,&SPI_S_config,&context);
    handle_error(result);

    Cy_SCB_SPI_Enable(SPI_S_HW);/* Enable SPI */
    /* Enable interrupts */
    __enable_irq();

    for (;;)
    {
        uint32_t read_value = CY_SCB_SPI_RX_NO_DATA;
        /* Read the command packet sent from master */
        while(read_value == CY_SCB_SPI_RX_NO_DATA)
        {
            read_value = Cy_SCB_SPI_Read(SPI_S_HW);
        }
        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, read_value);

    }
}


/* [] END OF FILE */

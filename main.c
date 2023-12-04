/**
 * @file main.c
 * @authors Victor Hugo Polli Neves, André Luiz Poloni and Isabela Amorim Siqueira
 * @brief Main file to initialize all the peripherals and tasks.
 * @version 0.1
 */

#include "elevator-mgr.h"
#include "elevator-rx-drv.h"
#include "elevator-tx-drv.h"
#include "defs.h"

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#define DEBUG
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
    while (1)
        ;
}
#endif

/* ================================ Defines ================================= */

#define LEDS_SYSCTL_PERIPH SYSCTL_PERIPH_GPIOF
#define LEDS_PORT GPIO_PORTF_BASE
#define RED_LED GPIO_PIN_1
#define BLUE_LED GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

/* ============================ Global Variables ============================ */

//*****************************************************************************
//
// System clock rate in Hz.
//
//*****************************************************************************
uint32_t g_ui32SysClock;

/* ---------------------------- MESSAGE QUEUES ------------------------------ */
osMessageQueueId_t elevator_mgr_incoming_queue_id;
const osMessageQueueAttr_t elevator_mgr_incoming_queue_attr = {.name = "ELEVATOR_MGR_QUEUE"};
ELEVATOR_MGR_HANDLERS_LIST elevator_mgr_handlers_list;

osMessageQueueId_t elevator_tx_incoming_queue_id;
const osMessageQueueAttr_t elevator_tx_incoming_queue_attr = {.name = "ELEVATOR_TX_QUEUE"};
ELEVATOR_TX_HANDLERS_LIST elevator_tx_handlers_list;

ELEVATOR_RX_HANDLERS_LIST elevator_rx_handlers_list;

/* -------------------------- Functions Prototypes -------------------------- */

/**
 * @brief Initializes the Message Queues
 *
 * @return osStatus_t operation result
 */
osStatus_t InitMessageQueues(void);

/**
 * @brief Initializes the Threads
 *
 * @return osStatus_t operation result
 */
osStatus_t InitThreads(void);

/**
 * @brief Enable and configure the GPIOs for the LEDs.
 */
void LEDS_INIT(void);

/* -------------------------------------------------------------------------- */

osStatus_t InitMessageQueues(void)
{
    osStatus_t result;

    elevator_mgr_incoming_queue_id = osMessageQueueNew(ELEVATOR_MGR_QUEUE_LENGTH, sizeof(elevator_mgr_msg_t), &elevator_mgr_incoming_queue_attr);
    elevator_tx_incoming_queue_id = osMessageQueueNew(ELEVATOR_TX_QUEUE_LENGTH, sizeof(elevator_tx_msg_t), &elevator_tx_incoming_queue_attr);

    if ((NULL == elevator_mgr_incoming_queue_id) ||
        (NULL == elevator_tx_incoming_queue_id))
    {
        result = osError;
    }
    else
    {
        // Data and messages flow: UART_RX -> ELEVATOR_RX -> ELEVATOR_MGR -> ELEVATOR_RX -> UART_TX
        elevator_rx_handlers_list.outgoing_msg_queue = elevator_mgr_incoming_queue_id;

        elevator_mgr_handlers_list.incoming_msg_queue = elevator_mgr_incoming_queue_id;
        elevator_mgr_handlers_list.outgoing_msg_queue = elevator_tx_incoming_queue_id;

        elevator_tx_handlers_list.incoming_msg_queue = elevator_tx_incoming_queue_id;

        result = osOK;
    }

    return result;
}

osStatus_t InitThreads(void)
{
    osStatus_t result = osOK;

    result |= ElevatorMgrInit(&elevator_mgr_handlers_list);
    result |= ElevatorRXInit(&elevator_rx_handlers_list);
    result |= ElevatorTXInit(&elevator_tx_handlers_list);

    return result;
}

void LEDS_INIT(void)
{
    //
    // Enable the GPIO port that is used for the LEDs.
    //
    MAP_SysCtlPeripheralEnable(LEDS_SYSCTL_PERIPH);

    //
    // Check if the peripheral access is enabled.
    //
    while (!MAP_SysCtlPeripheralReady(LEDS_SYSCTL_PERIPH))
    {
    }

    //
    // Enable the GPIO pin for the LEDs.
    // Set the direction as output, and enable the GPIO pin for digital function.
    //
    MAP_GPIOPinTypeGPIOOutput(LEDS_PORT, RED_LED | GREEN_LED | BLUE_LED);

    // Turn off all leds
    MAP_GPIOPinWrite(LEDS_PORT, RED_LED, 0);
    MAP_GPIOPinWrite(LEDS_PORT, GREEN_LED, 0);
    MAP_GPIOPinWrite(LEDS_PORT, BLUE_LED, 0);
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
    {
    }

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UART0))
    {
    }

    //
    // Configure the UART communication parameters.
    //
    MAP_UARTConfigSetExpClk(UART0_BASE, MAP_SysCtlClockGet(), 115200,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

int main(void)
{
    osStatus_t init_result = osOK;

    // Enables the FPU
    ROM_FPULazyStackingEnable();
    // Run from the PLL at 120 MHz.
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480),
                                            120000000);
    // Initialize the UART.
    ConfigureUART();
    // Configure SysTick
    MAP_SysTickEnable();
    // Enable Interrupts
    MAP_IntMasterEnable();
    // Initializing LEDs:
    LEDS_INIT();

    osKernelInitialize(); // Initialize CMSIS-RTOS

    init_result |= InitMessageQueues();
    init_result |= InitThreads();

    if (osOK == init_result)
    {
        MAP_GPIOPinWrite(LEDS_PORT, GREEN_LED, GREEN_LED);
        osKernelStart();
    }

    while (1)
    {
        // INIT ERROR
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0x0);
        SysCtlDelay(ROM_SysCtlClockGet() / 10);
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED);
        SysCtlDelay(ROM_SysCtlClockGet() / 10);
    }
}

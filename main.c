/**
 * @file main.c
 * @authors Joyce Carvalho Pereira (RA = 2090805)
 * e Victor Hugo Polli Neves (RA = 2128489)
 * @brief Disciplina: Sistemas Embarcados - Laboratorio 3
 * @version 0.1
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_gpio.h"
#include "driverlib/adc.h"
#include "cmsis_os2.h"
#include "elevator-mgr.h"
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

#define LAB3_LEDs_THREADS
#define LAB3_Sensors_THREADS

#define LEDS_SYSCTL_PERIPH SYSCTL_PERIPH_GPIOD
#define LEDS_PORT GPIO_PORTD_BASE
#define LED1_PIN GPIO_PIN_0
#define LED2_PIN GPIO_PIN_1
#define LED3_PIN GPIO_PIN_2
#define LED4_PIN GPIO_PIN_3

#define BUTTONS_SYSCTL_PERIPH SYSCTL_PERIPH_GPIOF
#define BUTTONS_PORT GPIO_PORTF_BASE
#define BUTTON1_PIN GPIO_PIN_4
#define BUTTON2_PIN GPIO_PIN_0

#define ADCS_SYSCTL_PERIPH SYSCTL_PERIPH_GPIOE
#define ADCS_PORT GPIO_PORTE_BASE
#define ADC_CHN_1_PIN GPIO_PIN_2
#define ADC_CHN_2_PIN GPIO_PIN_1

#define LED1_BLINK_TIME 1000
#define LED2_BLINK_TIME 250
#define LED3_BLINK_TIME 125
#define TEMPERATURE_READ_TIME 1000
#define LDR_READ_TIME 3000
#define SENSORS_READ_REPORT_TIME 1000

#define FLAGS_MSK1 0x00000001U

/* ================================= Enums ================================== */

/**
 * @brief LEDs identification for event flags operations.
 */
typedef enum
{
    LED1 = 1,
    LED2 = 2,
    LED3 = 3
} leds_id_t;

/**
 * @brief Sensor events identification.
 */
typedef enum
{
    READ_TEMPERATURE_SENSOR = 0,
    READ_LIGHT_SENSOR = 1,
    REPORT_SENSORS_DATA = 2
} sensors_events_id_t;

/* ================================ Structs ================================= */

/* ============================ Global Variables ============================ */

//*****************************************************************************
//
// System clock rate in Hz.
//
//*****************************************************************************
uint32_t g_ui32SysClock;

static leds_id_t led1_id = LED1;
static leds_id_t led2_id = LED2;
static leds_id_t led3_id = LED3;

static sensors_events_id_t temp_read_id = READ_TEMPERATURE_SENSOR;
static sensors_events_id_t ldr_read_id = READ_LIGHT_SENSOR;
static sensors_events_id_t report_sensors_id = REPORT_SENSORS_DATA;

static uint32_t g_temp_read_data[1];
static uint32_t g_ldr_read_data[1];

/* ================================= THREADS ================================ */
static osThreadId_t led1_thread_id;
static const osThreadAttr_t led1_thread_attr = {
    .priority = osPriorityNormal,
    .name = "LED1_THREAD",
    .stack_size = 256};

static osThreadId_t led2_thread_id;
static const osThreadAttr_t led2_thread_attr = {
    .priority = osPriorityNormal,
    .name = "LED2_THREAD",
    .stack_size = 256};

static osThreadId_t led3_thread_id;
static const osThreadAttr_t led3_thread_attr = {
    .priority = osPriorityNormal,
    .name = "LED3_THREAD",
    .stack_size = 256};

static osThreadId_t temperature_read_thread_id;
static const osThreadAttr_t temperature_read_thread_attr = {
    .priority = osPriorityNormal,
    .name = "TEMP_READ_THREAD",
    .stack_size = 256};

static osThreadId_t ldr_read_thread_id;
static const osThreadAttr_t ldr_read_thread_attr = {
    .priority = osPriorityNormal,
    .name = "LDR_READ_THREAD",
    .stack_size = 256};

static osThreadId_t sensors_report_thread_id;
static const osThreadAttr_t sensors_report_thread_attr = {
    .priority = osPriorityNormal,
    .name = "SENSORS_REPORT_THREAD",
    .stack_size = 256};

/* ================================= TIMERS ================================ */
static osTimerId_t led1_timer_id;
static const osTimerAttr_t led1_timer_attr = {.name = "LED1_TIMER"};

static osTimerId_t led2_timer_id;
static const osTimerAttr_t led2_timer_attr = {.name = "LED2_TIMER"};

static osTimerId_t led3_timer_id;
static const osTimerAttr_t led3_timer_attr = {.name = "LED3_TIMER"};

static osTimerId_t temperature_read_timer_id;
static const osTimerAttr_t temperature_read_timer_attr = {.name = "TEMP_READ_TIMER"};

static osTimerId_t ldr_read_timer_id;
static const osTimerAttr_t ldr_read_timer_attr = {.name = "LDR_READ_TIMER"};

static osTimerId_t sensors_report_timer_id;
static const osTimerAttr_t sensors_report_timer_attr = {.name = "SENSORS_REPORT_TIMER"};

/* =============================== EVENT FLAGS ============================== */

static osEventFlagsId_t led1_toogle_flag_id;
static osEventFlagsAttr_t led1_toogle_flag_attr = {.name = "LED1_TOOGLE_FLAG"};

static osEventFlagsId_t led2_toogle_flag_id;
static osEventFlagsAttr_t led2_toogle_flag_attr = {.name = "LED2_TOOGLE_FLAG"};

static osEventFlagsId_t led3_toogle_flag_id;
static osEventFlagsAttr_t led3_toogle_flag_attr = {.name = "LED3_TOOGLE_FLAG"};

static osEventFlagsId_t temperature_read_flag_id;
static osEventFlagsAttr_t temperature_read_flag_attr = {.name = "TEMP_READ_FLAG"};

static osEventFlagsId_t ldr_read_flag_id;
static osEventFlagsAttr_t ldr_read_flag_attr = {.name = "LDR_READ_FLAG"};

static osEventFlagsId_t sensors_report_flag_id;
static osEventFlagsAttr_t sensors_report_flag_attr = {.name = "SENSORS_REPORT_FLAG"};

/* =============================== SEMAPHORES =============================== */

static osSemaphoreId_t Sensors_operation_smph_id;
static osSemaphoreAttr_t Sensors_operation_smph_attr = {.name = "SENSORS_OP_SEMAPHORE"};

/* ========================== Functions Prototypes ========================== */
/**
 * @brief Delay function t o introduce a time delay in milliseconds.
 *
 * This function uses the SysCtlDelay function to introduce a time delay
 * in milliseconds. The SysCtlDelay function uses the system clock frequency
 * to calculate the delay.
 *
 * "SysCtlDelay() is a loop timer provided in TivaWare. The count parameter is the loop
 * count, not the actual delay in clock cycles. Each loop is 3 CPU cycles."
 *
 * @param ms time delay in milliseconds.
 */
void Delay(uint32_t ms);

/**
 * @brief Enable and configure the GPIOs for the LEDs.
 */
void LEDS_INIT(void);

// LAB 3.1:

/**
 * @brief Blinks the LED selected by the args, alternating its state.
 *
 * @param arg argument with gpio attributes.
 */
void LED_Blink_Set_Flag(void *arg);

/**
 * @brief LED 1 Task, that blinks it 1 time per second.
 */
void LED1_Thread(void);

/**
 * @brief LED 2 Task, that blinks it 2 time per second.
 */
void LED2_Thread(void);

/**
 * @brief LED 3 Task, that blinks it 2 time per second.
 */
void LED3_Thread(void);

/**
 * @brief Initializes all the event flags for LEDs (LAB 3.1).
 *
 * @return osStatus_t operation result.
 */
osStatus_t LEDsEventFlagsInit(void);

/**
 * @brief Initializes all the timers for LEDs (LAB 3.1).
 *
 * @return osStatus_t operation result.
 */
osStatus_t LEDsTimersInit(void);

/**
 * @brief Initializes all the threads for LEDs (LAB 3.1).
 *
 * @return osStatus_t operation result.
 */
osStatus_t LEDsThreadsInit(void);

// LAB 3.2:

/**
 * @brief Blinks the LED selected by the args, alternating its state.
 *
 * @param arg argument with gpio attributes.
 */
void LED_Blink_Set_Flag(void *arg);

/**
 * @brief Temperature Read Task, that reads the temperature sensor value.
 */
void Temperature_Read_Thread(void);

/**
 * @brief LDR Read Task, that reads the light sensor value.
 */
void LDR_Read_Thread(void);

/**
 * @brief Sensors Report Task, that sends the sensors read data to the UART.
 */
void Sensors_Report_Thread(void);

/**
 * @brief Initializes all the event flags for Sensors read and report (LAB 3.2).
 *
 * @return osStatus_t operation result.
 */
osStatus_t SensorsEventFlagsInit(void);

/**
 * @brief Initializes all the timers for Sensors read and report (LAB 3.2).
 *
 * @return osStatus_t operation result.
 */
osStatus_t SensorsTimersInit(void);

/**
 * @brief Initializes all the threads for Sensors read and report (LAB 3.2).
 *
 * @return osStatus_t operation result.
 */
osStatus_t SensorsThreadsInit(void);

/**
 * @brief Initializes the semaphore for Sensors read and report (LAB 3.2).
 *
 * @return osStatus_t operation result.
 */
osStatus_t SensorsSemaphoreInit(void);

/* ========================================================================== */

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

//*****************************************************************************
//
// ADC channels initialization
//
//*****************************************************************************
void ADC_INIT(void)
{
    //
    // The ADC0 and ADC1 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    //
    // For this exercice ADC0 is used with AIN1 on port E2 and ADC1 is used with AIN2 on port E1.
    // GPIO port E needs to be enabled so these pins can be used.
    //
    SysCtlPeripheralEnable(ADCS_SYSCTL_PERIPH);

    //
    // Select the analog ADC function for these pins.
    // Consult the data sheet to see which functions are allocated per pin.
    //
    GPIOPinTypeADC(ADCS_PORT, ADC_CHN_1_PIN | ADC_CHN_2_PIN);

    //
    // Enable sample sequence 3 with a processor signal trigger. Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion. Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.
    //
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3. Sample channel 1 and 2 (ADC_CTL_CH1 and ADC_CTL_CH2)
    // in single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done. Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END). Sequence
    // 3 has only one programmable step. Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps. Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0. For more
    // information on the ADC sequences and steps, reference the datasheet.
    //
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    //
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCSequenceEnable(ADC1_BASE, 3);

    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    ADCIntClear(ADC0_BASE, 3);
    ADCIntClear(ADC1_BASE, 3);
}

void Delay(uint32_t ms)
{
    SysCtlDelay(ms * (SysCtlClockGet() / 3000));
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
    MAP_GPIOPinTypeGPIOOutput(LEDS_PORT, LED1_PIN | LED2_PIN | LED3_PIN | LED4_PIN);
}

void LED_Blink_Set_Flag(void *arg)
{
    leds_id_t *led_id = (leds_id_t *)arg;
    switch (*led_id)
    {
    case LED1:
        osEventFlagsSet(led1_toogle_flag_id, FLAGS_MSK1);
        break;

    case LED2:
        osEventFlagsSet(led2_toogle_flag_id, FLAGS_MSK1);
        break;

    case LED3:
        osEventFlagsSet(led3_toogle_flag_id, FLAGS_MSK1);
        break;

    default:
        break;
    }
}

void LED1_Thread(void)
{
    osStatus_t timer_start_result;
    timer_start_result = osTimerStart(led1_timer_id, (uint32_t)(LED1_BLINK_TIME));
    if (osOK != timer_start_result)
    {
        UARTprintf("Error initializing LED 1 thread...\n");
    }
    else
    {
        for (;;)
        {
            // Waiting for the flag event
            osEventFlagsWait(led1_toogle_flag_id, FLAGS_MSK1, osFlagsWaitAny, osWaitForever);

            if (false == GPIOPinRead(LEDS_PORT, LED1_PIN))
            {
                GPIOPinWrite(LEDS_PORT, LED1_PIN, LED1_PIN);
            }
            else
            {
                GPIOPinWrite(LEDS_PORT, LED1_PIN, 0);
            }

            osEventFlagsClear(led1_toogle_flag_id, FLAGS_MSK1);
        }
    }
}

void LED2_Thread(void)
{
    osStatus_t timer_start_result;
    timer_start_result = osTimerStart(led2_timer_id, (uint32_t)(LED2_BLINK_TIME));
    if (osOK != timer_start_result)
    {
        UARTprintf("Error initializing LED 2 thread...\n");
    }
    else
    {
        for (;;)
        {
            // Waiting for the flag event
            osEventFlagsWait(led2_toogle_flag_id, FLAGS_MSK1, osFlagsWaitAny, osWaitForever);

            if (false == GPIOPinRead(LEDS_PORT, LED2_PIN))
            {
                GPIOPinWrite(LEDS_PORT, LED2_PIN, LED2_PIN);
            }
            else
            {
                GPIOPinWrite(LEDS_PORT, LED2_PIN, 0);
            }

            osEventFlagsClear(led2_toogle_flag_id, FLAGS_MSK1);
        }
    }
}

void LED3_Thread(void)
{
    osStatus_t timer_start_result;
    timer_start_result = osTimerStart(led3_timer_id, (uint32_t)(LED3_BLINK_TIME));
    if (osOK != timer_start_result)
    {
        UARTprintf("Error initializing LED 3 thread...\n");
    }
    else
    {
        for (;;)
        {
            // Waiting for the flag event
            osEventFlagsWait(led3_toogle_flag_id, FLAGS_MSK1, osFlagsWaitAny, osWaitForever);

            if (false == GPIOPinRead(LEDS_PORT, LED3_PIN))
            {
                GPIOPinWrite(LEDS_PORT, LED3_PIN, LED3_PIN);
            }
            else
            {
                GPIOPinWrite(LEDS_PORT, LED3_PIN, 0);
            }

            osEventFlagsClear(led3_toogle_flag_id, FLAGS_MSK1);
        }
    }
}

osStatus_t LEDsTimersInit(void)
{
    osStatus_t result;

    led1_timer_id = osTimerNew((osTimerFunc_t)LED_Blink_Set_Flag, osTimerPeriodic,
                               &led1_id, &led1_timer_attr);
    led2_timer_id = osTimerNew((osTimerFunc_t)LED_Blink_Set_Flag, osTimerPeriodic,
                               &led2_id, &led2_timer_attr);
    led3_timer_id = osTimerNew((osTimerFunc_t)LED_Blink_Set_Flag, osTimerPeriodic,
                               &led3_id, &led3_timer_attr);

    if ((NULL == led1_timer_id) ||
        (NULL == led2_timer_id) ||
        (NULL == led3_timer_id))
    {
        result = osError;
        UARTprintf("LEDs timers init error...\n");
    }
    else
    {
        result = osOK;
        UARTprintf("LEDs timers init success...\n");
    }
    return result;
}

osStatus_t LEDsThreadsInit(void)
{
    osStatus_t result;

    led1_thread_id = osThreadNew((osThreadFunc_t)LED1_Thread, NULL, &led1_thread_attr);
    led2_thread_id = osThreadNew((osThreadFunc_t)LED2_Thread, NULL, &led2_thread_attr);
    led3_thread_id = osThreadNew((osThreadFunc_t)LED3_Thread, NULL, &led3_thread_attr);

    if ((NULL == led1_thread_id) ||
        (NULL == led2_thread_id) ||
        (NULL == led3_thread_id))
    {
        result = osError;
        UARTprintf("LEDs treads init error...\n");
    }
    else
    {
        result = osOK;
        UARTprintf("LEDs treads init success...\n");
    }
    return result;
}

osStatus_t LEDsEventFlagsInit(void)
{
    osStatus_t result;

    led1_toogle_flag_id = osEventFlagsNew(&led1_toogle_flag_attr);
    led2_toogle_flag_id = osEventFlagsNew(&led2_toogle_flag_attr);
    led3_toogle_flag_id = osEventFlagsNew(&led3_toogle_flag_attr);

    if ((NULL == led1_toogle_flag_id) ||
        (NULL == led2_toogle_flag_id) ||
        (NULL == led3_toogle_flag_id))
    {
        result = osError;
        UARTprintf("LEDs event flags init error...\n");
    }
    else
    {
        result = osOK;
        UARTprintf("LEDs event flags init success...\n");
    }
    return result;
}

void Sensors_Set_Flag(void *arg)
{
    sensors_events_id_t *sensors_event_id = (sensors_events_id_t *)arg;
    switch (*sensors_event_id)
    {
    case READ_TEMPERATURE_SENSOR:
        osEventFlagsSet(temperature_read_flag_id, FLAGS_MSK1);
        break;

    case READ_LIGHT_SENSOR:
        osEventFlagsSet(ldr_read_flag_id, FLAGS_MSK1);
        break;

    case REPORT_SENSORS_DATA:
        osEventFlagsSet(sensors_report_flag_id, FLAGS_MSK1);
        break;

    default:
        break;
    }
}

void Temperature_Read_Thread(void)
{
    osStatus_t timer_start_result;
    timer_start_result = osTimerStart(temperature_read_timer_id, (uint32_t)(TEMPERATURE_READ_TIME));
    if (osOK != timer_start_result)
    {
        UARTprintf("Error initializing temperature read thread...\n");
    }
    else
    {
        for (;;)
        {
            // Waiting for the flag event
            osEventFlagsWait(temperature_read_flag_id, FLAGS_MSK1, osFlagsWaitAny, osWaitForever);
            osSemaphoreAcquire(Sensors_operation_smph_id, osWaitForever);

            // Trigger the ADC conversion.
            ADCProcessorTrigger(ADC0_BASE, 3);
            // Wait for conversion to be completed.
            while (!ADCIntStatus(ADC0_BASE, 3, false))
            {
            }
            // Clear the ADC interrupt flag.
            ADCIntClear(ADC0_BASE, 3);
            // Read ADC Value.
            ADCSequenceDataGet(ADC0_BASE, 3, g_temp_read_data);

            // float TempSensorVout = (float)(g_temp_read_data[0] * (float)(3.3 / 4096));
            // g_temp_read_celsius = ((TempSensorVout - 1) / (-0.0055)) - 20;

            osSemaphoreRelease(Sensors_operation_smph_id);
            osEventFlagsClear(temperature_read_flag_id, FLAGS_MSK1);
        }
    }
}

void LDR_Read_Thread(void)
{
    osStatus_t timer_start_result;
    timer_start_result = osTimerStart(ldr_read_timer_id, (uint32_t)(LDR_READ_TIME));
    if (osOK != timer_start_result)
    {
        UARTprintf("Error initializing LDR read thread...\n");
    }
    else
    {
        for (;;)
        {
            // Waiting for the flag event
            osEventFlagsWait(ldr_read_flag_id, FLAGS_MSK1, osFlagsWaitAny, osWaitForever);
            osSemaphoreAcquire(Sensors_operation_smph_id, osWaitForever);

            // Trigger the ADC conversion.
            ADCProcessorTrigger(ADC1_BASE, 3);
            // Wait for conversion to be completed.
            while (!ADCIntStatus(ADC1_BASE, 3, false))
            {
            }
            // Clear the ADC interrupt flag.
            ADCIntClear(ADC1_BASE, 3);
            // Read ADC Value.
            ADCSequenceDataGet(ADC1_BASE, 3, g_ldr_read_data);

            osSemaphoreRelease(Sensors_operation_smph_id);
            osEventFlagsClear(ldr_read_flag_id, FLAGS_MSK1);
        }
    }
}

void Sensors_Report_Thread(void)
{
    osStatus_t timer_start_result;
    timer_start_result = osTimerStart(sensors_report_timer_id, (uint32_t)(SENSORS_READ_REPORT_TIME));
    if (osOK != timer_start_result)
    {
        UARTprintf("Error initializing sensors report thread...\n");
    }
    else
    {
        for (;;)
        {
            // Waiting for the flag event
            osEventFlagsWait(sensors_report_flag_id, FLAGS_MSK1, osFlagsWaitAny, osWaitForever);
            // Waits for the acquire of the sensor read and report semaphore
            osSemaphoreAcquire(Sensors_operation_smph_id, osWaitForever);

            UARTprintf("Temperature: %d\nLDR: %d\n\n", g_temp_read_data[0], g_ldr_read_data[0]);

            // char temp_string[5];
            // snprintf(temp_string, sizeof(temp_string), "%.2f", g_temp_read_celsius);
            // UARTprintf("Temperature: %d = %s C\nLDR: %d\n\n", g_temp_read_data[0], temp_string, g_ldr_read_data[0]);

            osSemaphoreRelease(Sensors_operation_smph_id);
            osEventFlagsClear(sensors_report_flag_id, FLAGS_MSK1);
        }
    }
}

osStatus_t SensorsTimersInit(void)
{
    osStatus_t result;

    temperature_read_timer_id = osTimerNew((osTimerFunc_t)Sensors_Set_Flag, osTimerPeriodic,
                                           &temp_read_id, &temperature_read_timer_attr);
    ldr_read_timer_id = osTimerNew((osTimerFunc_t)Sensors_Set_Flag, osTimerPeriodic,
                                   &ldr_read_id, &ldr_read_timer_attr);
    sensors_report_timer_id = osTimerNew((osTimerFunc_t)Sensors_Set_Flag, osTimerPeriodic,
                                         &report_sensors_id, &sensors_report_timer_attr);

    if ((NULL == temperature_read_timer_id) ||
        (NULL == ldr_read_timer_id) ||
        (NULL == sensors_report_timer_id))
    {
        result = osError;
        UARTprintf("Sensors read and report timers init error...\n");
    }
    else
    {
        result = osOK;
        UARTprintf("Sensors read and report timers init success...\n");
    }
    return result;
}

osStatus_t SensorsThreadsInit(void)
{
    osStatus_t result;

    temperature_read_thread_id = osThreadNew((osThreadFunc_t)Temperature_Read_Thread, NULL, &temperature_read_thread_attr);
    ldr_read_thread_id = osThreadNew((osThreadFunc_t)LDR_Read_Thread, NULL, &ldr_read_thread_attr);
    sensors_report_thread_id = osThreadNew((osThreadFunc_t)Sensors_Report_Thread, NULL, &sensors_report_thread_attr);

    if ((NULL == temperature_read_thread_id) ||
        (NULL == ldr_read_thread_id) ||
        (NULL == sensors_report_thread_id))
    {
        result = osError;
        UARTprintf("Sensors read and report threads init error...\n");
    }
    else
    {
        result = osOK;
        UARTprintf("Sensors read and report threads init success...\n");
    }
    return result;
}

osStatus_t SensorsEventFlagsInit(void)
{
    osStatus_t result;

    temperature_read_flag_id = osEventFlagsNew(&temperature_read_flag_attr);
    ldr_read_flag_id = osEventFlagsNew(&ldr_read_flag_attr);
    sensors_report_flag_id = osEventFlagsNew(&sensors_report_flag_attr);

    if ((NULL == temperature_read_flag_id) ||
        (NULL == ldr_read_flag_id) ||
        (NULL == sensors_report_flag_id))
    {
        result = osError;
        UARTprintf("Sensors read and report event flags init error...\n");
    }
    else
    {
        result = osOK;
        UARTprintf("Sensors read and report event flags init success...\n");
    }
    return result;
}

osStatus_t SensorsSemaphoreInit(void)
{
    osStatus_t result;

    Sensors_operation_smph_id = osSemaphoreNew(1, 1, &Sensors_operation_smph_attr);

    if (NULL == Sensors_operation_smph_id)
    {
        result = osError;
        UARTprintf("Sensors read and report semaphore init error...\n");
    }
    else
    {
        result = osOK;
        UARTprintf("Sensors read and report semaphore init success...\n");
    }
    return result;
}

/**
 * @brief LAB 3 main process
 *
 * @return int main default return value: 0 if success
 */
int main(void)
{
    osStatus_t init_result = osOK;

    LEDS_INIT();

#ifdef LAB3_Sensors_THREADS
    ADC_INIT();
#endif /* LAB3_Sensors_THREADS */

    // Enables the FPU
    ROM_FPUEnable();

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

    // Turning off all LEDs
    MAP_GPIOPinWrite(LEDS_PORT, LED1_PIN, 0);
    MAP_GPIOPinWrite(LEDS_PORT, LED2_PIN, 0);
    MAP_GPIOPinWrite(LEDS_PORT, LED3_PIN, 0);
    MAP_GPIOPinWrite(LEDS_PORT, LED4_PIN, 0);

    osKernelInitialize(); // Initialize CMSIS-RTOS

#ifdef LAB3_LEDs_THREADS
    init_result |= LEDsEventFlagsInit();
    init_result |= LEDsTimersInit();
    init_result |= LEDsThreadsInit(); // Create LEDs threads
#endif                                /* LAB3_LEDs_THREADS */

#ifdef LAB3_Sensors_THREADS
    init_result |= SensorsEventFlagsInit();
    init_result |= SensorsTimersInit();
    init_result |= SensorsSemaphoreInit();
    init_result |= SensorsThreadsInit(); // Create sensors threads
#endif                                   /* LAB3_Sensors_THREADS */

    if (osOK == init_result)
    {
        UARTprintf("System Init successfully...\n\n");
    }
    else
    {
        UARTprintf("System Init error...\n\n");
    }

    osKernelStart(); // Start thread execution

    while (1)
    {
        // Nothing to do
    }
}

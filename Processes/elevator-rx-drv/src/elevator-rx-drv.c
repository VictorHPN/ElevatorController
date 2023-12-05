/**
 * @file elevator-rx-drv.c
 * @author Victor Hugo Polli Neves, André Luiz Poloni and Isabela Amorim Siqueira
 * @brief Elevator_RX task is responsible for constructing messages requested by
 * the Elevator_Manager task and then sending them through the UART serial port.
 * @version 0.1
 * @date 2023-11-25
 */

#include "elevator-rx-drv.h"
#include "FreeRTOS.h"
#include "message_buffer.h"

/* -------------------------- Defines -------------------------- */

#define UART_RX_BUFFER_SIZE 50

/* --------------------- Gloval Variables ---------------------- */
static osThreadId_t local_thread_handle;
static const osThreadAttr_t local_thread_attr = {
    .name = ELEVATOR_RX_TASK_NAME,
    .priority = ELEVATOR_RX_TASK_PRIORITY,
    .stack_size = ELEVATOR_RX_STACK_SIZE,
};

static ELEVATOR_RX_HANDLERS_LIST *elevator_rx_handlers_list = NULL;
static osMessageQueueId_t local_outgoing_msg_queue = NULL;

static MessageBufferHandle_t uartRxMsgBufferId;
static char rxBuffer[UART_RX_BUFFER_SIZE];
volatile uint8_t bufferIdx = 0;

/* --------------- Private Functions Prototypes ---------------- */

/**
 * @brief Gets the elevator identification index.

 * @param msg recieved uart message.
 * @param size size of the message.
 * @return elevator_idx_t elevator id index.
 */
elevator_idx_t getElevatorCode(char *msg, size_t size);

/**
 * @brief Elevator manager task main function.
 *
 * @param argument task argument.
 */
void ElevatorRXTask(void *argument);

/* ------------------------------------------------------------- */

elevator_idx_t getElevatorCode(char *msg, size_t size)
{
    elevator_idx_t elevatorIdx;
    if (size > 0)
    {
        elevator_id_code_t elevatorCode = (elevator_id_code_t)msg[0];
        switch (elevatorCode)
        {
        case LEFT_ELEVATOR:
            elevatorIdx = LEFT_ELEVATOR_IDX;
            break;
        case CENTER_ELEVATOR:
            elevatorIdx = CENTER_ELEVATOR_IDX;
            break;
        case RIGHT_ELEVATOR:
            elevatorIdx = RIGHT_ELEVATOR_IDX;
            break;
        default:
            elevatorIdx = ELEVATOR_IDX_UNKNWON;
            break;
        }
    }
    else
    {
        elevatorIdx = ELEVATOR_IDX_UNKNWON;
    }
    return elevatorIdx;
}

void UART0_Handler(void)
{
    // Get the interrupt status:
    uint32_t ui32Status = UARTIntStatus(UART0_BASE, true);
    // Clear the asserted interrupts.
    UARTIntClear(UART0_BASE, ui32Status);

    // Getting all the available bytes:
    while (UARTCharsAvail(UART0_BASE))
    {
        char receivedChar = UARTCharGetNonBlocking(UART0_BASE);
        rxBuffer[bufferIdx] = receivedChar;
        bufferIdx++;

        // Check if the end of the message is reached
        if (receivedChar == '\n')
        {
            // Sends the message to UART_RX task:
            xMessageBufferSendFromISR(uartRxMsgBufferId, rxBuffer, bufferIdx, NULL);
            // Resets the buffer
            memset(rxBuffer, 0, sizeof(rxBuffer));
            bufferIdx = 0;
            break;
        }
        else
        {
            // Nothing to do
        }

        // Handle buffer overflow
        if (UART_RX_BUFFER_SIZE <= bufferIdx)
        {
            // Reset buffer if it overflows
            memset(rxBuffer, 0, sizeof(rxBuffer));
            bufferIdx = 0;
        }
        else
        {
            // Nothing to do
        }
    }
}

void ElevatorRXTask(void *argument)
{
    char localRxBuffer[20];
    size_t bytesRecieved;
    int8_t msgSize;
    char event_signal;

    // Outgoing message parameters:
    msg_id_t msgId;
    elevator_idx_t elevatorIdx;
    uint32_t position;
    event_id_t eventId;
    uint8_t floorReached;
    button_id_t buttonFloor;
    ext_button_direction_t extButtonDirection;
    for (;;)
    {
        // Initializing/resetting message parameters
        msgId = MSG_ID_NONE;
        elevatorIdx = ELEVATOR_IDX_UNKNWON;
        position = 0;
        eventId = EVENT_ID_NONE;
        floorReached = 0;
        buttonFloor = BTT_NONE;
        extButtonDirection = DIRECTION_NONE;

        // Waiting for the UART RX message:
        bytesRecieved = xMessageBufferReceive(uartRxMsgBufferId, localRxBuffer, sizeof(localRxBuffer), osWaitForever);

        // Checking the <CR><LF> bytes:
        if (('\n' == localRxBuffer[bytesRecieved - 1]) &&
            ('\r' == localRxBuffer[bytesRecieved - 2]))
        {
            msgSize = bytesRecieved - 2;
            elevatorIdx = getElevatorCode(localRxBuffer, msgSize);
            if (0 < msgSize)
            {
                if (ELEVATOR_IDX_UNKNWON != elevatorIdx)
                {
                    switch (msgSize)
                    {
                    case 2:
                        msgId = RX_EVENT_REPORT;

                        event_signal = localRxBuffer[1];

                        if (('0' <= event_signal) &&
                            ('9' >= event_signal))
                        {
                            eventId = FLOOR_REACHED;

                            char floor_str_1b[2] = {localRxBuffer[1], '\0'};
                            floorReached = atoi(floor_str_1b);
                        }
                        else if (DOORS_OPENED == event_signal)
                        {
                            eventId = DOORS_OPENED;
                        }
                        else if (DOORS_CLOSED == event_signal)
                        {
                            eventId = DOORS_CLOSED;
                        }
                        else
                        {
                            // Nothing to do
                        }
                        break;
                    case 3:
                        if (INTERNAL_BUTTON == localRxBuffer[1])
                        {
                            msgId = RX_INTERNAL_BUTTON;

                            int_button_id_code_t buttonIdCode = (int_button_id_code_t)localRxBuffer[2];
                            buttonFloor = (button_id_t)(buttonIdCode - INT_BTT_GROUND_FLOOR_CODE);
                        }
                        else // Event signal:
                        {
                            msgId = RX_EVENT_REPORT;
                            eventId = FLOOR_REACHED;

                            char floor_str_2b[3] = {localRxBuffer[1], localRxBuffer[2], '\0'};
                            floorReached = atoi(floor_str_2b);
                        }
                        break;
                    case 5:
                        if (EXTERNAL_BUTTON == localRxBuffer[1])
                        {
                            msgId = RX_EXTERNAL_BUTTON;

                            char extButtonFloor_str[3] = {localRxBuffer[2], localRxBuffer[3], '\0'};
                            buttonFloor = (button_id_t)atoi(extButtonFloor_str);
                            extButtonDirection = (ext_button_direction_t)localRxBuffer[4];
                        }
                        else
                        {
                            // Nothing to do
                        }
                    default:
                        break;
                    }
                }
                else // Altitude report
                {
                    msgId = RX_POSITION_REPORT;

                    char altitude_str[6];
                    uint8_t str_idx;
                    for (str_idx = 0; str_idx < msgSize; str_idx++)
                    {
                        altitude_str[str_idx] = localRxBuffer[str_idx];
                    }
                    altitude_str[str_idx] = '\0';
                    position = atoi(altitude_str);
                }

                elevator_mgr_msg_t outgoingMsg = {
                    .msgId = msgId,
                    .elevatorIdx = elevatorIdx,
                    .position = position,
                    .eventId = eventId,
                    .floorReached = floorReached,
                    .buttonFloor = buttonFloor,
                    .extButtonDirection = extButtonDirection,
                };
                osMessageQueuePut(local_outgoing_msg_queue, &outgoingMsg, 0, DEFAULT_MESSAGE_PUT_TIMEOUT);
            } // if (0 < msgSize)
            else
            {
                // Nothing to do
            }
        } // if (('\n' == localRxBuffer[bytesRecieved - 1]) && ('\r' == localRxBuffer[bytesRecieved - 2]))
        else
        {
            // Nothing to do
        }
    }
}

osStatus_t ElevatorRXInit(void *configParams)
{
    osStatus_t init_result;

    elevator_rx_handlers_list = (ELEVATOR_RX_HANDLERS_LIST *)configParams;

    // Queues:
    local_outgoing_msg_queue = elevator_rx_handlers_list->outgoing_msg_queue;

    // Message Buffer:
    uartRxMsgBufferId = xMessageBufferCreate(UART_RX_BUFFER_SIZE);

    // Task:
    local_thread_handle = osThreadNew((osThreadFunc_t)ElevatorRXTask, NULL, &local_thread_attr);

    if (NULL == local_thread_handle)
    {
        init_result = osError;
    }
    else
    {
        init_result = osOK;
    }
    return init_result;
}

/**
 * @file elevator-tx-drv.c
 * @author Victor Hugo Polli Neves, André Luiz Poloni and Isabela Amorim Siqueira
 * @brief Elevator_TX task is responsible for constructing messages requested by
 * the Elevator_Manager task and then sending them through the UART serial port.
 * @version 0.1
 * @date 2023-11-25
 */

#include "elevator-tx-drv.h"

/* -------------------------- Defines -------------------------- */

/* --------------------- Gloval Variables ---------------------- */
static osThreadId_t local_thread_handle;
static const osThreadAttr_t local_thread_attr = {
    .name = ELEVATOR_TX_TASK_NAME,
    .priority = ELEVATOR_TX_TASK_PRIORITY,
    .stack_size = ELEVATOR_TX_STACK_SIZE,
};

static ELEVATOR_TX_HANDLERS_LIST *elevator_tx_handlers_list = NULL;
static osMessageQueueId_t local_incoming_msg_queue = NULL;

elevator_id_code_t elevator_id_codes[NUMBER_OF_ELEVATORS] = {
    LEFT_ELEVATOR,
    CENTER_ELEVATOR,
    RIGHT_ELEVATOR};

int_button_id_code_t buttons_id_codes[NUMBER_OF_BUTTONS] = {
    INT_BTT_GROUND_FLOOR_CODE,
    INT_BTT_FLOOR_1_CODE,
    INT_BTT_FLOOR_2_CODE,
    INT_BTT_FLOOR_3_CODE,
    INT_BTT_FLOOR_4_CODE,
    INT_BTT_FLOOR_5_CODE,
    INT_BTT_FLOOR_6_CODE,
    INT_BTT_FLOOR_7_CODE,
    INT_BTT_FLOOR_8_CODE,
    INT_BTT_FLOOR_9_CODE,
    INT_BTT_FLOOR_10_CODE,
    INT_BTT_FLOOR_11_CODE,
    INT_BTT_FLOOR_12_CODE,
    INT_BTT_FLOOR_13_CODE,
    INT_BTT_FLOOR_14_CODE,
    INT_BTT_FLOOR_15_CODE};

/* --------------- Private Functions Prototypes ---------------- */

/**
 * @brief Elevator manager task main function.
 *
 * @param argument task argument.
 */
void ElevatorTXTask(void *argument);

/* ------------------------------------------------------------- */

void ElevatorTXTask(void *argument)
{
    elevator_tx_msg_t incoming_message;
    elevator_id_code_t elevator_code;
    tx_command_id_t command;
    int_button_id_code_t buttonIdCode;
    bool msgIdFound;
    for (;;)
    {
        if (osOK == osMessageQueueGet(local_incoming_msg_queue, &incoming_message, NULL, osWaitForever))
        {
            msgIdFound = true;
            // Getting the command id:
            switch (incoming_message.msgId)
            {
            case TX_INIT_ELEVATOR:
                command = CMD_INIT;
                break;
            case TX_OPEN_DOORS:
                command = CMD_OPEN_DOORS;
                break;
            case TX_CLOSE_DOORS:
                command = CMD_CLOSE_DOORS;
                break;
            case TX_MOVE_UP:
                command = CMD_MOVE_UP;
                break;
            case TX_MOVE_DOWN:
                command = CMD_MOVE_DOWN;
                break;
            case TX_STOP:
                command = CMD_STOP;
                break;
            case TX_REQUEST_POSITION:
                command = CMD_REQUEST_POSITION;
                break;
            case TX_TURN_ON_BUTTON:
                command = CMD_TURN_ON_BUTTON;
                break;
            case TX_TURN_OFF_BUTTON:
                command = CMD_TURN_OFF_BUTTON;
                break;
            default:
                msgIdFound = false;
                break;
            }

            if (true == msgIdFound)
            {
                // Getting the elevator code id:
                elevator_code = elevator_id_codes[incoming_message.elevatorIdx];

                if ((CMD_TURN_ON_BUTTON == command) ||
                    (CMD_TURN_OFF_BUTTON == command))
                { 
                    // Getting button id code:
                    buttonIdCode = buttons_id_codes[incoming_message.buttonLightId];
                    UARTprintf("%c%c%c\r", elevator_code, command, buttonIdCode);
                }
                else
                {
                    UARTprintf("%c%c\r", elevator_code, command);
                }
            }
            else
            {
                // Nothing to do
            }
        }
        else
        {
            osThreadYield();
        } // if (osOK == osMessageQueueGet(local_incoming_msg_queue, &incoming_message, NULL, osWaitForever))
    }
}

osStatus_t ElevatorTXInit(void *configParams)
{
    osStatus_t init_result;

    elevator_tx_handlers_list = (ELEVATOR_TX_HANDLERS_LIST *)configParams;

    // Queues:
    local_incoming_msg_queue = elevator_tx_handlers_list->incoming_msg_queue;

    // Task:
    local_thread_handle = osThreadNew((osThreadFunc_t)ElevatorTXTask, NULL, &local_thread_attr);

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

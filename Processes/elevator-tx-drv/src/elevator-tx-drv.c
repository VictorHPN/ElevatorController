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
    msg_id_t msgId;
    for (;;)
    {
        if (osOK == osMessageQueueGet(local_incoming_msg_queue, &incoming_message, NULL, osWaitForever))
        {
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

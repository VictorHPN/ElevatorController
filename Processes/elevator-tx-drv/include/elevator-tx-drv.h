/**
 * @file elevator-tx-drv.h
 * @author Victor Hugo Polli Neves, André Luiz Poloni and Isabela Amorim Siqueira
 * @brief Elevator TX header files, containing it's data structures and public functions.
 * @version 0.1
 */

#ifndef ELEVATOR_TX_H
#define ELEVATOR_TX_H

#include "defs.h"

/* -------------------------- Defines -------------------------- */

#define ELEVATOR_TX_TASK_NAME "ELEVATOR_TX"
#define ELEVATOR_TX_TASK_PRIORITY osPriorityNormal
#define ELEVATOR_TX_STACK_SIZE (uint16_t)512*2
#define ELEVATOR_TX_QUEUE_LENGTH (uint8_t)20

/* -------------------------- Enums ---------------------------- */

/* ------------------------- Structs --------------------------- */

/**
 * @brief Elevator TX handlers
 * It contains its message queues handler.
 */
typedef struct
{
    osMessageQueueId_t incoming_msg_queue;
} ELEVATOR_TX_HANDLERS_LIST;

/* ---------------- Public Functions Prototypes ---------------- */

/**
 * @brief Elevator TX initialization process.
 * It initializes the process task, timers, semaphores etc.
 *
 * @param configParams process configuration parameters
 * @return osStatus_t operation result
 */
osStatus_t ElevatorTXInit(void *configParams);

/* ------------------------------------------------------------- */

#endif /* ELEVATOR_TX_H */

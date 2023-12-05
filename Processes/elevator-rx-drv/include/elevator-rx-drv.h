/**
 * @file elevator-rx-drv.h
 * @author Victor Hugo Polli Neves, André Luiz Poloni and Isabela Amorim Siqueira
 * @brief Elevator RX header files, containing it's data structures and public functions.
 * @version 0.1
 */

#ifndef ELEVATOR_RX_H
#define ELEVATOR_RX_H

#include "defs.h"

/* -------------------------- Defines -------------------------- */

#define ELEVATOR_RX_TASK_NAME "ELEVATOR_RX"
#define ELEVATOR_RX_TASK_PRIORITY osPriorityNormal
#define ELEVATOR_RX_STACK_SIZE (uint16_t)512

/* -------------------------- Enums ---------------------------- */

/* ------------------------- Structs --------------------------- */

/**
 * @brief Elevator RX handlers
 * It contains its message queues handler.
 */
typedef struct
{
    osMessageQueueId_t outgoing_msg_queue;
} ELEVATOR_RX_HANDLERS_LIST;

/* ---------------- Public Functions Prototypes ---------------- */

/**
 * @brief UART 0 interruption handler.
 * 
 * It handles the messages recieved from the elevator simulator.
 */
void UART0_Handler(void);

/**
 * @brief Elevator RX initialization process.
 * It initializes the process task, timers, semaphores etc.
 *
 * @param configParams process configuration parameters
 * @return osStatus_t operation result
 */
osStatus_t ElevatorRXInit(void *configParams);

/* ------------------------------------------------------------- */

#endif /* ELEVATOR_RX_H */

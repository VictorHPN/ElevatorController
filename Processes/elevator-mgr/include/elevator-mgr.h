/**
 * @file elevator-mgr.h
 * @author Victor Hugo Polli Neves, André Luiz Poloni and Isabela Amorim Siqueira
 * @brief Elevator manager header files, containing it's data structures and public functions.
 * @version 0.1
 */

#ifndef ELEVATOR_MGR_H
#define ELEVATOR_MGR_H

#include "defs.h"

/* -------------------------- Defines -------------------------- */

#define ELEVATOR_MGR_TASK_NAME "ELEVATOR_MGR"
#define ELEVATOR_MGR_TASK_PRIORITY osPriorityNormal
#define ELEVATOR_MGR_STACK_SIZE (uint16_t)(512 * 4)
#define ELEVATOR_MGR_QUEUE_LENGTH (uint8_t)30

#define MAX_NUMBER_OFF_STOPS (uint8_t)5
/* -------------------------- Enums ---------------------------- */

/**
 * @brief Elevator states
 */
typedef enum
{
    ELEVATOR_STATE_NONE = (uint8_t)0x00,
    IDLE,
    CLOSING_DOORS,
    MOVING,
    OPENING_DOORS,
    ELEVATOR_STATE_UNKNOWN = (uint8_t)0xFF
} elevator_state_t;

/**
 * @brief Elevator moving states
 */
typedef enum
{
    STOPPED,
    MOVING_UP,
    MOVING_DOWN
} moving_state_t;

/**
 * @brief Elevator door states
 */
typedef enum
{
    CLOSED,
    OPEN
} door_state_t;

/* ------------------------- Structs --------------------------- */

/**
 * @brief Elevator manager handlers
 * It contains its message queues handler.
 */
typedef struct
{
    osMessageQueueId_t incoming_msg_queue;
    osMessageQueueId_t outgoing_msg_queue;
} ELEVATOR_MGR_HANDLERS_LIST;

/**
 * @brief Elevator data structurs
 * This structure is used to store all the data of an elevator,
 * to control its states and actions.
 */
typedef struct
{
    elevator_state_t state;                       //!< Elevator current state
    uint8_t floor;                                //!< Elevator current floor
    uint32_t position;                            //!< Elevator current position [mm]
    uint8_t next_floor;                           //!< Elevator next floor
    uint8_t upStopFloors[MAX_NUMBER_OFF_STOPS];   //!< List of up floors to stop
    uint8_t downStopFloors[MAX_NUMBER_OFF_STOPS]; //!< List of down floors to stop
    uint8_t upStops;                              //!< Number of up floors to stop
    uint8_t downStops;                            //!< Number of down floors to stop
    moving_state_t movingSt;                      //!< Elevator current moving state (STOPPED, MOVING_UP or MOVING_DOWN)
    door_state_t doorsSt;                         //!< Elevator current door state (CLOSED or OPEN)
} elevator_t;

/* ---------------- Public Functions Prototypes ---------------- */

/**
 * @brief Elevator manager initialization process.
 * It initializes the process task, timers, semaphores etc.
 *
 * @param configParams process configuration parameters
 * @return osStatus_t operation result
 */
osStatus_t ElevatorMgrInit(void *configParams);

/* ------------------------------------------------------------- */

#endif /* ELEVATOR_MGR_H */

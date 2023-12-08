/**
 * @file elevator-mgr.c
 * @author Victor Hugo Polli Neves, André Luiz Poloni and Isabela Amorim Siqueira
 * @brief Elevator_Manager, responsible for storing and updating data and states
 * of three elevators, monitoring their positions, door states, buttons, etc.
 * Additionally, this task handles requesting message transmission by
 * communicating with the third task, Elevator_TX.
 * @version 0.1
 */

#include "elevator-mgr.h"

/* -------------------------- Defines -------------------------- */
#define DOORS_OPERATION_TIMEOUT 7500 // ms
#define STOP_OPEN_DOORS_TIME 3000    // ms

/* --------------------- Gloval Variables ---------------------- */
static osThreadId_t local_thread_handle;
static const osThreadAttr_t local_thread_attr = {
    .name = ELEVATOR_MGR_TASK_NAME,
    .priority = ELEVATOR_MGR_TASK_PRIORITY,
    .stack_size = ELEVATOR_MGR_STACK_SIZE,
};

static osTimerId_t elevators_doors_timeout_timers[NUMBER_OF_ELEVATORS];
static const osTimerAttr_t elevators_doors_timeout_timers_attrs[NUMBER_OF_ELEVATORS] = {
    {.name = "LE_DOORS_TIMEOUT"},
    {.name = "CE_DOORS_TIMEOUT"},
    {.name = "RE_DOORS_TIMEOUT"},
};

static osTimerId_t elevators_stop_open_doors_timers[NUMBER_OF_ELEVATORS];
static const osTimerAttr_t elevators_stop_open_doors_timers_attrs[NUMBER_OF_ELEVATORS] = {
    {.name = "LE_STOP_OPEN_DOORS_TIMER"},
    {.name = "CE_STOP_OPEN_DOORS_TIMER"},
    {.name = "RE_STOP_OPEN_DOORS_TIMER"},
};

static ELEVATOR_MGR_HANDLERS_LIST *elevator_mgr_handlers_list = NULL;
static osMessageQueueId_t local_incoming_msg_queue = NULL;
static osMessageQueueId_t local_outgoing_msg_queue = NULL;

static elevator_t elevators_list[NUMBER_OF_ELEVATORS];
static elevator_idx_t elevatorsIdxs[NUMBER_OF_ELEVATORS] = {
    LEFT_ELEVATOR_IDX,
    CENTER_ELEVATOR_IDX,
    RIGHT_ELEVATOR_IDX,
};

/* --------------- Private Functions Prototypes ---------------- */

/**
 * @brief Manages the timeout scenario for the door close or open operation.
 * This timeout callback is invoked when the elevator simulator does not receive an
 * open or close door command. In such a case, the simulator will not send the
 * fully open/closed doors event report, so the command needs to be resent.
 * @param arg argument - elevator idx.
 */
void DoorsTimeoutCallback(void *arg);

/**
 * @brief Starts the doors close or open timeout timer.
 *
 * @param elevatorIdx index of the elevator
 */
void DoorsTimemoutTimerStart(elevator_idx_t elevatorIdx);

/**
 * @brief Stops the doors close or open timeout timer.
 *
 * @param elevatorIdx index of the elevator
 */
void DoorsTimemoutTimerStop(elevator_idx_t elevatorIdx);

/**
 * @brief Closes the elevetor doors.
 * This function is the
 *
 * @param arg argument - elevator idx.
 */
void CloseDoorsAfterStop(void *arg);

/**
 * @brief Checks if a button was pressed to add a stop.
 *
 * @param elevator elevator that will be checked.
 * @param incoming_message incoming message from uart rx
 */
void CheckButtonPressed(elevator_t *elevator, elevator_mgr_msg_t *incoming_message);

/**
 * @brief Swaps the floors. That is used to sort the stops
 *
 * @param floorX floor 1 to swap
 * @param floorY floor 2 to swap
 */
void swapFLoor(uint8_t *floorX, uint8_t *floorY);

/**
 * @brief Sorts the up floors to stop.
 *
 * @param elevator elevator to sort the upStops
 */
void SortUpStops(elevator_t *elevator);

/**
 * @brief Sorts the down floors to stop.
 *
 * @param elevator elevator to sort the downStops
 */
void SortDownStops(elevator_t *elevator);

/**
 * @brief Verifies if the next floor to stop added is near the current
 * elevator position. If it is, the elevator should stop at it.
 *
 * @param elevator elevator to update the next stop
 * @param stopDirection direction of the added stop (1 for upStop and anything
 * different for downStop)
 */
void UpdateNextFloorToStop(elevator_t *elevator, uint8_t stopDirection);

/**
 * @brief Adds a floor to the stops list of the elevator
 *
 * @param elevator elevator to add the stop
 * @param incoming_message incoming message from uart rx
 * @return true if the stop was added
 * @return false if it could not add the stop
 */
bool ElevatorMgrAddFloorToStop(elevator_t *elevator, elevator_mgr_msg_t *incoming_message);

/**
 * @brief Add a stop to the list of up floors to stop
 *
 * @param elevator elevator to add the up floor stop
 * @param up_floor up floor to be added
 * @return true if the stop was added
 * @return false if it could not add the stop
 */
bool AddUpFloorStop(elevator_t *elevator, uint8_t up_floor);

/**
 * @brief Add a stop to the list of down floors to stop
 *
 * @param elevator elevator to add the down floor stop
 * @param down_floor down floor to be added
 * @return true if the stop was added
 * @return false if it could not add the stop
 */
bool AddDownFloorStop(elevator_t *elevator, uint8_t down_floor);

/**
 * @brief
 *
 * @param elevator elevator to get next stop
 * @return uint8_t Number of the next floor to stop. If returns the 0xFF value, there is no more stops.
 */
uint8_t ElevatorMgrGetNextFloor(elevator_t *elevator);

/**
 * @brief Get the Next up Floor to stop
 *
 * @param elevator elevator to get next up floor stop
 * @return uint8_t next up floor to stop
 */
uint8_t GetNextUpFloor(elevator_t *elevator);

/**
 * @brief Get the Next down Floor to stop
 *
 * @param elevator elevator to get next down floor stop
 * @return uint8_t next down floor to stop
 */
uint8_t GetNextDownFloor(elevator_t *elevator);

/**
 * @brief Sends a message to be trasmited to the elevator simulator.

 * @param msgId tx message id
 * @param elevatorIdx index of the elevator
 * @param buttonLightId button id (for turn its light on and off)
 */
void ElevatorMgrSendMsgToTX(msg_id_t msgId, elevator_idx_t elevatorIdx, button_id_t buttonLightId);

/**
 * @brief Initiliazes all elevators send the command to do it
 */
void ElevatorMgrInitElevators(void);

/**
 * @brief Elevator manager task main function.
 *
 * @param argument task argument.
 */
void ElevatorMgrTask(void *argument);

/* ------------------------------------------------------------- */

void DoorsTimeoutCallback(void *arg)
{
    elevator_idx_t *elevatorIdx = (elevator_idx_t *)arg;
    elevator_t *elevator = &elevators_list[*elevatorIdx];

    if (CLOSING_DOORS == elevator->state)
    {
        ElevatorMgrSendMsgToTX(TX_CLOSE_DOORS, *elevatorIdx, BTT_NONE);
    }
    else if (OPENING_DOORS == elevator->state)
    {
        ElevatorMgrSendMsgToTX(TX_OPEN_DOORS, *elevatorIdx, BTT_NONE);
    }
    else
    {
        // Nothing to do
    }
}

void DoorsTimemoutTimerStart(elevator_idx_t elevatorIdx)
{
    osStatus_t timerOperationResult;
    if (false == osTimerIsRunning(elevators_doors_timeout_timers[elevatorIdx]))
    {
        do
        {
            timerOperationResult = osTimerStart(elevators_doors_timeout_timers[elevatorIdx],
                                                DOORS_OPERATION_TIMEOUT);
        } while (osOK != timerOperationResult);
    }
}

void DoorsTimemoutTimerStop(elevator_idx_t elevatorIdx)
{
    osStatus_t timerOperationResult;
    if (true == osTimerIsRunning(elevators_doors_timeout_timers[elevatorIdx]))
    {
        do
        {
            timerOperationResult = osTimerStop(elevators_doors_timeout_timers[elevatorIdx]);
        } while (osOK != timerOperationResult);
    }
}

void CloseDoorsAfterStop(void *arg)
{
    elevator_idx_t *elevatorIdx = (elevator_idx_t *)arg;
    elevator_t *elevator = &elevators_list[*elevatorIdx];

    ElevatorMgrSendMsgToTX(TX_CLOSE_DOORS, *elevatorIdx, BTT_NONE);
    elevator->state = CLOSING_DOORS;
}

void CheckButtonPressed(elevator_t *elevator, elevator_mgr_msg_t *incoming_message)
{
    if ((RX_INTERNAL_BUTTON == incoming_message->msgId) ||
        (RX_EXTERNAL_BUTTON == incoming_message->msgId))
    {
        if (true == ElevatorMgrAddFloorToStop(elevator, incoming_message))
        {
            if (RX_INTERNAL_BUTTON == incoming_message->msgId)
            {
                ElevatorMgrSendMsgToTX(TX_TURN_ON_BUTTON,
                                       incoming_message->elevatorIdx,
                                       incoming_message->buttonFloor);
            }
        }
    }
}

bool ElevatorMgrAddFloorToStop(elevator_t *elevator, elevator_mgr_msg_t *incoming_message)
{
    bool stop_added = false;
    uint8_t new_floor = incoming_message->buttonFloor;

    if ((15 >= new_floor) &&
        (new_floor != elevator->next_floor))
    {
        if (RX_EXTERNAL_BUTTON == incoming_message->msgId)
        {
            if (MOVING_UP == elevator->movingSt)
            {
                if ((MOVE_UP_BUTTON == incoming_message->extButtonDirection &&
                     elevator->floor < new_floor))
                {
                    stop_added = AddUpFloorStop(elevator, new_floor);
                }
                else
                {
                    stop_added = AddDownFloorStop(elevator, new_floor);
                }
            }
            else if (MOVING_DOWN == elevator->movingSt)
            {
                if ((MOVE_DOWN_BUTTON == incoming_message->extButtonDirection) &&
                    elevator->floor >= new_floor)
                {
                    stop_added = AddDownFloorStop(elevator, new_floor);
                }
                else
                {
                    stop_added = AddUpFloorStop(elevator, new_floor);
                }
            }
            else
            {
                // Nothing to do
            }
        }
        else // RX_INTERNAL_BUTTON
        {
            if (elevator->floor < new_floor)
            {
                stop_added = AddUpFloorStop(elevator, new_floor);
            }
            else
            {
                stop_added = AddDownFloorStop(elevator, new_floor);
            }
        }
    }

    return stop_added;
}

bool AddUpFloorStop(elevator_t *elevator, uint8_t up_floor)
{
    bool stop_added = false;
    bool stopRepeated = false;

    // Checks if the new stop is already in the stops list:
    for (uint8_t stopIxd = 0; stopIxd < elevator->upStops; stopIxd++)
    {
        if (up_floor == elevator->upStopFloors[stopIxd])
        {
            stopRepeated = true;
            break;
        }
    }

    if ((elevator->upStops < MAX_NUMBER_OFF_STOPS) &&
        (false == stopRepeated))
    {
        elevator->upStopFloors[elevator->upStops] = up_floor;
        elevator->upStops++;
        stop_added = true;

        UpdateNextFloorToStop(elevator, 1);
        SortUpStops(elevator);
    }
    return stop_added;
}

bool AddDownFloorStop(elevator_t *elevator, uint8_t down_floor)
{
    bool stop_added = false;
    bool stopRepeated = false;

    // Checks if the new stop is already in the stops list:
    for (uint8_t stopIxd = 0; stopIxd < elevator->downStops; stopIxd++)
    {
        if (down_floor == elevator->downStopFloors[stopIxd])
        {
            stopRepeated = true;
            break;
        }
    }

    if ((elevator->downStops < MAX_NUMBER_OFF_STOPS) &&
        (false == stopRepeated))
    {
        elevator->downStopFloors[elevator->downStops] = down_floor;
        elevator->downStops++;
        stop_added = true;

        UpdateNextFloorToStop(elevator, 0);
        SortDownStops(elevator);
    }
    return stop_added;
}

void UpdateNextFloorToStop(elevator_t *elevator, uint8_t stopDirection)
{
    uint8_t currentNextFloor = elevator->next_floor;
    uint8_t *lastAddedFloor = NULL;

    if (MOVING == elevator->state)
    {
        if ((1 == stopDirection) &&
            (MOVING_UP == elevator->movingSt))
        {
            lastAddedFloor = &elevator->upStopFloors[elevator->upStops - 1];
            if (*lastAddedFloor < currentNextFloor)
            {
                elevator->next_floor = *lastAddedFloor;
                *lastAddedFloor = currentNextFloor;
            }
            else
            {
                // Nothing to
            }
        }
        else if ((1 != stopDirection) &&
                 (MOVING_DOWN == elevator->movingSt))
        {
            lastAddedFloor = &elevator->downStopFloors[elevator->downStops - 1];
            if (*lastAddedFloor > currentNextFloor)
            {
                elevator->next_floor = *lastAddedFloor;
                *lastAddedFloor = currentNextFloor;
            }
            else
            {
                // Nothing to
            }
        }
        else
        {
            // Nothing to
        }
    }
    else
    {
        // Nothing to
    }
}

void swapFLoor(uint8_t *floorX, uint8_t *floorY)
{
    uint8_t temp = *floorX;
    *floorX = *floorY;
    *floorY = temp;
}

void SortUpStops(elevator_t *elevator)
{
    uint8_t lowest_floor_idx;
    for (uint8_t floor_i = 0; floor_i < elevator->upStops - 1; floor_i++)
    {
        // Find the lowest floor
        lowest_floor_idx = floor_i;
        for (uint8_t floor_j = floor_i + 1; floor_j < elevator->upStops; floor_j++)
        {
            if (elevator->upStopFloors[floor_j] < elevator->upStopFloors[lowest_floor_idx])
            {
                lowest_floor_idx = floor_j;
            }
        }

        // Swap the found lowest floor
        swapFLoor(&elevator->upStopFloors[lowest_floor_idx], &elevator->upStopFloors[floor_i]);
    }
}

void SortDownStops(elevator_t *elevator)
{
    uint8_t highest_floor_idx;
    for (uint8_t floor_i = 0; floor_i < elevator->downStops - 1; floor_i++)
    {
        // Find the lowest floor
        highest_floor_idx = floor_i;
        for (uint8_t floor_j = floor_i + 1; floor_j < elevator->downStops; floor_j++)
        {
            if (elevator->downStopFloors[floor_j] > elevator->downStopFloors[highest_floor_idx])
            {
                highest_floor_idx = floor_j;
            }
        }

        // Swap the found lowest floor
        swapFLoor(&elevator->downStopFloors[highest_floor_idx], &elevator->downStopFloors[floor_i]);
    }
}

uint8_t ElevatorMgrGetNextFloor(elevator_t *elevator)
{
    uint8_t next_floor;

    if (MOVING_UP == elevator->movingSt)
    {
        if (0 < elevator->upStops)
        {
            next_floor = GetNextUpFloor(elevator);
        }
        else if (0 < elevator->downStops)
        {
            next_floor = GetNextDownFloor(elevator);
        }
        else
        {
            next_floor = ELEVATOR_FLOOR_NONE;
        }
    }
    else if (MOVING_DOWN == elevator->movingSt)
    {
        if (0 < elevator->downStops)
        {
            next_floor = GetNextDownFloor(elevator);
        }
        else if (0 < elevator->upStops)
        {
            next_floor = GetNextUpFloor(elevator);
        }
        else
        {
            next_floor = ELEVATOR_FLOOR_NONE;
        }
    }
    else
    {
        next_floor = ELEVATOR_FLOOR_NONE;
    }

    return next_floor;
}

uint8_t GetNextUpFloor(elevator_t *elevator)
{
    uint8_t next_floor = elevator->upStopFloors[0];
    elevator->upStops--;

    if (0 == elevator->upStops)
    {
        memset(elevator->upStopFloors, ELEVATOR_FLOOR_NONE, sizeof(elevator->upStopFloors));
    }
    else
    {
        memcpy(elevator->upStopFloors, elevator->upStopFloors + 1, elevator->upStops);
    }
    return next_floor;
}

uint8_t GetNextDownFloor(elevator_t *elevator)
{
    uint8_t next_floor = elevator->downStopFloors[0];
    elevator->downStops--;

    if (0 == elevator->downStops)
    {
        memset(elevator->downStopFloors, ELEVATOR_FLOOR_NONE, sizeof(elevator->downStopFloors));
    }
    else
    {
        memcpy(elevator->downStopFloors, elevator->downStopFloors + 1, elevator->downStops);
    }
    return next_floor;
}

void ElevatorMgrSendMsgToTX(msg_id_t msgId, elevator_idx_t elevatorIdx, button_id_t buttonLightId)
{
    elevator_tx_msg_t outgoingMsg =
        {
            .msgId = msgId,
            .elevatorIdx = elevatorIdx,
            .buttonLightId = buttonLightId,
        };
    osMessageQueuePut(local_outgoing_msg_queue, &outgoingMsg, 0, DEFAULT_MESSAGE_PUT_TIMEOUT);
}

void ElevatorMgrInitElevators(void)
{
    for (uint8_t elevatorIdx = 0; elevatorIdx < NUMBER_OF_ELEVATORS; elevatorIdx++)
    {
        elevators_list[elevatorIdx].state = IDLE;
        elevators_list[elevatorIdx].floor = 0;
        elevators_list[elevatorIdx].position = 0;
        elevators_list[elevatorIdx].next_floor = ELEVATOR_FLOOR_NONE;
        memset(elevators_list[elevatorIdx].upStopFloors, ELEVATOR_FLOOR_NONE, MAX_NUMBER_OFF_STOPS);
        memset(elevators_list[elevatorIdx].downStopFloors, ELEVATOR_FLOOR_NONE, MAX_NUMBER_OFF_STOPS);
        elevators_list[elevatorIdx].movingSt = STOPPED;
        elevators_list[elevatorIdx].doorsSt = OPEN;

        ElevatorMgrSendMsgToTX(TX_INIT_ELEVATOR, (elevator_idx_t)elevatorIdx, BTT_NONE);
    }
}

void ElevatorMgrTask(void *argument)
{
    ElevatorMgrInitElevators();

    elevator_mgr_msg_t incoming_message;
    elevator_idx_t elevatorIdx;
    elevator_t *elevator;
    msg_id_t msgId;
    for (;;)
    {
        if (osOK == osMessageQueueGet(local_incoming_msg_queue, &incoming_message, NULL, osWaitForever))
        {
            elevatorIdx = incoming_message.elevatorIdx;
            elevator = &elevators_list[elevatorIdx];
            msgId = incoming_message.msgId;
            switch (elevator->state)
            {
            case IDLE:
                elevator->movingSt = STOPPED;

                if ((RX_INTERNAL_BUTTON == msgId) ||
                    (RX_EXTERNAL_BUTTON == msgId))
                {
                    if (elevator->floor != incoming_message.buttonFloor)
                    {
                        elevator->next_floor = incoming_message.buttonFloor;

                        if (RX_INTERNAL_BUTTON == msgId)
                        {
                            ElevatorMgrSendMsgToTX(TX_TURN_ON_BUTTON, elevatorIdx, incoming_message.buttonFloor);
                        }
                        ElevatorMgrSendMsgToTX(TX_CLOSE_DOORS, elevatorIdx, BTT_NONE);

                        DoorsTimemoutTimerStart(elevatorIdx);

                        elevator->state = CLOSING_DOORS;
                    }
                }
                break;
            case CLOSING_DOORS:
                CheckButtonPressed(elevator, &incoming_message);

                if ((RX_EVENT_REPORT == msgId) &&
                    (DOORS_CLOSED == incoming_message.eventId))
                {
                    if (elevator->floor > elevator->next_floor)
                    {
                        ElevatorMgrSendMsgToTX(TX_MOVE_DOWN, elevatorIdx, BTT_NONE);
                        elevator->movingSt = MOVING_DOWN;
                    }
                    else
                    {
                        ElevatorMgrSendMsgToTX(TX_MOVE_UP, elevatorIdx, BTT_NONE);
                        elevator->movingSt = MOVING_UP;
                    }
                    ElevatorMgrSendMsgToTX(TX_REQUEST_POSITION, elevatorIdx, BTT_NONE);

                    DoorsTimemoutTimerStop(elevatorIdx);

                    elevator->doorsSt = CLOSED;
                    elevator->state = MOVING;
                }
                break;
            case MOVING:
                CheckButtonPressed(elevator, &incoming_message);

                if ((RX_EVENT_REPORT == msgId) &&
                    (FLOOR_REACHED == incoming_message.eventId))
                {
                    elevator->floor = incoming_message.floorReached;
                    if (elevator->next_floor == incoming_message.floorReached)
                    {
                        ElevatorMgrSendMsgToTX(TX_STOP, elevatorIdx, BTT_NONE);
                        ElevatorMgrSendMsgToTX(TX_TURN_OFF_BUTTON, elevatorIdx, (button_id_t)elevator->next_floor);
                        ElevatorMgrSendMsgToTX(TX_OPEN_DOORS, elevatorIdx, BTT_NONE);

                        DoorsTimemoutTimerStart(elevatorIdx);

                        elevator->floor = elevator->next_floor;
                        elevator->state = OPENING_DOORS;
                    }
                }
                else
                {
                    // Nothing to do
                }
                break;
            case OPENING_DOORS:
                CheckButtonPressed(elevator, &incoming_message);

                if ((RX_EVENT_REPORT == msgId) &&
                    (DOORS_OPENED == incoming_message.eventId) &&
                    false == osTimerIsRunning(elevators_stop_open_doors_timers[elevatorIdx]))
                {
                    elevator->doorsSt = OPEN;
                    elevator->next_floor = ElevatorMgrGetNextFloor(elevator);

                    DoorsTimemoutTimerStop(elevatorIdx);

                    if (ELEVATOR_FLOOR_NONE == elevator->next_floor)
                    {
                        elevator->state = IDLE;
                    }
                    else
                    {
                        // Wait 5 seconds to close the elevator door:
                        osTimerStart(elevators_stop_open_doors_timers[elevatorIdx], STOP_OPEN_DOORS_TIME);
                    }
                }
                break;
            default:
                // Nothing to do
                break;
            } // switch (elevatorState)
        }
        else
        {
            osThreadYield();
        } // if (osOK == osMessageQueueGet(local_incoming_msg_queue, &incoming_message, NULL, osWaitForever))
    }
}

osStatus_t ElevatorMgrInit(void *configParams)
{
    osStatus_t init_result;

    elevator_mgr_handlers_list = (ELEVATOR_MGR_HANDLERS_LIST *)configParams;

    // Queues:
    local_incoming_msg_queue = elevator_mgr_handlers_list->incoming_msg_queue;
    local_outgoing_msg_queue = elevator_mgr_handlers_list->outgoing_msg_queue;

    // Timers:
    for (uint8_t elevatorIdx = 0; elevatorIdx < NUMBER_OF_ELEVATORS; elevatorIdx++)
    {
        elevators_doors_timeout_timers[elevatorIdx] = osTimerNew((osTimerFunc_t)DoorsTimeoutCallback,
                                                                 osTimerPeriodic,
                                                                 &elevatorsIdxs[elevatorIdx],
                                                                 &elevators_doors_timeout_timers_attrs[elevatorIdx]);

        elevators_stop_open_doors_timers[elevatorIdx] = osTimerNew((osTimerFunc_t)CloseDoorsAfterStop,
                                                                   osTimerOnce,
                                                                   &elevatorsIdxs[elevatorIdx],
                                                                   &elevators_stop_open_doors_timers_attrs[elevatorIdx]);
    }

    // Task:
    local_thread_handle = osThreadNew((osThreadFunc_t)ElevatorMgrTask, NULL, &local_thread_attr);

    if ((NULL == local_thread_handle) ||
        (NULL == elevators_doors_timeout_timers[LEFT_ELEVATOR_IDX]) ||
        (NULL == elevators_doors_timeout_timers[CENTER_ELEVATOR_IDX]) ||
        (NULL == elevators_doors_timeout_timers[RIGHT_ELEVATOR_IDX]) ||
        (NULL == elevators_stop_open_doors_timers[LEFT_ELEVATOR_IDX]) ||
        (NULL == elevators_stop_open_doors_timers[CENTER_ELEVATOR_IDX]) ||
        (NULL == elevators_stop_open_doors_timers[RIGHT_ELEVATOR_IDX]))
    {
        init_result = osError;
    }
    else
    {
        init_result = osOK;
    }
    return init_result;
}

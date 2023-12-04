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

/* --------------------- Gloval Variables ---------------------- */
static osThreadId_t local_thread_handle;
static const osThreadAttr_t local_thread_attr = {
    .name = ELEVATOR_MGR_TASK_NAME,
    .priority = ELEVATOR_MGR_TASK_PRIORITY,
    .stack_size = ELEVATOR_MGR_STACK_SIZE,
};

static ELEVATOR_MGR_HANDLERS_LIST *elevator_mgr_handlers_list = NULL;
static osMessageQueueId_t local_incoming_msg_queue = NULL;
static osMessageQueueId_t local_outgoing_msg_queue = NULL;

static elevator_t elevators_list[NUMBER_OF_ELEVATORS];

/* --------------- Private Functions Prototypes ---------------- */

/**
 * @brief Adds a floor to the stops list of the elevator
 *
 * @param elevator elevator to add the stop
 * @param new_floor floor of the stop
 * @return true if the stop was added
 * @return false if it could not add the stop
 */
bool ElevatorMgrAddFloorToStop(elevator_t *elevator, uint8_t new_floor);

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

void CheckButtonPressed(elevator_t *elevator, elevator_mgr_msg_t *incoming_message)
{
    if ((RX_INTERNAL_BUTTON == incoming_message->msgId) ||
        (RX_EXTERNAL_BUTTON == incoming_message->msgId))
    {
        if (true == ElevatorMgrAddFloorToStop(elevator, incoming_message->buttonFloor))
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

bool ElevatorMgrAddFloorToStop(elevator_t *elevator, uint8_t new_floor)
{
    bool stop_added = false;

    if (15 >= new_floor)
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

    return stop_added;
}

bool AddUpFloorStop(elevator_t *elevator, uint8_t up_floor)
{
    bool stop_added = false;
    if (elevator->upStops < MAX_NUMBER_OFF_STOPS)
    {
        elevator->upStopFloors[elevator->upStops] = up_floor;
        elevator->upStops++;
        stop_added = true;
    }
    return stop_added;
}

bool AddDownFloorStop(elevator_t *elevator, uint8_t down_floor)
{
    bool stop_added = false;
    if (elevator->downStops < MAX_NUMBER_OFF_STOPS)
    {
        elevator->downStopFloors[elevator->downStops] = down_floor;
        elevator->downStops++;
        stop_added = true;
    }
    return stop_added;
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

                    elevator->doorsSt = CLOSED;
                    elevator->state = MOVING;
                }
                break;
            case MOVING:
                CheckButtonPressed(elevator, &incoming_message);

                if ((RX_EVENT_REPORT == msgId) &&
                    (FLOOR_REACHED == incoming_message.eventId) &&
                    (elevator->next_floor == incoming_message.floorReached))
                {
                    ElevatorMgrSendMsgToTX(TX_STOP, elevatorIdx, BTT_NONE);
                    ElevatorMgrSendMsgToTX(TX_TURN_OFF_BUTTON, elevatorIdx, (button_id_t)elevator->next_floor);
                    ElevatorMgrSendMsgToTX(TX_OPEN_DOORS, elevatorIdx, BTT_NONE);

                    elevator->floor = elevator->next_floor;
                    elevator->position = elevator->next_floor * 5000;
                    elevator->state = OPENING_DOORS;
                }
                else
                {
                    // Nothing to do
                }
                break;
            case OPENING_DOORS:
                CheckButtonPressed(elevator, &incoming_message);

                if ((RX_EVENT_REPORT == msgId) &&
                    (DOORS_OPENED == incoming_message.eventId))
                {
                    elevator->doorsSt = OPEN;
                    elevator->next_floor = ElevatorMgrGetNextFloor(elevator);

                    if (ELEVATOR_FLOOR_NONE == elevator->next_floor)
                    {
                        elevator->state = IDLE;
                    }
                    else
                    {
                        osDelay(5000); // Wait 5 seconds to close the elevator door
                        ElevatorMgrSendMsgToTX(TX_CLOSE_DOORS, elevatorIdx, BTT_NONE);
                        elevator->state = CLOSING_DOORS;
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

    // Task:
    local_thread_handle = osThreadNew((osThreadFunc_t)ElevatorMgrTask, NULL, &local_thread_attr);

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

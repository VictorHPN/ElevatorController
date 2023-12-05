

#ifndef DEFS_H
#define DEFS_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_ints.h"
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

#define DEFAULT_MESSAGE_PUT_TIMEOUT (uint8_t)200 // ms
#define ELEVATOR_FLOOR_NONE (uint8_t)0xFF

/**
 * @brief Message identifiers for use in message queues.
 */
typedef enum
{
    MSG_ID_NONE = (uint8_t)0x00, //!< None message id (not initialized)
    // RX messages types:
    RX_POSITION_REPORT, //!< Message: report of the position of the elevator in milimeters
    RX_EVENT_REPORT,    //!< Message: report of an event that ocurred in the simulator
    RX_INTERNAL_BUTTON, //!< Message: report of the pressing of an internal button
    RX_EXTERNAL_BUTTON, //!< Message: report of the pressing of an external button
    // TX messages types:
    TX_INIT_ELEVATOR,    //!< Message: sends the elevator to the ground floor with open doors
    TX_OPEN_DOORS,       //!< Message: opens the elevator doors
    TX_CLOSE_DOORS,      //!< Message: closes the elevator doors
    TX_MOVE_UP,          //!< Message: orders the elevator to move up until a stop command is received
    TX_MOVE_DOWN,        //!< Message: orders the elevator to move down until a stop command is received
    TX_STOP,             //!< Message: orders the elevator ot stop
    TX_REQUEST_POSITION, //!< Message: requests the elevator position
    TX_TURN_ON_BUTTON,   //!< Message: lights up the specified button
    TX_TURN_OFF_BUTTON,  //!< Message: turns off the light of the specified button

    MSG_ID_UNKNOWN = (uint8_t)0xFF //!< Unknown message id
} msg_id_t;

/**
 * @brief Elevators ids, used to communicate with
 * the elevator simulator via serial port.
 */
typedef enum
{
    LEFT_ELEVATOR = 'e',   //!< Left elevator identifier
    CENTER_ELEVATOR = 'c', //!< Center elevator identifier
    RIGHT_ELEVATOR = 'd'   //!< Right elevator identifier
} elevator_id_code_t;

typedef enum
{
    LEFT_ELEVATOR_IDX = 0,      //!< Left elevator identifier
    CENTER_ELEVATOR_IDX = 1,    //!< Center elevator identifier
    RIGHT_ELEVATOR_IDX = 2,     //!< Right elevator identifier
    NUMBER_OF_ELEVATORS,        //!< Number of elevators idxs
    ELEVATOR_IDX_UNKNWON = 0xFF //!< Unknown elevator idx
} elevator_idx_t;

/**
 * @brief Trasmission commands identification codes.
 * Command that are sent to the simulato by the elevator controller.
 */
typedef enum
{
    CMD_INIT = 'r',             //!< Sends the elevator to the ground floor with open doors
    CMD_OPEN_DOORS = 'a',       //!< Opens the elevator doors
    CMD_CLOSE_DOORS = 'f',      //!< Closes the elevator doors
    CMD_MOVE_UP = 's',          //!< Orders the elevator to move up until a stop command is received
    CMD_MOVE_DOWN = 'd',        //!< Orders the elevator to move down until a stop command is received
    CMD_STOP = 'p',             //!< Orders the elevator ot stop
    CMD_REQUEST_POSITION = 'x', //!< Requests the elevator position
    CMD_TURN_ON_BUTTON = 'L',   //!< Lights up the specified button
    CMD_TURN_OFF_BUTTON = 'D'   //!< Turns off the light of the specified button
} tx_command_id_t;

/**
 * @brief Elevators internal buttons identification codes.
 * This ids are used to turn off and on the buttons light.
 */
typedef enum
{
    INT_BTT_GROUND_FLOOR_CODE = 'a', //!< Ground floor internal button code
    INT_BTT_FLOOR_1_CODE = 'b',      //!< 1° floor internal button code
    INT_BTT_FLOOR_2_CODE = 'c',      //!< 2° floor internal button code
    INT_BTT_FLOOR_3_CODE = 'd',      //!< 3° floor internal button code
    INT_BTT_FLOOR_4_CODE = 'e',      //!< 4° floor internal button code
    INT_BTT_FLOOR_5_CODE = 'f',      //!< 5° floor internal button code
    INT_BTT_FLOOR_6_CODE = 'g',      //!< 6° floor internal button code
    INT_BTT_FLOOR_7_CODE = 'h',      //!< 7° floor internal button code
    INT_BTT_FLOOR_8_CODE = 'i',      //!< 8° floor internal button code
    INT_BTT_FLOOR_9_CODE = 'j',      //!< 9° floor internal button code
    INT_BTT_FLOOR_10_CODE = 'k',     //!< 10° floor internal button code
    INT_BTT_FLOOR_11_CODE = 'l',     //!< 11° floor internal button code
    INT_BTT_FLOOR_12_CODE = 'm',     //!< 12° floor internal button code
    INT_BTT_FLOOR_13_CODE = 'n',     //!< 13° floor internal button code
    INT_BTT_FLOOR_14_CODE = 'o',     //!< 14° floor internal button code
    INT_BTT_FLOOR_15_CODE = 'p',     //!< 15° floor internal button code
    INT_BTT_NONE = (uint8_t)0xFF
} int_button_id_code_t;
/**
 * @brief Simulator events identification codes.
 *
 */
typedef enum
{
    EVENT_ID_NONE = 0x00,   //!< None Event
    FLOOR_REACHED = '0',    //!< Signal recieved when the elevator reachs each floor
    DOORS_OPENED = 'A',     //!< Portas completamente abertas
    DOORS_CLOSED = 'F',     //!< Portas completamente fechadas
    EVENT_ID_UNKNOWN = 0xFF //!< Unknown Event
} event_id_t;

/**
 * @brief Elevators internal and external buttons identification codes.
 */
typedef enum
{
    BTT_GROUND_FLOOR = 0, //!< Ground floor internal button code
    BTT_FLOOR_1,          //!< 1° floor internal button code
    BTT_FLOOR_2,          //!< 2° floor internal button code
    BTT_FLOOR_3,          //!< 3° floor internal button code
    BTT_FLOOR_4,          //!< 4° floor internal button code
    BTT_FLOOR_5,          //!< 5° floor internal button code
    BTT_FLOOR_6,          //!< 6° floor internal button code
    BTT_FLOOR_7,          //!< 7° floor internal button code
    BTT_FLOOR_8,          //!< 8° floor internal button code
    BTT_FLOOR_9,          //!< 9° floor internal button code
    BTT_FLOOR_10,         //!< 10° floor internal button code
    BTT_FLOOR_11,         //!< 11° floor internal button code
    BTT_FLOOR_12,         //!< 12° floor internal button code
    BTT_FLOOR_13,         //!< 13° floor internal button code
    BTT_FLOOR_14,         //!< 14° floor internal button code
    BTT_FLOOR_15,         //!< 15° floor internal button code
    NUMBER_OF_BUTTONS,    //!< Number of buttons
    BTT_NONE = 0xFF       //!< None Button/floor
} button_id_t;

/**
 * @brief External buttons direction identification codes.
 * The external buttons are used to call the elevator to the
 * floor from it was pressed, indicating the direction of the
 * next floor.
 */
typedef enum
{
    MOVE_UP_BUTTON = 's',   //!< Move up extenal button code
    MOVE_DOWN_BUTTON = 'd', //!< Move down external button code
    DIRECTION_NONE = 0xFF   //!< None direction
} ext_button_direction_t;

/**
 * @brief Button event identification codes
 */
typedef enum
{
    INTERNAL_BUTTON = 'I', //!< Internal button identifier code
    EXTERNAL_BUTTON = 'E'  //!< External button identifier code
} button_id_code_t;

/**
 * @brief Elevator manager message structure.
 * It contains all the data that can be sent by
 * the elevator simulator.
 */
typedef struct
{
    msg_id_t msgId;                            //!< Message identification
    elevator_idx_t elevatorIdx;                //!< Elevator id - used to know to wich elevator the message was sent
    uint32_t position;                         //!< Position report value
    event_id_t eventId;                        //!< Event report identification
    uint8_t floorReached;                      //!< Floor reached signal
    button_id_t buttonFloor;                   //!< Internal or external button pressed floor
    ext_button_direction_t extButtonDirection; //!< External button direction
} elevator_mgr_msg_t;

/**
 * @brief Elevator transmission message, that its send to the elevator simulator
 */
typedef struct
{
    msg_id_t msgId;
    elevator_idx_t elevatorIdx;
    button_id_t buttonLightId;
} elevator_tx_msg_t;

typedef struct
{
    /* data */
} elevator_rx_msg_t;

#endif /* DEFS_H */

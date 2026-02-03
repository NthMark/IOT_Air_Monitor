/*******************************************************************************
 *
 * Copyright (c) 2025
 * All Rights Reserved
 *
 * Description: Simple MOSFET motor (fan) on/off control
 *
 * Author: Developer embedded team
 *
 ******************************************************************************/
#ifndef MOTOR_MOSFET_H
#define MOTOR_MOSFET_H
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdbool.h>
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void motor_mosfet_init(void);
void motor_mosfet_set(bool on);
void motor_mosfet_on(void);
void motor_mosfet_off(void);

#endif /* MOTOR_MOSFET_H */

/* END_FILE */

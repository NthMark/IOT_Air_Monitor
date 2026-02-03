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
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "motor_mosfet.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define MOSFET_GPIO           GPIOA
#define MOSFET_GPIO_RCC       RCC_APB2Periph_GPIOA
#define MOSFET_PIN            GPIO_Pin_6

#define MOSFET_ACTIVE_HIGH    1
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void motor_mosfet_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(MOSFET_GPIO_RCC, ENABLE);

    gpio.GPIO_Pin = MOSFET_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOSFET_GPIO, &gpio);
    GPIO_ResetBits(MOSFET_GPIO, MOSFET_PIN);

    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MOSFET_GPIO, &gpio);

    motor_mosfet_off();
}

void motor_mosfet_set(bool on)
{
#if MOSFET_ACTIVE_HIGH
    if (on) {
        GPIO_SetBits(MOSFET_GPIO, MOSFET_PIN);
    } else {
        GPIO_ResetBits(MOSFET_GPIO, MOSFET_PIN);
    }
#else
    if (on) {
        GPIO_ResetBits(MOSFET_GPIO, MOSFET_PIN);
    } else {
        GPIO_SetBits(MOSFET_GPIO, MOSFET_PIN);
    }
#endif
}

void motor_mosfet_on(void)
{
    motor_mosfet_set(true);
}

void motor_mosfet_off(void)
{
    motor_mosfet_set(false);
}

/* END_FILE */

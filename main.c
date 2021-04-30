/**
  ******************************************************************************
  * @file    main.c
  * @author  Johannes Schadow
  * @version V1.0
  * @date    15-April-2021
  ******************************************************************************
*/

//
//  Pin Reference
//  PB4,5 - Left Motor Controller IN1,2             - TIM3_CH1,2
//  PB0,1 - Right Motor Controller IN1,2            - TIM3_CH3,4

//  PA0,1 - Ultrasonic Proximity Sensor Echo,Echo   - TIM2_CH1,2
//  PA2,3 - Ultrasonic Proximity Sensor Echo,Echo   - TIM2_CH3,4
//  PA6   - Ultrasonic Proximity Sensor Trig        - TIM16_CH1

//  PC0   - Shovel IR Break Beam Data
//  PC1   - Container IR Break Beam Data

//  PC2   - Left Servo Data
//  PC3   - Right Servo Data

//  PA9   - FONA 808 RX
//  PA10  - FONA 808 TX
//  PA11  - FONA 808 RTS

//  PC4   - FONA 808 Reset
//  PB8   - FONA 808 NS
//  PC5   - FONA 808 PS
//  PC6   - FONA 808 Key
//  3.3V  - FONA 808 Vio
//  5V    - FONA 808 5V
//  GND   - FONA 808 GND

//  PB10  - bq Coulomb Counter SCL
//  PB11  - bq Coulomb Counter SDA
//  PA5   - bq Coulomb Counter ALERT1
//  PA4   - bq Coulomb Counter ALERT2

//  PC10  - Heartbeat Signal


#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>

#define SERVO_U 90
#define SERVO_N 0
#define SERVO_D -90

#define DROP_STOP 15
#define FRONT_STOP 40

enum gpio_mode {in=0, out=1, alt=2, analog=3};
volatile uint32_t distance_cm[4] = {0, 0, 0, 0};
volatile uint32_t last_capture[4] = {0, 0, 0, 0};
volatile uint32_t sig_polarity[4] = {0, 0, 0, 0};
volatile uint32_t prox_filter[4] = {0, 0, 0, 0};
volatile uint32_t ir_buffer[2] = {0, 0};
volatile uint32_t ticks = 0;
volatile uint32_t ticks_100ms = 0;
volatile uint32_t servo_tick = 999999;
int servo_angle = SERVO_N;
volatile int prev_IR_container = 0;

void servos_test() { //
    for(int i = 0; i < 5; i++) {
    GPIOC -> ODR |= GPIO_ODR_2;
    GPIOC -> ODR |= GPIO_ODR_3;
    delay_ms(150);
    GPIOC -> ODR &= ~GPIO_ODR_3;
    GPIOC -> ODR &= ~GPIO_ODR_2;
    delay_ms(1850);
    }

    delay_ms(200000);

    for(int i = 0; i < 5; i++) {
    GPIOC -> ODR |= GPIO_ODR_2;
    GPIOC -> ODR |= GPIO_ODR_3;
    delay_ms(110);
    GPIOC -> ODR &= ~GPIO_ODR_3;
    delay_ms(80);
    GPIOC -> ODR &= ~GPIO_ODR_2;
    delay_ms(1810);
    }

    delay_ms(200000);

    for(int i = 0; i < 5; i++) {
    GPIOC -> ODR |= GPIO_ODR_2;
    GPIOC -> ODR |= GPIO_ODR_3;
    delay_ms(110);
    GPIOC -> ODR &= ~GPIO_ODR_2;
    delay_ms(80);
    GPIOC -> ODR &= ~GPIO_ODR_3;
    delay_ms(1810);
    }

    delay_ms(200000);
}



void motors_stop() {
    while(TIM3 -> CCR4 > 0 || TIM3 -> CCR3 > 0 || TIM3 -> CCR2 > 0 || TIM3 -> CCR1 > 0) {
        if(TIM3 -> CCR4 > 0)    (TIM3 -> CCR4) -= 4;
        if(TIM3 -> CCR3 > 0)    (TIM3 -> CCR3) -= 4;
        if(TIM3 -> CCR2 > 0)    (TIM3 -> CCR2) -= 4;
        if(TIM3 -> CCR1 > 0)    (TIM3 -> CCR1) -= 4;
        delay_ms(1000);
    }
}

void motors_straight(int velocity) { // velocity = 0 is stopped; velocity = 128 is max forward; velocity = -128 is max backwards; velocity must be a factor of 4
    uint32_t speed = abs(velocity);
    motors_stop();
    delay_ms(50000);

    if(velocity >= 0) {
        while(TIM3 -> CCR1 < speed) {
            (TIM3 -> CCR1) += 4;
            (TIM3 -> CCR3) += 4;
            delay_ms(1000);
        }
    }
    else {
        while(TIM3 -> CCR2 < speed) {
            (TIM3 -> CCR2) += 4;
            (TIM3 -> CCR4) += 4;
            delay_ms(1000);
        }
    }
}

void motors_turn(int velocity) { // positive for right, negative for left?
    uint32_t speed = abs(velocity);
    motors_stop();
    delay_ms(50000);

    if(velocity >= 0) {
        while(TIM3 -> CCR1 < speed) {
            (TIM3 -> CCR1) += 4;
            (TIM3 -> CCR4) += 4;
            delay_ms(1000);
        }
    }
    else {
        while(TIM3 -> CCR2 < speed) {
            (TIM3 -> CCR2) += 4;
            (TIM3 -> CCR3) += 4;
            delay_ms(1000);
        }
    }
}

/*void set_speed(int velocity) { // velocity = 0 is stopped; velocity = 128 is max forward; velocity = -128 is max backwards;
    uint32_t speed = abs(velocity);
    //TIM3->CCR1 = speed;
    if(velocity >= 0) {
        if(TIM3 -> CCR2 > 0) {
            for(; TIM3 -> CCR2 > 0; (TIM3 -> CCR2) -= 4) {
                delay_ms(1000);
            }
            delay_ms(50000);
        }
        for(; TIM3 -> CCR1 < speed; (TIM3 -> CCR1) += 4) {
            delay_ms(1000);
        }
    }
    else {
        if(TIM3 -> CCR1 > 0) {
            for(; TIM3 -> CCR1 > 0; (TIM3 -> CCR1) -= 4) {
                delay_ms(1000);
            }
            delay_ms(50000);
        }
        for(; TIM3 -> CCR2 < speed; (TIM3 -> CCR2) += 4) {
            delay_ms(1000);
        }
    }
}*/

void SysTick_Handler(void) {
    ticks++;
    if(ticks % 300 == 0) {
        servo_tick = ticks + (servo_angle / 18 + 15) * 10;
        GPIOC -> ODR ^= GPIO_ODR_2 | GPIO_ODR_3;
    }
    if(ticks == servo_tick) {
        GPIOC -> ODR ^= GPIO_ODR_2 | GPIO_ODR_3;
    }
}

uint32_t get_ms() {
    return ticks;
}

void delay_ms(uint32_t delay) {
    uint32_t start = get_ms();
    uint32_t end = start + delay;
    if (start < end) {
        while(get_ms() >= start && get_ms() < end);
    }
    else {
        while(get_ms() >= start || get_ms() < end);
    }
}

void delay_sec(uint32_t delay) {
    delay_ms(100000 * delay);
}

//Only for A, B, C
void pin_config(char gpio, int pin_num, enum gpio_mode mode) {
    if(gpio == 'A' || gpio == 'a') {
        RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
        GPIOA -> MODER &= ~(ipow(2,2 * pin_num) + ipow(2,2 * pin_num + 1));
        GPIOA -> MODER |= ipow(2,2 * pin_num) * mode;
    }
    else if(gpio == 'B' || gpio == 'b') {
        RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
        GPIOB -> MODER &= ~(ipow(2,2 * pin_num) + ipow(2,2 * pin_num + 1));
        GPIOB -> MODER |= ipow(2,2 * pin_num) * mode;
    }
    else if(gpio == 'C' || gpio == 'c') {
        RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;
        GPIOC -> MODER &= ~(ipow(2,2 * pin_num) + ipow(2,2 * pin_num + 1));
        GPIOC -> MODER |= ipow(2,2 * pin_num) * mode;
    }
}

int ipow(int a, int b) {
    return (int)(pow(a,b)+0.5);
}

void pb01(){
    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB -> MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
    GPIOB -> MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0;
    GPIOB -> ODR |= GPIO_ODR_1 | GPIO_ODR_0;
}

void tim2_out() {
    pin_config('A', 3, alt);
    pin_config('A', 2, alt);
    pin_config('A', 1, alt);
    pin_config('A', 0, alt);
    GPIOA -> AFR[0] |= 2 << 12;
    GPIOA -> AFR[0] |= 2 << 8;
    GPIOA -> AFR[0] |= 2 << 4;
    GPIOA -> AFR[0] |= 2 << 0;
    RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2 -> PSC = 47;  //CNT increments every 1 us
    TIM2 -> ARR = 60000;  //Counts to 60 ms
    //TIM2 -> CCR4 = 10;    //High for 10 us
    //TIM2 -> CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; //set to PWM mode 1; high when CNT < CCR1
    TIM2 -> CCMR2 |= TIM_CCMR2_CC4S_0 | TIM_CCMR2_CC3S_0; //set to input mode
    TIM2 -> CCMR1 |= TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC1S_0; //set to input mode
    TIM2 -> CCER |= TIM_CCER_CC4E | TIM_CCER_CC4P | TIM_CCER_CC4NP
                  | TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NP
                  | TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP
                  | TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP;
    TIM2 -> DIER |= TIM_DIER_CC4IE | TIM_DIER_CC3IE | TIM_DIER_CC2IE | TIM_DIER_CC1IE;

    TIM2 -> CR1 = TIM_CR1_CEN;
    NVIC -> ISER[0] = 1<<TIM2_IRQn;
    NVIC_SetPriority(TIM2_IRQn,1);
}

void tim16_init() {
    pin_config('A', 6, alt);
    GPIOA -> AFR[0] |= 5 << 24;
    RCC -> APB2ENR |= RCC_APB2ENR_TIM16EN;
    TIM16 -> PSC = 47;
    TIM16 -> ARR = 60000;
    TIM16 -> CCR1 = 20;    //High for 10 us
    TIM16 -> CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; //set to PWM mode 1; high when CNT < CCR1
    TIM16 -> CCER |= TIM_CCER_CC1E;
    TIM16 -> BDTR |= TIM_BDTR_MOE;
    TIM16 -> CR1 = TIM_CR1_CEN;
}

void tim3_out() {
    pin_config('B', 4, alt);
    pin_config('B', 5, alt);
    pin_config('B', 0, alt);
    pin_config('B', 1, alt);
    GPIOB -> AFR[0] |= 1 << 20;
    GPIOB -> AFR[0] |= 1 << 16;
    GPIOB -> AFR[0] |= 1;
    GPIOB -> AFR[0] |= 1 << 4;
    RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3 -> PSC = 7499;
    TIM3 -> ARR = 127;   // 1 ms divided into 128 ticks
    TIM3 -> CCR1 = 0;    //High for 500 us
    TIM3 -> CCR2 = 0;    //High for 500 us
    TIM3 -> CCR3 = 0;    //High for 500 us
    TIM3 -> CCR4 = 0;    //High for 500 us
    TIM3 -> CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; //set to PWM mode 1; high when CNT < CCR1
    TIM3 -> CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; //set to PWM mode 1; high when CNT < CCR2
    TIM3 -> CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; //set to PWM mode 1; high when CNT < CCR3
    TIM3 -> CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; //set to PWM mode 1; high when CNT < CCR4
    TIM3 -> CCER |= TIM_CCER_CC4E | TIM_CCER_CC3E | TIM_CCER_CC2E | TIM_CCER_CC1E;

    TIM3 -> CR1 = TIM_CR1_CEN;
}

void TIM2_IRQHandler(void) {
    if(((TIM2 -> SR) & TIM_SR_CC1IF) != 0) {
        uint32_t curr_capture = TIM2 -> CCR1;
        sig_polarity[0] ^= 1;
        if(sig_polarity[0] == 0) {
            distance_cm[0] = (curr_capture - last_capture[0]) / 58; // in 1 us
            if(distance_cm[0] > DROP_STOP) {
                (prox_filter[0])++;
                if(prox_filter[0] >= 3) {
                    //GPIOC -> ODR |= GPIO_ODR_8;
                    avoid_obstacle();
                }
            }
            else {
                prox_filter[0] = 0;
                //GPIOC -> ODR &= ~GPIO_ODR_8;
            }
        }
        last_capture[0] = curr_capture;
    }
    if(((TIM2 -> SR) & TIM_SR_CC2IF) != 0) {
        uint32_t curr_capture = TIM2 -> CCR2;
        sig_polarity[1] ^= 1;
        if(sig_polarity[1] == 0) {
            distance_cm[1] = (curr_capture - last_capture[1]) / 58; // in 1 us
            if(distance_cm[1] > DROP_STOP) {
                (prox_filter[1])++;
                if(prox_filter[1] >= 3) {
                    //GPIOC -> ODR |= GPIO_ODR_8;
                    avoid_obstacle();
                }
            }
            else {
                prox_filter[1] = 0;
                //GPIOC -> ODR &= ~GPIO_ODR_8;
            }
        }
        last_capture[1] = curr_capture;
    }
    if(((TIM2 -> SR) & TIM_SR_CC3IF) != 0) {
        uint32_t curr_capture = TIM2 -> CCR3;
        sig_polarity[2] ^= 1;
        if(sig_polarity[2] == 0) {
            distance_cm[2] = (curr_capture - last_capture[2]) / 58; // in 1 us
            if(distance_cm[2] < FRONT_STOP) {
                (prox_filter[2])++;
                if(prox_filter[2] >= 3) {
                    //GPIOC -> ODR |= GPIO_ODR_8;
                    avoid_obstacle();
                }
            }
            else {
                prox_filter[2] = 0;
                //GPIOC -> ODR &= ~GPIO_ODR_8;
            }
        }
        last_capture[2] = curr_capture;
    }
    /*if(((TIM2 -> SR) & TIM_SR_CC4IF) != 0) {
        uint32_t curr_capture = TIM2 -> CCR4;
        sig_polarity[3] ^= 1;
        if(sig_polarity[3] == 0) {
            distance_cm[3] = (curr_capture - last_capture[3]) / 58; // in 1 us
            if(distance_cm[3] < FRONT_STOP) {
                (prox_filter[3])++;
                if(prox_filter[3] >= 3) {
                    GPIOC -> ODR |= GPIO_ODR_8;
                }
            }
            else {
                prox_filter[3] = 0;
                GPIOC -> ODR &= ~GPIO_ODR_8;
            }
        }
        last_capture[3] = curr_capture;
    }*/
}

void exti_init() {
    pin_config('C', 0, in);
    //pin_config('C', 1, in);
    //GPIOA -> PUPDR |= GPIO_PUPDR_PUPDR0_0;

    RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC; //| SYSCFG_EXTICR1_EXTI1_PC;
    EXTI -> IMR |= EXTI_IMR_MR0; //| EXTI_IMR_MR1;
    EXTI -> FTSR |= 1;
    //EXTI -> RTSR |= 1;
    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn,2);
}

void EXTI0_1_IRQHandler(void) {
    if((EXTI -> PR) & 1) {
        EXTI -> PR |= 1;
        motors_stop();
        delay_sec(1);
        servo_angle = 90;
        delay_ms(200000);
        servo_angle = -90;
        delay_sec(2);
        motors_straight(64);
    }
}

void avoid_obstacle() {
    motors_stop();
    delay_sec(1);

    motors_straight(-64);
    delay_sec(2);
    motors_stop();
    delay_sec(1);

    motors_turn(64);
    delay_sec(2);
    motors_stop();
    delay_sec(1);

    motors_straight(64);
    delay_sec(1);
    motors_stop();
    delay_sec(1);

    motors_turn(-64);
    delay_sec(2);
    motors_stop();
    delay_sec(1);

    motors_straight(64);
}

int main(void)
{
    //pb01();
    //pin_config('A', 0, out);
    //GPIOA -> ODR |= GPIO_ODR_0;
    tim2_out();
    pin_config('C', 2, out);
    pin_config('C', 3, out);
    GPIOC->ODR |= GPIO_ODR_3;   // On so inverted properly for servo 2
    pin_config('C', 8, out);
    pin_config('C', 9, out);
    pin_config('C', 10, out);
    pin_config('C', 1, in);
    //GPIOC -> PUPDR |= GPIO_PUPDR_PUPDR1_0;
    exti_init();
    tim16_init();
    tim3_out();
    SysTick_Config(480); //Event every 10 us
    NVIC_SetPriority(SysTick_IRQn,4);

    motors_straight(64);
    //GPIOC -> ODR ^= GPIO_ODR_10;

    for(;;) {

        //motors_straight(64);
        //delay_ms(500000);
        //motors_straight(-64);
        //delay_ms(500000);
        //motors_turn(64);
        //delay_ms(500000);
        //motors_turn(-64);
        //delay_ms(500000);

        //servo_angle = SERVO_N;
        //delay_ms(200000);
        //servo_angle = SERVO_D;
        delay_ms(300000);
        //servo_angle = SERVO_U;
        //servos_test();
        if(!((GPIOC -> IDR) & 2)) {
            if(prev_IR_container == 1) {
                GPIOC -> ODR |= GPIO_ODR_9;
            }
            prev_IR_container = 1;
        }
        else {
            prev_IR_container = 0;
            GPIOC -> ODR &= ~GPIO_ODR_9;
        }

        GPIOC -> ODR ^= GPIO_ODR_10;
        //GPIOC -> ODR &= ~GPIO_ODR_9;
    }
}

#include "stm32f4xx.h"
#include "lcd.h"

// DHT11 on PC3
#define DHT11_PORT GPIOC
#define DHT11_PIN  3
#define MOTOR_PORT GPIOB
#define MOTOR_PIN  0
#define TEMP_THRESHOLD 30


void DHT11_SetPinOutput(void) {
    DHT11_PORT->MODER &= ~(3U << (DHT11_PIN * 2));
    DHT11_PORT->MODER |=  (1U << (DHT11_PIN * 2));
}

//void DHT11_SetPinInput(void) {
//    DHT11_PORT->MODER &= ~(3U << (DHT11_PIN * 2));
//}

void DHT11_SetPinInput(void) {
    DHT11_PORT->MODER &= ~(3U << (DHT11_PIN * 2)); // Input
    DHT11_PORT->PUPDR &= ~(3U << (DHT11_PIN * 2)); // Clear
    DHT11_PORT->PUPDR |=  (1U << (DHT11_PIN * 2)); // Pull-up
}


void DHT11_Write(uint8_t val) {
    if (val)
        DHT11_PORT->BSRR = (1U << DHT11_PIN);
    else
        DHT11_PORT->BSRR = (1U << (DHT11_PIN + 16));
}

uint8_t DHT11_Read(void) {
    return (DHT11_PORT->IDR >> DHT11_PIN) & 1;
}

//void delay_us(uint32_t us) {
//    us *= 16;
//    while (us--) __NOP();
//}

void delay_us(uint32_t us) {
    SysTick->LOAD = (SystemCoreClock / 1000000) * us - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = 5; // Enable, use processor clock, no interrupt
    while ((SysTick->CTRL & 0x10000) == 0);
    SysTick->CTRL = 0;
}


void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 1000; i++) __NOP();
}

int dht11_read(uint8_t *temp) {
    uint8_t data[5] = {0};

    // Start signal
    DHT11_SetPinOutput();
    DHT11_Write(0);
    delay_ms(18);
    DHT11_Write(1);
    delay_us(30);
    DHT11_SetPinInput();

    // Wait for DHT11 response
    delay_us(40);
    if (DHT11_Read()) return -1;
    delay_us(80);
    if (!DHT11_Read()) return -1;
    delay_us(80);

    // Read 5 bytes (40 bits)
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 8; j++) {
            uint32_t timeout = 1000;
            while (!DHT11_Read() && timeout--) delay_us(1); // Wait for HIGH

            delay_us(40);
            if (DHT11_Read())
                data[i] |= (1 << (7 - j));

            timeout = 1000;
            while (DHT11_Read() && timeout--) delay_us(1); // Wait for LOW
        }
    }

    // Verify checksum
    if ((data[0] + data[1] + data[2] + data[3]) != data[4])
        return -1;

    *temp = data[2];  // Return only integer part of temperature
    return 0;
}


// Motor on PB0


void GPIO_init_motor(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    MOTOR_PORT->MODER &= ~(3U << (MOTOR_PIN * 2));
    MOTOR_PORT->MODER |=  (1U << (MOTOR_PIN * 2));
}

void motor_on(void) {
    MOTOR_PORT->BSRR = (1U << MOTOR_PIN);
}

void motor_off(void) {
    MOTOR_PORT->BSRR = (1U << (MOTOR_PIN + 16));
}

int main(void) {
    SystemCoreClockUpdate();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    GPIO_init_motor();
    lcd_gpio_init();
    lcd_init();

    uint32_t temp;

    while (1) {
//    	if (DHT11_Read()) {
//    		lcd(0x80, 0);  // Line 1
//    		lcd_string("Yes ");
//    	    // Toggle an LED to know something is being read
//    	}

    	if (dht11_read(&temp) == 0)
    	{

            lcd(0x80, 0);  // Line 1
            lcd_string("Temp: ");
            single_print(temp);
            lcd_string(" C");
            lcd(0xC0, 0);  // Line 2
            if (temp >= TEMP_THRESHOLD)
            {
                lcd_string("Mild Hot");
                motor_on();
            } else
            {
                lcd_string("Normal  ");
                motor_off();
            }

        } else {
            lcd(0x80, 0);
            lcd_string("DHT11 Error    ");
            lcd(0xC0, 0);
            lcd_string("Check Wiring   ");
            motor_off();
        }

        delay_ms(2000);
    }
}

//
//#include "stm32f4xx.h"
//#include "lcd.h"
//
//// DHT11 on PC3
//#define DHT11_PORT GPIOC
//#define DHT11_PIN  3
//
//void DHT11_GPIO_Init(void) {
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
//
//    // Initially configure as input (default state)
//    DHT11_PORT->MODER &= ~(3U << (DHT11_PIN * 2));
//    DHT11_PORT->PUPDR &= ~(3U << (DHT11_PIN * 2));
//    DHT11_PORT->PUPDR |= (2U << (DHT11_PIN * 2)); // Pull-down
//}
//
//void DHT11_SetPinOutput(void) {
//    DHT11_PORT->MODER &= ~(3U << (DHT11_PIN * 2));
//    DHT11_PORT->MODER |=  (1U << (DHT11_PIN * 2));
//}
//
//void DHT11_SetPinInput(void) {
//    DHT11_PORT->MODER &= ~(3U << (DHT11_PIN * 2));
//}
//
//void DHT11_Write(uint8_t val) {
//    if (val)
//        DHT11_PORT->BSRR = (1U << DHT11_PIN);
//    else
//        DHT11_PORT->BSRR = (1U << (DHT11_PIN + 16));
//}
//
//uint8_t DHT11_Read(void) {
//    return (DHT11_PORT->IDR >> DHT11_PIN) & 1;
//}
//
//// Better delay (still simple, but more predictable)
//void delay_us(uint32_t us) {
//    SysTick->LOAD = us * (SystemCoreClock / 1000000) - 1;
//    SysTick->VAL = 0;
//    SysTick->CTRL = 5; // Processor clock, no interrupt
//
//    while (!(SysTick->CTRL & 0x10000));
//    SysTick->CTRL = 0;
//}
//
//void delay_ms(uint32_t ms) {
//    while (ms--) delay_us(1000);
//}
//
//int dht11_read(uint8_t *temp) {
//    uint8_t data[5] = {0};
//
//    DHT11_SetPinOutput();
//    DHT11_Write(0);
//    delay_ms(20);  // 18–20ms
//    DHT11_Write(1);
//    delay_us(40);
//    DHT11_SetPinInput();
//
//    delay_us(40);
//    if (DHT11_Read()) return -1;
//    delay_us(80);
//    if (!DHT11_Read()) return -1;
//
//    delay_us(80);  // DHT response HIGH
//
//    for (int i = 0; i < 5; i++) {
//        for (int j = 0; j < 8; j++) {
//            while (!DHT11_Read());  // Wait for HIGH
//            delay_us(40);
//            if (DHT11_Read())
//                data[i] |= (1 << (7 - j));
//            while (DHT11_Read());  // Wait for LOW
//        }
//    }
//
//    if ((data[0] + data[1] + data[2] + data[3]) == data[4]) {
//        *temp = data[2];
//        return 0;
//    }
//
//    return -1;
//}
//
//// Motor on PB0
//#define MOTOR_PORT GPIOB
//#define MOTOR_PIN  0
//#define TEMP_THRESHOLD 30
//
//void GPIO_init_motor(void) {
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
//    MOTOR_PORT->MODER &= ~(3U << (MOTOR_PIN * 2));
//    MOTOR_PORT->MODER |=  (1U << (MOTOR_PIN * 2));
//}
//
//void motor_on(void) {
//    MOTOR_PORT->BSRR = (1U << MOTOR_PIN);
//}
//
//void motor_off(void) {
//    MOTOR_PORT->BSRR = (1U << (MOTOR_PIN + 16));
//}
//
//int main(void) {
//    SystemCoreClockUpdate();
//
//    lcd_gpio_init();
//        lcd_init();
//    DHT11_GPIO_Init();   // Important
//    GPIO_init_motor();
//
//
//    uint8_t temp;
//
//    while (1) {
//        if (dht11_read(&temp) == 0) {
//            lcd(0x80, 0);
//            lcd_string("Temp: ");
//            single_print(temp);
//            lcd_string(" C");
//
//            lcd(0xC0, 0);
//            if (temp >= TEMP_THRESHOLD) {
//                lcd_string("Mild Hot");
//                motor_on();
//            } else {
//                lcd_string("Normal  ");
//                motor_off();
//            }
//        } else {
//            lcd(0x80, 0);
//            lcd_string("DHT11 Error    ");
//            lcd(0xC0, 0);
//            lcd_string("Check Wiring   ");
//            motor_off();
//        }
//
//        delay_ms(2000);
//    }
//}


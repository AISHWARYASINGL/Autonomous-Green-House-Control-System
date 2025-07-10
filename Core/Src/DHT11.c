#include "stm32f4xx.h"
#include "lcd.h"

// DHT11 connected to PC3
#define DHT11_PORT GPIOC
#define DHT11_PIN  3

// Motor driver: PA4 (AIN1), PA5 (AIN2), PA10 (STBY)
#define MOTOR_PORT GPIOA
#define AIN1_PIN   4
#define AIN2_PIN   5
#define STBY_PIN   10

// Relay: PA1 (active LOW)
#define RELAY_PORT GPIOA
#define RELAY_PIN  3

// Thresholds
#define TEMP_THRESHOLD      25
#define HUMIDITY_THRESHOLD  60

// Function prototypes
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

// --- DHT11 FUNCTIONS ---
void DHT11_SetPinOutput(void)
{
    DHT11_PORT->MODER &= ~(3<< (DHT11_PIN * 2));    //DHT11_PORT->MODER &= ~(3 << (DHT11_PIN << 1));
    DHT11_PORT->MODER |=  (1 << (DHT11_PIN * 2));   //DHT11_PORT->MODER |=  (1 << (DHT11_PIN << 1));

}


void DHT11_SetPinInput(void)
{
    DHT11_PORT->MODER &= ~(3 << (DHT11_PIN * 2));   //Clears (resets) the two bits in the MODER register that control the pin mode
    DHT11_PORT->PUPDR &= ~(3 << (DHT11_PIN * 2));   //Clears the pull-up/pull-down configuration bits for the pin.
    DHT11_PORT->PUPDR |=  (1 << (DHT11_PIN * 2));  //Enables pull-up resistor for the pin.{(01)=as 1U}
}

void DHT11_Write(uint8_t val)
{
    if (val)
        DHT11_PORT->BSRR = (1 << DHT11_PIN);  //If value is non-zero.this line sets the pin HIGH
    else
        DHT11_PORT->BSRR = (1 << (DHT11_PIN + 16)); //Writing 1 to bit DHT11_PIN + 16 in BSRR sets that pin LOW.
}


uint8_t DHT11_Read(void)
{
    return (DHT11_PORT->IDR >> DHT11_PIN) & 1; //shifting the dht11 pin
}

void delay_us(uint32_t us)
{
    SysTick->LOAD = (SystemCoreClock / 1000000) * us - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = 5;
    while ((SysTick->CTRL & 0x10000) == 0);
    SysTick->CTRL = 0;
}


void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 1000; i++) __NOP();
}


int dht11_read(uint8_t *temp, uint8_t *humidity)
{
    uint8_t data[5] = {0};

    DHT11_SetPinOutput();
    DHT11_Write(0);
    delay_ms(18);
    DHT11_Write(1);
    delay_us(30);
    DHT11_SetPinInput();

    delay_us(40);
    if (DHT11_Read()) return -1;
    delay_us(80);
    if (!DHT11_Read()) return -1;
    delay_us(80);

    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            while (!DHT11_Read());
            delay_us(40);
            if (DHT11_Read()) data[i] |= (1 << (7 - j));
            while (DHT11_Read());
        }
    }

    if ((data[0] + data[1] + data[2] + data[3]) != data[4]) return -1;

    *humidity = data[0];
    *temp = data[2];
    return 0;
}

// --- GPIO INITIALIZATION ---
void GPIO_init_motor(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    MOTOR_PORT->MODER &= ~((3 << (AIN1_PIN * 2)) | (3 << (AIN2_PIN * 2)) | (3 << (STBY_PIN * 2)));
    MOTOR_PORT->MODER |=  ((1 << (AIN1_PIN * 2)) | (1 << (AIN2_PIN * 2)) | (1 << (STBY_PIN * 2)));
}

void GPIO_init_relay(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RELAY_PORT->MODER &= ~(3<< (RELAY_PIN * 2));
    RELAY_PORT->MODER |=  (1<< (RELAY_PIN * 2));
}


// --- CONTROL FUNCTIONS ---
void motor_on(void)
{
    MOTOR_PORT->BSRR = (1 << AIN1_PIN);
    MOTOR_PORT->BSRR = (1 << (AIN2_PIN + 16));
    MOTOR_PORT->BSRR = (1<< STBY_PIN);
}

void motor_off(void)
{
    MOTOR_PORT->BSRR = (1 << (AIN1_PIN + 16));
    MOTOR_PORT->BSRR = (1 << (AIN2_PIN + 16));
    MOTOR_PORT->BSRR = (1 << (STBY_PIN + 16));
}

// Active LOW relay
//void relay_on(void) {
//    RELAY_PORT->BSRR = (1U << (RELAY_PIN + 16));  // Drive PA1 LOW
//}
//
//void relay_off(void) {
//    RELAY_PORT->BSRR = (1U << RELAY_PIN);         // Drive PA1 HIGH
//}
void relay_on(void)
{
    RELAY_PORT->BSRR = (1 << (RELAY_PIN + 16));  // Pull PA2 LOW → Relay ON
}

void relay_off(void)
{
    RELAY_PORT->BSRR = (1 << RELAY_PIN);         // Pull PA2 HIGH → Relay OFF
}



// --- MAIN FUNCTION ---
int main(void) {
    SystemCoreClockUpdate();
    motor_on();


   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  // For DHT11 (PC3)
    GPIO_init_motor();
    GPIO_init_relay();
    lcd_gpio_init();
    lcd_init();

    uint8_t temp, humidity;
    relay_on();
    GPIOA->MODER |= (1<<6); //
    GPIOA->MODER &=~ (1<<7);

    GPIOA->ODR |= (1<<3);
    delay_ms(4000);
    while (1)
    {
        if (dht11_read(&temp, &humidity) == 0)
        {
            lcd(0x80, 0);  // Line 1
            lcd_string("T:");
            single_print(temp);
            lcd_string("C H:");
            single_print(humidity);
            lcd_string("%");

            lcd(0xC0, 0);  // Line 2

            // Temperature-based fan control
            if (temp >= TEMP_THRESHOLD)
            {
                lcd_string("Mild Hot");
                motor_on();
            }
            else
            {
                lcd_string("Normal");
                motor_off();
            }

            // Humidity-based relay control
            if (humidity >= HUMIDITY_THRESHOLD)
            {
                relay_on();
            }
            else
            {
                relay_off();
            }

        }
        else
        {
            lcd(0x80, 0);
            lcd_string("DHT11 Error    ");
            lcd(0xC0,0);
            lcd(0x01,0);
            lcd_string("Check Wiring   ");
            motor_off();
            relay_off();
        }

        delay_ms(2000);
    }
}

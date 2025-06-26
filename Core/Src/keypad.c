/*
 * Keypad Security
 */

#include <stdint.h>
#include <string.h>
#include "STM32F4xx.h"
#include "lcd.h"

// === TM1637 Pins ===
#define TM1_DIO_PIN 7  // PC7
#define TM1_CLK_PIN 6  // PC6
#define TM1_PORT GPIOC

#define PASSKEY_LENGTH 4
char passkey[PASSKEY_LENGTH + 1];
char input[PASSKEY_LENGTH + 1];

// Keypad definitions
char keypad_map[4][4] = {{'*', '0', '#', 'D'},
                         {'7', '8', '9', 'C'},
                         {'4', '5', '6', 'B'},
                         {'1', '2', '3', 'A'}};

#define ROW_PORT GPIOB
#define COL_PORTB GPIOB
#define COL_PORTC GPIOC

#define R1_PIN 3
#define R2_PIN 4
#define R3_PIN 5
#define R4_PIN 6
#define C1_PIN 7  // PB7
#define C2_PIN 0  // PC0
#define C3_PIN 1  // PC1
#define C4_PIN 2  // PC2


// === Clock Setup ===
void SystemClock_Config(void) {
  RCC->CR |= RCC_CR_HSEON;
  while (!(RCC->CR & RCC_CR_HSERDY));
  RCC->PLLCFGR = (8U) | (336U << 6) | (7U << 24) | RCC_PLLCFGR_PLLSRC_HSE;
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY));
  FLASH->ACR |= FLASH_ACR_LATENCY_5WS;
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
  SystemCoreClockUpdate();
}

void delay_us(uint32_t us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < ticks);
}

// === Segment Map ===
uint8_t segment_map[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66,
                           0x6D, 0x7D, 0x07, 0x7F, 0x6F};

// === TM1637 Display ===
void TM_init(GPIO_TypeDef* port, uint8_t clk, uint8_t dio) {
  port->MODER &= ~((3 << (clk * 2)) | (3 << (dio * 2)));
  port->MODER |= ((1 << (clk * 2)) | (1 << (dio * 2)));
}

void TM_all_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  TM_init(TM1_PORT, TM1_CLK_PIN, TM1_DIO_PIN);
}

void TM_start(GPIO_TypeDef* port, uint8_t clk, uint8_t dio) {
  port->ODR |= (1 << clk) | (1 << dio);
  delay_us(2);
  port->ODR &= ~(1 << dio);
  delay_us(2);
}

void TM_stop(GPIO_TypeDef* port, uint8_t clk, uint8_t dio) {
  port->ODR &= ~(1 << clk);
  port->ODR &= ~(1 << dio);
  delay_us(2);
  port->ODR |= (1 << clk);
  delay_us(2);
  port->ODR |= (1 << dio);
}

void TM_write_byte(GPIO_TypeDef* port, uint8_t clk, uint8_t dio, uint8_t b) {
  for (int i = 0; i < 8; i++) {
    port->ODR &= ~(1 << clk);
    if (b & 0x01)
      port->ODR |= (1 << dio);
    else
      port->ODR &= ~(1 << dio);
    delay_us(2);
    port->ODR |= (1 << clk);
    delay_us(2);
    b >>= 1;
  }
  port->ODR &= ~(1 << clk);
  port->ODR |= (1 << dio);
  delay_us(2);
  port->ODR |= (1 << clk);
  delay_us(2);
  port->ODR &= ~(1 << clk);
}

void TM_display_number(GPIO_TypeDef* port, uint8_t clk, uint8_t dio,
                       uint16_t num) {
  uint8_t d[4] = {segment_map[(num / 1000) % 10], segment_map[(num / 100) % 10],
                  segment_map[(num / 10) % 10], segment_map[num % 10]};

  TM_start(port, clk, dio);
  TM_write_byte(port, clk, dio, 0x40);
  TM_stop(port, clk, dio);

  TM_start(port, clk, dio);
  TM_write_byte(port, clk, dio, 0xC0);
  for (int i = 0; i < 4; i++) TM_write_byte(port, clk, dio, d[i]);
  TM_stop(port, clk, dio);

  TM_start(port, clk, dio);
  TM_write_byte(port, clk, dio, 0x88 | 0x07);
  TM_stop(port, clk, dio);
}

// === Delay Function ===
void delay(uint32_t t) {
  for (uint32_t i = 0; i < t * 4000; i++) __NOP();
}


// === Keypad Init and Scan ===
void keypad_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

  for (int pin = R1_PIN; pin <= R4_PIN; pin++) {
    ROW_PORT->MODER &= ~(0x3 << (pin * 2));
    ROW_PORT->PUPDR |= (0x1 << (pin * 2));  // Pull-up
  }

  COL_PORTB->MODER |= (0x1 << (C1_PIN * 2));
  COL_PORTC->MODER |=
      (0x1 << (C2_PIN * 2)) | (0x1 << (C3_PIN * 2)) | (0x1 << (C4_PIN * 2));
}

char keypad_scan(void) {
  uint8_t col_pins[4] = {C1_PIN, C2_PIN, C3_PIN, C4_PIN};
  GPIO_TypeDef* col_ports[4] = {COL_PORTB, COL_PORTC, COL_PORTC, COL_PORTC};
  uint8_t row_pins[4] = {R4_PIN, R3_PIN, R2_PIN, R1_PIN};

  for (int c = 0; c < 4; c++) {
    COL_PORTB->ODR |= (1 << C1_PIN);
    COL_PORTC->ODR |= (1 << C2_PIN) | (1 << C3_PIN) | (1 << C4_PIN);
    col_ports[c]->ODR &= ~(1 << col_pins[c]);
    delay(1);

    for (int r = 0; r < 4; r++) {
      if (!(ROW_PORT->IDR & (1 << row_pins[r]))) {
        while (!(ROW_PORT->IDR & (1 << row_pins[r])));
        return keypad_map[r][c];
      }
    }
  }
  return '\0';
}

uint16_t get_adc_noise_seed(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;         // Enable ADC1
    ADC1->SQR3 = 10;                            // Channel 10 (PC0)
    ADC1->CR2 |= ADC_CR2_ADON;                  // Power on ADC
    for (volatile int i = 0; i < 1000; i++);    // Short delay
    ADC1->CR2 |= ADC_CR2_SWSTART;              // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC));          // Wait for completion
    return ADC1->DR;                            // Return noisy value
}


uint16_t pseudo_rand(void) {
    static uint32_t seed = 1234;
    seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;
    return seed % 10000;
}

uint32_t reverse_number(uint32_t num) {
    uint32_t rev = 0;
    while (num > 0) {
        rev = rev * 10 + (num % 10);
        num = num / 10;
    }
    return rev;
}


// === Main ===
int main(void) {
  int idx;

  SysTick->LOAD = 0xFFFFFF;
  SysTick->CTRL = 5;

  SystemClock_Config();
  TM_all_init();
  lcd_gpio_init();
  lcd_init();
  keypad_init();

  uint16_t seed = get_adc_noise_seed();
  srand(seed);
  uint16_t a = rand() % 10000;
  TM_display_number(TM1_PORT, TM1_CLK_PIN, TM1_DIO_PIN, a);

  uint16_t b = reverse_number(a);
  char b_str[PASSKEY_LENGTH + 1];
  sprintf(b_str, "%04d", b);  // Convert reversed number to 4-digit string

  lcd_string("Welcome");
  delay(1000);
  lcd(0x01, 0);


  while (1) {
    lcd_string("Press * to Start");
    while (keypad_scan() != '*');

    lcd(0x01, 0);
    lcd_string("Enter Passkey:");
    // delay(6000);
    lcd(0xC0, 0);

    idx = 0;
    memset(input, 0, sizeof(input));

    while (1) {
      char key = keypad_scan();

      if (key >= '0' && key <= '9' && idx < PASSKEY_LENGTH) {
        input[idx++] = key;
        lcd(key, 1);
      } else if (key == '#') {
        input[idx] = '\0';
        lcd(0x01, 0);

        if (strcmp(input, b_str) == 0) {
          lcd_string("Access Granted");
          delay(1000);
          lcd(0x01, 0);
          char mode = '\0';

          break;
        } else {
          lcd_string("Wrong Passkey");
          lcd(0x01, 0);
          idx = 0;
          lcd_string("Retry Passkey:");
          lcd(0xC0, 0);
        }
      }
    }

    while (1) {
      lcd(0x01, 0);
      lcd_string("System Live ...");
      delay(3000);
    }
  }
}

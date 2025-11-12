#include "stm32f4xx.h"
#include <stdint.h>

/* ===================== Config macros ===================== */
// Polarity / board specifics
#define FLAME_ACTIVE_LOW      1    // most flame boards pull LOW on flame
#define BUZZER_ACTIVE_HIGH    1    // set 0 if your buzzer board is active-low
#define PC13_ACTIVE_LOW_LED   1    // BlackPill onboard LED is active-low

// Gas threshold (12-bit ADC)
#define GAS_THRESH            1500

// MPU6050
#define MPU6050_ADDR          0x68
#define MPU6050_REG_PWR1      0x6B
#define MPU6050_REG_SMPLRT    0x19
#define MPU6050_REG_CONFIG    0x1A
#define MPU6050_REG_GYROCFG   0x1B
#define MPU6050_REG_ACCELCFG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

// Accelerometer scale: �2 g => 1 g � 16384 LSB
#define ONE_G_RAW             16384

// Shake sensitivity (mg). Raise for less sensitive, lower for more.
// We'll convert to raw later, but we compare in mg for readability.
#define SHAKE_THRESH_MG       300   // ~0.30 g
#define QUIET_DUTY_PERCENT    5     // LED brightness when idle
#define SHAKE_DUTY_PERCENT    90    // LED brightness when shaking

/* ===================== Prototypes ===================== */
void GPIO_Init(void);               // PC13 LED
void FlameSensor_GPIO_Init(void);   // PA1 input (PU)
void Buzzer_GPIO_Init(void);        // PB0 output
void VibrationLED_PWM_Init(void);   // PB1 TIM3-CH4 PWM
void ADC_Init(void);                // MQ-2 on PA0
void USART1_Init(void);             // HC-05 PA9/PA10
void I2C1_Init(void);               // PB8/PB9

uint8_t MPU6050_Init(void);
uint8_t MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az);

uint8_t  FlameSensor_Read(void);
uint16_t GasSensor_Read(void);

void USART1_SendChar(char c);
void USART1_SendString(char* s);
void USART1_SendUInt(uint32_t v);

void VibrationLED_SetDuty(uint8_t duty_percent);

/* ===================== Small helpers ===================== */
static inline void LED_PC13_ON(void){
#if PC13_ACTIVE_LOW_LED
    GPIOC->ODR &= ~(1<<13);
#else
    GPIOC->ODR |=  (1<<13);
#endif
}
static inline void LED_PC13_OFF(void){
#if PC13_ACTIVE_LOW_LED
    GPIOC->ODR |=  (1<<13);//pc13
#else
    GPIOC->ODR &= ~(1<<13);
#endif
}
static inline void BUZZER_ON(void){
#if BUZZER_ACTIVE_HIGH
    GPIOB->ODR |=  (1<<0);//pb0
#else
    GPIOB->ODR &= ~(1<<0);
#endif
}
static inline void BUZZER_OFF(void){
#if BUZZER_ACTIVE_HIGH
    GPIOB->ODR &= ~(1<<0);
#else
    GPIOB->ODR |=  (1<<0);
#endif
}

/* ===== integer sqrt for 64-bit (for |accel| magnitude) ===== */
static uint32_t isqrt64(uint64_t x){
    uint64_t r = 0;
    uint64_t b = (uint64_t)1 << 62;           // highest even power
    while (b > x) b >>= 2;
    while (b){
        if (x >= r + b){ x -= r + b; r = (r >> 1) + b; }
        else            r >>= 1;
        b >>= 2;
    }
    return (uint32_t)r;
}

/* ===================== Main ===================== */
int main(void)
{
    GPIO_Init();               // PC13
    FlameSensor_GPIO_Init();   // PA1
    Buzzer_GPIO_Init();        // PB0
    VibrationLED_PWM_Init();   // PB1 -> PWM on TIM3-CH4
    ADC_Init();                // MQ-2 PA0
    USART1_Init();             // HC-05
    I2C1_Init();               // I2C1 PB8/PB9

    LED_PC13_OFF();
    BUZZER_OFF();
    VibrationLED_SetDuty(QUIET_DUTY_PERCENT);

    if (!MPU6050_Init()) USART1_SendString("MPU6050 init FAILED\r\n");
    else                 USART1_SendString("MPU6050 init OK\r\n");

    // simple debounce / state for shake alerts
    uint8_t shake_state = 0;   // 0=idle, 1=shaking

    while(1)
    {
        /* --------- Flame --------- */
        uint8_t flame = FlameSensor_Read();
        if (flame) LED_PC13_ON();
        else       LED_PC13_OFF();

        /* --------- Gas (ADC) --------- */
        uint16_t gasValue = GasSensor_Read();
        if (gasValue > GAS_THRESH) BUZZER_ON();
        else                       BUZZER_OFF();

        /* --------- Vibration (MPU6050) --------- */
        int16_t ax, ay, az;
        uint8_t mpu_ok = MPU6050_ReadAccel(&ax, &ay, &az);
        uint32_t dev_mg = 0;

        if (mpu_ok){
            // magnitude = sqrt(ax^2 + ay^2 + az^2)
            uint64_t mag2 = (uint64_t)ax*ax + (uint64_t)ay*ay + (uint64_t)az*az;
            uint32_t mag  = isqrt64(mag2); // raw LSB
            int32_t dev = (int32_t)mag - ONE_G_RAW; // deviation from ~1g
            if (dev < 0) dev = -dev;
            // convert to mg: dev_mg = dev * 1000 / 16384
            dev_mg = (uint32_t)(( (uint64_t)dev * 1000 ) / ONE_G_RAW);

            // LED brightness jump
            if (dev_mg >= SHAKE_THRESH_MG) {
                VibrationLED_SetDuty(SHAKE_DUTY_PERCENT);   // bright
            } else {
                VibrationLED_SetDuty(QUIET_DUTY_PERCENT);   // dim
            }

            // Debounced alerting on state change
            if (!shake_state && dev_mg >= SHAKE_THRESH_MG){
                shake_state = 1;
                USART1_SendString("ALERT! EARTHQUAKE detected, dev=");
                USART1_SendUInt(dev_mg);
                USART1_SendString(" mg\r\n");
            } else if (shake_state && dev_mg < (SHAKE_THRESH_MG/2)){
                // add a little hysteresis to drop back to idle
                shake_state = 0;
                USART1_SendString("Earthquake cleared\r\n");
            }
        } else {
            // if read fails, keep LED dim and state idle
            VibrationLED_SetDuty(QUIET_DUTY_PERCENT);
            if (shake_state){
                shake_state = 0;
                USART1_SendString("Earthquake cleared\r\n");
            }
        }

        /* --------- Bluetooth messages (flame/gas only) --------- */
        if (flame && gasValue > GAS_THRESH){
            USART1_SendString("ALERT! Flame + Gas\r\n");
        } else if (flame){
            USART1_SendString("ALERT! Flame Detected\r\n");
        } else if (gasValue > GAS_THRESH){
            USART1_SendString("ALERT! Gas Detected, ADC=");
            USART1_SendUInt(gasValue);
            USART1_SendString("\r\n");
        } else if (!shake_state){
            // Only spam SAFE when nothing is going on
            USART1_SendString("Status: SAFE\r\n");
        }

        for(volatile int i=0;i<600000;i++); // small delay
    }
}

/* ===================== GPIO ===================== */
void GPIO_Init(void)
{
    RCC->AHB1ENR |= (1<<2);       // GPIOC
    GPIOC->MODER &= ~(0x3<<26);
    GPIOC->MODER |=  (0x1<<26);   // PC13 output
}

void FlameSensor_GPIO_Init(void)
{
    RCC->AHB1ENR |= (1<<0);       // GPIOA
    GPIOA->MODER &= ~(0x3<<2);    // PA1 input
    GPIOA->PUPDR &= ~(0x3<<2);
    GPIOA->PUPDR |=  (0x1<<2);    // pull-up
}

void Buzzer_GPIO_Init(void)
{
    RCC->AHB1ENR |= (1<<1);       // GPIOB
    GPIOB->MODER &= ~(0x3<<0);
    GPIOB->MODER |=  (0x1<<0);    // PB0 output
    BUZZER_OFF();
}

/* ===================== PWM on PB1 (TIM3 CH4) ===================== */
void VibrationLED_PWM_Init(void)
{
    // Clocks
    RCC->AHB1ENR |= (1<<1);    // GPIOB
    RCC->APB1ENR |= (1<<1);    // TIM3

    // PB1 -> AF2 (TIM3_CH4), push-pull, high speed
    GPIOB->MODER &= ~(0x3<<(1*2));
    GPIOB->MODER |=  (0x2<<(1*2));      // AF
    GPIOB->OTYPER &= ~(1<<1);
    GPIOB->OSPEEDR |=  (0x3<<(1*2));
    GPIOB->PUPDR   &= ~(0x3<<(1*2));
    GPIOB->AFR[0]  &= ~(0xF<<(1*4));
    GPIOB->AFR[0]  |=  (0x2<<(1*4));    // AF2 for TIM3

    // Assume APB1 timer clock = 16 MHz (HSI, no PLL).
    // Target PWM ~ 1 kHz: PSC=15, ARR=999 -> f = 16MHz/(16*1000)=1kHz
    TIM3->PSC = 15;
    TIM3->ARR = 999;

    // CH4 PWM mode 1, enable preload
    TIM3->CCMR2 &= ~(0xFF<<8);
    TIM3->CCMR2 |=  (0x6<<12);    // OC4M: PWM mode 1
    TIM3->CCMR2 |=  (1<<11);      // OC4PE
    TIM3->CCER  &= ~(1<<13);      // active high
    TIM3->CCER  |=  (1<<12);      // CC4E enable

    TIM3->CCR4 = (QUIET_DUTY_PERCENT * (TIM3->ARR+1))/100;  // start dim
    TIM3->CR1  |=  (1<<7);        // ARPE
    TIM3->EGR  |=  (1<<0);        // UG
    TIM3->CR1  |=  (1<<0);        // CEN
}

void VibrationLED_SetDuty(uint8_t duty_percent)
{
    if (duty_percent > 100) duty_percent = 100;
    uint32_t arrp1 = (TIM3->ARR + 1);
    TIM3->CCR4 = (arrp1 * duty_percent)/100;
}

/* ===================== ADC (MQ-2 on PA0) ===================== */
void ADC_Init(void)
{
    RCC->APB2ENR |= (1<<8);        // ADC1
    RCC->AHB1ENR |= (1<<0);        // GPIOA
    GPIOA->MODER &= ~(0x3<<0);
    GPIOA->MODER |=  (0x3<<0);     // PA0 analog
    ADC1->SQR3 = 0;                // ch0
    ADC1->CR2 |= (1<<0);           // ADON
}

uint16_t GasSensor_Read(void)
{
    ADC1->CR2 |= (1<<30);          // SWSTART
    while(!(ADC1->SR & (1<<1)));   // EOC
    return ADC1->DR;
}

/* ===================== Flame read ===================== */
uint8_t FlameSensor_Read(void)
{
    uint8_t raw = (GPIOA->IDR & (1<<1)) ? 1 : 0;
#if FLAME_ACTIVE_LOW
    return (raw==0);   // LOW -> flame
#else
    return (raw==1);   // HIGH -> flame
#endif
}

/* ===================== USART1 (HC-05) ===================== */
void USART1_Init(void)
{
    RCC->APB2ENR |= (1<<4);    // USART1
    RCC->AHB1ENR |= (1<<0);    // GPIOA

    // PA9 TX, PA10 RX, AF7
    GPIOA->MODER &= ~((0x3<<18)|(0x3<<20));
    GPIOA->MODER |=  ((0x2<<18)|(0x2<<20));
    GPIOA->AFR[1] &= ~((0xF<<4)|(0xF<<8));
    GPIOA->AFR[1] |=  ((0x7<<4)|(0x7<<8));

    USART1->BRR = 0x0683;      // 9600 @ 16 MHz
    USART1->CR1 |= (1<<3)|(1<<2)|(1<<13); // TE, RE, UE
}

void USART1_SendChar(char c)
{
    while(!(USART1->SR & (1<<7)));
    USART1->DR = c;
}
void USART1_SendString(char* s)
{
    while(*s) USART1_SendChar(*s++);
}
void USART1_SendUInt(uint32_t v)
{
    char buf[12]; int i=0;
    if (v==0){ USART1_SendChar('0'); return; }
    while(v){ buf[i++] = '0' + (v%10); v/=10; }
    while(i--) USART1_SendChar(buf[i]);
}

/* ===================== I2C1 low-level ===================== */
void I2C1_Init(void)
{
    RCC->APB1ENR |= (1<<21);   // I2C1
    RCC->AHB1ENR |= (1<<1);    // GPIOB

    // PB8 SCL, PB9 SDA, AF4, open-drain, PU
    GPIOB->MODER &= ~((0x3<<(8*2))|(0x3<<(9*2)));
    GPIOB->MODER |=  ((0x2<<(8*2))|(0x2<<(9*2)));
    GPIOB->OTYPER |=  (1<<8)|(1<<9);
    GPIOB->OSPEEDR |= (0x3<<(8*2))|(0x3<<(9*2));
    GPIOB->PUPDR &= ~((0x3<<(8*2))|(0x3<<(9*2)));
    GPIOB->PUPDR |=  ((0x1<<(8*2))|(0x1<<(9*2)));
    GPIOB->AFR[1] &= ~((0xF<<0)|(0xF<<4));
    GPIOB->AFR[1] |=  ((0x4<<0)|(0x4<<4));

    // 100 kHz @ PCLK1=16 MHz
    I2C1->CR1 = (1<<15); I2C1->CR1 = 0;
    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= (1<<0);
}

static void I2C1_Start(void){ I2C1->CR1 |= (1<<8); while(!(I2C1->SR1 & (1<<0))); }
static void I2C1_Stop(void){ I2C1->CR1 |= (1<<9); }
static void I2C1_SendAddr(uint8_t rw){ I2C1->DR = rw; while(!(I2C1->SR1 & (1<<1))); (void)I2C1->SR1; (void)I2C1->SR2; }
static void I2C1_WriteByte(uint8_t d){ while(!(I2C1->SR1 & (1<<7))); I2C1->DR = d; while(!(I2C1->SR1 & (1<<2))); }
static uint8_t I2C1_ReadByte_Ack(void){ I2C1->CR1 |= (1<<10); while(!(I2C1->SR1 & (1<<6))); return I2C1->DR; }
static uint8_t I2C1_ReadByte_Nack(void){ I2C1->CR1 &= ~(1<<10); while(!(I2C1->SR1 & (1<<6))); return I2C1->DR; }
static uint8_t I2C1_WriteReg(uint8_t addr, uint8_t reg, uint8_t val){
    I2C1_Start(); I2C1_SendAddr((addr<<1)|0); I2C1_WriteByte(reg); I2C1_WriteByte(val); I2C1_Stop(); return 1;
}
static uint8_t I2C1_ReadRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len){
    if(!len) return 0;
    I2C1_Start(); I2C1_SendAddr((addr<<1)|0); I2C1_WriteByte(reg);
    I2C1_Start(); I2C1_SendAddr((addr<<1)|1);
    for(uint8_t i=0;i<len;i++){
        if(i<(len-1)) buf[i]=I2C1_ReadByte_Ack();
        else{ buf[i]=I2C1_ReadByte_Nack(); I2C1_Stop(); }
    }
    return 1;
}

/* ===================== MPU6050 ===================== */
uint8_t MPU6050_Init(void)
{
    if (!I2C1_WriteReg(MPU6050_ADDR, MPU6050_REG_PWR1, 0x00)) return 0; // wake
    if (!I2C1_WriteReg(MPU6050_ADDR, MPU6050_REG_SMPLRT, 0x09)) return 0; // ~100Hz
    if (!I2C1_WriteReg(MPU6050_ADDR, MPU6050_REG_CONFIG, 0x03)) return 0; // DLPF 42Hz
    if (!I2C1_WriteReg(MPU6050_ADDR, MPU6050_REG_GYROCFG, 0x00)) return 0; // �250 dps
    if (!I2C1_WriteReg(MPU6050_ADDR, MPU6050_REG_ACCELCFG, 0x00)) return 0; // �2g
    return 1;
}
uint8_t MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t b[6];
    if (!I2C1_ReadRegs(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, b, 6)) return 0;
    *ax = (int16_t)((b[0]<<8)|b[1]);
    *ay = (int16_t)((b[2]<<8)|b[3]);
    *az = (int16_t)((b[4]<<8)|b[5]);
    return 1;
}

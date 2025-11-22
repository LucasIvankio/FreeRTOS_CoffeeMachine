/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "stm32l475e_iot01_tsensor.h"

#include "st7735.h"
#include "fonts.h"
#include "display_img.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEBOUNCE_MS 200

/* Brew times in ms per cup size (example values) */
#define BREW_TIME_CUP1_MS   6000
#define BREW_TIME_CUP2_MS   8000
#define BREW_TIME_CUP3_MS   10000
#define BREW_TIME_CUP4_MS   12000

/* Auto-off timeout (ms) after brew finished & idle */
#define AUTO_OFF_TIMEOUT_MS   10000UL   // 60 seconds

/* Water level thresholds (0–100%) */
#define WATER_LEVEL_MIN_PERCENT   25    // below this => Add water

/* LED blink period for the active cup (ms) */
#define CUP_LED_BLINK_PERIOD_MS  300

/* Colors */
#define COLOR_BG          ST7735_BLACK
#define COLOR_TEXT        ST7735_WHITE
#define COLOR_HILIGHT     ST7735_YELLOW
#define COLOR_ERROR       ST7735_RED
#define COLOR_WATER_OK    ST7735_GREEN
#define COLOR_WATER_LOW   ST7735_MAGENTA
#define COLOR_COFFEE      ST7735_COLOR565(139, 69, 19)
#define COLOR_FRAME       ST7735_WHITE

#define CUP_X   90
#define CUP_Y   40
#define CUP_W   30
#define CUP_H   80

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId CoffStateMachinHandle;
osThreadId CoffSensorTaskHandle;
osThreadId CoffConsoleTaskHandle;
osThreadId CoffDisplayTaskHandle;
osMessageQId xEventQueueHandle;
osMessageQId xConsoleQueueHandle;

/* USER CODE BEGIN PV */

static volatile TickType_t gLastExtiTick[6]; // map: [0]=PC13, [1]=PC0, [2]=PC1,[3]=PC2,[4]=PC3,[5]=PC4

typedef enum
{
    COFFEE_STATE_OFF = 0,
    COFFEE_STATE_IDLE,
    COFFEE_STATE_BREWING,
	COFFEE_STATE_DONE,
    COFFEE_STATE_ERROR_NO_WATER
} CoffeeState_t;

typedef enum
{
    EVT_NONE = 0,
    EVT_POWER_TOGGLE,       // Blue button (BUTTON_EXTI13)
    EVT_STRONG_TOGGLE,      // Strong brew button
    EVT_CUP_1,
    EVT_CUP_2,
    EVT_CUP_3,
    EVT_CUP_4
} CoffeeEvent_t;

typedef enum
{
    CUP_NONE = 0,
    CUP_1,
    CUP_2,
    CUP_3,
    CUP_4
} CupSize_t;

typedef struct
{
    char text[100];
} ConsoleMessage_t;

/* State variables */
volatile CoffeeState_t gCoffeeState = COFFEE_STATE_OFF;
volatile uint8_t gStrongBrew = 0;        // 0 = normal, 1 = strong
volatile uint8_t gAutoOffFlag = 0;       // controls LED_AUTO_OFF
volatile uint32_t gLastActivityTick = 0; // for auto-off countdown
volatile CupSize_t gCurrentCup = CUP_NONE;

volatile TickType_t gBrewEndTick     = 0;      // absolute tick when brew ends
volatile uint32_t   gBrewDurationMs  = 0;      // duration of current brew

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
void StartCoffeeStateMachineTask(void const * argument);
void StartCoffeeSensorTask(void const * argument);
void StartCoffeeConsoleTask(void const * argument);
void StartCoffeeDisplayTask(void const * argument);

/* USER CODE BEGIN PFP */
static uint8_t Coffee_ReadWaterLevelPercent(void);
static float   Coffee_ReadTemperatureDegC(void);
static void    Coffee_UpdateLeds(void);
static void    Coffee_SendConsole(const char *fmt, ...);

static void DrawHeader(const char *title);
static void DrawCupFrame(void);
static void DrawCupFill(uint8_t percent);
static void Screen_OffOrIdle(uint8_t water, float tempC);
static void Screen_Brewing(uint8_t water, float tempC);
static void Screen_Done(void);
static void Screen_AutoOff(void);
static void Screen_ErrorNoWater(void);
static uint8_t GetBrewProgressPercent(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void beep(uint8_t times) {
	for(uint8_t i=0; i < times; i++) {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		osDelay(100);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		osDelay(100);
	}
}

//No Context Switch needed
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    CoffeeEvent_t evt = EVT_NONE;
    TickType_t now = xTaskGetTickCountFromISR();

    /* Determine index for debounce table */
    int idx = -1;

    if (GPIO_Pin == BUTTON_EXTI13_Pin)         idx = 0;  // Blue onboard button
    else if (GPIO_Pin == BUTTON_CUP_SIZE_1_Pin) idx = 1;
    else if (GPIO_Pin == BUTTON_CUP_SIZE_2_Pin) idx = 2;
    else if (GPIO_Pin == BUTTON_CUP_SIZE_3_Pin) idx = 3;
    else if (GPIO_Pin == BUTTON_CUP_SIZE_4_Pin) idx = 4;
    else if (GPIO_Pin == BUTTON_STRONG_BREW_Pin) idx = 5;

    if (idx < 0)
        return;

    if ((now - gLastExtiTick[idx]) < pdMS_TO_TICKS(DEBOUNCE_MS))
        return; // ignore bounce

    gLastExtiTick[idx] = now;

    /* Map pin to event */
    if (GPIO_Pin == BUTTON_EXTI13_Pin)
    {
        evt = EVT_POWER_TOGGLE;
    }
    else if (GPIO_Pin == BUTTON_STRONG_BREW_Pin)
    {
        evt = EVT_STRONG_TOGGLE;
    }
    else if (GPIO_Pin == BUTTON_CUP_SIZE_1_Pin)
    {
        evt = EVT_CUP_1;
    }
    else if (GPIO_Pin == BUTTON_CUP_SIZE_2_Pin)
    {
        evt = EVT_CUP_2;
    }
    else if (GPIO_Pin == BUTTON_CUP_SIZE_3_Pin)
    {
        evt = EVT_CUP_3;
    }
    else if (GPIO_Pin == BUTTON_CUP_SIZE_4_Pin)
    {
        evt = EVT_CUP_4;
    }

    if (evt != EVT_NONE && xEventQueueHandle != NULL)
    {
        xQueueSendFromISR(xEventQueueHandle, &evt, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

/* Read water level as percentage (0–100) from ADC on PC5 (WATER_LVL_Pin) */
static uint8_t Coffee_ReadWaterLevelPercent(void) {
    uint32_t adc_in = 0;

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
    	adc_in = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    // 12-bit ADC: 0–4095 -> 0–100%
    uint8_t water_pct = (uint8_t)((adc_in * 100U) / 4095U);
    return water_pct;
}

/* Simulated temperature: here just a fake ramp or fixed value.
   You can replace this with real sensor reading (HTS221, LPS22, etc.) */
static float Coffee_ReadTemperatureDegC(void) {
    /* For now, return a dummy 85.0C as “hot water” */
    return BSP_TSENSOR_ReadTemp();
}

/* Update LEDs according to state and strong brew flag */
static void Coffee_UpdateLeds(void) {
    /* Power LED: LED2 on PB14 */
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,
    				gCoffeeState == COFFEE_STATE_OFF ? GPIO_PIN_RESET : GPIO_PIN_SET);
    /* Auto-off LED */
    HAL_GPIO_WritePin(LED_AUTO_OFF_GPIO_Port, LED_AUTO_OFF_Pin,
                      gAutoOffFlag ? GPIO_PIN_SET : GPIO_PIN_RESET);
    /* Strong brew LED */
    HAL_GPIO_WritePin(LED_STRONG_BREW_GPIO_Port, LED_STRONG_BREW_Pin,
                      gStrongBrew ? GPIO_PIN_SET : GPIO_PIN_RESET);
    /* Cup size LEDs */
    static TickType_t lastBlinkTick = 0;
    static GPIO_PinState blinkState = GPIO_PIN_RESET;

    TickType_t now = xTaskGetTickCount();

    if (gCoffeeState == COFFEE_STATE_BREWING) {
        /* Blink at CUP_LED_BLINK_PERIOD_MS while brewing */
        if ((now - lastBlinkTick) >= pdMS_TO_TICKS(CUP_LED_BLINK_PERIOD_MS)) {
            blinkState = (blinkState == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
            lastBlinkTick = now;
        }
    }
    else if (gCoffeeState == COFFEE_STATE_IDLE)
        /* In IDLE, show selected cup as solid ON */
        blinkState = GPIO_PIN_SET;
    else
        /* OFF or ERROR: all cup LEDs OFF, no blinking */
        blinkState = GPIO_PIN_RESET;
    GPIO_PinState cup1 = GPIO_PIN_RESET;
    GPIO_PinState cup2 = GPIO_PIN_RESET;
    GPIO_PinState cup3 = GPIO_PIN_RESET;
    GPIO_PinState cup4 = GPIO_PIN_RESET;

    if (gCoffeeState != COFFEE_STATE_OFF &&
        gCoffeeState != COFFEE_STATE_ERROR_NO_WATER)
    {
        switch (gCurrentCup)
        {
        case CUP_1: cup1 = blinkState; break;
        case CUP_2: cup2 = blinkState; break;
        case CUP_3: cup3 = blinkState; break;
        case CUP_4: cup4 = blinkState; break;
        default: /* CUP_NONE: all remain OFF */ break;
        }
    }

    HAL_GPIO_WritePin(LED_CUP_SIZE_1_GPIO_Port, LED_CUP_SIZE_1_Pin, cup1);
    HAL_GPIO_WritePin(LED_CUP_SIZE_2_GPIO_Port, LED_CUP_SIZE_2_Pin, cup2);
    HAL_GPIO_WritePin(LED_CUP_SIZE_3_GPIO_Port, LED_CUP_SIZE_3_Pin, cup3);
    HAL_GPIO_WritePin(LED_CUP_SIZE_4_GPIO_Port, LED_CUP_SIZE_4_Pin, cup4);
}

static void Coffee_SendConsole(const char *fmt, ...) {
    if (xConsoleQueueHandle == NULL) return;

    ConsoleMessage_t msg;
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg.text, sizeof(msg.text), fmt, args);
    va_end(args);

    /* Send, drop if queue full (non-blocking) */
    (void)xQueueSend(xConsoleQueueHandle, &msg, 0);
}

/* ------------------------------------------------------------------ */
/* Screen helpers                                                     */
/* ------------------------------------------------------------------ */

static void DrawHeader(const char *title) {
    ST7735_FillRectangleFast(0, 0, ST7735_WIDTH, 22, ST7735_BLUE);
    ST7735_WriteString(20, 2, title, Font_11x18, ST7735_WHITE, ST7735_BLUE);
}

static void DrawCupFrame(void) {
    /* Outline */
    ST7735_FillRectangleFast(CUP_X, CUP_Y, CUP_W, CUP_H, COLOR_FRAME);
    /* Hollow inside */
    ST7735_FillRectangleFast(CUP_X + 2, CUP_Y + 2, CUP_W - 4, CUP_H - 4, COLOR_BG);
}

static void DrawCupFill(uint8_t percent) {
    if (percent > 100) percent = 100;

    uint16_t innerH = CUP_H - 4;
    uint16_t innerW = CUP_W - 4;

    uint16_t fillH  = (innerH * percent) / 100;
    uint16_t startY = CUP_Y + 2 + innerH - fillH;

    /* Clear inside first */
    ST7735_FillRectangleFast(CUP_X + 2, CUP_Y + 2, innerW, innerH, COLOR_BG);

    if (fillH > 0)
    {
        ST7735_FillRectangleFast(CUP_X + 2, startY, innerW, fillH, COLOR_COFFEE);
    }
}

static void Screen_OffOrIdle(uint8_t water, float tempC) {
    char line[32];

    const char *stateText = (gCoffeeState == COFFEE_STATE_OFF) ? "OFF" : "IDLE";
    DrawHeader(stateText);

    /* Main info block */
    snprintf(line, sizeof(line), "Water: %3u%%", water);
    ST7735_WriteString(2, 24, line, Font_7x10,
                       (water < WATER_LEVEL_MIN_PERCENT) ? COLOR_WATER_LOW : COLOR_WATER_OK,
                       COLOR_BG);

    snprintf(line, sizeof(line), "Temp : %4.1fC", tempC);
    ST7735_WriteString(2, 44, line, Font_7x10, COLOR_TEXT, COLOR_BG);

    snprintf(line, sizeof(line), "Brew : %s", gStrongBrew ? "STRONG" : "NORMAL");
    ST7735_WriteString(2, 64, line, Font_7x10,
                       gStrongBrew ? COLOR_HILIGHT : COLOR_TEXT, COLOR_BG);

    if (gAutoOffFlag)
    {
        ST7735_WriteString(2, 110, "AUTO OFF READY", Font_7x10,
                           COLOR_HILIGHT, COLOR_BG);
    }
    else
    {
        ST7735_WriteString(2, 84, "Choose Brew Size:", Font_7x10,
                           COLOR_TEXT, COLOR_BG);
        ST7735_WriteString(6, 84+12, "Cup Size 1: 6oz", Font_7x10,
                                   COLOR_TEXT, COLOR_BG);
        ST7735_WriteString(6, 84+12*2, "Cup Size 2: 8oz", Font_7x10,
                                   COLOR_TEXT, COLOR_BG);
        ST7735_WriteString(6, 84+12*3, "Cup Size 3: 10oz", Font_7x10,
                                   COLOR_TEXT, COLOR_BG);
        ST7735_WriteString(6, 84+12*4, "Cup Size 4: 12oz", Font_7x10,
                                   COLOR_TEXT, COLOR_BG);
    }
}

static void Screen_Brewing(uint8_t water, float tempC) {
    char line[32];

    DrawHeader("BREWING");

    /* Text on left side */
    snprintf(line, sizeof(line), "Cup Size: %u", gCurrentCup);
    ST7735_WriteString(2, 24, line, Font_7x10, COLOR_TEXT, COLOR_BG);

    snprintf(line, sizeof(line), "%s", gStrongBrew ? "STRONG" : "NORMAL");
    ST7735_WriteString(2, 44, line, Font_7x10,
                       gStrongBrew ? COLOR_HILIGHT : COLOR_TEXT, COLOR_BG);

//    snprintf(line, sizeof(line), "Temp: %4.1fC", tempC);
//    ST7735_WriteString(2, 64, line, Font_7x10, COLOR_TEXT, COLOR_BG);

    uint8_t progress = GetBrewProgressPercent();
    snprintf(line, sizeof(line), "%3u%%", progress);
    ST7735_WriteString(2, 84, line, Font_11x18, COLOR_HILIGHT, COLOR_BG);

    /* Cup animation on the right */
    DrawCupFrame();
    DrawCupFill(progress);
}

static void Screen_Done(void) {
	DrawHeader("DONE!");
	ST7735_DrawImage(0, 20, ST7735_WIDTH, ST7735_HEIGHT-22, (uint16_t*)enjoy_coffee_128x128);
	ST7735_FillRectangleFast(0, 140, ST7735_WIDTH, ST7735_HEIGHT-140, ST7735_WHITE);
}

static void Screen_AutoOff(void) {
    DrawHeader("STANDBY");
    ST7735_WriteString(10, 40, "AUTO OFF",   Font_7x10, COLOR_HILIGHT, COLOR_BG);
    ST7735_WriteString(10, 62, "Press POWER", Font_7x10, COLOR_TEXT,   COLOR_BG);
    ST7735_WriteString(10, 84, "to WAKE",    Font_7x10, COLOR_TEXT,   COLOR_BG);
}

static void Screen_ErrorNoWater(void) {
    DrawHeader("ERROR");
    ST7735_WriteString(10, 40, "NO WATER!",  Font_7x10, COLOR_ERROR, COLOR_BG);
    ST7735_WriteString(10, 62, "Refill tank",Font_7x10, COLOR_TEXT,  COLOR_BG);
    ST7735_WriteString(10, 84, "Press POWER",Font_7x10, COLOR_TEXT,  COLOR_BG);
}

/* ------------------------------------------------------------------ */
/* Brew progress based on FreeRTOS ticks                              */
/* ------------------------------------------------------------------ */

static uint8_t GetBrewProgressPercent(void) {
    if (gCoffeeState != COFFEE_STATE_BREWING || gBrewDurationMs == 0)
        return 0;

    TickType_t now        = xTaskGetTickCount();
    TickType_t totalTicks = pdMS_TO_TICKS(gBrewDurationMs);
    TickType_t startTick  = gBrewEndTick - totalTicks;

    TickType_t elapsed;
    if (now <= startTick)
        elapsed = 0;
    else
        elapsed = now - startTick;

    if (elapsed >= totalTicks)
        return 100;

    uint32_t pct = (uint32_t)elapsed * 100U / totalTicks;
    if (pct > 100U) pct = 100U;
    return (uint8_t)pct;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  BSP_TSENSOR_Init();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of xEventQueue */
  osMessageQDef(xEventQueue, 10, CoffeeEvent_t);
  xEventQueueHandle = osMessageCreate(osMessageQ(xEventQueue), NULL);

  /* definition and creation of xConsoleQueue */
  osMessageQDef(xConsoleQueue, 10, ConsoleMessage_t);
  xConsoleQueueHandle = osMessageCreate(osMessageQ(xConsoleQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  if ((xEventQueueHandle == NULL) || (xConsoleQueueHandle == NULL))
  {
	  // If something failed, stay here
	  while (1) { }
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of CoffStateMachin */
  osThreadDef(CoffStateMachin, StartCoffeeStateMachineTask, osPriorityRealtime, 0, 512);
  CoffStateMachinHandle = osThreadCreate(osThread(CoffStateMachin), NULL);

  /* definition and creation of CoffSensorTask */
  osThreadDef(CoffSensorTask, StartCoffeeSensorTask, osPriorityHigh, 0, 256);
  CoffSensorTaskHandle = osThreadCreate(osThread(CoffSensorTask), NULL);

  /* definition and creation of CoffConsoleTask */
  osThreadDef(CoffConsoleTask, StartCoffeeConsoleTask, osPriorityAboveNormal, 0, 256);
  CoffConsoleTaskHandle = osThreadCreate(osThread(CoffConsoleTask), NULL);

  /* definition and creation of CoffDisplayTask */
  osThreadDef(CoffDisplayTask, StartCoffeeDisplayTask, osPriorityLow, 0, 256);
  CoffDisplayTaskHandle = osThreadCreate(osThread(CoffDisplayTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  gLastActivityTick = xTaskGetTickCount();

  Coffee_SendConsole("Power Off\r\n");
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000E14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_ADD_WATER_Pin|LED_AUTO_OFF_Pin|ARD_D10_Pin|LED_CUP_SIZE_2_Pin
                          |SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_CUP_SIZE_1_Pin|LED_CUP_SIZE_4_Pin|ARD_D8_Pin|ISM43362_BOOT0_Pin
                          |ISM43362_WAKEUP_Pin|LED2_Pin|SPSGRF_915_SDN_Pin|LED_CUP_SIZE_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|LED_STRONG_BREW_Pin|TFT_RESET_Pin|TFT_DC_Pin
                          |TFT_CS_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin */
  GPIO_InitStruct.Pin = SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_CUP_SIZE_1_Pin BUTTON_CUP_SIZE_2_Pin BUTTON_CUP_SIZE_3_Pin BUTTON_CUP_SIZE_4_Pin
                           BUTTON_STRONG_BREW_Pin */
  GPIO_InitStruct.Pin = BUTTON_CUP_SIZE_1_Pin|BUTTON_CUP_SIZE_2_Pin|BUTTON_CUP_SIZE_3_Pin|BUTTON_CUP_SIZE_4_Pin
                          |BUTTON_STRONG_BREW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_ADD_WATER_Pin LED_AUTO_OFF_Pin ARD_D10_Pin LED_CUP_SIZE_2_Pin
                           SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = LED_ADD_WATER_Pin|LED_AUTO_OFF_Pin|ARD_D10_Pin|LED_CUP_SIZE_2_Pin
                          |SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_CUP_SIZE_1_Pin LED_CUP_SIZE_4_Pin ARD_D8_Pin ISM43362_BOOT0_Pin
                           ISM43362_WAKEUP_Pin LED2_Pin SPSGRF_915_SDN_Pin LED_CUP_SIZE_3_Pin
                           SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = LED_CUP_SIZE_1_Pin|LED_CUP_SIZE_4_Pin|ARD_D8_Pin|ISM43362_BOOT0_Pin
                          |ISM43362_WAKEUP_Pin|LED2_Pin|SPSGRF_915_SDN_Pin|LED_CUP_SIZE_3_Pin
                          |SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin HTS221_DRDY_EXTI15_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|HTS221_DRDY_EXTI15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin LED_STRONG_BREW_Pin TFT_RESET_Pin
                           TFT_DC_Pin TFT_CS_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|LED_STRONG_BREW_Pin|TFT_RESET_Pin
                          |TFT_DC_Pin|TFT_CS_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(PMOD_UART2_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCoffeeStateMachineTask */
/**
  * @brief  Function implementing the CoffStateMachin thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCoffeeStateMachineTask */
void StartCoffeeStateMachineTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
    CoffeeEvent_t evt;
    uint8_t uWater_lvl;

    for (;;) {
        /* Wait for an event (buttons) with 100ms timeout to also handle timers */
        if (xQueueReceive(xEventQueueHandle, &evt, pdMS_TO_TICKS(100)) == pdPASS) {
            gLastActivityTick = xTaskGetTickCount();

            switch (evt) {
            case EVT_POWER_TOGGLE:
            	beep(1);
                if (gCoffeeState == COFFEE_STATE_OFF) {
                	gAutoOffFlag = 0;
                    gCoffeeState = COFFEE_STATE_IDLE;
                    Coffee_SendConsole("Power On\r\n");

                }
                else {
                    gCoffeeState = COFFEE_STATE_OFF;
                    gStrongBrew  = 0;
                    gAutoOffFlag = 0;
                    gCurrentCup  = CUP_NONE;   // clear selection
                    Coffee_SendConsole("Power Off\r\n");
                }
                break;

            case EVT_STRONG_TOGGLE:
            	beep(1);
                if (gCoffeeState != COFFEE_STATE_OFF) {
                    gStrongBrew = !gStrongBrew;
                    if (gStrongBrew)
                    	Coffee_SendConsole("Strong Brew: ON\r\n");
                    else
                    	Coffee_SendConsole("Strong Brew: OFF\r\n");
                }
                break;

            case EVT_CUP_1:
            case EVT_CUP_2:
            case EVT_CUP_3:
            case EVT_CUP_4:
            	beep(1);
                if (gCoffeeState == COFFEE_STATE_IDLE) {
                	uWater_lvl = Coffee_ReadWaterLevelPercent();
                    if (uWater_lvl < WATER_LEVEL_MIN_PERCENT) {
                        gCoffeeState = COFFEE_STATE_ERROR_NO_WATER;
                        Coffee_SendConsole("Error: Low water level (%u%%)\r\n", uWater_lvl);
                    }
                    else {
                        /* Select brew time */
                        switch (evt) {
                        case EVT_CUP_1:
                        	gBrewDurationMs = BREW_TIME_CUP1_MS;
                        	gCurrentCup = CUP_1;
                        	break;
                        case EVT_CUP_2:
                        	gBrewDurationMs = BREW_TIME_CUP2_MS;
                        	gCurrentCup = CUP_2;
                        	break;
                        case EVT_CUP_3:
                        	gBrewDurationMs = BREW_TIME_CUP3_MS;
                        	gCurrentCup = CUP_3;
                        	break;
                        case EVT_CUP_4:
                        	gBrewDurationMs = BREW_TIME_CUP4_MS;
                        	gCurrentCup = CUP_4;
                        	break;
                        default:
                        	gBrewDurationMs = BREW_TIME_CUP1_MS;
                        	gCurrentCup = CUP_1;
                        	break;
                        }

                        gCoffeeState = COFFEE_STATE_BREWING;
                        gBrewEndTick = xTaskGetTickCount() + pdMS_TO_TICKS(gBrewDurationMs);


                        Coffee_SendConsole("Brewing started (%lu ms) %s\r\n", (unsigned long)gBrewDurationMs, gStrongBrew ? "[STRONG]" : "");
                    }
                }
                break;

            default:
                break;
            }
        }

        /* State-based housekeeping */
        TickType_t now = xTaskGetTickCount();

        if (gCoffeeState == COFFEE_STATE_BREWING) {

            if (now >= gBrewEndTick) {
            	beep(3);
            	Coffee_SendConsole("Brewing complete\r\n");
                gCoffeeState = COFFEE_STATE_DONE;
                gAutoOffFlag = 0;
                gLastActivityTick = now;
                gCurrentCup = 0;           // clear selection when done
				gBrewDurationMs = 0;
            }
            uWater_lvl = Coffee_ReadWaterLevelPercent();
			if (uWater_lvl < WATER_LEVEL_MIN_PERCENT) {
				gCoffeeState = COFFEE_STATE_ERROR_NO_WATER;
				Coffee_SendConsole("Error: Low water level (%u%%)\r\n", uWater_lvl);
			}
        }
        if (gCoffeeState == COFFEE_STATE_DONE) {
        	if ((now - gBrewEndTick) > pdMS_TO_TICKS(2000)) {
        		gCoffeeState = COFFEE_STATE_IDLE;
        	}

        }

        if (gCoffeeState == COFFEE_STATE_ERROR_NO_WATER) {
            /* Blink Add Water LED, state cleared when water OK and user presses power */
            HAL_GPIO_TogglePin(LED_ADD_WATER_GPIO_Port, LED_ADD_WATER_Pin);
        }

        /* Auto-off: after timeout in IDLE, turn machine off and set auto-off LED */
        if ((gCoffeeState == COFFEE_STATE_IDLE) &&
            ((now - gLastActivityTick) > pdMS_TO_TICKS(AUTO_OFF_TIMEOUT_MS))) {
            gCoffeeState = COFFEE_STATE_OFF;
            gStrongBrew = 0;
            gAutoOffFlag = 1;
            Coffee_SendConsole("Auto Off\r\n");
        }

        Coffee_UpdateLeds();
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCoffeeSensorTask */
/**
* @brief Function implementing the CoffSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCoffeeSensorTask */
void StartCoffeeSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartCoffeeSensorTask */
  /* Infinite loop */
    for (;;)
    {
        uint8_t water = Coffee_ReadWaterLevelPercent();
        float tempC   = Coffee_ReadTemperatureDegC();

        /* Add Water LED: on if low water, off otherwise (if not blinking for error) */
        if (gCoffeeState != COFFEE_STATE_OFF &&gCoffeeState != COFFEE_STATE_ERROR_NO_WATER)
        {
            if (water < WATER_LEVEL_MIN_PERCENT)
                HAL_GPIO_WritePin(LED_ADD_WATER_GPIO_Port, LED_ADD_WATER_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(LED_ADD_WATER_GPIO_Port, LED_ADD_WATER_Pin, GPIO_PIN_RESET);

            /* Periodic console status like a real coffee maker display */
			Coffee_SendConsole("Water: %u%%  Temp: %.1fC\r\n", water, tempC);
        }


        vTaskDelay(pdMS_TO_TICKS(1000)); // every 1s
    }
  /* USER CODE END StartCoffeeSensorTask */
}

/* USER CODE BEGIN Header_StartCoffeeConsoleTask */
/**
* @brief Function implementing the CoffConsoleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCoffeeConsoleTask */
void StartCoffeeConsoleTask(void const * argument)
{
  /* USER CODE BEGIN StartCoffeeConsoleTask */
  /* Infinite loop */
	ConsoleMessage_t msg;
	for (;;)
	{
		HAL_GPIO_TogglePin(LED3_WIFI__LED4_BLE_GPIO_Port, LED3_WIFI__LED4_BLE_Pin);
		if (xQueueReceive(xConsoleQueueHandle, &msg, portMAX_DELAY) == pdPASS)
		{
			HAL_UART_Transmit(&huart1, (uint8_t*)msg.text, strlen(msg.text), HAL_MAX_DELAY);
		}
	}
  /* USER CODE END StartCoffeConsoleTask */
}

/* USER CODE BEGIN Header_StartCoffeeDisplayTask */
/**
* @brief Function implementing the CoffDisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCoffeeDisplayTask */
void StartCoffeeDisplayTask(void const * argument)
{
    /* USER CODE BEGIN StartCoffeeDisplayTask */


    /* Initialize LCD --------------------------------------------------------*/
    ST7735_Init();
    ST7735_FillScreenFast(COLOR_BG);
    ST7735_SetGamma(GAMMA_22);

    CoffeeState_t lastState = (CoffeeState_t)(-1);

    /* Main display loop -----------------------------------------------------*/
    for (;;) {
        CoffeeState_t state = gCoffeeState;
        uint8_t water   = Coffee_ReadWaterLevelPercent();
        float tempC     = Coffee_ReadTemperatureDegC();

        if (state != lastState) {
            ST7735_FillScreenFast(COLOR_BG);
            lastState = state;
        }

        switch (state) {
        case COFFEE_STATE_OFF:
        	if (state == COFFEE_STATE_OFF && gAutoOffFlag)
				Screen_AutoOff();
        	else
        		ST7735_FillScreen(ST7735_BLACK);
        	break;
        case COFFEE_STATE_IDLE:
            Screen_OffOrIdle(water, tempC);
            break;

        case COFFEE_STATE_BREWING:
            Screen_Brewing(water, tempC);
            break;

        case COFFEE_STATE_DONE:
        	Screen_Done();
        	break;

        case COFFEE_STATE_ERROR_NO_WATER:
            Screen_ErrorNoWater();
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(250));  // ~4 FPS
    }

    /* USER CODE END StartCoffeeDisplayTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

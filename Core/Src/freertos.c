/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "tim.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint8_t G[8];
  uint8_t R[8];
  uint8_t B[8];
} rgb_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_LEDS  1
#define NUM_RESET 64
#define NUM_DATA  (NUM_LEDS + NUM_RESET)
#define T_LOW     2
#define T_HIGH    5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static rgb_t pixels[NUM_DATA];
extern int32_t x_raw; 
extern int32_t y_raw;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int getRGB(int rx, int ry, int i, int j);
void set_RGB(int i, int j, int r, int g, int b);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);


extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */


/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  // force re-enumeration by pulling down USB_DP for a couple of ms
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(5);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  for (size_t i = 0; i < NUM_DATA; ++i) {
    memset(pixels, 0, NUM_DATA * sizeof(rgb_t));
  }

  for (size_t i = 0; i < NUM_LEDS; ++i) {
    memset(pixels, T_LOW, NUM_LEDS * sizeof(rgb_t));
  }

  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, (uint32_t*)pixels, NUM_DATA * sizeof(rgb_t));
  /* Infinite loop */
  while (1) {
    HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
    osDelay(500);

    //set_RGB(0,0,0,1,1);
    //wait
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);

    if(flags){
    
    // memset(pixels[0].R, T_HIGH, 8);
    // memset(pixels[0].G, T_LOW, 8);
    // memset(pixels[0].B, T_LOW, 8);
    // osDelay(500);

    // memset(pixels[0].R, T_LOW, 8);
    // memset(pixels[0].G, T_HIGH, 8);
    // memset(pixels[0].B, T_LOW, 8);
    // osDelay(500);

    // memset(pixels[0].R, T_LOW, 8);
    // memset(pixels[0].G, T_LOW, 8);
    // memset(pixels[0].B, T_HIGH, 8);
    // osDelay(500);
    }
    
    
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void set_RGB(int i, int j, int r, int g, int b){
  int r_val; 
  int g_val;
  int b_val;
  int index;   
  if(r==0)
  r_val = T_LOW; 
  else
  r_val = T_HIGH; 

  if(g==0)
  g_val = T_LOW; 
  else
  g_val = T_HIGH;

  if(b==0)
  b_val = T_LOW; 
  else
  b_val = T_HIGH;
  
  //case1: normal case  
  if(i%2 == 0){
    index = i*8+j;
    memset(pixels[index].R, r_val, 8);
    memset(pixels[index].G, g_val, 8);
    memset(pixels[index].B, b_val, 8);
    osDelay(500);
  }

  //case2: odd number of rows 
  else{
    int j_fix = 7-j;
    index = i*8+j_fix;
    memset(pixels[index].R, r_val, 8);
    memset(pixels[index].G, g_val, 8);
    memset(pixels[index].B, b_val, 8);
  }

}


int getRGB(int rx, int ry, int i, int j){
  /*
    Retval:
          0: Dim
          1: Red
          2: Green
          3: Blue  
  */

  //if inner frame, return and illuminate blue
  if ( (i==2 && j >= 2 && j <= 5) || (j==2 && i >= 2 && i <=5) )
  return 3; 
  
  //First convert to an index compatible with LED matrix (all integers ) 
  int flag;
  if (rx > 0)
  rx = rx + 3;
  else
  rx = rx + 4;

  if (ry > 0)
  ry = ry + 3;
  else 
  ry = ry + 4; 

  //then check if this is a good alignment
  if(rx >= 2 && rx <= 5&& ry >=2 && ry <=5 )
  flag = 2; 
  else
  flag = 1;

  //if on the frame or overlap with the actual drone position, light to corresponding color  
  if(i == 0 || i == 7|| j == 0 || j ==7 || (i == rx && j == ry) )
  return flag;
  //if not, stay dim
  else 
  return 0;  
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

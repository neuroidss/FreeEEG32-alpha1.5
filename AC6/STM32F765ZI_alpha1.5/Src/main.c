/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ad7779.h"
#include "platform_drivers.h"
#include "string.h"

#define ARM_MATH_CM7
#include "core_cm7.h"

//#include "arm_common_tables.h"

#define __FPU_PRESENT             1U       /*!< FPU present */

#include <stdlib.h>
#include <math.h>
//#include "arm_math.h"
#include <stdlib.h>
#include <string.h>

#include <math.h>
//#include "arm_math.h"

#include <time.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
SAI_HandleTypeDef hsai_BlockA2;
SAI_HandleTypeDef hsai_BlockB2;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi6;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

PCD_HandleTypeDef hpcd_USB_OTG_HS;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int SPI_RxCplt=0;
int SAI_RxCplt=0;
int I2S_RxCplt=0;
int SPI_RxTxCplt=0;


//GPIO_TypeDef * led_ports[] = {LED_YELLOW_GPIO_Port,LED_RED_GPIO_Port,LED_GREEN_GPIO_Port,LED_BLUE_GPIO_Port};
//uint16_t led_pins[] = {LED_YELLOW_Pin,LED_RED_Pin,LED_GREEN_Pin,LED_BLUE_Pin};

//GPIO_TypeDef * led_ports[] = {LED_RED_GPIO_Port,LED_GREEN_GPIO_Port,LED_BLUE_GPIO_Port,LED_YELLOW_GPIO_Port};
//uint16_t led_pins[] = {LED_RED_Pin,LED_GREEN_Pin,LED_BLUE_Pin,LED_YELLOW_Pin};
//GPIO_TypeDef * led_ports[] = {LED_RED_GPIO_Port,LED_GREEN_GPIO_Port};
//uint16_t led_pins[] = {LED_RED_Pin,LED_GREEN_Pin};

//GPIO_TypeDef * reset_ports[] = {AD1_RESET_GPIO_Port,AD2_RESET_GPIO_Port,AD3_RESET_GPIO_Port,AD4_RESET_GPIO_Port};
//uint16_t reset_pins[] = {AD1_RESET_Pin,AD2_RESET_Pin,AD3_RESET_Pin,AD4_RESET_Pin};

int32_t samples_int32_average[average_number][uint8_ad_adc_number][uint8_ad_chan_number];
uint8_t datas_out[uint8_ad_adc_number][uint8_ad_number];
uint8_t datas[uint8_ad_adc_number][uint8_ad_number];
uint8_t data1[uint8_ad_number];
uint8_t data2[uint8_ad_number];
uint8_t data3[uint8_ad_number];
uint8_t data4[uint8_ad_number];

#define datasBuffersize (4)

//#define dataBuffer110size (2 + uint8_ad_chan_number * (3) * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1)
#define dataBuffer110size (2 + uint8_ad_chan_number * (1 + 3) * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1)

uint8_t dataBuffer110[dataBuffer110size];

uint8_t datasBuffer[datasBuffersize][uint8_ad_number];
//uint8_t aTxdatasBuffer[datasBuffersize][uint8_ad_number];
//uint8_t aRxdatasBuffer[datasBuffersize][uint8_ad_number];

uint8_t aTxBuffer[uint8_ad_number];
uint8_t aRxBuffer[uint8_ad_number];

ad7779_dev *devices[uint8_ad_adc_number];

uint8_t ui8SampleNumber=-1;
uint32_t ui32SampleNumber=-1;

//const uint8_t uint8_data_number_print = 200;
//const uint8_t uint8_data_number_print2 = 200;
#define uint8_data_number_print  300
#define uint8_data_number_print2  300
#define uint8_data_number_print7  300
uint8_t dataBuffer_print2[uint8_data_number_print2];
uint8_t dataBuffer_print[uint8_data_number_print];
uint8_t dataBuffer_print7[uint8_data_number_print7];


#define LCD_RAM   *(uint16_t *)((uint32_t)0x60020000)  //disp Data ADDR
#define LCD_REG   *(uint16_t *)((uint32_t)0x60000000)    //disp Reg  ADDR

#define LCD_NOOP            0x00   /* No Operation*/
#define LCD_SWRESET             0x01   /* Software Reset */
#define LCD_RDDPM               0x0A   /* Read Display Power Mode */
#define LCD_RDDMADCTL           0x0B   /* Read Display MADCTL */
#define LCD_RDDCOLMOD           0x0C   /* Read Display Pixel Format */
#define LCD_RDDIM               0x0D   /* Read Display Image Format */
#define LCD_RDDSM               0x0E   /* Read Display Signal Mode */
#define LCD_RDDSDR              0x0F   /* Read Display Self-Diagnostic Result */
#define LCD_SPLIN               0x10   /* Enter Sleep Mode */
#define LCD_SLEEP_OUT           0x11   /* Sleep out register */
#define LCD_PTLON               0x12   /* Partial Mode ON */
#define LCD_NORMAL_MODE_ON      0x13   /* Normal Display Mode ON */
#define LCD_DINVOFF             0x20   /* Display Inversion OFF */
#define LCD_DINVON              0x21   /* Display Inversion ON */
#define LCD_GAMMA               0x26   /* Gamma register */
#define LCD_DISPLAY_OFF         0x28   /* Display off register */
#define LCD_DISPLAY_ON          0x29   /* Display on register */
#define LCD_COLUMN_ADDR         0x2A   /* Column address register */
#define LCD_PAGE_ADDR           0x2B   /* Page address register */
#define LCD_GRAM                0x2C   /* GRAM register */
#define LCD_RGBSET              0x2D   /* Color SET */
#define LCD_RAMRD               0x2E   /* Memory Read */
#define LCD_PLTAR               0x30   /* Partial Area */
#define LCD_VSCRDEF             0x33   /* Vertical Scrolling Definition */
#define LCD_TEOFF               0x34   /* Tearing Effect Line OFF */
#define LCD_TEON                0x35   /* Tearing Effect Line ON */
#define LCD_MAC                 0x36   /* Memory Access Control register*/
#define LCD_VSCRSADD            0x37   /* Vertical Scrolling Start Address */
#define LCD_IDMOFF              0x38   /* Idle Mode OFF */
#define LCD_IDMON               0x39   /* Idle Mode ON */
#define LCD_PIXEL_FORMAT        0x3A   /* Pixel Format register */
#define LCD_WRITE_MEM_CONTINUE  0x3C   /* Write Memory Continue */
#define LCD_READ_MEM_CONTINUE   0x3E   /* Read Memory Continue */
#define LCD_SET_TEAR_SCANLINE   0x44   /* Set Tear Scanline */
#define LCD_GET_SCANLINE        0x45   /* Get Scanline */
#define LCD_READ_DDB_START      0xA1   /* Read DDB start */

/* Level 2 Commands */
#define LCD_CMDACCPRTC          0xB0   /* Command Access Protect  */
#define LCD_FRMCTR              0xB3   /* Frame Memory Access and Interface setting  */
#define LCD_DMFMCTR             0xB4   /* Display Mode and Frame Memory Write Mode
setting */
#define LCD_DEVCODERD           0xBF   /* Device code read */
#define LCD_PANEL_DRV_CTL       0xC0   /* Panel Driving Setting */
#define LCD_NORMAL_TIMING_WR    0xC1   /* Display Timing Setting for Normal Mode  */
#define LCD_PARTIAL_TIMING_WR   0xC2   /* Display Timing Setting for Partial Mode  */
#define LCD_IDLE_TIMING_WR      0xC3   /* Display Timing Setting for Idle Mode  */
#define LCD_FR_INV_CTL          0xC5   /* Frame rate and Inversion Control  */
#define LCD_INTERFACE           0xC6   /* Interface Control */
#define LCD_GAMMAWR            0xC8   /* Gamma Setting */
#define LCD_POWER               0xD0   /* POWER CONTROL */
#define LCD_VCOM               0xD1   /* VCOM Control */
#define LCD_NORMAL_PWR_WR       0xD2   /* Power Setting for Normal Mode  */
#define LCD_PARTIAL_PWR_WR      0xD3   /* Power Setting for Partial Mode  */
#define LCD_IDLE_PWR_WR         0xD4   /* Power Setting for Idle Mode  */
#define LCD_NVMEMWR             0xE0   /* NV Memory Write  */
#define LCD_NVMEMCTRL           0xE1   /* NV Memory Control */
#define LCD_NVMEMRD             0xE2   /* NV Memory Status */
#define LCD_NVMEMPRT            0xE3   /* NV Memory Protection  */
#define LCD_EEPROMWR_ENABLE      0xE8   /* EEPROM Write Enable  */
#define LCD_EEPROMWR_DISABLE    0xE9   /* EEPROM Write Disable  */
#define LCD_EEPROMWR         0xEA   /* EEPROM Word Write */
#define LCD_EEPROMRD            0xEB   /* EEPROM Word Read   */
#define LCD_EEPROM_ADR_SET       0xEC   /* EEPROM Address Set */
#define LCD_EEPROM_ERASE        0xED   /* EEPROM Erase */
#define LCD_EEPROM_ERASE_ALL    0xEE   /* EEPROM Erase All  */

//RTC_TimeTypeDef sTime;

__IO ITStatus UartReady = RESET;

/* defines number of us per second */
#define US_IN_SECOND            ((uint32_t)1000000)

/* Systick Start */
#define TimerCount_Start()  do{\
              SysTick->LOAD  =  0xFFFFFF  ; /* set reload register */\
              SysTick->VAL  =  0  ;     /* Clear Counter */    \
              SysTick->CTRL  =  0x5 ;     /* Enable Counting*/   \
              }while(0)

/* Systick Stop and retrieve CPU Clocks count */
#define TimerCount_Stop(Value)  do {\
                SysTick->CTRL  =0;  /* Disable Counting */         \
                Value = SysTick->VAL;/* Load the SysTick Counter Value */\
                Value = 0xFFFFFF - Value;/* Capture Counts in CPU Cycles*/\
                }while(0)
#define FIR_PROCESS 0
#define FFT_PROCESS 1

#define Float32 0
#define Q15     1
#define Q31     2
#define LPF     0
#define HPF     1

#define FFT_INVERSE_FLAG        ((uint8_t)0)
#define FFT_Normal_OUTPUT_FLAG  ((uint8_t)1)
#define BUFFER_SIZE       240
#define NB_SAMPLES        64
#define SAMPLES           2048      /* 256 real party and 256 imaginary parts */

//extern
uint32_t _aADC1ConvertedValue_s [SAMPLES];
//extern
uint32_t _uhADCxConvertedValue  ;
//extern
uint32_t _nb_cycles             ;

#define FFT_Length_Tab 1024
//#define FFT_Length_Tab 128

//extern int FFT_SIZE_CHOOSE;

//float32_t aFFT_Input_f32[FFT_Length_Tab*2];
//float32_t aFFT_Output_f32 [FFT_Length_Tab];
//
//float32_t FFT_Input_Q15_f[FFT_Length_Tab*2];
//q15_t aFFT_Input_Q15[FFT_Length_Tab*2];
//q15_t FFT_Output_Q15[FFT_Length_Tab];
//
//float32_t FFT_Input_Q31_f[FFT_Length_Tab*2];
//q31_t aFFT_Input_Q31[FFT_Length_Tab*2];
//q31_t FFT_Output_Q31[FFT_Length_Tab];



uint8_t crc8_01;
uint8_t crc8_23;
uint8_t crc8_45;
uint8_t crc8_67;

uint8_t crc8fromad[4];
uint8_t crc8[4];
uint8_t crcBuffer[4][1+3+3+1];
//uint8_t crcBuffer[8/2][3+3];
uint8_t crc8ok[4];
uint8_t channelnumberok[8];

long int c000 = 0;
long int c_0 = 0;
long int c_1 = 0;
long int c_2 = 0;
long int c_3 = 0;
long int c_00 = 0;
long int c_01 = 0;
long int c_02 = 0;
long int c_03 = 0;
long int c00 = 0;
long int c01 = 0;
long int c0 = 0;
long int c1 = 0;
long int c2 = 0;
long int c3 = 0;
long int c4 = 0;
long int c5 = 0;
long int c6 = 0;
long int c7 = 0;

char str_in[] = "%1.6f";
char str_out[] = "0000000000000000000";

uint8_t uicount=0;

uint32_t last_index_fill_adc_buffer=0;

int32_t last_stddev_count=0;


time_t time1;
#define times_max 20
time_t times[times_max];
long stddevmaxs[times_max];
int times_count=0;
float frac_secs1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SAI1_Init(void);
static void MX_SAI2_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI6_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_USB_OTG_HS_PCD_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void print_hex(int v, int num_places);
void print7_hex(int v, int num_places);
void print2_hex(int v, int num_places);
void print_binary(int v, int num_places );
void print_symbol(uint8_t v);
void print7_symbol(uint8_t v);
void print_text(const char * t);
void print_line();
void print7_line();
void print2_line();
void print_text_line(const char * t);

//void FFT_PROCESSING_F32Process(uint32_t FFT_Length);
//void FFT_PROCESSING_Q15Process(uint32_t FFT_Length);
//void FFT_PROCESSING_Q31Process(uint32_t FFT_Length);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void rtc_read_frac_s(time_t * t, float * secfrac)
//{
//    RTC_HandleTypeDef RtcHandle;
//    RTC_DateTypeDef dateStruct;
//    RTC_TimeTypeDef timeStruct;
//    struct tm timeinfo;
//
//    RtcHandle.Instance = RTC;
//
//    HAL_RTC_GetTime(&RtcHandle, &timeStruct, FORMAT_BIN);
//    HAL_RTC_GetDate(&RtcHandle, &dateStruct, FORMAT_BIN);
//
//    // Setup a tm structure based on the RTC
//    timeinfo.tm_wday = dateStruct.WeekDay;
//    timeinfo.tm_mon  = dateStruct.Month - 1;
//    timeinfo.tm_mday = dateStruct.Date;
//    timeinfo.tm_year = dateStruct.Year + 100;
//    timeinfo.tm_hour = timeStruct.Hours;
//    timeinfo.tm_min  = timeStruct.Minutes;
//    timeinfo.tm_sec  = timeStruct.Seconds;
//
//    // Convert to timestamp
//    *t = mktime(&timeinfo);
//
//// Standard granularity is ~4ms (1/256sec)
//// Import mbed-dev & modify rtc_api.c's rtc_init() with these values for ~1ms (1/1024sec) granularity:
////    RtcHandle.Init.AsynchPrediv   = 0x1f;
////    RtcHandle.Init.SynchPrediv    = (rtc_freq / 0x20) - 1;
//
//    *secfrac = ((float)(timeStruct.SecondFraction-timeStruct.SubSeconds)) / ((float)(timeStruct.SecondFraction+1));
//    return;
//}

void print_hex(int v, int num_places)
{
  const uint32_t uint8_data_number = (num_places % 4 == 0 ? num_places / 4 : num_places / 4 + 1);
//  uint8_t dataBuffer[uint8_data_number];
  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
  {
  }
  int mask=0, n, num_nibbles, digit, nibbles_max;

    for (n=1; n<=num_places; n++)
    {
        mask = (mask << 1) | 0x0001;
    }
    v = v & mask; // truncate v to specified number of places

    num_nibbles = num_places / 4;
    if ((num_places % 4) != 0)
    {
        ++num_nibbles;
    }
    nibbles_max=num_nibbles;

    do
    {
        digit = ((v >> (num_nibbles-1) * 4)) & 0x0f;
        if(digit == 0)  dataBuffer_print[ nibbles_max-num_nibbles ] = '0';
        if(digit == 1)  dataBuffer_print[ nibbles_max-num_nibbles ] = '1';
        if(digit == 2)  dataBuffer_print[ nibbles_max-num_nibbles ] = '2';
        if(digit == 3)  dataBuffer_print[ nibbles_max-num_nibbles ] = '3';
        if(digit == 4)  dataBuffer_print[ nibbles_max-num_nibbles ] = '4';
        if(digit == 5)  dataBuffer_print[ nibbles_max-num_nibbles ] = '5';
        if(digit == 6)  dataBuffer_print[ nibbles_max-num_nibbles ] = '6';
        if(digit == 7)  dataBuffer_print[ nibbles_max-num_nibbles ] = '7';
        if(digit == 8)  dataBuffer_print[ nibbles_max-num_nibbles ] = '8';
        if(digit == 9)  dataBuffer_print[ nibbles_max-num_nibbles ] = '9';
        if(digit == 10) dataBuffer_print[ nibbles_max-num_nibbles ] = 'A';
        if(digit == 11) dataBuffer_print[ nibbles_max-num_nibbles ] = 'B';
        if(digit == 12) dataBuffer_print[ nibbles_max-num_nibbles ] = 'C';
        if(digit == 13) dataBuffer_print[ nibbles_max-num_nibbles ] = 'D';
        if(digit == 14) dataBuffer_print[ nibbles_max-num_nibbles ] = 'E';
        if(digit == 15) dataBuffer_print[ nibbles_max-num_nibbles ] = 'F';
    } while(--num_nibbles);
//    HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
//    HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);

    if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer_print, uint8_data_number) != HAL_OK)
    {
      Error_Handler();
    }
//    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    {
    }
}

void print7_hex(int v, int num_places)
{
  const uint32_t uint8_data_number = (num_places % 4 == 0 ? num_places / 4 : num_places / 4 + 1);
//  uint8_t dataBuffer[uint8_data_number];
  while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
  {
  }
  int mask=0, n, num_nibbles, digit, nibbles_max;

    for (n=1; n<=num_places; n++)
    {
        mask = (mask << 1) | 0x0001;
    }
    v = v & mask; // truncate v to specified number of places

    num_nibbles = num_places / 4;
    if ((num_places % 4) != 0)
    {
        ++num_nibbles;
    }
    nibbles_max=num_nibbles;

    do
    {
        digit = ((v >> (num_nibbles-1) * 4)) & 0x0f;
        if(digit == 0)  dataBuffer_print7[ nibbles_max-num_nibbles ] = '0';
        if(digit == 1)  dataBuffer_print7[ nibbles_max-num_nibbles ] = '1';
        if(digit == 2)  dataBuffer_print7[ nibbles_max-num_nibbles ] = '2';
        if(digit == 3)  dataBuffer_print7[ nibbles_max-num_nibbles ] = '3';
        if(digit == 4)  dataBuffer_print7[ nibbles_max-num_nibbles ] = '4';
        if(digit == 5)  dataBuffer_print7[ nibbles_max-num_nibbles ] = '5';
        if(digit == 6)  dataBuffer_print7[ nibbles_max-num_nibbles ] = '6';
        if(digit == 7)  dataBuffer_print7[ nibbles_max-num_nibbles ] = '7';
        if(digit == 8)  dataBuffer_print7[ nibbles_max-num_nibbles ] = '8';
        if(digit == 9)  dataBuffer_print7[ nibbles_max-num_nibbles ] = '9';
        if(digit == 10) dataBuffer_print7[ nibbles_max-num_nibbles ] = 'A';
        if(digit == 11) dataBuffer_print7[ nibbles_max-num_nibbles ] = 'B';
        if(digit == 12) dataBuffer_print7[ nibbles_max-num_nibbles ] = 'C';
        if(digit == 13) dataBuffer_print7[ nibbles_max-num_nibbles ] = 'D';
        if(digit == 14) dataBuffer_print7[ nibbles_max-num_nibbles ] = 'E';
        if(digit == 15) dataBuffer_print7[ nibbles_max-num_nibbles ] = 'F';
    } while(--num_nibbles);
//    HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
//    HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);

    if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer_print7, uint8_data_number) != HAL_OK)
    {
      Error_Handler();
    }
//    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    {
    }
}

//void print2_hex(int v, int num_places)
//{
//  const uint32_t uint8_data_number = (num_places % 4 == 0 ? num_places / 4 : num_places / 4 + 1);
////  uint8_t dataBuffer[uint8_data_number];
//  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
//  {
//  }
//  int mask=0, n, num_nibbles, digit, nibbles_max;
//
//    for (n=1; n<=num_places; n++)
//    {
//        mask = (mask << 1) | 0x0001;
//    }
//    v = v & mask; // truncate v to specified number of places
//
//    num_nibbles = num_places / 4;
//    if ((num_places % 4) != 0)
//    {
//        ++num_nibbles;
//    }
//    nibbles_max=num_nibbles;
//
//    do
//    {
//        digit = ((v >> (num_nibbles-1) * 4)) & 0x0f;
//        if(digit == 0)  dataBuffer_print2[ nibbles_max-num_nibbles ] = '0';
//        if(digit == 1)  dataBuffer_print2[ nibbles_max-num_nibbles ] = '1';
//        if(digit == 2)  dataBuffer_print2[ nibbles_max-num_nibbles ] = '2';
//        if(digit == 3)  dataBuffer_print2[ nibbles_max-num_nibbles ] = '3';
//        if(digit == 4)  dataBuffer_print2[ nibbles_max-num_nibbles ] = '4';
//        if(digit == 5)  dataBuffer_print2[ nibbles_max-num_nibbles ] = '5';
//        if(digit == 6)  dataBuffer_print2[ nibbles_max-num_nibbles ] = '6';
//        if(digit == 7)  dataBuffer_print2[ nibbles_max-num_nibbles ] = '7';
//        if(digit == 8)  dataBuffer_print2[ nibbles_max-num_nibbles ] = '8';
//        if(digit == 9)  dataBuffer_print2[ nibbles_max-num_nibbles ] = '9';
//        if(digit == 10) dataBuffer_print2[ nibbles_max-num_nibbles ] = 'A';
//        if(digit == 11) dataBuffer_print2[ nibbles_max-num_nibbles ] = 'B';
//        if(digit == 12) dataBuffer_print2[ nibbles_max-num_nibbles ] = 'C';
//        if(digit == 13) dataBuffer_print2[ nibbles_max-num_nibbles ] = 'D';
//        if(digit == 14) dataBuffer_print2[ nibbles_max-num_nibbles ] = 'E';
//        if(digit == 15) dataBuffer_print2[ nibbles_max-num_nibbles ] = 'F';
//    } while(--num_nibbles);
////    HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
////    HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
//
//    if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer_print2, uint8_data_number) != HAL_OK)
//    {
//      Error_Handler();
//    }
////    while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
//    {
//    }
//}

void print_binary(int v, int num_places  )
{
  const uint32_t uint8_data_number = num_places;
//  uint8_t dataBuffer[uint8_data_number];
  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
  {
  }
    int mask=0, n;

    for (n=1; n<=num_places; n++)
    {
        mask = (mask << 1) | 0x0001;
    }
    v = v & mask;  // truncate v to specified number of places

    while(num_places)
    {

        if (v & (0x0001 << (num_places-1)))
        {
//             Serial.print("1");
             dataBuffer_print[ uint8_data_number-num_places ] = '1';
        }
        else
        {
//             Serial.print("0");
          dataBuffer_print[ uint8_data_number-num_places ] = '0';
         }

        --num_places;
        if(((num_places%4) == 0) && (num_places != 0))
        {
//            Serial.print("_");
        }
    }
//    HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
//    HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
    if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer_print, uint8_data_number) != HAL_OK)
    {
      Error_Handler();
    }
//    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    {
    }
}

void print7_binary(int v, int num_places  )
{
  const uint32_t uint8_data_number = num_places;
//  uint8_t dataBuffer[uint8_data_number];
  while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
  {
  }
    int mask=0, n;

    for (n=1; n<=num_places; n++)
    {
        mask = (mask << 1) | 0x0001;
    }
    v = v & mask;  // truncate v to specified number of places

    while(num_places)
    {

        if (v & (0x0001 << (num_places-1)))
        {
//             Serial.print("1");
             dataBuffer_print7[ uint8_data_number-num_places ] = '1';
        }
        else
        {
//             Serial.print("0");
          dataBuffer_print7[ uint8_data_number-num_places ] = '0';
         }

        --num_places;
        if(((num_places%4) == 0) && (num_places != 0))
        {
//            Serial.print("_");
        }
    }
//    HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
//    HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
    if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer_print7, uint8_data_number) != HAL_OK)
    {
      Error_Handler();
    }
//    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    {
    }
}

//void print2_binary(int v, int num_places  )
//{
//  const uint32_t uint8_data_number = num_places;
////  uint8_t dataBuffer[uint8_data_number];
//  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
//  {
//  }
//    int mask=0, n;
//
//    for (n=1; n<=num_places; n++)
//    {
//        mask = (mask << 1) | 0x0001;
//    }
//    v = v & mask;  // truncate v to specified number of places
//
//    while(num_places)
//    {
//
//        if (v & (0x0001 << (num_places-1)))
//        {
////             Serial.print("1");
//             dataBuffer_print2[ uint8_data_number-num_places ] = '1';
//        }
//        else
//        {
////             Serial.print("0");
//          dataBuffer_print2[ uint8_data_number-num_places ] = '0';
//         }
//
//        --num_places;
//        if(((num_places%4) == 0) && (num_places != 0))
//        {
////            Serial.print("_");
//        }
//    }
////    HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
////    HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
//    if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer_print2, uint8_data_number) != HAL_OK)
//    {
//      Error_Handler();
//    }
////    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
//    {
//    }
//}
void print_symbol(uint8_t v)
{
  const uint8_t uint8_data_number = 1;
//  uint8_t dataBuffer[uint8_data_number];
  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
  {
  }
  dataBuffer_print[ 0 ] = v;
//  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number);
//  HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
//  HAL_UART_Transmit_IT(&huart1, (uint8_t*)dataBuffer, uint8_data_number);
  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer_print, uint8_data_number) != HAL_OK)
  {
    Error_Handler();
  }
//  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
  {
  }
}

void print7_symbol(uint8_t v)
{
  const uint8_t uint8_data_number = 1;
//  uint8_t dataBuffer[uint8_data_number];
  while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
  {
  }
  dataBuffer_print7[ 0 ] = v;
//  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number);
//  HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
//  HAL_UART_Transmit_IT(&huart1, (uint8_t*)dataBuffer, uint8_data_number);
  if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer_print7, uint8_data_number) != HAL_OK)
  {
    Error_Handler();
  }
//  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
  {
  }
}

//void print2_symbol(uint8_t v)
//{
//  const uint8_t uint8_data_number = 1;
////  uint8_t dataBuffer[uint8_data_number];
//  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
//  {
//  }
//  dataBuffer_print2[ 0 ] = v;
////  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number);
////  HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
////  HAL_UART_Transmit_IT(&huart1, (uint8_t*)dataBuffer, uint8_data_number);
//  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer_print2, uint8_data_number) != HAL_OK)
//  {
//    Error_Handler();
//  }
////  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
//  {
//  }
//}
void print_text(const char * t)
{
  int text_length = strlen(t);
  const uint32_t uint8_data_number = text_length;
  uint8_t dataBuffer[uint8_data_number];
  for (int n=0; n<text_length; n++)
  {
    dataBuffer[n] = (uint8_t)(t[n]);
  }
//  HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
//  HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
//  HAL_UART_Transmit_IT(&huart1, (uint8_t*)dataBuffer, uint8_data_number);
  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
  {
  }
  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
  {
    Error_Handler();
  }
  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
  {
  }
}

void print7_text(const char * t)
{
  int text_length = strlen(t);
  const uint32_t uint8_data_number = text_length;
  uint8_t dataBuffer[uint8_data_number];
  for (int n=0; n<text_length; n++)
  {
    dataBuffer[n] = (uint8_t)(t[n]);
  }
//  HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
//  HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
//  HAL_UART_Transmit_IT(&huart1, (uint8_t*)dataBuffer, uint8_data_number);
  while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
  {
  }
  if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
  {
    Error_Handler();
  }
  while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
  {
  }
}

//void print2_text(const char * t)
//{
//  int text_length = strlen(t);
//  const uint32_t uint8_data_number = text_length;
//  uint8_t dataBuffer[uint8_data_number];
//  for (int n=0; n<text_length; n++)
//  {
//    dataBuffer[n] = (uint8_t)(t[n]);
//  }
////  HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
////  HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
////  HAL_UART_Transmit_IT(&huart1, (uint8_t*)dataBuffer, uint8_data_number);
//  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
//  {
//  }
//  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
//  {
//  }
//}
void print_line()
{
  const uint8_t uint8_data_number = 2;
  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
  {
  }
  dataBuffer_print[ 0 ] = '\r';
  dataBuffer_print[ 1 ] = '\n';
  //    HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
//      HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer_print, uint8_data_number) != HAL_OK)
  {
    Error_Handler();
  }
//  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
  {
  }
}

void print7_line()
{
  const uint8_t uint8_data_number = 2;
  while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
  {
  }
  dataBuffer_print7[ 0 ] = '\r';
  dataBuffer_print7[ 1 ] = '\n';
  //    HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
//      HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
  if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer_print7, uint8_data_number) != HAL_OK)
  {
    Error_Handler();
  }
//  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
  {
  }
}

//void print2_line()
//{
//  const uint8_t uint8_data_number = 2;
//  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
//  {
//  }
//  dataBuffer_print2[ 0 ] = '\r';
//  dataBuffer_print2[ 1 ] = '\n';
//  //    HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
////      HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
//  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer_print2, uint8_data_number) != HAL_OK)
//  {
//    Error_Handler();
//  }
////  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
//  {
//  }
//}
void print_text_line(const char * t)
{
  print_text(t);
  print_line();
}

void print7_text_line(const char * t)
{
  print7_text(t);
  print7_line();
}

uint8_t aui8_6b[6] = {0,1,2,3,4,5};
uint8_t aui8_7b[7] = {0,1,2,3,4,5,6};
uint8_t aui8_8b[8] = {0,1,2,3,4,5,6,7};
uint8_t aui8_8b_[8] = {1,2,3,4,5,6,7,8};

uint8_t crc8da_(uint8_t *data, uint8_t data_size)
{
//  Serial.println("");
//  for(int16_t byte_in_data = data_size - 1; byte_in_data >= 0; byte_in_data--)
//  {
//    print_binary(data[byte_in_data],8);    Serial.print(";");
//  }
//  Serial.println("");
  for(int16_t bit_in_data = data_size * 8 - 1; bit_in_data >= 7; bit_in_data--)
//  for(int16_t bit_in_data = data_size * 8 - 1; bit_in_data >= 8; bit_in_data--)
  {
//    uint16_t bit_in_leftmost_byte_in_data = bit_in_data / 8;
//    uint16_t leftmost_byte_in_data = bit_in_data / 8;
    if(data[bit_in_data / 8] & (1 << (bit_in_data % 8)))
    {
//      if(bit_in_data == 8)
      if(bit_in_data == 7)
      {
        if(data[0]<0x83)
        {
          return data[0];
        }
      }
      data[bit_in_data / 8] = data[bit_in_data / 8] ^ (1 << (bit_in_data % 8));
      data[(bit_in_data - 6) / 8] = data[(bit_in_data - 6) / 8] ^ (1 << ((bit_in_data - 6) % 8));
      data[(bit_in_data - 7) / 8] = data[(bit_in_data - 7) / 8] ^ (1 << ((bit_in_data - 7) % 8));
//      data[(bit_in_data - 8) / 8] = data[(bit_in_data - 8) / 8] ^ (1 << ((bit_in_data - 8) % 8));
    }
//    for(int16_t byte_in_data = data_size - 1; byte_in_data >= 0; byte_in_data--)
//    {
//      print_binary(data[byte_in_data],8);    Serial.print(";");
//    }
//    Serial.println("");
  }
  return data[0];
}

uint8_t crc8da(uint8_t *data, uint8_t data_size)
{
//  Serial.println("");
//  for(int16_t byte_in_data = 0; byte_in_data < data_size; byte_in_data++)
//  {
//    print_binary(data[byte_in_data],8);    Serial.print(";");
//  }
//  Serial.println("");
//  data[0] = data[0] ^ 0xff;
//  for(int16_t byte_in_data = 0; byte_in_data < data_size; byte_in_data++)
//  {
//    print_binary(data[byte_in_data],8);    Serial.print(";");
//  }
//  Serial.println("");
  for(int16_t bit_in_data = (data_size * 8 - 1); bit_in_data >= (9 - 1); bit_in_data--)
  {
//    uint16_t bit_in_leftmost_byte_in_data = bit_in_data / 8;
//    uint16_t leftmost_byte_in_data = bit_in_data / 8;
    if(data[((data_size * 8 - 1) - bit_in_data) / 8] & (1 << (bit_in_data % 8)))
    {
      if(bit_in_data == (9 - 1))
      {
        if(data[(data_size - 1)] < 0x07)
        {
          return data[(data_size - 1)];
        }
      }
      data[(((data_size * 8 - 1) - (bit_in_data - 0))) / 8] =
       data[(((data_size * 8 - 1) - (bit_in_data - 0))) / 8] ^ (1 << ((bit_in_data - 0) % 8));
      data[(((data_size * 8 - 1) - (bit_in_data - 6))) / 8] =
       data[(((data_size * 8 - 1) - (bit_in_data - 6))) / 8] ^ (1 << ((bit_in_data - 6) % 8));
      data[(((data_size * 8 - 1) - (bit_in_data - 7))) / 8] =
       data[(((data_size * 8 - 1) - (bit_in_data - 7))) / 8] ^ (1 << ((bit_in_data - 7) % 8));
      data[(((data_size * 8 - 1) - (bit_in_data - 8))) / 8] =
       data[(((data_size * 8 - 1) - (bit_in_data - 8))) / 8] ^ (1 << ((bit_in_data - 8) % 8));
    }
//    for(int16_t byte_in_data = 0; byte_in_data < data_size; byte_in_data++)
//    {
//      print_binary(data[byte_in_data],8);    Serial.print(";");
//    }
//    Serial.println("");
  }
  return data[(data_size - 1)];
}

uint8_t crc8da_7(const uint8_t *addr)
{
/*  aui8_7b[6]=((addr[4] & 0xf0) >> 4) | ((addr[0] & 0xf0));
  aui8_7b[5]=addr[7];
  aui8_7b[4]=addr[6];
  aui8_7b[3]=addr[5];
  aui8_7b[2]=addr[3];
  aui8_7b[1]=addr[2];
  aui8_7b[0]=addr[1];
  return crc8da(aui8_7b,7);*/

/*  aui8_7b[6]=addr[7];
  aui8_7b[5]=addr[6];
  aui8_7b[4]=addr[5];
  aui8_7b[3]=addr[3];
  aui8_7b[2]=addr[2];
  aui8_7b[1]=addr[1];
  aui8_7b[0]=0xff;
  return crc8da(aui8_7b,7);*/

/*  aui8_7b[6]=addr[7];
  aui8_7b[5]=addr[6];
  aui8_7b[4]=addr[5];
  aui8_7b[3]=((addr[4] & 0xf0) >> 4) | ((addr[3] & 0x0f) << 4);
  aui8_7b[2]=((addr[3] & 0xf0) >> 4) | ((addr[2] & 0x0f) << 4);
  aui8_7b[1]=((addr[2] & 0xf0) >> 4) | ((addr[1] & 0x0f) << 4);
  aui8_7b[0]=((addr[1] & 0xf0) >> 4) | ((addr[0] & 0xf0));
  return crc8da(aui8_7b,7);*/

  aui8_8b_[0]=((addr[0] & 0xf0)) | ((addr[1] & 0xf0) >> 4);
  aui8_8b_[0] = aui8_8b_[0] ^ 0xff;
  aui8_8b_[1]=((addr[1] & 0x0f) << 4) | ((addr[2] & 0xf0) >> 4);
  aui8_8b_[2]=((addr[2] & 0x0f) << 4) | ((addr[3] & 0xf0) >> 4);
  aui8_8b_[3]=((addr[3] & 0x0f) << 4) | ((addr[4] & 0xf0) >> 4);
  aui8_8b_[4]=addr[5];
  aui8_8b_[5]=addr[6];
  aui8_8b_[6]=addr[7];
  aui8_8b_[7]=0x00;
  return crc8da(aui8_8b_,8);
//  return crc8ccitt(aui8_8b_,8);


//  aui8_8b[7]=addr[7];
/*  aui8_7b[6]=addr[6];
  aui8_7b[5]=addr[5];
  aui8_7b[4]=((addr[4] & 0xf0) >> 4) | ((addr[3] & 0x0f) << 4);
  aui8_7b[3]=((addr[3] & 0xf0) >> 4) | ((addr[2] & 0x0f) << 4);
  aui8_7b[2]=((addr[2] & 0xf0) >> 4) | ((addr[1] & 0x0f) << 4);
  aui8_7b[1]=((addr[1] & 0xf0) >> 4) | ((addr[0] & 0xf0));
  aui8_7b[0]=0xff;
  return crc8da(aui8_7b,7);*/

/*  aui8_7b[6]=addr[7];
  aui8_7b[5]=addr[6];
  aui8_7b[4]=addr[5];
  aui8_7b[3]=((addr[4] & 0xf0) >> 4) | ((addr[3] & 0x0f) << 4);
  aui8_7b[2]=((addr[3] & 0xf0) >> 4) | ((addr[2] & 0x0f) << 4);
  aui8_7b[1]=((addr[2] & 0xf0) >> 4) | ((addr[1] & 0x0f) << 4);
  aui8_7b[0]=((addr[1] & 0xf0) >> 4) | ((addr[0] & 0xf0));
  return crc8da(aui8_7b,7);*/
}


///**
//  * @brief  This function Calculate FFT in Q15.
//  * @param  FFT Length : 1024, 256, 64
//  * @retval None
//  */
//void FFT_PROCESSING_Q15Process(uint32_t FFT_Length)
//{
//
//  arm_cfft_radix4_instance_q15  FFT_Q15_struct;
//
//  q15_t maxValue;    /* Max FFT value is stored here */
//  uint32_t maxIndex;    /* Index in Output array where max value is */
//
//  uint32_t index_fill_input_buffer, index_fill_output_buffer, index_fill_adc_buffer = 0;
//  uint32_t duration_us = 0x00;
//
//  for (index_fill_adc_buffer = 0; index_fill_adc_buffer < FFT_Length*2; index_fill_adc_buffer ++)
//  {
//
//    int crc_ok = 0;
////    while (!crc_ok)
//    {
//
//      int drdy_pin = HAL_GPIO_ReadPin(AD1_DRDY_GPIO_Port, AD1_DRDY_Pin);
//      while ((drdy_pin == 0))
//      {
//        drdy_pin = HAL_GPIO_ReadPin(AD1_DRDY_GPIO_Port, AD1_DRDY_Pin);
//      }
//
//      if(0)
//      {
////        HAL_SPI_Receive(&hspi1, data, uint8_ad_number, 5000);
//      }
//      else
//      {
//        if(HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)aRxBuffer, uint8_ad_number) != HAL_OK)
//  //        if(HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)data, uint8_ad_number) != HAL_OK)
//  //        if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, aRxBuffer, uint8_ad_number) != HAL_OK)
//        {
//          // Transfer error in transmission process
//          Error_Handler();
//        }
//
//
//        /*##-3- Wait for the end of the transfer ###################################*/
//        /*  Before starting a new communication transfer, you need to check the current
//            state of the peripheral; if it’s busy you need to wait for the end of current
//            transfer before starting a new one.
//            For simplicity reasons, this example is just waiting till the end of the
//            transfer, but application may perform other tasks while transfer operation
//            is ongoing. */
//        while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
//        {
//        }
//
//        for(int ad_data_byte = 0; ad_data_byte < uint8_ad_number; ad_data_byte ++)
//        {
////          data[ad_data_byte] = aRxBuffer[ad_data_byte];
//        }
//      }
//
////      crc8_01=crc8da_7(&(data[0]));
////
////      if(!(((uint8_t)crc8_01)-((uint8_t)((data[0]<<4)|(data[4]&0x0f)))))
////      if(
////        (((data[0]>=0b10000000)&&(data[0]<0b10010000))||((data[0]>=0b00000000)&&(data[0]<0b00010000)))
////      &&(((data[4]>=0b10010000)&&(data[4]<0b10100000))||((data[4]>=0b00010000)&&(data[4]<0b00100000)))
////      )
////      {
//////        c0++;
////        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
////        crc_ok = 1;
////      }
////      else
////      {
////        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
////      }
//
//    }
//
////    _uhADCxConvertedValue = data[1]<<16 | data[2]<<8 | data[3];
//
//    _aADC1ConvertedValue_s[index_fill_adc_buffer] = _uhADCxConvertedValue;
////    TIM2_Config();
//  }
//  for (index_fill_input_buffer = 0; index_fill_input_buffer < FFT_Length*2; index_fill_input_buffer += 2)
//  {
//    FFT_Input_Q15_f[(uint16_t)index_fill_input_buffer] = (float32_t)_uhADCxConvertedValue / (float32_t)4096.0;
//    /* Imaginary part */
//    FFT_Input_Q15_f[(uint16_t)(index_fill_input_buffer + 1)] = 0;
//
////    TIM2_Config();
//  }
//
//  arm_float_to_q15((float32_t *)&FFT_Input_Q15_f[0], (q15_t *)&aFFT_Input_Q15[0], FFT_Length*2);
//
//  /* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
//  arm_cfft_radix4_init_q15(&FFT_Q15_struct, FFT_Length, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);
//
////  TimerCount_Start();
//  arm_cfft_radix4_q15(&FFT_Q15_struct, aFFT_Input_Q15);
////  TimerCount_Stop(nb_cycles);
//
////  GUI_Clear();
////  LCD_OUTPUT_Cycles(5, 305, nb_cycles);
////  duration_us = (uint32_t)(((uint64_t)US_IN_SECOND * (nb_cycles)) / SystemCoreClock);
////  LCD_OUTPUT_DURATION(120, 305, duration_us);
//
//  /* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
//  arm_cmplx_mag_q15(aFFT_Input_Q15, FFT_Output_Q15, FFT_Length);
//
//  /* Calculates maxValue and returns corresponding value */
//  arm_max_q15(FFT_Output_Q15, FFT_Length, &maxValue, &maxIndex);
//  maxValue = 0;
//
//  if(0)
//  {
//    print_text("a_");
//    for (index_fill_input_buffer = 0; index_fill_input_buffer < FFT_Length*2; index_fill_input_buffer++)
//    {
//  //    GRAPH_DATA_YT_SetAlign(aGraph_Data[1], GRAPH_ALIGN_LEFT);
//  //    GRAPH_DATA_YT_MirrorX (aGraph_Data[1], 1);
//  //    GRAPH_DATA_YT_AddValue(aGraph_Data[1], aADC1ConvertedValue_s[index_fill_input_buffer] / 100 + 50);
//      print_hex(_aADC1ConvertedValue_s[index_fill_input_buffer] / 100 + 50, 8);
//      print_text(";");
//    }
//    print_line();
//    print_text("f_");
//  }
//  print_hex(uicount++,8);
//  print_text(":");
//  for (index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length; index_fill_output_buffer++)
//  {
////    GRAPH_DATA_YT_SetAlign(aGraph_Data[0], GRAPH_ALIGN_LEFT);
////    GRAPH_DATA_YT_MirrorX (aGraph_Data[0], 1);
////    GRAPH_DATA_YT_AddValue(aGraph_Data[0], FFT_Output_Q15[index_fill_output_buffer] / 50 + 10);
//    print_hex(FFT_Output_Q15[index_fill_output_buffer] / 50 + 10, 8);
//    print_text(";");
//  }
//  print_line();
//
//}
//
///**
//  * @brief  This function Calculate FFT in F32.
//  * @param  FFT Length : 1024, 256, 64
//  * @retval None
//  */
//void FFT_PROCESSING_F32Process(uint32_t FFT_Length)
//{
//  arm_cfft_radix4_instance_f32  FFT_F32_struct;
//
//  float32_t maxValue;    /* Max FFT value is stored here */
//  uint32_t maxIndex;    /* Index in Output array where max value is */
//
//  unsigned index_fill_input_buffer, index_fill_output_buffer, index_fill_adc_buffer = 0;
//  uint32_t duration_us = 0x00;
//
//  for (index_fill_adc_buffer = 0; index_fill_adc_buffer < FFT_Length*2; index_fill_adc_buffer ++)
//  {
//
//    int crc_ok = 0;
////    while (!crc_ok)
//    {
//
//      int drdy_pin = HAL_GPIO_ReadPin(AD1_DRDY_GPIO_Port, AD1_DRDY_Pin);
//      while ((drdy_pin == 0))
//      {
//        drdy_pin = HAL_GPIO_ReadPin(AD1_DRDY_GPIO_Port, AD1_DRDY_Pin);
//      }
//
//      if(0)
//      {
////        HAL_SPI_Receive(&hspi1, data, uint8_ad_number, 5000);
//      }
//      else
//      {
//        if(HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)aRxBuffer, uint8_ad_number) != HAL_OK)
//  //        if(HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)data, uint8_ad_number) != HAL_OK)
//  //        if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, aRxBuffer, uint8_ad_number) != HAL_OK)
//        {
//          // Transfer error in transmission process
//          Error_Handler();
//        }
//
//
//        /*##-3- Wait for the end of the transfer ###################################*/
//        /*  Before starting a new communication transfer, you need to check the current
//            state of the peripheral; if it’s busy you need to wait for the end of current
//            transfer before starting a new one.
//            For simplicity reasons, this example is just waiting till the end of the
//            transfer, but application may perform other tasks while transfer operation
//            is ongoing. */
//        while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
//        {
//        }
//
//        for(int ad_data_byte = 0; ad_data_byte < uint8_ad_number; ad_data_byte ++)
//        {
////          data[ad_data_byte] = aRxBuffer[ad_data_byte];
//        }
//      }
//
////      crc8_01=crc8da_7(&(data[0]));
////
////      if(!(((uint8_t)crc8_01)-((uint8_t)((data[0]<<4)|(data[4]&0x0f)))))
////      if(
////        (((data[0]>=0b10000000)&&(data[0]<0b10010000))||((data[0]>=0b00000000)&&(data[0]<0b00010000)))
////      &&(((data[4]>=0b10010000)&&(data[4]<0b10100000))||((data[4]>=0b00010000)&&(data[4]<0b00100000)))
////      )
////      {
//////        c0++;
////        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
////        crc_ok = 1;
////      }
////      else
////      {
////        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
////      }
//
//    }
//
////    _uhADCxConvertedValue = data[1]<<16 | data[2]<<8 | data[3];
//
//    _aADC1ConvertedValue_s[index_fill_adc_buffer] = _uhADCxConvertedValue;
/////    TIM2_Config();
//  }
//  for (index_fill_input_buffer = 0; index_fill_input_buffer < FFT_Length*2; index_fill_input_buffer += 2)
//  {
//    aFFT_Input_f32[(uint16_t)index_fill_input_buffer] = (float32_t)_uhADCxConvertedValue / (float32_t)4096.0;
//    /* Imaginary part */
//    aFFT_Input_f32[(uint16_t)(index_fill_input_buffer + 1)] = 0;
/////    TIM2_Config();
//  }
//  /* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
//  arm_cfft_radix4_init_f32(&FFT_F32_struct, FFT_Length, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);
//
////  TimerCount_Start();
//  arm_cfft_radix4_f32(&FFT_F32_struct, aFFT_Input_f32);
////  TimerCount_Stop(nb_cycles);
//
/////  GUI_Clear();
/////  LCD_OUTPUT_Cycles(5, 305, nb_cycles);
////  duration_us = (uint32_t)(((uint64_t)US_IN_SECOND * (nb_cycles)) / SystemCoreClock);
/////  LCD_OUTPUT_DURATION(120, 305, duration_us);
//
//  /* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
//  arm_cmplx_mag_f32(aFFT_Input_f32, aFFT_Output_f32, FFT_Length);
//
//  /* Calculates maxValue and returns corresponding value */
//  arm_max_f32(aFFT_Output_f32, FFT_Length, &maxValue, &maxIndex);
//  maxValue = 0;
//
//  if(0)
//  {
//    print_text("a_");
//    for (index_fill_input_buffer = 0; index_fill_input_buffer < FFT_Length*2; index_fill_input_buffer++)
//    {
//  ///    GRAPH_DATA_YT_SetAlign(aGraph_Data[1], GRAPH_ALIGN_LEFT);
//  ///    GRAPH_DATA_YT_MirrorX (aGraph_Data[1], 1);
//  ///    GRAPH_DATA_YT_AddValue(aGraph_Data[1], aADC1ConvertedValue_s[index_fill_input_buffer] / 50 + 50);
//      print_hex(_aADC1ConvertedValue_s[index_fill_input_buffer] / 50 + 50, 8);
//      print_text(";");
//    }
//    print_line();
//    print_text("f_");
//  }
//  print_hex(uicount++,8);
//  print_text(":");
//  for (index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length; index_fill_output_buffer++)
//  {
/////    GRAPH_DATA_YT_SetAlign(aGraph_Data[0], GRAPH_ALIGN_LEFT);
/////    GRAPH_DATA_YT_MirrorX (aGraph_Data[0], 1);
/////    GRAPH_DATA_YT_AddValue(aGraph_Data[0], aFFT_Output_f32[index_fill_output_buffer] + 10);
//    print_hex((int)aFFT_Output_f32[index_fill_output_buffer] + 10, 8);
//    print_text(";");
//  }
//  print_line();
//}
//
///**
//  * @brief  This function Calculate FFT in Q31.
//  * @param  FFT Length : 1024, 256, 64
//  * @retval None
//  */
//void FFT_PROCESSING_Q31Process(uint32_t FFT_Length)
//{
////  arm_cfft_radix4_instance_q31  FFT_Q31_struct;
//  arm_cfft_instance_q31  FFT_Q31_struct;
//
//  q31_t maxValue;    /* Max FFT value is stored here */
//  uint32_t maxIndex;    /* Index in Output array where max value is */
//
//  uint32_t index_fill_input_buffer, index_fill_output_buffer, index_fill_adc_buffer = 0;
//  uint32_t duration_us = 0x00;
//
//  if(last_index_fill_adc_buffer==FFT_Length*2)
//  {
//    last_index_fill_adc_buffer--;
//    for (index_fill_adc_buffer = 0; index_fill_adc_buffer < FFT_Length*2-1; index_fill_adc_buffer ++)
//    {
//      _aADC1ConvertedValue_s[index_fill_adc_buffer] = _aADC1ConvertedValue_s[index_fill_adc_buffer+1];
//    }
//  }
//
////  last_index_fill_adc_buffer = 0;
//
//  for (index_fill_adc_buffer = last_index_fill_adc_buffer; index_fill_adc_buffer < FFT_Length*2; index_fill_adc_buffer ++)
////  for (index_fill_adc_buffer = 0; index_fill_adc_buffer < FFT_Length*2; index_fill_adc_buffer ++)
//  {
//
////    int crc_ok = 0;
////    while (!crc_ok)
//    {
//
//
//      int drdy_pin = HAL_GPIO_ReadPin(AD1_DRDY_GPIO_Port, AD1_DRDY_Pin);
//      while ((drdy_pin == 0))
//      {
//        drdy_pin = HAL_GPIO_ReadPin(AD1_DRDY_GPIO_Port, AD1_DRDY_Pin);
//      }
////      int miso = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
////      if ((drdy_pin == 0))
//      {   // ADC Converter ready ?
////        if (miso == 0)
//        {   // ADC Converter ready ?
//
//          if(0)
//          {
////            HAL_SPI_Receive(&hspi1, data, uint8_ad_number, 5000);
//          }
//          else
//          {
////            if(HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)aRxBuffer, uint8_ad_number) != HAL_OK)
////            if(HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)data, uint8_ad_number) != HAL_OK)
//      //        if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, aRxBuffer, uint8_ad_number) != HAL_OK)
//            {
//              // Transfer error in transmission process
//              Error_Handler();
//            }
//
//
//            /*##-3- Wait for the end of the transfer ###################################*/
//            /*  Before starting a new communication transfer, you need to check the current
//                state of the peripheral; if it’s busy you need to wait for the end of current
//                transfer before starting a new one.
//                For simplicity reasons, this example is just waiting till the end of the
//                transfer, but application may perform other tasks while transfer operation
//                is ongoing. */
//            while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
//            {
//            }
//
////            for(int ad_data_byte = 0; ad_data_byte < uint8_ad_number; ad_data_byte ++)
////            {
////              data[ad_data_byte] = aRxBuffer[ad_data_byte];
////            }
//          }
//
////          crc8_01=crc8da_7(&(data[0]));
////
////          if(
////              ((!(((uint8_t)crc8_01)-((uint8_t)((data[0]<<4)|(data[4]&0x0f)))))
////             ||(!(((uint8_t)crc8_01)-((~((uint8_t)((data[0]<<4)|(data[4]&0x0f))))&0b00000111)))
////              )
////          &&(
////            (((data[0]>=0b10000000)&&(data[0]<0b10010000))||((data[0]>=0b00000000)&&(data[0]<0b00010000)))
////          &&(((data[4]>=0b10010000)&&(data[4]<0b10100000))||((data[4]>=0b00010000)&&(data[4]<0b00100000)))
////          ))
////          {
////            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
////            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
////          }
////          else
////          {
////            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
////            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
////          }
//          if(1)
//          {
//    //        c0++;
////            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
////            crc_ok = 1;
//          }
//          else
//          {
////            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
//          }
//
//        }
//      }
//    }
//
////    _uhADCxConvertedValue = data[1]<<16 | data[2]<<8 | data[3];
//
//    _aADC1ConvertedValue_s[index_fill_adc_buffer] = _uhADCxConvertedValue;
/////    TIM2_Config();
//  }
//  last_index_fill_adc_buffer=index_fill_adc_buffer;
//  for (index_fill_input_buffer = 0; index_fill_input_buffer < FFT_Length*2; index_fill_input_buffer += 2)
//  {
//    FFT_Input_Q31_f[(uint16_t)index_fill_input_buffer] = (float32_t)_uhADCxConvertedValue;// / (float32_t)4096.0;
//    /* Imaginary part */
//    FFT_Input_Q31_f[(uint16_t)(index_fill_input_buffer + 1)] = 0;
//
/////    TIM2_Config();
//  }
//
//  arm_float_to_q31((float32_t *)&FFT_Input_Q31_f[0], (q31_t *)&aFFT_Input_Q31[0], FFT_Length*2);
//
//  /* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
////  arm_cfft_radix4_init_q31(&FFT_Q31_struct, FFT_Length, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);
//  arm_cfft_q31(&FFT_Q31_struct, aFFT_Input_Q31, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);
//
////  TimerCount_Start();
////  arm_cfft_radix4_q31(&FFT_Q31_struct, aFFT_Input_Q31);
////  TimerCount_Stop(nb_cycles);
//
/////  GUI_Clear();
/////  LCD_OUTPUT_Cycles(5, 305, nb_cycles);
////  duration_us = (uint32_t)(((uint64_t)US_IN_SECOND * (nb_cycles)) / SystemCoreClock);
/////  LCD_OUTPUT_DURATION(120, 305, duration_us);
//
//  /* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
//  arm_cmplx_mag_q31(aFFT_Input_Q31, FFT_Output_Q31, FFT_Length);
//
//  /* Calculates maxValue and returns corresponding value */
//  arm_max_q31(FFT_Output_Q31, FFT_Length, &maxValue, &maxIndex);
//  maxValue = 0;
//
//  if(0)
//  {
//    print_text("a_");
//    for (index_fill_input_buffer = 0; index_fill_input_buffer < FFT_Length*2; index_fill_input_buffer++)
//    {
//  ///    GRAPH_DATA_YT_SetAlign(aGraph_Data[1], GRAPH_ALIGN_LEFT);
//  ///    GRAPH_DATA_YT_MirrorX (aGraph_Data[1], 1);
//  ///    GRAPH_DATA_YT_AddValue(aGraph_Data[1], aADC1ConvertedValue_s[index_fill_input_buffer] / 50 + 50);
//      print_hex(_aADC1ConvertedValue_s[index_fill_input_buffer] / 50 + 50, 24);
//      print_text(";");
//    }
//    print_line();
//    print_text("f_");
//  }
//
//  uicount++;
//  print_hex(uicount,8);
//  print_symbol(':');
//  print2_hex(uicount,8);
//  print2_symbol(':');
//  for (index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length; index_fill_output_buffer++)
////  for (index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length; index_fill_output_buffer++)
//  {
/////    GRAPH_DATA_YT_SetAlign(aGraph_Data[0], GRAPH_ALIGN_LEFT);
/////    GRAPH_DATA_YT_MirrorX (aGraph_Data[0], 1);
/////    GRAPH_DATA_YT_AddValue(aGraph_Data[0], FFT_Output_Q31[index_fill_output_buffer] / 5376212 + 10);
//    print_hex(FFT_Output_Q31[index_fill_output_buffer] / 5376212 + 10, 8);
//    print_symbol(':');
//    print2_hex(FFT_Output_Q31[index_fill_output_buffer] / 5376212 + 10, 8);
//    print2_symbol(':');
//  }
//  print_line();
//  print2_line();
//
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_SPI4_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_SAI1_Init();
  MX_SAI2_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI6_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USB_OTG_HS_PCD_Init();
  /* USER CODE BEGIN 2 */

  //  BSP_SD_Init();

    if((FREESMARTEEG_OUT & FREESMARTEEG_TEST_COUNTER)
        ||(FREESMARTEEG_OUT & FREESMARTEEG_TEST_BLINK))
    {
      uint32_t test_counter=0;
      uint32_t test_blink_counter=0;
      while (1)
      {
        if(FREESMARTEEG_OUT & FREESMARTEEG_TEST_COUNTER)
        {
          if(test_counter%1000000==0)
  //        if(test_counter%10000<5000)
          {
//            print_hex(test_counter, 32);
//            print_symbol(';');
            print7_hex(test_counter, 32);
            print7_symbol(';');
  //        print2_hex(test_counter, 32);
  //        print2_symbol(';');
          }
          test_counter++;
        }

//        if(FREESMARTEEG_OUT & FREESMARTEEG_TEST_BLINK)
//        {
//          if(test_blink_counter%1000000<50000)
//  //        if(test_blink_counter%1000000<500000)
//  //        if(test_blink_counter%1000000<500000)
//          {
//            HAL_GPIO_WritePin(led_ports[3], led_pins[3], GPIO_PIN_SET);
//  //          HAL_GPIO_WritePin(led_ports[0], led_pins[0], GPIO_PIN_SET);
//          }
//          else
//          {
//            HAL_GPIO_WritePin(led_ports[3], led_pins[3], GPIO_PIN_RESET);
//  //          HAL_GPIO_WritePin(led_ports[0], led_pins[0], GPIO_PIN_RESET);
//          }
//          test_blink_counter++;
//        }

      }
    }

    if((FREESMARTEEG_OUT & FREESMARTEEG_TEST_ACCGYR_SPI3))
    {
      uint32_t test_counter=0;
      uint32_t test_blink_counter=0;

//      {
//        aTxBuffer[0] = 0x10;//CTRL1_XL (10h)
//        aTxBuffer[1] = (0<<7)|(0<<6)|(0<<5)|(1<<4);//12.5 Hz (high performance)
////        aTxBuffer[1] = ((1<<3)|(0<<2)|(0<<1)|(0<<0))<<4;//1.66 kHz (high performance)
////        aTxBuffer[1] = (1<<3|0<<2|1<<1|0<<0)<<4;//6.66 kHz (high performance)
//        if(HAL_SPI_Transmit_DMA(&hspi3, aTxBuffer, 2) != HAL_OK)
////            if(HAL_SPI_TransmitReceive_DMA(&hspi3, aTxBuffer, datas[2], 2) != HAL_OK)
//        {
//          Error_Handler();
//        }
//        while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY)
//        {
//        }
//      }
//      {
//        aTxBuffer[0] = 0x11;//CTRL2_G (11h)
//        aTxBuffer[1] = (0<<7)|(0<<6)|(0<<5)|(1<<4);//12.5 Hz (high performance)
////        aTxBuffer[1] = (1<<3|0<<2|0<<1|0<<0)<<4;//1.66 kHz (high performance)
//        if(HAL_SPI_Transmit_DMA(&hspi3, aTxBuffer, 2) != HAL_OK)
////        if(HAL_SPI_TransmitReceive_DMA(&hspi3, aTxBuffer, datas[2], 2) != HAL_OK)
//        {
//          Error_Handler();
//        }
//        while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY)
//        {
//        }
//      }
//      {
//        aTxBuffer[0] = 0x13;//CTRL4_C (13h)
//        aTxBuffer[1] = ((0<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(1<<2)|(0<<1)|(0<<0));//I2C_disable
//        if(HAL_SPI_Transmit_DMA(&hspi3, aTxBuffer, 2) != HAL_OK)
////        if(HAL_SPI_TransmitReceive_DMA(&hspi3, aTxBuffer, datas[2], 2) != HAL_OK)
//        {
//          Error_Handler();
//        }
//        while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY)
//        {
//        }
//      }

      while (1)
      {
        if(FREESMARTEEG_OUT & FREESMARTEEG_TEST_ACCGYR_SPI3)
        {


//            {
//                aTxBuffer[0] = 0x80|0x0F;//WHO_AM_I (0F)
////                aTxBuffer[0] = (1<<7)|0x0F;//WHO_AM_I (0F)
//              aTxBuffer[1] = 0;
//              if(HAL_SPI_TransmitReceive(&hspi3, aTxBuffer, datas[2], 2,5000) != HAL_OK)
////              if(HAL_SPI_TransmitReceive_DMA(&hspi3, aTxBuffer, datas[2], 2) != HAL_OK)
//              {
//                Error_Handler();
//              }
//              while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY)
//              {
//              }
//            }

            print7_binary(aTxBuffer[0], 8);
//              print7_hex(aTxBuffer[0], 8);
            print7_symbol(';');
            print7_binary(aTxBuffer[1], 8);
//              print7_hex(aTxBuffer[1], 8);
            print7_symbol(';');

            print7_symbol(';');

            print7_binary(datas[2][0], 8);
//              print7_hex(datas[2][0], 8);
            print7_symbol(';');
            print7_binary(datas[2][1], 8);
//              print7_hex(datas[2][1], 8);
            print7_symbol(';');
//            print7_binary(datas[2][2], 8);
////              print7_hex(datas[2][2], 8);
//            print7_symbol(';');

            print7_symbol(';');
//              print7_line();


//              {
//                aTxBuffer[0] = 0x80|0x1E;//STATUS_REG (1Eh)
////                aTxBuffer[0] = (1<<7)|0x1E;//STATUS_REG (1Eh)
////                aTxBuffer[1] = 0;
//                if(HAL_SPI_TransmitReceive_DMA(&hspi3, aTxBuffer, datas[2], 2) != HAL_OK)
//                {
//                  Error_Handler();
//                }
//                while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY)
//                {
//                }
//              }
//
//              print7_binary(aTxBuffer[0], 8);
////              print7_hex(aTxBuffer[0], 8);
//              print7_symbol(';');
//              print7_binary(aTxBuffer[1], 8);
////              print7_hex(aTxBuffer[1], 8);
//              print7_symbol(';');
//
//              print7_symbol(';');
//
//              print7_binary(datas[2][0], 8);
////              print7_hex(datas[2][0], 8);
//              print7_symbol(';');
//              print7_binary(datas[2][1], 8);
////              print7_hex(datas[2][1], 8);
//              print7_symbol(';');
//              print7_symbol(';');
////              print7_line();
//
//
//
//              {
//                  aTxBuffer[0] = 0x80|0x22;//OUTX_L_G (22h)
////                  aTxBuffer[0] = (1<<7)|0x22;//OUTX_L_G (22h)
//                aTxBuffer[1] = 0;
////                if(HAL_SPI_TransmitReceive_DMA(&hspi3, aTxBuffer, datas[3], 2) != HAL_OK)
//                if(HAL_SPI_TransmitReceive_DMA(&hspi3, aTxBuffer, datas[3], 2*3*2) != HAL_OK)
//                {
//                  Error_Handler();
//                }
//                while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY)
//                {
//                }
//              }
//
//
//              for(int accgyr_data = 0; accgyr_data < 2*3*2; accgyr_data ++)
//              {
//
//            	  print7_binary(datas[3][accgyr_data], 8);
////              	print7_hex(datas[3][accgyr_data], 8);
//              	  print7_symbol(';');
//              }
//              print7_symbol(';');


              print7_line();

//          if(test_counter%1000000==0)
//  //        if(test_counter%10000<5000)
//          {
//            print_hex(test_counter, 32);
//            print_symbol(';');
//            print7_hex(test_counter, 32);
//            print7_symbol(';');
//  //        print2_hex(test_counter, 32);
//  //        print2_symbol(';');
//          }
//          test_counter++;
        }

      }
    }

    //START
//    HAL_GPIO_WritePin(AD1_START_GPIO_Port, AD1_START_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(AD2_START_GPIO_Port, AD2_START_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(AD3_START_GPIO_Port, AD3_START_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(AD4_START_GPIO_Port, AD4_START_Pin, GPIO_PIN_RESET);

    if(!(SPI_DMA))
    {
        HAL_GPIO_WritePin(SPI2_NSS_AD1_GPIO_Port, SPI2_NSS_AD1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SPI2_NSS_AD2_GPIO_Port, SPI2_NSS_AD2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SPI2_NSS_AD3_GPIO_Port, SPI2_NSS_AD3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SPI2_NSS_AD4_GPIO_Port, SPI2_NSS_AD4_Pin, GPIO_PIN_SET);
    }

    //RESET
    HAL_GPIO_WritePin(AD_RESET_GPIO_Port, AD_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(AD_RESET_GPIO_Port, AD_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    if(!(SPI_DMA))
    {
        HAL_GPIO_WritePin(SPI2_NSS_AD1_GPIO_Port, SPI2_NSS_AD1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SPI2_NSS_AD2_GPIO_Port, SPI2_NSS_AD2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SPI2_NSS_AD3_GPIO_Port, SPI2_NSS_AD3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SPI2_NSS_AD4_GPIO_Port, SPI2_NSS_AD4_Pin, GPIO_PIN_RESET);
    }

//    RESET
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
//    HAL_Delay(100);
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
//    HAL_Delay(100);

    if(!(SPI_DMA))
    {
        HAL_GPIO_WritePin(SPI2_NSS_AD1_GPIO_Port, SPI2_NSS_AD1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SPI2_NSS_AD2_GPIO_Port, SPI2_NSS_AD2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SPI2_NSS_AD3_GPIO_Port, SPI2_NSS_AD3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SPI2_NSS_AD4_GPIO_Port, SPI2_NSS_AD4_Pin, GPIO_PIN_SET);
    }

//    //RESET
//    HAL_GPIO_WritePin(AD1_RESET_GPIO_Port, AD1_RESET_Pin, GPIO_PIN_RESET);
//    HAL_Delay(1);
//    HAL_GPIO_WritePin(AD1_RESET_GPIO_Port, AD1_RESET_Pin, GPIO_PIN_SET);
//    //2RESET
//    HAL_GPIO_WritePin(AD2_RESET_GPIO_Port, AD2_RESET_Pin, GPIO_PIN_RESET);
//    HAL_Delay(1);
//    HAL_GPIO_WritePin(AD2_RESET_GPIO_Port, AD2_RESET_Pin, GPIO_PIN_SET);
//    //3RESET
//    HAL_GPIO_WritePin(AD3_RESET_GPIO_Port, AD3_RESET_Pin, GPIO_PIN_RESET);
//    HAL_Delay(1);
//    HAL_GPIO_WritePin(AD3_RESET_GPIO_Port, AD3_RESET_Pin, GPIO_PIN_SET);
//    //4RESET
//    HAL_GPIO_WritePin(AD4_RESET_GPIO_Port, AD4_RESET_Pin, GPIO_PIN_RESET);
//    HAL_Delay(1);
//    HAL_GPIO_WritePin(AD4_RESET_GPIO_Port, AD4_RESET_Pin, GPIO_PIN_SET);

  //  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
  //  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);

    //ESP RESET
  //  GPIO_InitTypeDef GPIO_InitStruct;
  //  GPIO_InitStruct.Pin = GPIO_PIN_11;
  //  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //  GPIO_InitStruct.Pull = GPIO_NOPULL;
  //  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  //  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //  HAL_Delay(1);
  //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  //  HAL_Delay(1);
  //  HAL_GPIO_WritePin(ESP_RESET_GPIO_Port, ESP_RESET_Pin, GPIO_PIN_RESET);
  //  HAL_Delay(1);
  //  HAL_GPIO_WritePin(ESP_RESET_GPIO_Port, ESP_RESET_Pin, GPIO_PIN_SET);
  //  HAL_Delay(1);
  //  GPIO_InitStruct.Pin = GPIO_PIN_11;
  //  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  //  GPIO_InitStruct.Pull = GPIO_NOPULL;
  //  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  //  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  //  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  //  if(1)
  //  {
  //    ad7779_init_param init_params[uint8_ad_adc_number];
//      ad7779_dev *devices[uint8_ad_adc_number];
      ad7779_dev *device1;
      ad7779_init_param init_param;
      uint8_t i;

  //    init_param.spi_dev = &hspi1;
//      init_param.spi_dev.dev = &hspi1;
//      init_param.spi_dev.dev = &hspi2;
  //    init_param.spi_dev.chip_select_port = AD_CS_GPIO_Port;
  //    init_param.spi_dev.chip_select_pin = AD_CS_Pin;
      init_param.ctrl_mode = AD7779_SPI_CTRL;

      init_param.spi_crc_en = AD7779_DISABLE;
  //    init_param.spi_crc_en = AD7779_ENABLE;

      if(FREESMARTEEG_OUT & FREESMARTEEG_TEXT)
      {
        init_param.spi_crc_en = AD7779_DISABLE;
      }

      for (i = AD7779_CH0; i <= AD7779_CH7; i++)
      {
//        init_param.state[i] = AD7779_DISABLE;
        init_param.state[i] = AD7779_ENABLE;
      }
//      init_param.state[0] = AD7779_DISABLE;
  //    init_param.state[1] = AD7779_DISABLE;
  //    init_param.state[2] = AD7779_DISABLE;
  //    init_param.state[0] = AD7779_ENABLE;
  //      init_param.state[1] = AD7779_ENABLE;
  //      init_param.state[2] = AD7779_ENABLE;
  //      init_param.state[3] = AD7779_ENABLE;
  //      init_param.state[4] = AD7779_ENABLE;
  //      init_param.state[1] = AD7779_ENABLE;

      for (i = AD7779_CH0; i <= AD7779_CH7; i++) {
//        init_param.gain[i] = AD7779_GAIN_1;
  //      init_param.gain[i] = AD7779_GAIN_2;
//        init_param.gain[i] = AD7779_GAIN_4;
        init_param.gain[i] = AD7779_GAIN_8;
      }

      init_param.pwr_mode = AD7779_HIGH_RES;
//  //    init_param.dec_rate_int = 0xfff;//hr 0.5001221 kHz
      init_param.dec_rate_int = 0xfa0;//hr 0.512 kHz
//      init_param.dec_rate_int = 0xc80;//hr 0.640 kHz
  //    init_param.dec_rate_int = 0xa00;//hr 0.800 kHz
//      init_param.dec_rate_int = 0x800;//hr 1 kHz
//      init_param.dec_rate_int = 0x400;//hr 2 kHz
//      init_param.dec_rate_int = 0x200;//hr 4 kHz
//      init_param.dec_rate_int = 0x100;//hr 8 kHz
//      init_param.dec_rate_int = 0x80;//hr 16 kHz
//          init_param.dec_rate_int = 0x40;//hr 32 kHz
//          init_param.dec_rate_int = 0x20;//hr 64 kHz
//          init_param.dec_rate_int = 0x10;//hr 128 kHz

//      init_param.pwr_mode = AD7779_LOW_PWR;
//      init_param.dec_rate_int = 0xfff;//lp 0.25006105 kHz
  //    init_param.dec_rate_int = 0xfa0;//lp 0.256 kHz
  //    init_param.dec_rate_int = 0xc80;//lp 0.320 kHz
  //    init_param.dec_rate_int = 0xa00;//lp 0.400 kHz
  //    init_param.dec_rate_int = 0x7d0;//lp 0.512 kHz
  //    init_param.dec_rate_int = 0x800;//lp 0.500 kHz
  //    init_param.dec_rate_int = 0x400;//lp 1 kHz
  //    init_param.dec_rate_int = 0x200;//lp 2 kHz
  //    init_param.dec_rate_int = 0x100;//lp 4 kHz
  //    init_param.dec_rate_int = 0x80;//lp 8 kHz
  //    init_param.dec_rate_int = 0x40;//lp 16 kHz

  //    init_param.dec_rate_int = 0x20;//lp 64 kHz

      init_param.dec_rate_dec = 0;
  //    init_param.dec_rate_dec = 0xfff;

      init_param.ref_type = AD7779_EXT_REF;
//      init_param.ref_type = AD7779_INT_REF;

  //    init_param.pwr_mode = AD7779_HIGH_RES;
  //    init_param.pwr_mode = AD7779_LOW_PWR;

      init_param.dclk_div = AD7779_DCLK_DIV_1;
//      init_param.dclk_div = AD7779_DCLK_DIV_2;
//      init_param.dclk_div = AD7779_DCLK_DIV_128;

      for (i = AD7779_CH0; i <= AD7779_CH7; i++) {
        init_param.sync_offset[i] = 0;
        init_param.offset_corr[i] = 0;
        init_param.gain_corr[i] = 0;
      }

  //    init_param.spi_dev.dev = &hspi3;
      init_param.spi_dev.dev = &hspi2;
//      init_param.spi_dev.dev = &hspi1;
      init_param.spi_dev.chip_select_port = SPI2_NSS_AD1_GPIO_Port;
      init_param.spi_dev.chip_select_pin = SPI2_NSS_AD1_Pin;
//      init_param.spi_dev.chip_select_port = SPI2_NSS_AD1_GPIO_Port;
//      init_param.spi_dev.chip_select_pin = SPI2_NSS_AD1_Pin;
  //    init_param.spi_dev.dev = &hspi3;
  //    init_param.spi_dev.chip_select_port = AD4_CS_GPIO_Port;
  //    init_param.spi_dev.chip_select_pin = AD4_CS_Pin;
  //    init_param.spi_dev.dev = &hspi3;
  //    init_param.spi_dev.chip_select_port = AD3_CS_GPIO_Port;
  //    init_param.spi_dev.chip_select_pin = AD3_CS_Pin;

  //    init_param.dec_rate_int = 0x1000;//hr 0.5 kHz
  //    init_param.dec_rate_int = 0x80;//hr 0.5 kHz
  //    init_param.dec_rate_int = 0x40;//hr 0.5 kHz
  //    init_param.dec_rate_int = 0x1000;//hr 0.5 kHz

  //    HAL_GPIO_WritePin(init_param.spi_dev.chip_select_port, init_param.spi_dev.chip_select_pin, GPIO_PIN_SET);

//      if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_DATA_READ)
//      {
//          HAL_Delay(1);
//          {
////            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//            {
//                aTxBuffer[0] = AD7779_REG_GENERAL_USER_CONFIG_1;
//                aTxBuffer[1] = AD7779_SOFT_RESET(3);
//                if(HAL_SPI_TransmitReceive_DMA(device1, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
////                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
//              {
//                Error_Handler();
//              }
//              while (HAL_SPI_GetState(device1) != HAL_SPI_STATE_READY)
//              {
//              }
//            }
//          }
//      }

//      ad7779_setup(&device1, init_param);
//
//      ad7779_spi_int_reg_write(device1,AD7779_REG_GENERAL_USER_CONFIG_1,AD7779_SOFT_RESET(3));
//      ad7779_spi_int_reg_write(device1,AD7779_REG_GENERAL_USER_CONFIG_1,AD7779_SOFT_RESET(2));

////      if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_DATA_READ)
//      {
//          HAL_Delay(1);
//          {
//            for(int to_reset = 0; to_reset < 64; to_reset ++)
//            {
//                aTxBuffer[0] = 0x80;
//                aTxBuffer[1] = 0x00;
//                if(HAL_SPI_TransmitReceive_DMA(&hspi2, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
////                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
//              {
//                Error_Handler();
//              }
//              while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
//              {
//              }
//            }
//          }
//      }
////      if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_DATA_READ)
//      {
//          HAL_Delay(1);
//          {
//            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//            {
//                aTxBuffer[0] = AD7779_REG_GENERAL_USER_CONFIG_1;
//                aTxBuffer[1] = AD7779_SOFT_RESET(2);
//                if(HAL_SPI_TransmitReceive_DMA(&hspi2, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
////                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
//              {
//                Error_Handler();
//              }
//              while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
//              {
//              }
//            }
//          }
//      }

//      print7_text_line(">>ad7779_setup(&device1, init_param);");

      ad7779_setup(&device1, init_param);

//      print7_text_line("<<ad7779_setup(&device1, init_param);");

      ad7779_spi_int_reg_write_mask(device1,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_SOURCE,AD7779_DISABLE);
      ad7779_spi_int_reg_write_mask(device1,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_UPDATE,AD7779_ENABLE);
      HAL_Delay(1);
      ad7779_spi_int_reg_write_mask(device1,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_UPDATE,AD7779_DISABLE);

      devices[0]=device1;

  //    ad7779_set_spi_op_mode(device, AD7779_SD_CONV);

      if(uint8_ad_adc_number >= 2)
      {
      ad7779_dev *device2;
      ad7779_init_param init_param2 = init_param;

  //    init_param2.spi_dev.dev = &hspi4;
      init_param2.spi_dev.dev = &hspi3;
//      init_param2.spi_dev.dev = &hspi2;
  //    init_param2.spi_dev.dev = &hspi1;
      init_param2.spi_dev.chip_select_port = SPI2_NSS_AD2_GPIO_Port;
      init_param2.spi_dev.chip_select_pin = SPI2_NSS_AD2_Pin;
  //    init_param2.spi_dev.dev = &hspi3;
  //    init_param2.spi_dev.chip_select_port = AD4_CS_GPIO_Port;
  //    init_param2.spi_dev.chip_select_pin = AD4_CS_Pin;
  //    init_param2.spi_dev.dev = &hspi3;
  //    init_param2.spi_dev.chip_select_port = AD3_CS_GPIO_Port;
  //    init_param2.spi_dev.chip_select_pin = AD3_CS_Pin;

  //    init_param2.dec_rate_int = 0x1000;//hr 0.5 kHz

  //    HAL_GPIO_WritePin(init_param2.spi_dev.chip_select_port, init_param2.spi_dev.chip_select_pin, GPIO_PIN_SET);

      ad7779_setup(&device2, init_param2);

      ad7779_spi_int_reg_write_mask(device2,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_SOURCE,AD7779_DISABLE);
      ad7779_spi_int_reg_write_mask(device2,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_UPDATE,AD7779_ENABLE);
      HAL_Delay(1);
      ad7779_spi_int_reg_write_mask(device2,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_UPDATE,AD7779_DISABLE);

      devices[1]=device2;
      }

      if(uint8_ad_adc_number >= 3)
      {
      ad7779_dev *device3;
      ad7779_init_param init_param3 = init_param;

//              init_param3.state[7] = AD7779_DISABLE;

  //    init_param3.spi_dev.dev = &hspi1;
//      init_param3.spi_dev.dev = &hspi2;
//      init_param3.spi_dev.dev = &hspi3;
      init_param3.spi_dev.dev = &hspi4;
      init_param3.spi_dev.chip_select_port = SPI2_NSS_AD3_GPIO_Port;
      init_param3.spi_dev.chip_select_pin = SPI2_NSS_AD3_Pin;
  //    init_param3.spi_dev.dev = &hspi3;
  //    init_param3.spi_dev.chip_select_port = AD4_CS_GPIO_Port;
  //    init_param3.spi_dev.chip_select_pin = AD4_CS_Pin;
  //    init_param3.spi_dev.dev = &hspi1;
  //    init_param3.spi_dev.chip_select_port = AD2_CS_GPIO_Port;
  //    init_param3.spi_dev.chip_select_pin = AD2_CS_Pin;
  //    init_param3.spi_dev.dev = &hspi1;
  //    init_param3.spi_dev.chip_select_port = AD_CS_GPIO_Port;
  //    init_param3.spi_dev.chip_select_pin = AD_CS_Pin;

  //    init_param3.dec_rate_int = 0x100;//hr 16 kHz
  //    init_param3.dec_rate_int = 0x1000;//hr 1 kHz
  //    init_param3.dec_rate_int = 0x800;//hr 1 kHz

  //    HAL_GPIO_WritePin(init_param3.spi_dev.chip_select_port, init_param3.spi_dev.chip_select_pin, GPIO_PIN_SET);

      ad7779_setup(&device3, init_param3);

      ad7779_spi_int_reg_write_mask(device3,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_SOURCE,AD7779_DISABLE);
      ad7779_spi_int_reg_write_mask(device3,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_UPDATE,AD7779_ENABLE);
      HAL_Delay(1);
      ad7779_spi_int_reg_write_mask(device3,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_UPDATE,AD7779_DISABLE);

      devices[2]=device3;
      }

      if(uint8_ad_adc_number >= 4)
      {
      ad7779_dev *device4;
      ad7779_init_param init_param4 = init_param;

  //    init_param4.spi_dev.dev = &hspi2;
//      init_param4.spi_dev.dev = &hspi4;
      init_param4.spi_dev.dev = &hspi2;
  //    init_param4.spi_dev.dev = &hspi3;
      init_param4.spi_dev.chip_select_port = SPI2_NSS_AD4_GPIO_Port;
      init_param4.spi_dev.chip_select_pin = SPI2_NSS_AD4_Pin;
  //    init_param4.spi_dev.dev = &hspi1;
  //    init_param4.spi_dev.chip_select_port = AD2_CS_GPIO_Port;
  //    init_param4.spi_dev.chip_select_pin = AD2_CS_Pin;
  //    init_param4.spi_dev.dev = &hspi1;
  //    init_param4.spi_dev.chip_select_port = AD_CS_GPIO_Port;
  //    init_param4.spi_dev.chip_select_pin = AD_CS_Pin;

  //    init_param4.dec_rate_int = 0x1000;//hr 1 kHz
  //    init_param4.dec_rate_int = 0x800;//hr 1 kHz

  //    HAL_GPIO_WritePin(init_param4.spi_dev.chip_select_port, init_param4.spi_dev.chip_select_pin, GPIO_PIN_SET);

      ad7779_setup(&device4, init_param4);

      ad7779_spi_int_reg_write_mask(device4,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_SOURCE,AD7779_DISABLE);
      ad7779_spi_int_reg_write_mask(device4,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_UPDATE,AD7779_ENABLE);
      HAL_Delay(1);
      ad7779_spi_int_reg_write_mask(device4,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_UPDATE,AD7779_DISABLE);

      devices[3]=device4;
      }

  //    init_params[0]=init_param;
  //    init_params[1]=init_param2;
  //    init_params[2]=init_param3;
  //    init_params[3]=init_param4;

  //    ad7779_set_spi_op_mode(device2, AD7779_SD_CONV);

  //    ad7779_set_spi_op_mode(0, AD7779_SD_CONV);
  //  }


  //    FATFS fileSystem;
  //    FIL testFile;
  //    FIL testFile2;
  ////    uint8_t testBuffer[160];
  ////    uint8_t testBuffer2[160];
  //    UINT testBytes;
  //    UINT testBytes2;
  //    FRESULT res;
  //    FRESULT res2;
  ////    uint8_t path[13];
  ////    uint8_t path2[13];
  //
  //    int usart1_cts_pin = 0;
  ////    int usart1_cts_pin = 1;
  //
  //
  //    if(FREESMARTEEG_OUT & FREESMARTEEG_SDCARD_FORMAT)
  //    {
  //      print_line();
  //
  //      FATFS fs;
  ////      uint32_t i;
  ////      char buffer[512];
  ////
  ////      print_text("disk_initialize");
  ////      print_line();
  ////
  ////      memset(buffer, 0, 512);
  ////      if( disk_initialize(0)==RES_OK)
  ////      {
  ////        print_text("disk_initialize -- OK");
  ////        print_line();
  ////      }
  //////      f_mount(0, &fs);
  ////
  ////      print_text("f_mount");
  ////      print_line();
  ////
  ////      f_mount(&fs, "", 0);
  ////
  ////      print_text("disk_write");
  ////      print_line();
  ////
  ////      for (i = fs.fatbase; i < fs.database; i++) {
  ////        disk_write(0, buffer, i, 1);
  ////      }
  //
  ////          FATFS fs;           /* Filesystem object */
  //          FIL fil;            /* File object */
  //          FRESULT res;        /* API result code */
  //          UINT bw;            /* Bytes written */
  //#define FF_MAX_SS (8*1024)
  //          BYTE work[FF_MAX_SS]; /* Work area (larger is better for processing time) */
  ////          BYTE work[_MAX_SS]; /* Work area (larger is better for processing time) */
  //
  ////          print_text("LinkDriver");
  ////          print_line();
  ////
  ////          if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)//we call FATFS_LinkDriver directly to initialize the fatfs
  ////          {
  ////            print_text("LinkDriver: OK");
  ////            print_line();
  ////
  ////          }
  //
  //          print_text("Create FAT volume");
  //          print_line();
  //
  //          /* Create FAT volume */
  ////          res = f_mkfs(SDPath, FM_FAT32, 512, work, sizeof work);
  ////          res = f_mkfs(SDPath, FM_FAT32, 4096, work, sizeof work);
  //          res = f_mkfs(SDPath, FM_EXFAT, 4096, work, sizeof work);
  //          if (res == FR_OK)
  //          {
  //            print_text("Create FAT volume: OK");
  //            print_line();
  //          }
  //          else
  //          {
  //            print_text("Create FAT volume: error = 0x");
  //            print_hex(res,8);
  //            print_line();
  //
  //          }
  //
  //          print_text("Register work area");
  //          print_line();
  //
  //          /* Register work area */
  //          f_mount(&fs, "", 0);
  //
  //          print_text("Create a file as new");
  //          print_line();
  //
  //          /* Create a file as new */
  //          res = f_open(&fil, "hello.txt", FA_CREATE_NEW | FA_WRITE);
  //          if (res == FR_OK)
  //          {
  //            print_text("Create a file as new -- OK");
  //            print_line();
  //          }
  //
  //          print_text("Write a message");
  //          print_line();
  //
  //          /* Write a message */
  //          f_write(&fil, "Hello, World!\r\n", 15, &bw);
  //          if (bw != 15)
  //          {
  //            print_text("Write a message -- FAIL");
  //            print_line();
  //          }
  //
  //          print_text("Close the file");
  //          print_line();
  //
  //          /* Close the file */
  //          f_close(&fil);
  //
  //          print_text("Unregister work area");
  //          print_line();
  //
  //          /* Unregister work area */
  //          f_mount(0, "", 0);
  //
  //    }
  //
  //    if(FREESMARTEEG_OUT & FREESMARTEEG_WRITE_SDCARD_OPENVIBE)
  //    {
  //      usart1_cts_pin = HAL_GPIO_ReadPin(USART1_CTS_GPIO_Port, USART1_CTS_Pin);
  //
  //    }
  //
  //    if((FREESMARTEEG_OUT & FREESMARTEEG_WRITE_SDCARD)
  //        && (usart1_cts_pin || !(FREESMARTEEG_OUT & FREESMARTEEG_WRITE_SDCARD_OPENVIBE)))
  //    {
  ////      rtc_read_frac_s(&time1, &frac_secs1);
  //          print_line();
  //
  //          print_text("f_mount");
  //          print_line();
  //      if(f_mount(&fileSystem, SDPath, 1) == FR_OK)
  //      {
  //        print_text("f_mount: OK");
  //        print_line();
  //                uint8_t path[13] = "testfi10.fe3";
  ////        path = "testfil1.fe3";
  //        path[12] = '\0';
  //
  //        print_text("Create a file as new ");
  //        print_text("'");
  //        print_text(path);
  //        print_text("'");
  //        print_line();
  //        res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
  ////          res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
  ////        res = f_open(&testFile, (char*)path, FA_READ | FA_OPEN_EXISTING);
  //          if (res == FR_OK)
  //          {
  //            print_text("Create a file as new: OK");
  //            print_line();
  //          }
  ////          res = f_lseek(&testFile, 5*1024*1024*1024);
  //          FSIZE_t filesize = 5*1024*1024*1024;
  //          print_text("Expand a file with 0x");
  //          print_hex(filesize,32);
  //          print_text(" bytes");
  //          print_line();
  //          res = f_expand(&testFile, filesize, 1);
  //          if (res == FR_OK)
  //          {
  //            print_text("Expand a file: OK");
  //            print_line();
  //          }
  ////          res = f_close(&testFile);
  ////          res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
  ////          res = f_lseek(&testFile, 0);
  //      }
  //    }
  //
  //    if((FREESMARTEEG_OUT & FREESMARTEEG_SDCARD_OPENVIBE)
  //        && ((!usart1_cts_pin) || !(FREESMARTEEG_OUT & FREESMARTEEG_WRITE_SDCARD_OPENVIBE)))
  //    {
  //
  //      if(f_mount(&fileSystem, SDPath, 1) == FR_OK)
  //      {
  //                uint8_t path[13] = "testfi10.fe3";
  ////        path = "testfil1.fe3";
  //        path[12] = '\0';
  //
  //  //      res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
  ////                res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
  //        res = f_open(&testFile, (char*)path, FA_READ | FA_OPEN_EXISTING);
  //      }
  //    }
  //
  //    if(FREESMARTEEG_OUT & FREESMARTEEG_SDCARD_TEXT)
  //    {
  //      if(f_mount(&fileSystem, SDPath, 1) == FR_OK)
  //      {
  //                uint8_t path[13] = "testfi10.fe3";
  ////        path = "testfil1.fe3";
  //        path[12] = '\0';
  //
  //  //      res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
  ////                res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
  //        res = f_open(&testFile, (char*)path, FA_READ | FA_OPEN_EXISTING);
  //      }
  //    }
  //
  //   if(FREESMARTEEG_OUT & FREESMARTEEG_TEST_SDCARD)
  //   {
  //
  ////    FATFS fileSystem;
  ////    FIL testFile;
  ////    FIL testFile2;
  //     uint8_t testBuffer[160] = "SD write\n";
  //     uint8_t testBuffer2[160] = "";
  ////     testBuffer = "SD write\n";
  ////     testBuffer2 = "";
  ////    UINT testBytes;
  ////    UINT testBytes2;
  ////    FRESULT res;
  ////    FRESULT res2;
  //
  //    print_text_line("<begin>");
  ////    print_line();
  //
  //    if(f_mount(&fileSystem, SDPath, 1) == FR_OK)
  //    {
  //      uint8_t path[13] = "testfil3.txt";
  ////      path = "testfil3.txt";
  //      path[12] = '\0';
  //
  ////      res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
  //      res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
  ////      res = f_open(&testFile, (char*)path, FA_READ | FA_OPEN_EXISTING);
  //
  //      res = f_lseek(&testFile, f_size(&testFile));
  //      res = f_write(&testFile, testBuffer, 16, &testBytes);
  ////      res = f_read(&testFile, testBuffer, 16, &testBytes);
  //
  //      res = f_close(&testFile);
  //      print_text_line(testBuffer);
  //    }
  //
  //    print_text_line("<1>");
  ////    print_line();
  //
  //    if(f_mount(&fileSystem, SDPath, 1) == FR_OK)
  //    {
  //      uint8_t path2[13] = "testfil3.txt";
  ////      path2 = "testfil3.txt";
  //      path2[12] = '\0';
  //
  ////      res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
  ////      res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
  //      res2 = f_open(&testFile2, (char*)path2, FA_READ | FA_OPEN_EXISTING);
  //
  ////      res = f_write(&testFile, testBuffer, 16, &testBytes);
  //      res2 = f_read(&testFile2, testBuffer2, 160, &testBytes2);
  //
  //      res2 = f_close(&testFile2);
  //      print_text_line(testBuffer2);
  //    }
  //
  //    print_text_line("<end>");
  ////    print_line();
  //   }
  //
  //    if((FREESMARTEEG_OUT & FREESMARTEEG_WRITE_SDCARD))// && usart1_cts_pin)
  //    {
  ////      rtc_read_frac_s(&time1, &frac_secs1);
  //      if(f_mount(&fileSystem, SDPath, 1) == FR_OK)
  //      {
  //                uint8_t path[13] = "testfil7.fe3";
  ////        path = "testfil1.fe3";
  //        path[12] = '\0';
  //
  ////        res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
  //          res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
  ////        res = f_open(&testFile, (char*)path, FA_READ | FA_OPEN_EXISTING);
  //          res = f_lseek(&testFile, 5*1024*1024*1024);
  //          res = f_close(&testFile);
  //          res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
  ////          res = f_lseek(&testFile, 0);
  //      }
  //    }
  //
  //    if((FREESMARTEEG_OUT & FREESMARTEEG_SDCARD_OPENVIBE))// && !usart1_cts_pin)
  //    {
  //      if(f_mount(&fileSystem, SDPath, 1) == FR_OK)
  //      {
  //                uint8_t path[13] = "testfil7.fe3";
  ////        path = "testfil1.fe3";
  //        path[12] = '\0';
  //
  //  //      res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
  ////                res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
  //        res = f_open(&testFile, (char*)path, FA_READ | FA_OPEN_EXISTING);
  //      }
  //    }
  //
  //    if(FREESMARTEEG_OUT & FREESMARTEEG_SDCARD_TEXT)
  //    {
  //      if(f_mount(&fileSystem, SDPath, 1) == FR_OK)
  //      {
  //                uint8_t path[13] = "testfil5.fe3";
  ////        path = "testfil1.fe3";
  //        path[12] = '\0';
  //
  //  //      res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
  ////                res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
  //        res = f_open(&testFile, (char*)path, FA_READ | FA_OPEN_EXISTING);
  //      }
  //    }
  //
  //   if(FREESMARTEEG_OUT & FREESMARTEEG_TEST_SDCARD)
  //   {
  //
  ////    FATFS fileSystem;
  ////    FIL testFile;
  ////    FIL testFile2;
  //     uint8_t testBuffer[160] = "SD write\n";
  //     uint8_t testBuffer2[160] = "";
  ////     testBuffer = "SD write\n";
  ////     testBuffer2 = "";
  ////    UINT testBytes;
  ////    UINT testBytes2;
  ////    FRESULT res;
  ////    FRESULT res2;
  //
  //    print_text_line("<begin>");
  ////    print_line();
  //
  //    if(f_mount(&fileSystem, SDPath, 1) == FR_OK)
  //    {
  //      uint8_t path[13] = "testfil3.txt";
  ////      path = "testfil3.txt";
  //      path[12] = '\0';
  //
  ////      res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
  //      res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
  ////      res = f_open(&testFile, (char*)path, FA_READ | FA_OPEN_EXISTING);
  //
  //      res = f_lseek(&testFile, f_size(&testFile));
  //      res = f_write(&testFile, testBuffer, 16, &testBytes);
  ////      res = f_read(&testFile, testBuffer, 16, &testBytes);
  //
  //      res = f_close(&testFile);
  //      print_text_line(testBuffer);
  //    }
  //
  //    print_text_line("<1>");
  ////    print_line();
  //
  //    if(f_mount(&fileSystem, SDPath, 1) == FR_OK)
  //    {
  //      uint8_t path2[13] = "testfil3.txt";
  ////      path2 = "testfil3.txt";
  //      path2[12] = '\0';
  //
  ////      res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
  ////      res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
  //      res2 = f_open(&testFile2, (char*)path2, FA_READ | FA_OPEN_EXISTING);
  //
  ////      res = f_write(&testFile, testBuffer, 16, &testBytes);
  //      res2 = f_read(&testFile2, testBuffer2, 160, &testBytes2);
  //
  //      res2 = f_close(&testFile2);
  //      print_text_line(testBuffer2);
  //    }
  //
  //    print_text_line("<end>");
  ////    print_line();
  //   }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      //  for(int ad_data_byte = 0; ad_data_byte < 100; ad_data_byte ++)
      if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_DATA_READ)
      {
          HAL_Delay(1);
          {
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
                aTxBuffer[0] = AD7779_REG_DOUT_FORMAT;
                aTxBuffer[1] = AD7779_DOUT_FORMAT(3);
                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
//                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
              {
                Error_Handler();
              }
              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }
            }
          }
      }
      if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_DATA_READ_INT)
      {
          HAL_Delay(1);
          {
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
                aTxBuffer[0] = AD7779_REG_DOUT_FORMAT;
                aTxBuffer[1] = AD7779_DOUT_FORMAT(3);
                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
//                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
              {
                Error_Handler();
              }
              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }
            }
          }
      }
      if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_SAI_READ_INT)
      {
          HAL_Delay(100);
          {
//              print7_text_line(">>AD7779_REG_DOUT_FORMAT");
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
                aTxBuffer[0] = AD7779_REG_DOUT_FORMAT;
//                aTxBuffer[1] = AD7779_DOUT_FORMAT(0);//4 DOUTx lines
//                aTxBuffer[1] = AD7779_DOUT_FORMAT(1);//2 DOUTx lines
                aTxBuffer[1] = AD7779_DOUT_FORMAT(2);//1 DOUTx lines
//                aTxBuffer[1] = AD7779_DOUT_FORMAT(3);//1 DOUTx lines

                if(SPI_DMA)
                {
                    if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
                    {
                      Error_Handler();
                    }
                } else {
                    HAL_GPIO_WritePin(devices[ad_adc]->spi_dev.chip_select_port, devices[ad_adc]->spi_dev.chip_select_pin, GPIO_PIN_RESET);
                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
                    {
                      Error_Handler();
                    }
                    HAL_GPIO_WritePin(devices[ad_adc]->spi_dev.chip_select_port, devices[ad_adc]->spi_dev.chip_select_pin, GPIO_PIN_SET);
                }

                while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }
            }
//            print7_text_line("<<AD7779_REG_DOUT_FORMAT");
//            print7_text_line(">>AD7779_REG_GENERAL_USER_CONFIG_3");
            for(int ad_adc = uint8_ad_adc_number-1; ad_adc >= 0; ad_adc --)
//            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
//            	int ad_adc = 0;
                aTxBuffer[0] = AD7779_REG_GENERAL_USER_CONFIG_2;
                aTxBuffer[1] = AD7779_SDO_DRIVE_STR(1) | AD7779_DOUT_DRIVE_STR(0) | AD7779_SPI_SYNC;
//                aTxBuffer[1] = AD7779_SDO_DRIVE_STR(1) | AD7779_DOUT_DRIVE_STR(0);
//                aTxBuffer[1] = AD7771_FILTER_MODE | AD7779_SDO_DRIVE_STR(1) | AD7779_DOUT_DRIVE_STR(1) | AD7779_SPI_SYNC;
//                aTxBuffer[1] = AD7771_FILTER_MODE | AD7779_SDO_DRIVE_STR(1) | AD7779_DOUT_DRIVE_STR(0) | AD7779_SPI_SYNC;

                if(SPI_DMA)
                {
                    if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
                    {
                      Error_Handler();
                    }
                } else {
                    HAL_GPIO_WritePin(devices[ad_adc]->spi_dev.chip_select_port, devices[ad_adc]->spi_dev.chip_select_pin, GPIO_PIN_RESET);
                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
                    {
                      Error_Handler();
                    }
                    HAL_GPIO_WritePin(devices[ad_adc]->spi_dev.chip_select_port, devices[ad_adc]->spi_dev.chip_select_pin, GPIO_PIN_SET);
                }

              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }
                HAL_Delay(100);
            }
//            print7_text_line("<<AD7779_REG_GENERAL_USER_CONFIG_3");
          }
      }
      if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_SAI_READ)
      {
          HAL_Delay(1);
          {
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
                aTxBuffer[0] = AD7779_REG_DOUT_FORMAT;
                aTxBuffer[1] = AD7779_DOUT_FORMAT(3);
                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
//                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
              {
                Error_Handler();
              }
              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }
            }
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
                aTxBuffer[0] = AD7779_REG_GENERAL_USER_CONFIG_2;
                aTxBuffer[1] = AD7771_FILTER_MODE | AD7779_SDO_DRIVE_STR(1) |
                		AD7779_DOUT_DRIVE_STR(0) | AD7779_SPI_SYNC;
                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
//                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
              {
                Error_Handler();
              }
              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }
            }
          }
      }
      if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_SAI_READ_TIMER)
      {
          HAL_Delay(1);
          {
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
                aTxBuffer[0] = AD7779_REG_DOUT_FORMAT;
                aTxBuffer[1] = AD7779_DOUT_FORMAT(3);
                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
//                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
              {
                Error_Handler();
              }
              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }
            }
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
                aTxBuffer[0] = AD7779_REG_GENERAL_USER_CONFIG_2;
                aTxBuffer[1] = AD7771_FILTER_MODE | AD7779_SDO_DRIVE_STR(1) |
                		AD7779_DOUT_DRIVE_STR(1) | AD7779_SPI_SYNC;
                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
//                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
              {
                Error_Handler();
              }
              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }
            }
          }
      }
      if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_I2S_READ_INT)
      {
          HAL_Delay(1);
          {
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
                aTxBuffer[0] = AD7779_REG_DOUT_FORMAT;
                aTxBuffer[1] = AD7779_DOUT_FORMAT(3);
                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
//                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
              {
                Error_Handler();
              }
              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }
            }
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
                aTxBuffer[0] = AD7779_REG_GENERAL_USER_CONFIG_2;
                aTxBuffer[1] = AD7771_FILTER_MODE | AD7779_SDO_DRIVE_STR(1) |
                		AD7779_DOUT_DRIVE_STR(0) | AD7779_SPI_SYNC;
                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
//                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
              {
                Error_Handler();
              }
              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }
            }
          }
      }
      if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_I2S_READ)
      {
          HAL_Delay(1);
          {
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
                aTxBuffer[0] = AD7779_REG_DOUT_FORMAT;
                aTxBuffer[1] = AD7779_DOUT_FORMAT(3);
                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
//                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
              {
                Error_Handler();
              }
              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }
            }
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
                aTxBuffer[0] = AD7779_REG_GENERAL_USER_CONFIG_2;
                aTxBuffer[1] = AD7771_FILTER_MODE | AD7779_SDO_DRIVE_STR(1) |
                		AD7779_DOUT_DRIVE_STR(0) | AD7779_SPI_SYNC;
                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
//                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
              {
                Error_Handler();
              }
              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }
            }
          }
      }

      if(0)
      if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_DATA_READ)
      {
          HAL_Delay(1);
//          aTxBuffer[0] = AD7779_REG_CH_CONFIG(0);
//          aTxBuffer[1] = AD7779_CH_GAIN(3);
//          aTxBuffer[1] = AD7779_DOUT_FORMAT(3)|AD7779_DCLK_CLK_DIV(7);
        //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        //  HAL_SPI_Transmit(&hspi1, data, 2, 5000);
    //      if(HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)data, 2) != HAL_OK)
    //      if(SPI_DMA)
    //      {
    //        if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, data, 2) != HAL_OK)
    //        {
    //          Error_Handler();
    //        }
    //      }
    //      else
          {
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//                if(ad_adc != 0)
//                    if(ad_adc != 1)
//              if(ad_adc != 2)
                for(int ad_chan = 0; ad_chan < uint8_ad_chan_number; ad_chan ++)
            {
                aTxBuffer[0] = AD7779_REG_CH_CONFIG(ad_chan);
                aTxBuffer[1] = AD7779_CH_GAIN(3);

    //          HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_RESET);
                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
//                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
    //          if(HAL_SPI_TransmitReceive(device2->spi_dev.dev, aTxBuffer, data2, 2, 5000) != HAL_OK)
              {
                Error_Handler();
              }
    //          HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_SET);

              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }

            }

          }
        //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
      }
      HAL_Delay(1);
      if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_SPI_READ_INT)
      {
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//                for(int ad_adc = uint8_ad_adc_number-1; ad_adc >= 0; ad_adc --)
            {
                aTxBuffer[0] = AD7779_REG_GENERAL_USER_CONFIG_2;
                aTxBuffer[1] = AD7779_SDO_DRIVE_STR(1) | AD7779_DOUT_DRIVE_STR(0) | AD7779_SPI_SYNC;
//                aTxBuffer[1] = AD7779_SDO_DRIVE_STR(3) | AD7779_DOUT_DRIVE_STR(0) | AD7779_SPI_SYNC;
//                aTxBuffer[1] = AD7771_FILTER_MODE | AD7779_SDO_DRIVE_STR(3) |
//                		AD7779_DOUT_DRIVE_STR(0) | AD7779_SPI_SYNC;
                if(SPI_DMA)
                {
                    if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
                    {
                      Error_Handler();
                    }
                } else {
                    HAL_GPIO_WritePin(devices[ad_adc]->spi_dev.chip_select_port, devices[ad_adc]->spi_dev.chip_select_pin, GPIO_PIN_RESET);
                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
                    {
                      Error_Handler();
                    }
                    HAL_GPIO_WritePin(devices[ad_adc]->spi_dev.chip_select_port, devices[ad_adc]->spi_dev.chip_select_pin, GPIO_PIN_SET);
                }
              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }
            }
            HAL_Delay(1);
              for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//            	  if(ad_adc != 2)
//                	  if(ad_adc != 1)
              {
                aTxBuffer[0] = AD7779_REG_GENERAL_USER_CONFIG_3;
                aTxBuffer[1] = AD7779_SPI_SLAVE_MODE_EN;

                if(SPI_DMA)
                {
                    if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
                    {
                      Error_Handler();
                    }
                } else {
                    HAL_GPIO_WritePin(devices[ad_adc]->spi_dev.chip_select_port, devices[ad_adc]->spi_dev.chip_select_pin, GPIO_PIN_RESET);
                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2, 5000) != HAL_OK)
                    {
                      Error_Handler();
                    }
                    HAL_GPIO_WritePin(devices[ad_adc]->spi_dev.chip_select_port, devices[ad_adc]->spi_dev.chip_select_pin, GPIO_PIN_SET);
                }

                while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
                {
                }
              }

      }

//          HAL_Delay(1);
          if(0)
      if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_SPI_READ)
      {
          aTxBuffer[0] = AD7779_REG_GEN_ERR_REG_1_EN;
          aTxBuffer[1] = AD7779_SPI_CRC_TEST_EN;
        //  HAL_SPI_Transmit(&hspi1, data, 2, 5000);
    //      if(HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)data, 2) != HAL_OK)
    //      if(SPI_DMA)
    //      {
    //        if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, data, 2) != HAL_OK)
    //        {
    //          Error_Handler();
    //        }
    //      }
    //      else
          {
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
              aTxBuffer[0] = AD7779_REG_GEN_ERR_REG_1_EN;
              aTxBuffer[1] = AD7779_SPI_CRC_TEST_EN;

    //          HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_RESET);
              if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
    //          if(HAL_SPI_TransmitReceive(device2->spi_dev.dev, aTxBuffer, data2, 2, 5000) != HAL_OK)
              {
                Error_Handler();
              }
    //          HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_SET);

              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
              {
              }

            }

  ////          HAL_GPIO_WritePin(device->spi_dev.chip_select_port, device->spi_dev.chip_select_pin, GPIO_PIN_RESET);
  //          if(HAL_SPI_TransmitReceive_DMA(device1->spi_dev.dev, aTxBuffer, data1, 2) != HAL_OK)
  ////          if(HAL_SPI_TransmitReceive(device->spi_dev.dev, aTxBuffer, data, 2, 5000) != HAL_OK)
  //          {
  //            Error_Handler();
  //          }
  ////          HAL_GPIO_WritePin(device->spi_dev.chip_select_port, device->spi_dev.chip_select_pin, GPIO_PIN_SET);
  //
  //          while (HAL_SPI_GetState(device1->spi_dev.dev) != HAL_SPI_STATE_READY)
  //          {
  //          }
  //
  //          aTxBuffer[0] = AD7779_REG_GEN_ERR_REG_1_EN;
  //          aTxBuffer[1] = AD7779_SPI_CRC_TEST_EN;
  //
  ////          HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_RESET);
  //          if(HAL_SPI_TransmitReceive_DMA(device2->spi_dev.dev, aTxBuffer, data2, 2) != HAL_OK)
  ////          if(HAL_SPI_TransmitReceive(device2->spi_dev.dev, aTxBuffer, data2, 2, 5000) != HAL_OK)
  //          {
  //            Error_Handler();
  //          }
  ////          HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_SET);
  //
  //          while (HAL_SPI_GetState(device2->spi_dev.dev) != HAL_SPI_STATE_READY)
  //          {
  //          }
  //
  //          aTxBuffer[0] = AD7779_REG_GEN_ERR_REG_1_EN;
  //          aTxBuffer[1] = AD7779_SPI_CRC_TEST_EN;
  //
  ////          HAL_GPIO_WritePin(device3->spi_dev.chip_select_port, device3->spi_dev.chip_select_pin, GPIO_PIN_RESET);
  //          if(HAL_SPI_TransmitReceive_DMA(device3->spi_dev.dev, aTxBuffer, data3, 2) != HAL_OK)
  ////          if(HAL_SPI_TransmitReceive(device3->spi_dev.dev, aTxBuffer, data3, 2, 5000) != HAL_OK)
  //          {
  //            Error_Handler();
  //          }
  ////          HAL_GPIO_WritePin(device3->spi_dev.chip_select_port, device3->spi_dev.chip_select_pin, GPIO_PIN_SET);
  //
  //          while (HAL_SPI_GetState(device3->spi_dev.dev) != HAL_SPI_STATE_READY)
  //          {
  //          }
  //
  //          aTxBuffer[0] = AD7779_REG_GEN_ERR_REG_1_EN;
  //          aTxBuffer[1] = AD7779_SPI_CRC_TEST_EN;
  //
  ////          HAL_GPIO_WritePin(device4->spi_dev.chip_select_port, device4->spi_dev.chip_select_pin, GPIO_PIN_RESET);
  //          if(HAL_SPI_TransmitReceive_DMA(device4->spi_dev.dev, aTxBuffer, data4, 2) != HAL_OK)
  ////          if(HAL_SPI_TransmitReceive(device4->spi_dev.dev, aTxBuffer, data4, 2, 5000) != HAL_OK)
  //          {
  //            Error_Handler();
  //          }
  ////          HAL_GPIO_WritePin(device4->spi_dev.chip_select_port, device4->spi_dev.chip_select_pin, GPIO_PIN_SET);
  //
  //          while (HAL_SPI_GetState(device4->spi_dev.dev) != HAL_SPI_STATE_READY)
  //          {
  //          }

          }
        //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
      }

//      HAL_GPIO_WritePin(AD1_START_GPIO_Port, AD1_START_Pin, GPIO_PIN_RESET);
//      HAL_Delay(1);
//      HAL_GPIO_WritePin(AD1_START_GPIO_Port, AD1_START_Pin, GPIO_PIN_SET);
//      HAL_Delay(1);
//      HAL_GPIO_WritePin(AD1_START_GPIO_Port, AD1_START_Pin, GPIO_PIN_RESET);

  //    HAL_GPIO_WritePin(AD3_START_GPIO_Port, AD3_START_Pin, GPIO_PIN_RESET);
  //    HAL_Delay(1);
  //    HAL_GPIO_WritePin(AD3_START_GPIO_Port, AD3_START_Pin, GPIO_PIN_SET);
  //    HAL_Delay(1);
  //    HAL_GPIO_WritePin(AD3_START_GPIO_Port, AD3_START_Pin, GPIO_PIN_RESET);

//      HAL_GPIO_WritePin(AD4_START_GPIO_Port, AD4_START_Pin, GPIO_PIN_RESET);
//      HAL_Delay(1);
//      HAL_GPIO_WritePin(AD4_START_GPIO_Port, AD4_START_Pin, GPIO_PIN_SET);
//      HAL_Delay(1);
//      HAL_GPIO_WritePin(AD4_START_GPIO_Port, AD4_START_Pin, GPIO_PIN_RESET);

//      for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//      {
//        for(int ad_data_byte = 0; ad_data_byte < uint8_ad_number; ad_data_byte ++)
//        {
//          datas[ad_adc][ad_data_byte] = 0;
//          aRxBuffer[ad_data_byte] = 0;
//          aTxBuffer[ad_data_byte] = 0;
//        }
//      }

          if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_I2S_READ)
          {
              while(1)
              {
                  int drdy_pin = HAL_GPIO_ReadPin(AD_DRDY_GPIO_Port, AD_DRDY_Pin);
  //                print7_hex(drdy_pin, 1);
  //                print7_symbol(';');
                  int drdy_count=0;
                  while (drdy_pin == 0)
                  {
                  	drdy_pin = HAL_GPIO_ReadPin(AD_DRDY_GPIO_Port, AD_DRDY_Pin);
  //                    print7_hex(drdy_pin, 1);
  //                    print7_symbol(';');
                  	drdy_count++;
                  }

//                  HAL_SAI_Receive(&hsai_BlockB2, datasBuffer[0], SAI_DATASIZE_32, 5000);


//                  HAL_I2S_Receive(&hi2s1, (uint16_t *)(datasBuffer[0]), 32, 5000);


//                  HAL_I2S_Receive_DMA(&hi2s1, (uint16_t *)(datasBuffer[0]), 32);
  //                            aTxBuffer[0]=0x80;
  //                            aTxBuffer[1]=0x00;
  //                            HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, datasBuffer[0], 8*4);
  //                HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, datasBuffer[0], 8*4, 100);


//                              while (HAL_I2S_GetState(&hi2s1) != HAL_I2S_STATE_READY)


                              {
                              }

          	    if(FREESMARTEEG_OUT & FREESMARTEEG_I2S_TEXT_UART7)
          	    {
          	        for(int ad_adc = 0; ad_adc < 1; ad_adc ++)
          	        {
          	        	for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
          //		        	for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
          	        	{
          //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
          //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
          //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
          //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
          //                    CLEAR_BIT(hspi6.Instance->CR1, SPI_CR1_SSI);

          	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
          	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
          //                    SET_BIT(hspi6.Instance->CR1, SPI_CR1_SSI);
          	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
          	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');


          	        		print7_symbol(';');
          	        	}
          	        	print7_line();
          	        }
          	    }//FREESMARTEEG_I2S_TEXT_UART7

                if(FREESMARTEEG_OUT & FREESMARTEEG_I2S_OPENVIBE_FREEEEG32_CUSTOM)
        //            if(0)//openbci
                    if(SPI_RxCplt)
                {
                    	SPI_RxCplt=0;

                  if(1)
                  {
                              const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 1;
        //            const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1;
                    uint8_t dataBuffer[uint8_data_number];

                    dataBuffer[0] = 0xA0;
                    dataBuffer[1] = ui8SampleNumber++;

                    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                    {
                      for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
                      {
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 1];
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 2];
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 3];
                      }
                    }
                    for(int accel_data_channel = 0; accel_data_channel < uint8_accel_chan_number; accel_data_channel ++)
                    {
                      dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 0] = 0;
                      dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 1] = 0;
                    }
                            dataBuffer[2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 0] = 0xC0;
        //            dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0] = 0xC0;

                    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
                    {
                      while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
                      {
                      }
                      if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                      {
                        Error_Handler();
                      }
                    }
                    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART7)
                    {
                      while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
                      {
                      }
                      if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                      {
                        Error_Handler();
                      }
                    }
                  }
                }//FREESMARTEEG_I2S_OPENVIBE_FREEEEG32_CUSTOM

              }

          }

          if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_SAI_READ)
          {
              while(1)
              {
                  int drdy_pin = HAL_GPIO_ReadPin(AD_DRDY_GPIO_Port, AD_DRDY_Pin);
  //                print7_hex(drdy_pin, 1);
  //                print7_symbol(';');
                  int drdy_count=0;
                  while (drdy_pin == 0)
                  {
                  	drdy_pin = HAL_GPIO_ReadPin(AD_DRDY_GPIO_Port, AD_DRDY_Pin);
  //                    print7_hex(drdy_pin, 1);
  //                    print7_symbol(';');
                  	drdy_count++;
                  }

//                  HAL_SAI_Receive(&hsai_BlockB2, datasBuffer[0], SAI_DATASIZE_32, 5000);
                  HAL_SAI_Receive_DMA(&hsai_BlockB1, datasBuffer[0], SAI_DATASIZE_32);
//                  HAL_SAI_Receive_DMA(&hsai_BlockA1, datasBuffer[2], SAI_DATASIZE_32);
//                  HAL_SAI_Receive_DMA(&hsai_BlockB2, datasBuffer[0], SAI_DATASIZE_32);
  //                            aTxBuffer[0]=0x80;
  //                            aTxBuffer[1]=0x00;
  //                            HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, datasBuffer[0], 8*4);
  //                HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, datasBuffer[0], 8*4, 100);
                  while (HAL_SAI_GetState(&hsai_BlockB1) != HAL_SAI_STATE_READY)
//                              while (HAL_SAI_GetState(&hsai_BlockB2) != HAL_SAI_STATE_READY)
                              {
                              }

          	    if(FREESMARTEEG_OUT & FREESMARTEEG_SAI_TEXT_UART7)
          	    {
          	        for(int ad_adc = 0; ad_adc < 1; ad_adc ++)
          	        {
          	        	for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
          //		        	for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
          	        	{
          //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
          //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
          //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
          //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
          //                    CLEAR_BIT(hspi6.Instance->CR1, SPI_CR1_SSI);

          	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
          	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
          //                    SET_BIT(hspi6.Instance->CR1, SPI_CR1_SSI);
          	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
          	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');


          	        		print7_symbol(';');
          	        	}
          	        	print7_line();
          	        }
          	    }//FREESMARTEEG_SAI_TEXT_UART7

                if(FREESMARTEEG_OUT & FREESMARTEEG_SAI_OPENVIBE_FREEEEG32_CUSTOM)
        //            if(0)//openbci
                    if(SPI_RxCplt)
                {
                    	SPI_RxCplt=0;

                  if(1)
                  {
                              const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 1;
        //            const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1;
                    uint8_t dataBuffer[uint8_data_number];

                    dataBuffer[0] = 0xA0;
                    dataBuffer[1] = ui8SampleNumber++;

                    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                    {
                      for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
                      {
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 1];
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 2];
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 3];
                      }
                    }
                    for(int accel_data_channel = 0; accel_data_channel < uint8_accel_chan_number; accel_data_channel ++)
                    {
                      dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 0] = 0;
                      dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 1] = 0;
                    }
                            dataBuffer[2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 0] = 0xC0;
        //            dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0] = 0xC0;

                    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
                    {
                      while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
                      {
                      }
                      if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                      {
                        Error_Handler();
                      }
                    }
                    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART7)
                    {
                      while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
                      {
                      }
                      if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                      {
                        Error_Handler();
                      }
                    }
                  }
                }//FREESMARTEEG_SAI_OPENVIBE_FREEEEG32_CUSTOM

              }

          }

          if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_SAI_READ_TIMER)
    {


//        	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//        	  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
//        	  TIM_MasterConfigTypeDef sMasterConfig = {0};
//        	  TIM_IC_InitTypeDef sConfigIC = {0};
//
//        	  /* USER CODE BEGIN TIM1_Init 1 */
//
//        	  /* USER CODE END TIM1_Init 1 */
//        	  htim1.Instance = TIM1;
//        	  htim1.Init.Prescaler = 0;
//        	  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//        	  htim1.Init.Period = 0;
//        	  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//        	  htim1.Init.RepetitionCounter = 0;
//        	  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//        	  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
//        	  {
//        	    Error_Handler();
//        	  }
////        	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//        	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
//        	  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
//        	  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
//        	  sClockSourceConfig.ClockFilter = 0;
//        	  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
//        	  {
//        	    Error_Handler();
//        	  }
//        	  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
//        	  {
//        	    Error_Handler();
//        	  }
//        	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
//        	  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
////        	  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
//        	  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
//        	  sSlaveConfig.TriggerFilter = 0;
//        	  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
//        	  {
//        	    Error_Handler();
//        	  }
//        	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
//        	  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//        	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//        	  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//        	  {
//        	    Error_Handler();
//        	  }
////        	  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
//        	  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
//        	  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
//        	  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
//        	  sConfigIC.ICFilter = 0;
//        	  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
//        	  {
//        	    Error_Handler();
//        	  }
//
//        	  /* USER CODE BEGIN TIM1_Init 1 */
//
//        	  /* USER CODE END TIM1_Init 1 */
//        	  htim8.Instance = TIM8;
//        	  htim8.Init.Prescaler = 0;
//        	  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
//        	  htim8.Init.Period = 256;
//        	  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//        	  htim8.Init.RepetitionCounter = 0;
//        	  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//        	  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
//        	  {
//        	    Error_Handler();
//        	  }
////        	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//        	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
//        	  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
//        	  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
//        	  sClockSourceConfig.ClockFilter = 0;
//        	  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
//        	  {
//        	    Error_Handler();
//        	  }
//        	  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
//        	  {
//        	    Error_Handler();
//        	  }
//        	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
//        	  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
//        	  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
//        	  {
//        	    Error_Handler();
//        	  }
//        	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//        	  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//        	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//        	  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
//        	  {
//        	    Error_Handler();
//        	  }
////        	  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
//        	  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
//        	  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
//        	  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
//        	  sConfigIC.ICFilter = 0;
//        	  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
//        	  {
//        	    Error_Handler();
//        	  }
//        	  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
//        	  {
//        	    Error_Handler();
//        	  }
//
//        	    /* TIM1 DMA Init */
//        	    /* TIM1_CH1 Init */
//        	    hdma_tim1_ch1.Instance = DMA2_Stream3;
//        	    hdma_tim1_ch1.Init.Channel = DMA_CHANNEL_6;
////        	    hdma_tim1_ch1.Init.Direction = DMA_PERIPH_TO_MEMORY;
//        	    hdma_tim1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
//        	    hdma_tim1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
//        	    hdma_tim1_ch1.Init.MemInc = DMA_MINC_ENABLE;
//        	    hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//        	    hdma_tim1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//        	    hdma_tim1_ch1.Init.Mode = DMA_CIRCULAR;
//        	    hdma_tim1_ch1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
////        	    hdma_tim1_ch1.Init.Priority = DMA_PRIORITY_LOW;
//        	    hdma_tim1_ch1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
//        	    hdma_tim1_ch1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
//        	    hdma_tim1_ch1.Init.MemBurst = DMA_MBURST_SINGLE;
//        	    hdma_tim1_ch1.Init.PeriphBurst = DMA_PBURST_SINGLE;
//        	    if (HAL_DMA_Init(&hdma_tim1_ch1) != HAL_OK)
//        	    {
//        	      Error_Handler();
//        	    }
//
////        	    __HAL_LINKDMA(htim1,hdma[TIM_DMA_ID_CC1],hdma_tim1_ch1);
//
//        	    /* TIM1_CH4_TRIG_COM Init */
//        	    hdma_tim1_ch4_trig_com.Instance = DMA2_Stream4;
//        	    hdma_tim1_ch4_trig_com.Init.Channel = DMA_CHANNEL_6;
//        	    hdma_tim1_ch4_trig_com.Init.Direction = DMA_PERIPH_TO_MEMORY;
//        	    hdma_tim1_ch4_trig_com.Init.PeriphInc = DMA_PINC_DISABLE;
//        	    hdma_tim1_ch4_trig_com.Init.MemInc = DMA_MINC_ENABLE;
//        	    hdma_tim1_ch4_trig_com.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//        	    hdma_tim1_ch4_trig_com.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//        	    hdma_tim1_ch4_trig_com.Init.Mode = DMA_CIRCULAR;
//        	    hdma_tim1_ch4_trig_com.Init.Priority = DMA_PRIORITY_VERY_HIGH;
////        	    hdma_tim1_ch4_trig_com.Init.Priority = DMA_PRIORITY_LOW;
//        	    hdma_tim1_ch4_trig_com.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
//        	    hdma_tim1_ch4_trig_com.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
//        	    hdma_tim1_ch4_trig_com.Init.MemBurst = DMA_MBURST_SINGLE;
//        	    hdma_tim1_ch4_trig_com.Init.PeriphBurst = DMA_PBURST_SINGLE;
//        	    if (HAL_DMA_Init(&hdma_tim1_ch4_trig_com) != HAL_OK)
//        	    {
//        	      Error_Handler();
//        	    }
//
//        	    /* Several peripheral DMA handle pointers point to the same DMA handle.
//        	     Be aware that there is only one stream to perform all the requested DMAs. */
////        	    __HAL_LINKDMA(htim1,hdma[TIM_DMA_ID_CC4],hdma_tim1_ch4_trig_com);
////        	    __HAL_LINKDMA(htim1,hdma[TIM_DMA_ID_TRIGGER],hdma_tim1_ch4_trig_com);
////        	    __HAL_LINKDMA(htim1,hdma[TIM_DMA_ID_COMMUTATION],hdma_tim1_ch4_trig_com);
//
//        	    /* TIM1_UP Init */
//        	    hdma_tim1_up.Instance = DMA2_Stream5;
//        	    hdma_tim1_up.Init.Channel = DMA_CHANNEL_6;
//        	    hdma_tim1_up.Init.Direction = DMA_PERIPH_TO_MEMORY;
//        	    hdma_tim1_up.Init.PeriphInc = DMA_PINC_DISABLE;
//        	    hdma_tim1_up.Init.MemInc = DMA_MINC_ENABLE;
//        	    hdma_tim1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//        	    hdma_tim1_up.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//        	    hdma_tim1_up.Init.Mode = DMA_CIRCULAR;
//        	    hdma_tim1_up.Init.Priority = DMA_PRIORITY_VERY_HIGH;
////        	    hdma_tim1_up.Init.Priority = DMA_PRIORITY_LOW;
//        	    hdma_tim1_up.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
//        	    hdma_tim1_up.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
//        	    hdma_tim1_up.Init.MemBurst = DMA_MBURST_SINGLE;
//        	    hdma_tim1_up.Init.PeriphBurst = DMA_PBURST_SINGLE;
//        	    if (HAL_DMA_Init(&hdma_tim1_up) != HAL_OK)
//        	    {
//        	      Error_Handler();
//        	    }
//
////        	    __HAL_LINKDMA(htim1,hdma[TIM_DMA_ID_UPDATE],hdma_tim1_up);
//
//        	    /* TIM8_CH1 Init */
//        	    hdma_tim8_ch1.Instance = DMA2_Stream2;
//        	    hdma_tim8_ch1.Init.Channel = DMA_CHANNEL_7;
//        	    hdma_tim8_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
//        	    hdma_tim8_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
//        	    hdma_tim8_ch1.Init.MemInc = DMA_MINC_ENABLE;
//        	    hdma_tim8_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//        	    hdma_tim8_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//        	    hdma_tim8_ch1.Init.Mode = DMA_CIRCULAR;
//        	    hdma_tim8_ch1.Init.Priority = DMA_PRIORITY_LOW;
//        	    hdma_tim8_ch1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
//        	    hdma_tim8_ch1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
//        	    hdma_tim8_ch1.Init.MemBurst = DMA_MBURST_SINGLE;
//        	    hdma_tim8_ch1.Init.PeriphBurst = DMA_PBURST_SINGLE;
//        	    if (HAL_DMA_Init(&hdma_tim8_ch1) != HAL_OK)
//        	    {
//        	      Error_Handler();
//        	    }
//
////        	    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC1],hdma_tim8_ch1);
//
//        	    /* TIM8_CH4_TRIG_COM Init */
//        	    hdma_tim8_ch4_trig_com.Instance = DMA2_Stream7;
//        	    hdma_tim8_ch4_trig_com.Init.Channel = DMA_CHANNEL_7;
//        	    hdma_tim8_ch4_trig_com.Init.Direction = DMA_PERIPH_TO_MEMORY;
//        	    hdma_tim8_ch4_trig_com.Init.PeriphInc = DMA_PINC_DISABLE;
//        	    hdma_tim8_ch4_trig_com.Init.MemInc = DMA_MINC_ENABLE;
//        	    hdma_tim8_ch4_trig_com.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//        	    hdma_tim8_ch4_trig_com.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//        	    hdma_tim8_ch4_trig_com.Init.Mode = DMA_CIRCULAR;
//        	    hdma_tim8_ch4_trig_com.Init.Priority = DMA_PRIORITY_LOW;
//        	    hdma_tim8_ch4_trig_com.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
//        	    hdma_tim8_ch4_trig_com.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
//        	    hdma_tim8_ch4_trig_com.Init.MemBurst = DMA_MBURST_SINGLE;
//        	    hdma_tim8_ch4_trig_com.Init.PeriphBurst = DMA_PBURST_SINGLE;
//        	    if (HAL_DMA_Init(&hdma_tim8_ch4_trig_com) != HAL_OK)
//        	    {
//        	      Error_Handler();
//        	    }
//
//        	    /* Several peripheral DMA handle pointers point to the same DMA handle.
//        	     Be aware that there is only one stream to perform all the requested DMAs. */
////        	    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC4],hdma_tim8_ch4_trig_com);
////        	    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_TRIGGER],hdma_tim8_ch4_trig_com);
////        	    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_COMMUTATION],hdma_tim8_ch4_trig_com);
//
//        	    /* TIM8_UP Init */
//        	    hdma_tim8_up.Instance = DMA2_Stream1;
//        	    hdma_tim8_up.Init.Channel = DMA_CHANNEL_7;
//        	    hdma_tim8_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
//        	    hdma_tim8_up.Init.PeriphInc = DMA_PINC_DISABLE;
//        	    hdma_tim8_up.Init.MemInc = DMA_MINC_ENABLE;
//        	    hdma_tim8_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//        	    hdma_tim8_up.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//        	    hdma_tim8_up.Init.Mode = DMA_CIRCULAR;
//        	    hdma_tim8_up.Init.Priority = DMA_PRIORITY_LOW;
//        	    hdma_tim8_up.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
//        	    hdma_tim8_up.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
//        	    hdma_tim8_up.Init.MemBurst = DMA_MBURST_SINGLE;
//        	    hdma_tim8_up.Init.PeriphBurst = DMA_PBURST_SINGLE;
//        	    if (HAL_DMA_Init(&hdma_tim8_up) != HAL_OK)
//        	    {
//        	      Error_Handler();
//        	    }


              HAL_SAI_Receive_DMA(&hsai_BlockB1, datas[0], SAI_DATASIZE_32);
//              HAL_SAI_Receive_DMA(&hsai_BlockB2, datas[0], SAI_DATASIZE_32);
//              HAL_SAI_Receive_DMA(&hsai_BlockB2, dataBuffer110[2], SAI_DATASIZE_32);
//              HAL_SAI_Receive_DMA(&hsai_BlockB2, (&(dataBuffer110[2])), SAI_DATASIZE_32);
              while (HAL_SAI_GetState(&hsai_BlockB1) != HAL_SAI_STATE_READY)
//                          while (HAL_SAI_GetState(&hsai_BlockB2) != HAL_SAI_STATE_READY)
                          {
                          }

//              HAL_DMA_Start(htim1.hdma[TIM_DMA_ID_UPDATE], (uint32_t)&hsai_BlockB2.Instance->DR, (uint32_t)(&(dataBuffer110[2])), 32);
//              HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_UPDATE], (uint32_t)&hsai_BlockB2.Instance->DR, (uint32_t)(&(dataBuffer110[2])), 32);
//              __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);

//              HAL_DMA_Start_IT(htim8.hdma[TIM_DMA_ID_TRIGGER], (uint32_t)&hsai_BlockB2.Instance->DR, (uint32_t)(datas[0]), 32);
//              __HAL_TIM_ENABLE_DMA(&htim8, TIM_DMA_TRIGGER);
//              HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_TRIGGER], (uint32_t)&hsai_BlockB2.Instance->DR, (uint32_t)(&(dataBuffer110[2])), 32);
//              __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_TRIGGER);



//
//              HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_TRIGGER], (uint32_t)&hsai_BlockB1.Instance->DR, (uint32_t)(datas[0]), 32);
////             HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_TRIGGER], (uint32_t)&hsai_BlockB2.Instance->DR, (uint32_t)(datas[0]), 32);
//             __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_TRIGGER);
//
//



//             HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_CC1], (uint32_t)&hsai_BlockB2.Instance->DR, (uint32_t)(datas[0]), 32);
//             __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);
//              HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_UPDATE], (uint32_t)&hsai_BlockB2.Instance->DR, (uint32_t)(datas[0]), 32);
//                 __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);

//        	    HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_TRIGGER], (uint32_t)&(dataBuffer110[1]), (uint32_t)&huart7.Instance->TDR, 1);
//        	   __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_TRIGGER);
////
        	      dataBuffer110[0] = 0xA0;
        	      dataBuffer110[dataBuffer110size-1] = 0xC0;
////
//        	      HAL_DMA_Start(htim8.hdma[TIM_DMA_ID_TRIGGER], (uint32_t)&htim8.Instance->CNT, (uint32_t)(&(dataBuffer110[1])), 1);
//        	      HAL_DMA_Start_IT(htim8.hdma[TIM_DMA_ID_TRIGGER], (uint32_t)&htim8.Instance->CNT, (uint32_t)(&(dataBuffer110[1])), 1);



//
//        	      HAL_DMA_Start_IT(htim8.hdma[TIM_DMA_ID_TRIGGER], (uint32_t)&htim8.Instance->CNT, (uint32_t)(datas[1]), 4);
//        	      __HAL_TIM_ENABLE_DMA(&htim8, TIM_DMA_TRIGGER);
//




       while(1)
       {


   	    if(FREESMARTEEG_OUT & FREESMARTEEG_DMA_UART7_SAI2B_TIMER)
   //	            if(0)
   	    {

//   	    	if(datasBuffer[2][0]!=dataBuffer110[1])
   //	      if((datasBuffer[2][0]!=datasBuffer[0][0])
   //	    			||(datasBuffer[2][1]!=datasBuffer[0][1])
   //	    			||(datasBuffer[2][2]!=datasBuffer[0][2])
   //	    			||(datasBuffer[2][3]!=datasBuffer[0][3])
   //	    			||(datasBuffer[2][4]!=datasBuffer[0][4])
   //	    			||(datasBuffer[2][5]!=datasBuffer[0][5])
   //	    			||(datasBuffer[2][6]!=datasBuffer[0][6])
   //					||(datasBuffer[2][7]!=datasBuffer[0][7]))
   	      {

      		print7_binary(dataBuffer110[0], 8);          print7_symbol(';');
//    		print7_binary(dataBuffer110[1], 8);          print7_symbol(';');
    		print7_binary(datas[1][0], 8);          print7_symbol('.');
    		print7_binary(datas[1][1], 8);          print7_symbol('.');
    		print7_binary(datas[1][2], 8);          print7_symbol('.');
    		print7_binary(datas[1][3], 8);          print7_symbol(';');
//    	    	  if(0)
//     	        for(int ad_adc = 0; ad_adc < 1; ad_adc ++)
//    	        for(int ad_adc = 0; ad_adc < 4; ad_adc ++)
   	        {
//   	        	for(int ad_data_out = 0; ad_data_out < dataBuffer110size; ad_data_out ++)
   	        	for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
   	        	   //		        	for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
   	        	{
   //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
   //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
   //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
   //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
   //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
   //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
   //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
   //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
   //                    CLEAR_BIT(hspi6.Instance->CR1, SPI_CR1_SSI);

   //	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
   //	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
   ////                    SET_BIT(hspi6.Instance->CR1, SPI_CR1_SSI);
   //	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
   //	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');

//   	        		print7_binary(dataBuffer110[ad_data_channel * 4 + 0+2], 8);//          print2_symbol(';');
//   	        		print7_binary(dataBuffer110[ad_data_channel * 4 + 1+2], 8);//          print2_symbol(';');
//   	        		print7_binary(dataBuffer110[ad_data_channel * 4 + 2+2], 8);//          print2_symbol(';');
//   	        		print7_binary(dataBuffer110[ad_data_channel * 4 + 3+2], 8);//          print2_symbol(';');
	        		print7_binary(datas[0][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
	        		print7_binary(datas[0][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
	        		print7_binary(datas[0][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
	        		print7_binary(datas[0][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');

   //	        		print7_binary(dataBuffer110[2 + ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
   //	        		print7_binary(dataBuffer110[2 + ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
   //	        		print7_binary(dataBuffer110[2 + ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
   //	        		print7_binary(dataBuffer110[2 + ad_data_channel * 4 + 3], 8);//          print2_symbol(';');

   	        		print7_symbol(';');
   	        	}


   //        		print7_symbol(';');
   	        }
        		print7_binary(dataBuffer110[dataBuffer110size-1], 8);          print7_symbol(';');

   	    	  //	        	print7_hex(dataBuffer110[0], 8);//          print2_symbol(';');
   	    	  //	        	print7_hex(dataBuffer110[1], 8);//          print2_symbol(';');
   	    	  //	      		print7_binary(dataBuffer110[2], 8);//          print2_symbol(';');

   	    	  //      		print7_symbol(';');
   //	    	  	        	for(int Buffer_data = 0; Buffer_data < dataBuffer110size; Buffer_data ++)
   //	    	  	        	{
   //	    	  	        		print7_hex(dataBuffer110[Buffer_data], 8);
   //	    	  	        	}

   	    	  //	        	print7_hex(dataBuffer110[2 + uint8_ad_chan_number * (1 + 3) * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0], 8);//          print2_symbol(';');

           	print7_line();

//                                while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
//                                {
//                                }
////                                if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)&(dataBuffer110[1]), 1) != HAL_OK)
//                                if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)&(datasBuffer[0]), 1) != HAL_OK)
//    //                            if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer110, dataBuffer110size) != HAL_OK)
//                                {
//                                  Error_Handler();
//                                }

            	    	datasBuffer[2][0]=dataBuffer110[1];
   //	    	datasBuffer[2][0]=datasBuffer[0][0];
   //	    	datasBuffer[2][1]=datasBuffer[0][1];
   //	    	datasBuffer[2][2]=datasBuffer[0][2];
   //	    	datasBuffer[2][3]=datasBuffer[0][3];
   //	    	datasBuffer[2][4]=datasBuffer[0][4];
   //	    	datasBuffer[2][5]=datasBuffer[0][5];
   //	    	datasBuffer[2][6]=datasBuffer[0][6];
   //			datasBuffer[2][7]=datasBuffer[0][7];
   	      }

   	    }//FREESMARTEEG_DMA_UART7_SAI2B_TIMER

    	    if(FREESMARTEEG_OUT & FREESMARTEEG_TEXT_UART7_SAI2B_TIMER)
    //	            if(0)
    	    {

    //	      if((datasBuffer[2][0]!=datasBuffer[0][0])
    //	    			||(datasBuffer[2][1]!=datasBuffer[0][1])
    //	    			||(datasBuffer[2][2]!=datasBuffer[0][2])
    //	    			||(datasBuffer[2][3]!=datasBuffer[0][3])
    //	    			||(datasBuffer[2][4]!=datasBuffer[0][4])
    //	    			||(datasBuffer[2][5]!=datasBuffer[0][5])
    //	    			||(datasBuffer[2][6]!=datasBuffer[0][6])
    //					||(datasBuffer[2][7]!=datasBuffer[0][7]))
    	      {

//    	    	  if(0)
      	        for(int ad_adc = 0; ad_adc < 1; ad_adc ++)
//    	        for(int ad_adc = 0; ad_adc < 4; ad_adc ++)
    	        {
    	        	for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
    //		        	for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
    	        	{
    //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
    //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    //	        		print7_hex(datasBuffer[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
    //                    CLEAR_BIT(hspi6.Instance->CR1, SPI_CR1_SSI);

    //	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    //	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    ////                    SET_BIT(hspi6.Instance->CR1, SPI_CR1_SSI);
    //	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    //	        		print7_binary(datasBuffer[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');

    	        		print7_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    	        		print7_binary(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    	        		print7_binary(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    	        		print7_binary(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');

    //	        		print7_binary(dataBuffer110[2 + ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    //	        		print7_binary(dataBuffer110[2 + ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    //	        		print7_binary(dataBuffer110[2 + ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    //	        		print7_binary(dataBuffer110[2 + ad_data_channel * 4 + 3], 8);//          print2_symbol(';');

    	        		print7_symbol(';');
    	        	}


    //        		print7_symbol(';');
    	        }

    	    	  //	        	print7_hex(dataBuffer110[0], 8);//          print2_symbol(';');
    	    	  //	        	print7_hex(dataBuffer110[1], 8);//          print2_symbol(';');
    	    	  //	      		print7_binary(dataBuffer110[2], 8);//          print2_symbol(';');

    	    	  //      		print7_symbol(';');
    //	    	  	        	for(int Buffer_data = 0; Buffer_data < dataBuffer110size; Buffer_data ++)
    //	    	  	        	{
    //	    	  	        		print7_hex(dataBuffer110[Buffer_data], 8);
    //	    	  	        	}

    	    	  //	        	print7_hex(dataBuffer110[2 + uint8_ad_chan_number * (1 + 3) * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0], 8);//          print2_symbol(';');

            	print7_line();

//                                while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
//                                {
//                                }
////                                if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)&(dataBuffer110[1]), 1) != HAL_OK)
//                                if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)&(datasBuffer[0]), 1) != HAL_OK)
//    //                            if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer110, dataBuffer110size) != HAL_OK)
//                                {
//                                  Error_Handler();
//                                }

    //	    	datasBuffer[2][0]=datasBuffer[0][0];
    //	    	datasBuffer[2][1]=datasBuffer[0][1];
    //	    	datasBuffer[2][2]=datasBuffer[0][2];
    //	    	datasBuffer[2][3]=datasBuffer[0][3];
    //	    	datasBuffer[2][4]=datasBuffer[0][4];
    //	    	datasBuffer[2][5]=datasBuffer[0][5];
    //	    	datasBuffer[2][6]=datasBuffer[0][6];
    //			datasBuffer[2][7]=datasBuffer[0][7];
    	      }

    	    }//FREESMARTEEG_TEXT_UART7_SAI2B_TIMER
            if(FREESMARTEEG_OUT & FREESMARTEEG_OPENVIBE_FREEEEG32_UART7_SAI2B_TIMER)
     	    {
                if(1)
                {
                  const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1;
                  uint8_t dataBuffer[uint8_data_number];

                  dataBuffer[0] = 0xA0;
                  dataBuffer[1] = ui8SampleNumber++;

                  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                  {
                    for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
                    {
                      dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 1];
                      dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 2];
                      dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 3];
                    }
                  }
                  for(int accel_data_channel = 0; accel_data_channel < uint8_accel_chan_number; accel_data_channel ++)
                  {
                    dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 0] = 0;
                    dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 1] = 0;
                  }
                  dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0] = 0xC0;

                  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
                  {
                    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
                    {
                    }
                    if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                    {
                      Error_Handler();
                    }
                  }
                  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART7)
                  {
                    while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
                    {
                    }
                    if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                    {
                      Error_Handler();
                    }
                  }
                }
     	    }//FREESMARTEEG_OPENVIBE_FREEEEG32_UART7_SAI2B_TIMER


       }
    }


    //  if(0)
      if(FREESMARTEEG_OUT & FREESMARTEEG_FFT)
      while (1)
      {
        // data is in 11.21(q21) format for the 1024 point as there are 3 middle stages
        // data is in 9.23(q23) format for the 256 point as there are 2 middle stages
        // data is in 7.25(q25) format for the 64 point as there are 1 middle stage
        // data is in 5.27(q27) format for the 16 point as there are no middle stages
  //      uint32_t FFT_Length = 16;
    //    uint32_t FFT_Length = 64;
    //    uint32_t FFT_Length = 256;
        //    uint32_t FFT_Length = 1024;
    //    FFT_PROCESSING_Q15Process(FFT_Length);
    //    FFT_PROCESSING_F32Process(FFT_Length);
  ////      FFT_PROCESSING_Q31Process(FFT_Length);
      }

      if(0)
      while (1)
      {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_Delay(40);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_Delay(40);
      }

      long test_counter=0;
      long test_blink_counter=0;

  //    const int sample_number_max = 480;
  //    const int sample_number_max = 240;
  //    const int sample_number_max = 15;
      const int sample_number_max = 128;
  //    const int sample_number_max = 256;
  //    const int sample_number_max = 512;
  //    const int sample_number_max = 1024;
  //    const int sample_number_max = 2048;
      int sample_freq_fft = 64;
      int fft_out=16;
      int fft_out_shift=5;
      int32_t sample_int[sample_number_max];
      uint8_t sample_byte[sample_number_max];
      int sample_number_now = 0;
  //    int sample_average = 0;
      int64_t sample_sum=0;
      int32_t sample_max=0;
      int32_t sample_min=0;
      int32_t sample_max_last=0;
      int32_t sample_min_last=0;

  //    int32_t samples_int[uint8_ad_adc_number][8][sample_number_max];
      int32_t samples_int_part[sample_number_max];
  //    uint8_t samples_byte[uint8_ad_adc_number][8][sample_number_max];
      int32_t samples_max[uint8_ad_adc_number][8];//={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
      int32_t samples_min[uint8_ad_adc_number][8];//={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
      int32_t samples_max_min_dif[uint8_ad_adc_number][8];
      int32_t sample_max_min_dif_max=0;
      int32_t sample_max_min_dif_min=0;
      int32_t sample_max_new;
      int32_t sample_min_new;
      int32_t samples_max_new[uint8_ad_adc_number][8];
      int32_t samples_min_new[uint8_ad_adc_number][8];

      int32_t sample_int_out;
      uint8_t sample_byte_out;
  //    double sample_int_byte_scale;
  //    int sample_int_byte_shift;

      const uint32_t sample_data_number = sample_number_max;
      uint8_t sample_dataBuffer[sample_data_number];

      const uint32_t telnet_packet_data_number_max = 100;//32;
      const uint32_t telnet_data_number_max = 2 + 8 * 3 + 3 * 2 + 1;
  //                const uint32_t uint8_data_number = 2 + 8 * 3 + 8 * 3 + 3 * 2 + 1;
      uint8_t telnet_dataBuffer[telnet_packet_data_number_max][telnet_data_number_max];

      int telnet_packet_data_number_now = 0;

      long data_counter=0;
      long stable_crc[uint8_ad_adc_number][8/2]={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};

  //    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
  //    {
  //      for(int ad_data_channel_pair = 0; ad_data_channel_pair < (uint8_ad_chan_number / 2); ad_data_channel_pair ++)
  //      {
  //        stable_crc[adc][ad_data_channel_pair]=0;
  //      }
  //    }

      int ad_adc1=0;

      if(1)

    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


        if(FREESMARTEEG_OUT & FREESMARTEEG_DATA_TEXT_UART7_INT)
            if(SPI_RxCplt)
        {
            	SPI_RxCplt=0;

//              int crc_pair_ok[uint8_ad_adc_number][4];


//                  print7_hex(ad_adc1, 32);//          print2_symbol(';');
//                  print7_symbol(';');
          for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
          {
            for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
            {
                print7_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
                print7_binary(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
                print7_binary(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
                print7_binary(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');

//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
              print7_symbol(';');
            }
            print7_symbol(';');
          }
          print7_symbol(';');

          print7_line();

        }//FREESMARTEEG_DATA_TEXT_UART7_INT

        if(FREESMARTEEG_OUT & FREESMARTEEG_DATA_OPENVIBE_FREEEEG32_CUSTOM_INT)
//            if(0)//openbci
            if(SPI_RxCplt)
        {
            	SPI_RxCplt=0;

          if(1)
          {
              const uint32_t uint8_data_number = 2 + 1 * 3 * 1 + 0 * 2 + 1;
//              const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 1;
//            const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1;
            uint8_t dataBuffer[uint8_data_number];

            dataBuffer[0] = 0xA0;
            dataBuffer[1] = ui8SampleNumber++;

            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
              for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
              {
                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 1];
                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 2];
                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 3];
              }
            }
            for(int accel_data_channel = 0; accel_data_channel < 0; accel_data_channel ++)
//                for(int accel_data_channel = 0; accel_data_channel < uint8_accel_chan_number; accel_data_channel ++)
            {
              dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 0] = 0;
              dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 1] = 0;
            }
            dataBuffer[2 + 1 * 3 * 1 + 0 * 2 + 0] = 0xC0;
//            dataBuffer[2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 0] = 0xC0;
//            dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0] = 0xC0;

            if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
            {
              while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
              {
              }
              if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
              {
                Error_Handler();
              }
            }
            if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART7)
            {
              while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
              {
              }
              if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
              {
                Error_Handler();
              }
            }
          }


        }//FREESMARTEEG_DATA_OPENVIBE_FREEEEG32_CUSTOM_INT
//        print7_text_line(".");

        if(FREESMARTEEG_OUT & FREESMARTEEG_SAI_TEXT_UART7_INT)
            if(SAI_RxCplt)
        {
            	SAI_RxCplt=0;

//              int crc_pair_ok[uint8_ad_adc_number][4];


//                  print7_hex(ad_adc1, 32);//          print2_symbol(';');
//                  print7_symbol(';');
//                for(int ad_adc = 0; ad_adc < 1; ad_adc ++)
                    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//                int ad_adc = 0;
//                int ad_adc = 1;
          {
            for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
            {
//                print7_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
//                print7_binary(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//                print7_binary(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//                print7_binary(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');

//                print7_binary(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
//                print7_binary(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//                print7_binary(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//                print7_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');

//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');

                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');

              print7_symbol(';');
            }
            print7_symbol(';');
          }
          print7_symbol(';');

          print7_line();

        }//FREESMARTEEG_SAI_TEXT_UART7_INT

        if(FREESMARTEEG_OUT & FREESMARTEEG_SAI_OPENVIBE_FREEEEG32_CUSTOM_INT)
//            if(0)//openbci
            if(SAI_RxCplt)
        {
            	SAI_RxCplt=0;

          if(1)
          {
              const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1;
//              const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 1;
//              const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * 1 + 0 * 2 + 1;
//              const uint32_t uint8_data_number = 2 + 1 * 3 * 1 + 0 * 2 + 1;
//              const uint32_t uint8_data_number = 2 + 1 * 3 * 1 + 1 * 2 + 1;
//              const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1;
            uint8_t dataBuffer[uint8_data_number];

            dataBuffer[0] = 0xA0;
            dataBuffer[1] = ui8SampleNumber++;

//            for(int ad_adc = 0; ad_adc < 1; ad_adc ++)
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
//                for(int ad_data_channel = 0; ad_data_channel < 1; ad_data_channel ++)
              for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
              {
//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 1];
//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 2];
//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 3];

                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 2];
                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 1];
                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 0];
              }
            }
//            for(int accel_data_channel = 0; accel_data_channel < 0; accel_data_channel ++)
//            for(int accel_data_channel = 0; accel_data_channel < 1; accel_data_channel ++)
            for(int accel_data_channel = 0; accel_data_channel < uint8_accel_chan_number; accel_data_channel ++)
            {
              dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 0] = 0;
              dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 1] = 0;
            }
//            dataBuffer[2 + uint8_ad_chan_number * 3 * 1 + 0 * 2 + 0] = 0xC0;
//            dataBuffer[2 + 1 * 3 * 1 + 0 * 2 + 0] = 0xC0;
//            dataBuffer[2 + 1 * 3 * 1 + 1 * 2 + 0] = 0xC0;
//            dataBuffer[2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 0] = 0xC0;
            dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0] = 0xC0;

            if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
            {
              while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
              {
              }
              if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
              {
                Error_Handler();
              }
            }
            if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART7)
            {
              while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
              {
              }
              if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
              {
                Error_Handler();
              }
            }
          }


        }//FREESMARTEEG_SAI_OPENVIBE_FREEEEG32_CUSTOM_INT

        if(FREESMARTEEG_OUT & FREESMARTEEG_I2S_TEXT_UART7_INT)
            if(I2S_RxCplt)
        {
            	I2S_RxCplt=0;

//              int crc_pair_ok[uint8_ad_adc_number][4];


//                  print7_hex(ad_adc1, 32);//          print2_symbol(';');
//                  print7_symbol(';');
          for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
          {
            for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
            {
//                print7_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
//                print7_binary(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//                print7_binary(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//                print7_binary(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');

                print7_binary(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
                print7_binary(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
                print7_binary(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
                print7_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');

//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
              print7_symbol(';');
            }
            print7_symbol(';');
          }
          print7_symbol(';');

          print7_line();

        }//FREESMARTEEG_I2S_TEXT_UART7_INT

        if(FREESMARTEEG_OUT & FREESMARTEEG_I2S_OPENVIBE_FREEEEG32_CUSTOM_INT)
//            if(0)//openbci
            if(I2S_RxCplt)
        {
            	I2S_RxCplt=0;

          if(1)
          {
//              const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 1;
              const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * 1 + 0 * 2 + 1;
//              const uint32_t uint8_data_number = 2 + 1 * 3 * 1 + 0 * 2 + 1;
//              const uint32_t uint8_data_number = 2 + 1 * 3 * 1 + 1 * 2 + 1;
//              const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1;
            uint8_t dataBuffer[uint8_data_number];

            dataBuffer[0] = 0xA0;
            dataBuffer[1] = ui8SampleNumber++;

//            for(int ad_adc = 0; ad_adc < 1; ad_adc ++)
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
//                for(int ad_data_channel = 0; ad_data_channel < 1; ad_data_channel ++)
              for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
              {
//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 1];
//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 2];
//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 3];

                  dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 2];
                  dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 2];
                  dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 1];
                  dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 0];

//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 2];
//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 1];
//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 0];
              }
            }
            for(int accel_data_channel = 0; accel_data_channel < 0; accel_data_channel ++)
//            for(int accel_data_channel = 0; accel_data_channel < 1; accel_data_channel ++)
//            for(int accel_data_channel = 0; accel_data_channel < uint8_accel_chan_number; accel_data_channel ++)
            {
              dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 0] = 0;
              dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 1] = 0;
            }
//            dataBuffer[2 + 1 * 3 * 1 + 0 * 2 + 0] = 0xC0;
//            dataBuffer[2 + 1 * 3 * 1 + 1 * 2 + 0] = 0xC0;
            dataBuffer[2 + uint8_ad_chan_number * 3 * 1 + 0 * 2 + 0] = 0xC0;
//            dataBuffer[2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 0] = 0xC0;
//            dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0] = 0xC0;

            if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
            {
              while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
              {
              }
              if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
              {
                Error_Handler();
              }
            }
            if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART7)
            {
              while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
              {
              }
              if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
              {
                Error_Handler();
              }
            }
          }


        }//FREESMARTEEG_I2S_OPENVIBE_FREEEEG32_CUSTOM_INT

  	    if(FREESMARTEEG_OUT & FREESMARTEEG_SPI_TEXT_UART7_INT)
  	    {
            if(SPI_RxTxCplt)
          {
            	SPI_RxTxCplt=0;

            	  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            	  {
            	//                print2_symbol('0');
            	    aTxBuffer[0]=0x80;
            	    aTxBuffer[1]=0x00;
            	//              HAL_GPIO_WritePin(device->spi_dev.chip_select_port, device->spi_dev.chip_select_pin, GPIO_PIN_RESET);
            	//              if(HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
            //	    print7_symbol('+');
      //    	    if(HAL_SPI_Receive_DMA(&hspi2, datas[ad_adc], uint8_ad_number) != HAL_OK)
      //        	    if(HAL_SPI_Receive_DMA(&hspi1, datas[ad_adc], uint8_ad_number) != HAL_OK)

                    if(SPI_DMA)
                    {
                        if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], uint8_ad_number) != HAL_OK)
                        {
                          Error_Handler();
                        }
                    } else {
                        HAL_GPIO_WritePin(devices[ad_adc]->spi_dev.chip_select_port, devices[ad_adc]->spi_dev.chip_select_pin, GPIO_PIN_RESET);
                        if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], uint8_ad_number, 5000) != HAL_OK)
                        {
                          Error_Handler();
                        }
                        HAL_GPIO_WritePin(devices[ad_adc]->spi_dev.chip_select_port, devices[ad_adc]->spi_dev.chip_select_pin, GPIO_PIN_SET);
                    }


      //            if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], uint8_ad_number) != HAL_OK)
            	//                print2_symbol('1');
            	//                if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, data1, uint8_ad_number) != HAL_OK)
            	//                if(HAL_SPI_Receive_DMA(&hspi1, data1, uint8_ad_number) != HAL_OK)
            	//              if(HAL_SPI_TransmitReceive(device->spi_dev.dev, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
            //	    print7_symbol('-');
            	  }

                for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                {
                  while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
  //                while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
                  {
                  }
                }

  	        for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
  	        {
//  	        	for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
  		        	for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
  	        	{
  	        		print7_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
  	        		print7_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
  	        		print7_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
  	        		print7_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');

//                    CLEAR_BIT(hspi6.Instance->CR1, SPI_CR1_SSI);

//  	        		print7_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
//  	        		print7_binary(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//  	        		print7_binary(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//  	        		print7_binary(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');

//                    SET_BIT(hspi6.Instance->CR1, SPI_CR1_SSI);


  	        		print7_symbol(';');
  	        	}
  	        		print7_symbol(';');
  	        }
	        	print7_line();
          }
  	    }//FREESMARTEEG_SPI_TEXT_UART7_INT

        if(FREESMARTEEG_OUT & FREESMARTEEG_SPI_OPENVIBE_FREEEEG32_CUSTOM_INT)
//            if(0)//openbci
            if(SPI_RxTxCplt)
        {
            	SPI_RxTxCplt=0;


                if(1)
                {
                	  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//              	  for(int ad_adc = 0; ad_adc < 2; ad_adc ++)
              	  {
              	//                print2_symbol('0');
              	    aTxBuffer[0]=0x80;
              	    aTxBuffer[1]=0x00;

                    if(SPI_DMA)
                    {
                        if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], uint8_ad_number) != HAL_OK)
                        {
                          Error_Handler();
                        }
                    } else {
                        HAL_GPIO_WritePin(devices[ad_adc]->spi_dev.chip_select_port, devices[ad_adc]->spi_dev.chip_select_pin, GPIO_PIN_RESET);
                        if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], uint8_ad_number, 5000) != HAL_OK)
                        {
                          Error_Handler();
                        }
                        HAL_GPIO_WritePin(devices[ad_adc]->spi_dev.chip_select_port, devices[ad_adc]->spi_dev.chip_select_pin, GPIO_PIN_SET);
                    }

              	//              HAL_GPIO_WritePin(device->spi_dev.chip_select_port, device->spi_dev.chip_select_pin, GPIO_PIN_RESET);
              	//              if(HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
              //	    print7_symbol('+');
        //    	    if(HAL_SPI_Receive_DMA(&hspi2, datas[ad_adc], uint8_ad_number) != HAL_OK)
        //        	    if(HAL_SPI_Receive_DMA(&hspi1, datas[ad_adc], uint8_ad_number) != HAL_OK)
//                        if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], uint8_ad_number) != HAL_OK)
        //            if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], uint8_ad_number) != HAL_OK)
              	//                print2_symbol('1');
              	//                if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, data1, uint8_ad_number) != HAL_OK)
              	//                if(HAL_SPI_Receive_DMA(&hspi1, data1, uint8_ad_number) != HAL_OK)
              	//              if(HAL_SPI_TransmitReceive(device->spi_dev.dev, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
//              	    {
//              	      Error_Handler();
//              	    }
              //	    print7_symbol('-');
              	  }

                  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//                      for(int ad_adc = 0; ad_adc < 2; ad_adc ++)
                  {
                    while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
    //                while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
                    {
                    }
                  }


              //	    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
              //	    {
              //	      for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
              //	      {
              //	          print7_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
              //	          print7_binary(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
              //	          print7_binary(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
              //	          print7_binary(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
              //
              //	//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
              //	//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
              //	//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
              //	//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
              //	        print7_symbol(';');
              //	      }
              //	      print7_symbol(';');
              //	    }
              //	    print7_symbol(';');
              //
              //	    print7_line();

              //	  dataBuffer_print7[0]=';';
              //	  int uint8_data_number=1;
              //	  if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer_print7, uint8_data_number) != HAL_OK)
              //	  {
              //	    Error_Handler();
              //	  }

//              		SPI_RxTxCplt=1;
                }


          if(1)
          {
//                      const uint32_t uint8_data_number = 2 + 8 * 3 * 4 + 3 * 2 + 1;
            const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1;
            uint8_t dataBuffer[uint8_data_number];

            dataBuffer[0] = 0xA0;
            dataBuffer[1] = ui8SampleNumber++;

            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
            {
              for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
              {
                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 1];
                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 2];
                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 3];
              }
            }
            for(int accel_data_channel = 0; accel_data_channel < uint8_accel_chan_number; accel_data_channel ++)
            {
              dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 0] = 0;
              dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 1] = 0;
            }
//                    dataBuffer[2 + 8 * 3 * 4 + 3 * 2 + 0] = 0xC0;
            dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0] = 0xC0;

            if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
            {
              while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
              {
              }
              if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
              {
                Error_Handler();
              }
            }
            if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART7)
            {
              while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
              {
              }
              if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
              {
                Error_Handler();
              }
            }
          }


        }//FREESMARTEEG_SPI_OPENVIBE_FREEEEG32_CUSTOM_INT

        if(FREESMARTEEG_OUT & FREESMARTEEG_TEST_COUNTER2)
        {
            if(test_counter%1000==0)
//                if(test_counter%1000000==0)
        	  //        if(test_counter%10000<5000)
          {
            print_hex(test_counter, 32);
            print_symbol(';');
    //        print2_hex(test_counter, 32);
    //        print2_symbol(';');
          }
          test_counter++;
        }

//        if(FREESMARTEEG_OUT & FREESMARTEEG_TEST_BLINK2)
//        {
//          if(test_blink_counter%1000000<50000)
//    //        if(test_blink_counter%1000000<500000)
//    //        if(test_blink_counter%1000000<500000)
//          {
//            HAL_GPIO_WritePin(led_ports[3], led_pins[3], GPIO_PIN_SET);
//    //          HAL_GPIO_WritePin(led_ports[0], led_pins[0], GPIO_PIN_SET);
//          }
//          else
//          {
//            HAL_GPIO_WritePin(led_ports[3], led_pins[3], GPIO_PIN_RESET);
//    //          HAL_GPIO_WritePin(led_ports[0], led_pins[0], GPIO_PIN_RESET);
//          }
//          test_blink_counter++;
//        }

        //    data[0] = AD7779_REG_GENERAL_USER_CONFIG_1;
        //    data[1] = 0;
            if(0)
            {
              data1[0] = AD7779_REG_GENERAL_USER_CONFIG_3;
              data1[1] = AD7779_SPI_SLAVE_MODE_EN;
          //    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    //          HAL_SPI_Transmit(&hspi4, data1, 2, 5000);
//              HAL_SPI_Transmit_DMA(&hspi4, data1, 2);
          //    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
              data1[0] = AD7779_REG_GEN_ERR_REG_1_EN;
              data1[1] = AD7779_SPI_CRC_TEST_EN;
            //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    //          HAL_SPI_Transmit(&hspi4, data1, 2, 5000);
//              HAL_SPI_Transmit_DMA(&hspi4, data1, 2);
            //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            }

            if(0)
            {
              int ad_adc = 3;
    //          for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
              {
                aTxBuffer[0] = AD7779_REG_GENERAL_USER_CONFIG_3;
                aTxBuffer[1] = AD7779_SPI_SLAVE_MODE_EN;
    //            aTxBuffer[0]=0x80;
    //            aTxBuffer[1]=0x00;
      //              HAL_GPIO_WritePin(device->spi_dev.chip_select_port, device->spi_dev.chip_select_pin, GPIO_PIN_RESET);
      //              if(HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
      //                print2_symbol('1');
      //                if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, data1, uint8_ad_number) != HAL_OK)
      //                if(HAL_SPI_Receive_DMA(&hspi1, data1, uint8_ad_number) != HAL_OK)
      //              if(HAL_SPI_TransmitReceive(device->spi_dev.dev, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
                {
                  // Transfer error in transmission process
                  Error_Handler();
      //                  print2_symbol('?');
                }
                while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
    //                while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
                {
                }
              }

            }

        //    for(int ad_data_byte = 0; ad_data_byte < uint8_ad_number; ad_data_byte ++)
        //    {
        //      data[ad_data_byte] = 0;
        //    }

        //    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        //    HAL_Delay(50);
        //    HAL_SPI_TransmitReceive_DMA(&hspi1, data, data, 2);
        //    HAL_SPI_TransmitReceive(&hspi1, data, data, 2, 5000);

//            print7_symbol(';');
//            print7_line();

            if(FREESMARTEEG_OUT & FREESMARTEEG_DEVICES_STATE)
            {
    //          int ad_adc = 2;
              for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
              {
//                  data1[0] = 0x80 | AD7779_REG_CH_CONFIG(0);
                  data1[0] = 0x80 | AD7779_REG_GENERAL_USER_CONFIG_1;
//                  data1[0] = 0x80 | AD7779_REG_GENERAL_USER_CONFIG_2;
//                  data1[0] = 0x80 | AD7779_REG_GENERAL_USER_CONFIG_3;
//                  data1[0] = 0x80 | AD7779_REG_DOUT_FORMAT;
//                  data1[0] = 0x80 | AD7779_REG_BUFFER_CONFIG_1;
//                  data1[0] = 0x80 | AD7779_REG_BUFFER_CONFIG_2;
//                  data1[0] = 0x80 | AD7779_REG_CHX_ERR_REG_EN;
//                  data1[0] = 0x80 | AD7779_REG_CH_ERR_REG(0);
//                  data1[0] = 0x80 | AD7779_REG_CH0_1_SAT_ERR;
//                  data1[0] = 0x80 | AD7779_REG_GEN_ERR_REG_1_EN;
//                  data1[0] = 0x80 | AD7779_REG_GEN_ERR_REG_1;
//                  data1[0] = 0x80 | AD7779_REG_GEN_ERR_REG_2_EN;
//                  data1[0] = 0x80 | AD7779_REG_GEN_ERR_REG_2;
//                  data1[0] = 0x80 | AD7779_REG_STATUS_REG_1;
//                  data1[0] = 0x80 | AD7779_REG_STATUS_REG_2;
//                  data1[0] = 0x80 | AD7779_REG_STATUS_REG_3;
                data1[1] = 0;

//                print_binary(data1[0], 8);    print_symbol(';');
//                print_binary(data1[1], 8);    print_symbol(';');
//                print_symbol(';');
                print7_binary(data1[0], 8);    print7_symbol(';');
                print7_binary(data1[1], 8);    print7_symbol(';');
                print7_symbol(';');

          //      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
      //         spi_write_and_read(0, data, 2);
          //      spi_write_and_read(0, data, 2);
//                if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, data1, data1, 2,5000) != HAL_OK)

                if(SPI_DMA)
                {
                    if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, data1, data1, 2) != HAL_OK)
                    {
                      Error_Handler();
                    }
                } else {
                    HAL_GPIO_WritePin(devices[ad_adc]->spi_dev.chip_select_port, devices[ad_adc]->spi_dev.chip_select_pin, GPIO_PIN_RESET);
                    if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, data1, data1, 2, 5000) != HAL_OK)
                    {
                      Error_Handler();
                    }
                    HAL_GPIO_WritePin(devices[ad_adc]->spi_dev.chip_select_port, devices[ad_adc]->spi_dev.chip_select_pin, GPIO_PIN_SET);
                }


      //                print2_symbol('1');
      //                if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, data1, uint8_ad_number) != HAL_OK)
      //                if(HAL_SPI_Receive_DMA(&hspi1, data1, uint8_ad_number) != HAL_OK)
      //              if(HAL_SPI_TransmitReceive(device->spi_dev.dev, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
                while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
      //                while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
                {
                }
      //          HAL_SPI_TransmitReceive(&hspi1, data1, data1, 2, 5000);
            //    HAL_SPI_Transmit(&hspi1, data, 2, 5000);
          //      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

//                print_binary(data1[0], 8);    print_symbol(';');
//                print_binary(data1[1], 8);    print_symbol(';');
//                print_symbol(';');
//                print_symbol(';');
                print7_binary(data1[0], 8);    print7_symbol(';');
                print7_binary(data1[1], 8);    print7_symbol(';');
                print7_symbol(';');
                print7_symbol(';');

            //    print_line();
              }
//              print_line();
              print7_line();
            }

        //    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        //    HAL_Delay(50);
        //    print_text_line("hello world!");
        //    print_symbol('!');
        //    print_binary(data[0], 8);
        //    print_line();
        //    HAL_UART_Transmit(&huart1, data, 1, 5000);

        //    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        //    HAL_Delay(200);
        //    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        //    HAL_Delay(200);

    //        int drdy_pin = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);

            if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_DATA_READ)
            {

                int drdy_pin = HAL_GPIO_ReadPin(AD_DRDY_GPIO_Port, AD_DRDY_Pin);
//                print7_hex(drdy_pin, 1);
//                print7_symbol(';');
                int drdy_count=0;
                while (drdy_pin == 0)
                {
                	drdy_pin = HAL_GPIO_ReadPin(AD_DRDY_GPIO_Port, AD_DRDY_Pin);
//                    print7_hex(drdy_pin, 1);
//                    print7_symbol(';');
                	drdy_count++;
                }

//                ad_adc1=4;
////                ad_adc1=3;
////                ad_adc1=0x35A;
////                ad_adc1=0x15A0;
////                ad_adc1=0x7b0;
////                ad_adc1=0x800;
////                for(int ad_adc2 = 0; ad_adc2 < 0xF00/100; ad_adc2 ++)
//                for(int ad_adc2 = 0; ad_adc2 < ad_adc1; ad_adc2 ++)
//                    for(int ad_adc2 = 0; ad_adc2 < ad_adc1; ad_adc2 ++)
//                {
//                }
//                ad_adc1++;

__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
////
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();

//                int ad_adc = 0;
                for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                {
  //                print2_symbol('0');
                  aTxBuffer[0]=0x80;
                  aTxBuffer[1]=0x00;
    //              HAL_GPIO_WritePin(device->spi_dev.chip_select_port, device->spi_dev.chip_select_pin, GPIO_PIN_RESET);
    //              if(HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
//                  if(HAL_SPI_Receive_DMA(&hspi3, datas[ad_adc], uint8_ad_number) != HAL_OK)
//                  if(HAL_SPI_Receive_DMA(&hspi1, datas[ad_adc], uint8_ad_number) != HAL_OK)
  //                print2_symbol('1');
  //                if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, data1, uint8_ad_number) != HAL_OK)
  //                if(HAL_SPI_Receive_DMA(&hspi1, data1, uint8_ad_number) != HAL_OK)
    //              if(HAL_SPI_TransmitReceive(device->spi_dev.dev, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
                  {
                    Error_Handler();
                  }
                }

                for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                {
//                    while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY)
//                  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
  //                while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
                  {
                  }
                }



                if(FREESMARTEEG_OUT & FREESMARTEEG_DATA_TEXT_UART7)
    //            if(0)
                {

    //              int crc_pair_ok[uint8_ad_adc_number][4];


//                  print7_hex(ad_adc1, 32);//          print2_symbol(';');
//                  print7_symbol(';');
                  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                  {
                    for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
                    {
                        print7_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
                        print7_binary(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
                        print7_binary(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
                        print7_binary(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');

//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
                      print7_symbol(';');
                    }
                    print7_symbol(';');
                  }
                  print7_symbol(';');

                  print7_line();

                }//FREESMARTEEG_DATA_TEXT_UART7

                if(FREESMARTEEG_OUT & FREESMARTEEG_DATA_OPENVIBE_FREEEEG32)
    //            if(0)//openbci
                {

                  if(1)
                  {
                      const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 1;
//                    const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1;
                    uint8_t dataBuffer[uint8_data_number];

                    dataBuffer[0] = 0xA0;
                    dataBuffer[1] = ui8SampleNumber++;

                    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                    {
                      for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
                      {
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 1];
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 2];
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 3];
                      }
                    }
                    for(int accel_data_channel = 0; accel_data_channel < uint8_accel_chan_number; accel_data_channel ++)
                    {
                      dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 0] = 0;
                      dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 1] = 0;
                    }
                    dataBuffer[2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 0] = 0xC0;
//                    dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0] = 0xC0;

                    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
                    {
                      while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
                      {
                      }
                      if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                      {
                        Error_Handler();
                      }
                    }
                    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART7)
                    {
                      while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
                      {
                      }
                      if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                      {
                        Error_Handler();
                      }
                    }
                  }


                }//FREESMARTEEG_DATA_OPENVIBE_FREEEEG32

                if(FREESMARTEEG_OUT & FREESMARTEEG_DATA_OPENVIBE_FREEEEG32_CUSTOM)
    //            if(0)//openbci
                {

                  if(1)
                  {
//                      const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 1;
                    const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1;
                    uint8_t dataBuffer[uint8_data_number];

                    dataBuffer[0] = 0xA0;
                    dataBuffer[1] = ui8SampleNumber++;

                    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                    {
                      for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
                      {
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 1];
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 2];
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 3];
                      }
                    }
                    for(int accel_data_channel = 0; accel_data_channel < uint8_accel_chan_number; accel_data_channel ++)
                    {
                      dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 0] = 0;
                      dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 1] = 0;
                    }
//                    dataBuffer[2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 0] = 0xC0;
                    dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0] = 0xC0;

                    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
                    {
                      while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
                      {
                      }
                      if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                      {
                        Error_Handler();
                      }
                    }
                    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART7)
                    {
                      while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
                      {
                      }
                      if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                      {
                        Error_Handler();
                      }
                    }
                  }


                }//FREESMARTEEG_DATA_OPENVIBE_FREEEEG32_CUSTOM
            }


            if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_SPI_READ)
//           if((FREESMARTEEG_ADC & FREESMARTEEG_ADC_SPI_READ( && (!(FREESMARTEEG_ADC & FREESMARTEEG_ADC_SPI_READ_INT)))
           {

               int drdy_pin = HAL_GPIO_ReadPin(AD_DRDY_GPIO_Port, AD_DRDY_Pin);
//               int drdy_pin = HAL_GPIO_ReadPin(AD1_DRDY_GPIO_Port, AD1_DRDY_Pin);
    //        int drdy2_pin = HAL_GPIO_ReadPin(AD2_DRDY_GPIO_Port, AD2_DRDY_Pin);
    //        int drdy3_pin = HAL_GPIO_ReadPin(AD3_DRDY_GPIO_Port, AD3_DRDY_Pin);
//            int drdy4_pin = HAL_GPIO_ReadPin(AD4_DRDY_GPIO_Port, AD4_DRDY_Pin);
//            print7_hex(drdy_pin, 1);
//            print7_symbol(';');
            int drdy_count=0;
            while (drdy_pin == 0)
    //        while (drdy2_pin == 0)
    //        while (drdy3_pin == 0)
//            while (drdy4_pin == 0)
    //        while ((drdy_pin == 0)||(drdy2_pin == 0)||(drdy3_pin == 0)||(drdy4_pin == 0))
    //        while ((drdy_pin == 0)&&(drdy2_pin == 0)&&(drdy3_pin == 0)&&(drdy4_pin == 0))
    //        while ((drdy_pin == 0)||(drdy2_pin == 0))
            {
                drdy_pin = HAL_GPIO_ReadPin(AD_DRDY_GPIO_Port, AD_DRDY_Pin);
//                drdy_pin = HAL_GPIO_ReadPin(AD1_DRDY_GPIO_Port, AD1_DRDY_Pin);
    //          drdy2_pin = HAL_GPIO_ReadPin(AD2_DRDY_GPIO_Port, AD2_DRDY_Pin);
    //          drdy3_pin = HAL_GPIO_ReadPin(AD3_DRDY_GPIO_Port, AD3_DRDY_Pin);
//              drdy4_pin = HAL_GPIO_ReadPin(AD4_DRDY_GPIO_Port, AD4_DRDY_Pin);
    //          if(!drdy_pin)
    //          {
    //            drdy_pin = HAL_GPIO_ReadPin(AD_DRDY_GPIO_Port, AD_DRDY_Pin);
    //          }
    //          if(!drdy2_pin)
    //          {
    //            drdy2_pin = HAL_GPIO_ReadPin(AD2_DRDY_GPIO_Port, AD2_DRDY_Pin);
    //          }
    //          if(!drdy3_pin)
    //          {
    //            drdy3_pin = HAL_GPIO_ReadPin(AD3_DRDY_GPIO_Port, AD3_DRDY_Pin);
    //          }
    //          if(!drdy4_pin)
    //          {
    //            drdy4_pin = HAL_GPIO_ReadPin(AD4_DRDY_GPIO_Port, AD4_DRDY_Pin);
    //          }
//              print7_hex(drdy_pin, 1);
//              print7_symbol(';');
              drdy_count++;
    //          if(drdy_count>2000)
    //          {
    //            drdy_pin=-1;
    ////            drdy_count=0;
    ////            print2_symbol(';');
    ////            print2_line();
    //          }
            }

    //        int drdy2_pin = HAL_GPIO_ReadPin(AD2_DRDY_GPIO_Port, AD2_DRDY_Pin);
    //        while ((drdy2_pin == 0))
    //        {
    //          drdy2_pin = HAL_GPIO_ReadPin(AD2_DRDY_GPIO_Port, AD2_DRDY_Pin);
    //        }

    //        int miso = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
        //    if ((drdy_pin == 0))
            {   // ADC Converter ready ?
        //      if (miso == 0)
              {   // ADC Converter ready ?

        //        spi_write_and_read(0, data, uint8_ad_number);
        //        HAL_SPI_TransmitReceive_DMA(&hspi1, data, data, uint8_ad_number);
        //        HAL_SPI_TransmitReceive(&hspi1, data, data, uint8_ad_number, 5000);

        //        HAL_SPI_Receive(&hspi1, data, uint8_ad_number, 5000);
        //        HAL_SPI_Receive_DMA(&hspi1, data, uint8_ad_number);


                if(0)
                {
                  /*##-2- Start the Full Duplex Communication process ########################*/
                  /* While the SPI in TransmitReceive process, user can transmit data through
                     "aTxBuffer" buffer & receive data through "aRxBuffer" */
                  //        HAL_SPI_TransmitReceive_DMA(&hspi1, data, data, uint8_ad_number);

          //        if(HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)aRxBuffer, uint8_ad_number) != HAL_OK)
        //          if(HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)data, uint8_ad_number) != HAL_OK)
                  aTxBuffer[0]=0x80;
                  aTxBuffer[1]=0x00;
                  if(SPI_DMA)
                  {
//                      if(HAL_SPI_TransmitReceive_DMA(&hspi3, aTxBuffer, data1, uint8_ad_number) != HAL_OK)
//                    if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, data1, uint8_ad_number) != HAL_OK)
          //          if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, aRxBuffer, uint8_ad_number) != HAL_OK)
                    {
                      // Transfer error in transmission process
                      Error_Handler();
                    }
                  }
                  else
                  {
//                      if(HAL_SPI_TransmitReceive(&hspi3, aTxBuffer, data1, uint8_ad_number, 5000) != HAL_OK)
//                          if(HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, data1, uint8_ad_number, 5000) != HAL_OK)
          //          if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, aRxBuffer, uint8_ad_number) != HAL_OK)
                    {
                      // Transfer error in transmission process
                      Error_Handler();
                    }
                  }


                  /*##-3- Wait for the end of the transfer ###################################*/
                  /*  Before starting a new communication transfer, you need to check the current
                      state of the peripheral; if it’s busy you need to wait for the end of current
                      transfer before starting a new one.
                      For simplicity reasons, this example is just waiting till the end of the
                      transfer, but application may perform other tasks while transfer operation
                      is ongoing. */
//                  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
//                      while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
                  {
                  }

          //        for(int ad_data_byte = 0; ad_data_byte < uint8_ad_number; ad_data_byte ++)
          //        {
          //          data[ad_data_byte] = aRxBuffer[ad_data_byte];
          //        }

                }
                else
                {

    //              while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    //              {
    //              }

                  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                  {
    //                print2_symbol('0');
                    aTxBuffer[0]=0x80;
                    aTxBuffer[1]=0x00;
      //              HAL_GPIO_WritePin(device->spi_dev.chip_select_port, device->spi_dev.chip_select_pin, GPIO_PIN_RESET);
      //              if(HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
                    if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], uint8_ad_number) != HAL_OK)
    //                print2_symbol('1');
    //                if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, data1, uint8_ad_number) != HAL_OK)
    //                if(HAL_SPI_Receive_DMA(&hspi1, data1, uint8_ad_number) != HAL_OK)
      //              if(HAL_SPI_TransmitReceive(device->spi_dev.dev, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
                    {
                      // Transfer error in transmission process
                      Error_Handler();
    //                  print2_symbol('?');
                    }
    //                print2_symbol('2');
      //              HAL_SPI_Receive(&hspi1, data, uint8_ad_number, 5000);
      //              HAL_GPIO_WritePin(device->spi_dev.chip_select_port, device->spi_dev.chip_select_pin, GPIO_PIN_SET);

      //              while (HAL_SPI_GetState(device->spi_dev.dev) != HAL_SPI_STATE_READY)
      //              {
      //              }

                  }

    ////              HAL_SPI_Receive(device.spi_dev.dev, data, uint8_ad_number, 5000);
    //
    //              aTxBuffer[0]=0x80;
    //              aTxBuffer[1]=0x00;
    ////              HAL_GPIO_WritePin(device->spi_dev.chip_select_port, device->spi_dev.chip_select_pin, GPIO_PIN_RESET);
    ////              if(HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
    //              if(HAL_SPI_TransmitReceive_DMA(device1->spi_dev.dev, aTxBuffer, data1, uint8_ad_number) != HAL_OK)
    ////              if(HAL_SPI_TransmitReceive(device->spi_dev.dev, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
    //              {
    //                // Transfer error in transmission process
    //                Error_Handler();
    //              }
    ////              HAL_SPI_Receive(&hspi1, data, uint8_ad_number, 5000);
    ////              HAL_GPIO_WritePin(device->spi_dev.chip_select_port, device->spi_dev.chip_select_pin, GPIO_PIN_SET);
    //
    ////              while (HAL_SPI_GetState(device->spi_dev.dev) != HAL_SPI_STATE_READY)
    ////              {
    ////              }
    //
    //              aTxBuffer[0]=0x80;
    //              aTxBuffer[1]=0x00;
    //
    ////              HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_RESET);
    ////              if(HAL_SPI_Transmit(&hspi1, aTxBuffer, uint8_ad_number, 5000) != HAL_OK)
    //              if(HAL_SPI_TransmitReceive_DMA(device2->spi_dev.dev, aTxBuffer, data2, uint8_ad_number) != HAL_OK)
    ////              if(HAL_SPI_TransmitReceive(device2->spi_dev.dev, aTxBuffer, data2, uint8_ad_number, 5000) != HAL_OK)
    ////              if(HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, data2, uint8_ad_number, 5000) != HAL_OK)
    //              {
    //                // Transfer error in transmission process
    //                Error_Handler();
    //              }
    ////              HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_SET);
    //
    ////              while (HAL_SPI_GetState(device2->spi_dev.dev) != HAL_SPI_STATE_READY)
    ////              {
    ////              }
    //
    //              aTxBuffer[0]=0x80;
    //              aTxBuffer[1]=0x00;
    //
    ////              HAL_GPIO_WritePin(device3->spi_dev.chip_select_port, device3->spi_dev.chip_select_pin, GPIO_PIN_RESET);
    ////              if(HAL_SPI_Transmit(&hspi1, aTxBuffer, uint8_ad_number, 5000) != HAL_OK)
    //              if(HAL_SPI_TransmitReceive_DMA(device3->spi_dev.dev, aTxBuffer, data3, uint8_ad_number) != HAL_OK)
    ////              if(HAL_SPI_TransmitReceive(device3->spi_dev.dev, aTxBuffer, data3, uint8_ad_number, 5000) != HAL_OK)
    ////              if(HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, data2, uint8_ad_number, 5000) != HAL_OK)
    //              {
    //                // Transfer error in transmission process
    //                Error_Handler();
    //              }
    ////              HAL_GPIO_WritePin(device3->spi_dev.chip_select_port, device3->spi_dev.chip_select_pin, GPIO_PIN_SET);
    //
    ////              while (HAL_SPI_GetState(device3->spi_dev.dev) != HAL_SPI_STATE_READY)
    ////              {
    ////              }
    //
    //              aTxBuffer[0]=0x80;
    //              aTxBuffer[1]=0x00;
    //
    ////              HAL_GPIO_WritePin(device4->spi_dev.chip_select_port, device4->spi_dev.chip_select_pin, GPIO_PIN_RESET);
    ////              if(HAL_SPI_Transmit(&hspi1, aTxBuffer, uint8_ad_number, 5000) != HAL_OK)
    //              if(HAL_SPI_TransmitReceive_DMA(device4->spi_dev.dev, aTxBuffer, data4, uint8_ad_number) != HAL_OK)
    ////              if(HAL_SPI_TransmitReceive(device4->spi_dev.dev, aTxBuffer, data4, uint8_ad_number, 5000) != HAL_OK)
    ////              if(HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, data2, uint8_ad_number, 5000) != HAL_OK)
    //              {
    //                // Transfer error in transmission process
    //                Error_Handler();
    //              }
    ////              HAL_GPIO_WritePin(device4->spi_dev.chip_select_port, device4->spi_dev.chip_select_pin, GPIO_PIN_SET);
    //
    ////              while (HAL_SPI_GetState(device4->spi_dev.dev) != HAL_SPI_STATE_READY)
    ////              {
    ////              }

                  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                  {
                    while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
    //                while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
                    {
                    }
                  }
    //              HAL_GPIO_WritePin(AD_START_GPIO_Port, AD_START_Pin, GPIO_PIN_SET);
    //              HAL_Delay(1);
    //              HAL_GPIO_WritePin(AD_START_GPIO_Port, AD_START_Pin, GPIO_PIN_RESET);


                }

                data_counter++;
              }

                if(FREESMARTEEG_OUT & FREESMARTEEG_OPENBCI_TELNET_PACKET)
    //            if(0)//openbci
                {

                  crc8_01=crc8da_7(&(datas[0][0]));

                  if(
                      ((!(((uint8_t)crc8_01)-((uint8_t)((datas[0][0]<<4)|(datas[0][4]&0x0f)))))
                     ||(!(((uint8_t)crc8_01)-((~((uint8_t)((datas[0][0]<<4)|(datas[0][4]&0x0f))))&0b00000111)))
                      )
                  &&(
                    (//((datas[0][0]>=0b10000000)&&(datas[0][0]<0b10010000))||
                        ((datas[0][0]>=0b00000000)&&(datas[0][0]<0b00010000)))
                  &&(//((datas[0][4]>=0b10010000)&&(datas[0][4]<0b10100000))||
                      ((datas[0][4]>=0b00010000)&&(datas[0][4]<0b00100000)))
                  ))
                  {
//                    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
//                    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
                  }
                  else
                  {
//                    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
//                    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
                  }

    //              crc8_01=crc8da_7(&(datas[1][0]));
    //
    //              if(
    //                  ((!(((uint8_t)crc8_01)-((uint8_t)((datas[1][0]<<4)|(datas[1][4]&0x0f)))))
    //                 ||(!(((uint8_t)crc8_01)-((~((uint8_t)((datas[1][0]<<4)|(datas[1][4]&0x0f))))&0b00000111)))
    //                  )
    //              &&(
    //                (//((datas[1][0]>=0b10000000)&&(datas[1][0]<0b10010000))||
    //                    ((datas[1][0]>=0b00000000)&&(datas[1][0]<0b00010000)))
    //              &&(//((datas[1][4]>=0b10010000)&&(datas[1][4]<0b10100000))||
    //                  ((datas[1][4]>=0b00010000)&&(datas[1][4]<0b00100000)))
    //              ))
    //              {
    ////                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    //              }
    //              else
    //              {
    ////                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    //              }

    //              crc8_01=crc8da_7(&(data[0]));
    //              crc8_23=crc8da_7(&(data[8]));
    //              crc8_45=crc8da_7(&(data[16]));
    //              crc8_67=crc8da_7(&(data[24]));
    //
    //              if(((!(((uint8_t)crc8_01)-((uint8_t)((data[0]<<4)|(data[4]&0x0f)))))
    //                &&(!(((uint8_t)crc8_23)-((uint8_t)((data[8]<<4)|(data[12]&0x0f)))))
    //                &&(!(((uint8_t)crc8_45)-((uint8_t)((data[16]<<4)|(data[20]&0x0f)))))
    //                &&(!(((uint8_t)crc8_67)-((uint8_t)((data[24]<<4)|(data[28]&0x0f))))))
    //              && (
    //                (((data[0]>=0b10000000)&&(data[0]<0b10010000))||((data[0]>=0b00000000)&&(data[0]<0b00010000)))
    //              &&(((data[4]>=0b10010000)&&(data[4]<0b10100000))||((data[4]>=0b00010000)&&(data[4]<0b00100000)))
    //              &&(((data[8]>=0b10100000)&&(data[8]<0b10110000))||((data[8]>=0b00100000)&&(data[8]<0b00110000)))
    //              &&(((data[12]>=0b10110000)&&(data[12]<0b11000000))||((data[12]>=0b00110000)&&(data[12]<0b01000000)))
    //              &&(((data[16]>=0b11000000)&&(data[16]<0b11010000))||((data[16]>=0b01000000)&&(data[16]<0b01010000)))
    //              &&(((data[20]>=0b11010000)&&(data[20]<0b11100000))||((data[20]>=0b01010000)&&(data[20]<0b01100000)))
    //              &&(((data[24]>=0b11100000)&&(data[24]<0b11110000))||((data[24]>=0b01100000)&&(data[24]<0b01110000)))
    //              &&(((data[28]>=0b11110000)&&(data[28]<=0b11111111))||((data[28]>=0b01110000)&&(data[28]<=0b01111111)))
    //              ))
                  if(1)
                  {
    //                const uint32_t uint8_data_number = 2 + 8 * 3 + 3 * 2 + 1;
    //                const uint32_t uint8_data_number = 2 + 8 * 3 + 8 * 3 + 3 * 2 + 1;
    //                uint8_t dataBuffer[uint8_data_number];

                    telnet_dataBuffer[telnet_packet_data_number_now][0] = 0xA0;
                    telnet_dataBuffer[telnet_packet_data_number_now][1] = ui8SampleNumber++;

                    for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
                    {
                      telnet_dataBuffer[telnet_packet_data_number_now][2 + ad_data_channel * 3 + 0] = datas[0][ad_data_channel * 4 + 1];
                      telnet_dataBuffer[telnet_packet_data_number_now][2 + ad_data_channel * 3 + 1] = datas[0][ad_data_channel * 4 + 2];
                      telnet_dataBuffer[telnet_packet_data_number_now][2 + ad_data_channel * 3 + 2] = datas[0][ad_data_channel * 4 + 3];
                    }
    //                for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
    //                {
    //                  dataBuffer[2 + ad_data_channel * 3 + 0] = data2[ad_data_channel * 4 + 1];
    //                  dataBuffer[2 + ad_data_channel * 3 + 1] = data2[ad_data_channel * 4 + 2];
    //                  dataBuffer[2 + ad_data_channel * 3 + 2] = data2[ad_data_channel * 4 + 3];
    //                }
                    for(int accel_data_channel = 0; accel_data_channel < 3; accel_data_channel ++)
                    {
                      telnet_dataBuffer[telnet_packet_data_number_now][2 + 8 * 3 + accel_data_channel * 2 + 0] = 0;
                      telnet_dataBuffer[telnet_packet_data_number_now][2 + 8 * 3 + accel_data_channel * 2 + 1] = 0;
                    }
                    telnet_dataBuffer[telnet_packet_data_number_now][2 + 8 * 3 + 3 * 2 + 0] = 0xC0;


                    if((data_counter%telnet_data_number_max==0))
                    {
                      if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
                      {
                        while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
                        {
                        }
                        if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)telnet_dataBuffer, telnet_packet_data_number_max*telnet_data_number_max) != HAL_OK)
                        {
                          Error_Handler();
                        }
                      }
    //                  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
    //                  {
    //                    while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
    //                    {
    //                    }
    //                    if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)telnet_dataBuffer, telnet_packet_data_number_max*telnet_data_number_max) != HAL_OK)
    //                    {
    //                      Error_Handler();
    //                    }
    //                  }
                      telnet_packet_data_number_now=0;
                    }
                    else
                    {
                      telnet_packet_data_number_now++;
                    }


    //                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    //              }
    //              else
    //              {
    //                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
                  }


                }//FREESMARTEEG_OPENBCI_TELNET_PACKET

    //            if(FREESMARTEEG_OUT & FREESMARTEEG_OPENVIBE_TELNET_PACKET)
    //            {
    //
    ////              int crc_pair_ok[uint8_ad_adc_number][4];
    //
    //              for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    //              {
    //                int crc_ok = 1;
    //                for(int ad_data_channel_pair = 0; ad_data_channel_pair < 4; ad_data_channel_pair ++)
    //                {
    ////                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 1];
    ////                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 2];
    ////                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 3];
    //
    //                  uint8_t crc8_=crc8da_7(&(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]));
    //
    ////                  uint8_t crc1=((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    ////                      |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)));
    ////                  uint8_t chan_0=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b01110000)>>4);
    ////                  uint8_t chan_1=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b01110000)>>4);
    ////                  print2_hex(chan_0,8);
    ////                  print2_symbol(':');
    ////                  print2_hex(crc8_,8);
    ////                  print2_symbol('-');
    ////                  print2_hex(crc1,8);
    //////                  print2_symbol(';');
    ////                  print2_symbol(',');
    ////                  print2_hex((uint8_t)(!(((uint8_t)crc8_)
    ////                      -((uint8_t)crc1))),8);
    ////                  print2_symbol(',');
    ////                  print2_hex((uint8_t)!(((uint8_t)crc8_)
    ////                     -((~((uint8_t)crc1))&0b00000111)),8);
    ////                  print2_symbol(',');
    ////                  print2_hex((uint8_t)(chan_0==ad_data_channel_pair*2),8);
    ////                  print2_symbol(',');
    ////                  print2_hex((uint8_t)(chan_1==(ad_data_channel_pair*2+1)),8);
    ////                  print2_symbol(';');
    ////                  if(
    ////                      ((!(((uint8_t)crc8_)
    ////                          -((uint8_t)crc1)))
    ////                     ||(!(((uint8_t)crc8_)
    ////                         -((~((uint8_t)crc1))&0b00000111)))
    ////                      )
    ////                  &&(
    ////                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
    ////                        ((chan_0==ad_data_channel_pair)))
    ////                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
    ////                      ((chan_1==(ad_data_channel_pair+1))))
    ////                  ))
    //                  if(
    //                      ((!(((uint8_t)crc8_)
    //                          -((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    //                              |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)))))
    //                     ||(!(((uint8_t)crc8_)
    //                         -((~((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    //                             |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f))))&0b00000111)))
    //                      )
    //                  &&(
    //                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
    //                        ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b11110000)>>4)==ad_data_channel_pair*2)))
    //                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
    //                      ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b11110000)>>4)==ad_data_channel_pair*2+1)))
    //                  ))
    //                  {
    ////                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 1;
    //                  }
    //                  else
    //                  {
    ////                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 0;
    //                    crc_ok = 0;
    //                  }
    //
    //                }
    //                if(crc_ok)
    //                {
    ////                  print2_symbol('1');
    //                  HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
    //  //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    //                }
    //                else
    //                {
    ////                  print2_symbol('0');
    //                  HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
    //  //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    //
    //                }
    ////                print2_symbol(';');
    //              }
    //
    //              int ad_adc = 0;
    ////              for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    //              {
    //                int ad_data_channel = 0;
    ////                for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
    //                {
    ////                  print2_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);          print2_symbol(';');
    ////                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    ////                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    ////                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    ////                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
    //      //            print_binary(data[ad_data_channel * 4 + 1], 8);          print_symbol(';');
    //      //            print_binary(data[ad_data_channel * 4 + 2], 8);          print_symbol(';');
    //      //            print_binary(data[ad_data_channel * 4 + 3], 8);          print_symbol(';');
    ////                  print2_symbol(';');
    //                  datas_out[ad_adc][ad_data_channel * 4 + 0]=datas[ad_adc][ad_data_channel * 4 + 1]&0b10000000;
    //                  datas_out[ad_adc][ad_data_channel * 4 + 1]=datas[ad_adc][ad_data_channel * 4 + 1]&0b01111111;
    //                  datas_out[ad_adc][ad_data_channel * 4 + 2]=datas[ad_adc][ad_data_channel * 4 + 2];
    //                  datas_out[ad_adc][ad_data_channel * 4 + 3]=datas[ad_adc][ad_data_channel * 4 + 3];
    //
    ////                  const int sample_number_max = 240;
    ////                  int sample[sample_number_max];
    ////                  int sample_number_last = -1;
    ////                  int sample_average = 0;
    ////
    ////                  int sample_int_out;
    ////                  uint8_t sample_byte_out;
    ////                  double sample_int_byte_scale;
    ////                  int sample_int_byte_shift;
    //
    //                  sample_int_out = (int)(datas_out[ad_adc][ad_data_channel * 4 + 0]<<24)
    //                          +(int)(datas_out[ad_adc][ad_data_channel * 4 + 1]<<16)
    //                          +(int)(datas_out[ad_adc][ad_data_channel * 4 + 2]<<8)
    //                          +(int)(datas_out[ad_adc][ad_data_channel * 4 + 3]<<0);
    //                  if(sample_number_now==sample_number_max)
    //                  {
    //                    for(int sample_count=0;sample_count<sample_number_now-1;sample_count++)
    //                    {
    //                      sample_int[sample_count]=sample_int[sample_count+1];
    //                      sample_byte[sample_count]=sample_byte[sample_count+1];
    //                    }
    //                  }
    //                  else
    //                  {
    //                    sample_number_now++;
    //                  }
    //                  sample_int[sample_number_now-1]=sample_int_out;
    //
    ////                  if((test_counter%sample_number_max==0)||(test_counter<sample_number_max))
    //                  {
    //                    sample_sum=0;
    //
    //                    int sample_max_new=sample_int[0];
    //                    int sample_min_new=sample_int[0];
    //
    //                    for(int sample_count=0;sample_count<sample_number_now;sample_count++)
    //                    {
    //                      sample_sum+=sample_int[sample_count];
    //                      if(sample_max_new<sample_int[sample_count])
    //                      {
    //                        sample_max_new=sample_int[sample_count];
    //                      }
    //                      if(sample_min_new>sample_int[sample_count])
    //                      {
    //                        sample_min_new=sample_int[sample_count];
    //                      }
    //                    }
    ////                    sample_average=sample_sum/sample_number_now;
    //
    //                    if(((sample_max_last<sample_max_new)||(sample_min_last>sample_min_new))||
    //                        ((sample_max_last>sample_max_new+(sample_max+sample_min)/2)
    //                            ||(sample_min_last<sample_min_new-(sample_max+sample_min)/2)))
    //                    {
    //                      sample_max=sample_max_new;
    //                      sample_min=sample_min_new;
    //                    }
    //
    //                    sample_max_last=sample_max;
    //                    sample_min_last=sample_min;
    //
    //                  }
    //
    //
    ////                  sample_int_byte_shift = (1<<8)/(1<<32);
    ////                  sample_byte_out = sample_int_out + sample_int_byte_shift;
    ////                  sample_int_byte_scale = 1;//(1<<8)/(1<<32);
    ////                  sample_byte_out = (uint8_t)(sample_int_out * sample_int_byte_scale);
    //                  sample_byte_out = (uint8_t)((sample_int_out-sample_min)*255/(sample_max-sample_min+1));
    //                  sample_byte[sample_number_now-1]=sample_byte_out;
    //
    ////                  print2_symbol(sample_byte_out);
    ////                  print2_hex(sample_byte_out, 8);//          print2_symbol(';');
    ////                  print2_line();
    //
    //                  if((test_counter%sample_number_max==0)||(test_counter<sample_number_max))
    //                  {
    //                    for(int sample_count=0;sample_count<sample_number_now;sample_count++)
    //                    {
    //
    //                      sample_dataBuffer[sample_count]=sample_byte[sample_count];
    //                    }
    //
    //                    while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
    //                    {
    //                    }
    //                    if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)sample_dataBuffer, sample_data_number) != HAL_OK)
    //                    {
    //                      Error_Handler();
    //                    }
    //                  }
    //                }
    //              }
    //
    //            }//FREESMARTEEG_OPENVIBE_TELNET_PACKET


                if(FREESMARTEEG_OUT & FREESMARTEEG_CHANNELS_STATE)
                {
                  if(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_PIN)
                  {
    //              int crc_pair_ok[uint8_ad_adc_number][4];

                    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                    {
                      int crc_ok = 1;
                      for(int ad_data_channel_pair = 0; ad_data_channel_pair < 4; ad_data_channel_pair ++)
                      {
      //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 1];
      //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 2];
      //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 3];

                        uint8_t crc8_=crc8da_7(&(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]));

      //                  uint8_t crc1=((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
      //                      |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)));
      //                  uint8_t chan_0=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b01110000)>>4);
      //                  uint8_t chan_1=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b01110000)>>4);
      //                  print2_hex(chan_0,8);
      //                  print2_symbol(':');
      //                  print2_hex(crc8_,8);
      //                  print2_symbol('-');
      //                  print2_hex(crc1,8);
      ////                  print2_symbol(';');
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(!(((uint8_t)crc8_)
      //                      -((uint8_t)crc1))),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)!(((uint8_t)crc8_)
      //                     -((~((uint8_t)crc1))&0b00000111)),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(chan_0==ad_data_channel_pair*2),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(chan_1==(ad_data_channel_pair*2+1)),8);
      //                  print2_symbol(';');
      //                  if(
      //                      ((!(((uint8_t)crc8_)
      //                          -((uint8_t)crc1)))
      //                     ||(!(((uint8_t)crc8_)
      //                         -((~((uint8_t)crc1))&0b00000111)))
      //                      )
      //                  &&(
      //                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
      //                        ((chan_0==ad_data_channel_pair)))
      //                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
      //                      ((chan_1==(ad_data_channel_pair+1))))
      //                  ))
                        if(
                            ((!(((uint8_t)crc8_)
                                -((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                    |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)))))
                           ||(!(((uint8_t)crc8_)
                               -((~((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                   |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f))))&0b00000111)))
                            )
                        &&(
                          (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
                              ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b11110000)>>4)==ad_data_channel_pair*2)))
                        &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
                            ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b11110000)>>4)==ad_data_channel_pair*2+1)))
                        ))
                        {
      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 1;
                        }
                        else
                        {
      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 0;

                          crc_ok = 0;
                        }

                      }
                      if(crc_ok)
                      {
      //                  print2_symbol('1');
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
      //                  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
                      }
                      else
                      {
      //                  print2_symbol('0');
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
      //                  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

                      }
      //                print2_symbol(';');
                    }
                  }

                  if(data_counter%sample_number_max==0)
                  {
                    sample_number_now=sample_number_max;
                  }
                  else
                  {
                    sample_number_now=data_counter%sample_number_max;
                  }

    //              int ad_adc = 0;
                  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                  {
    //                int ad_data_channel = 0;
                    for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
                    {
    //                  print2_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);          print2_symbol(';');
    //                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    //                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    //                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    //                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
          //            print_binary(data[ad_data_channel * 4 + 1], 8);          print_symbol(';');
          //            print_binary(data[ad_data_channel * 4 + 2], 8);          print_symbol(';');
          //            print_binary(data[ad_data_channel * 4 + 3], 8);          print_symbol(';');
    //                  print2_symbol(';');
    //                  datas_out[ad_adc][ad_data_channel * 4 + 0]=datas[ad_adc][ad_data_channel * 4 + 1]&0b10000000;
    //                  datas_out[ad_adc][ad_data_channel * 4 + 1]=datas[ad_adc][ad_data_channel * 4 + 1]&0b01111111;
    //                  datas_out[ad_adc][ad_data_channel * 4 + 2]=datas[ad_adc][ad_data_channel * 4 + 2];
    //                  datas_out[ad_adc][ad_data_channel * 4 + 3]=datas[ad_adc][ad_data_channel * 4 + 3];

    //                  const int sample_number_max = 240;
    //                  int sample[sample_number_max];
    //                  int sample_number_last = -1;
    //                  int sample_average = 0;
    //
    //                  int sample_int_out;
    //                  uint8_t sample_byte_out;
    //                  double sample_int_byte_scale;
    //                  int sample_int_byte_shift;

    //                  sample_int_out = (int)((datas_out[ad_adc][ad_data_channel * 4 + 0]<<24)
    //                          |(datas_out[ad_adc][ad_data_channel * 4 + 1]<<16)
    //                          |(datas_out[ad_adc][ad_data_channel * 4 + 2]<<8)
    //                          |(datas_out[ad_adc][ad_data_channel * 4 + 3]<<0));

    //                  int32_t newInt = (byteBuffer[2] << 24) | (byteBuffer[1] << 16) | byteBuffer[0] << 8;
    //                  // depending on most significant byte, set positive or negative value
    //                  if ((newInt & 0x00008000) > 0) {
    //                    newInt |= 0x000000FF;
    //                  } else {
    //                    newInt &= 0xFFFFFF00;
    //                  }

    //                  sample_int_out = (datas[ad_adc][ad_data_channel * 4 + 3]<<24)
    //                          |(datas[ad_adc][ad_data_channel * 4 + 2]<<16)
    //                          |(datas[ad_adc][ad_data_channel * 4 + 1]<<8);
                      sample_int_out = (datas[ad_adc][ad_data_channel * 4 + 1]<<24)
                              |(datas[ad_adc][ad_data_channel * 4 + 2]<<16)
                              |(datas[ad_adc][ad_data_channel * 4 + 3]<<8);
                      if ((sample_int_out & 0x00008000) > 0) {
                        sample_int_out |= 0x000000FF;
                      } else {
                        sample_int_out &= 0xFFFFFF00;
                      }
    //                  sample_int_out >>= 8;
    //
    //                  sample_int_out = (datas[ad_adc][ad_data_channel * 4 + 3]<<16)
    //                          |(datas[ad_adc][ad_data_channel * 4 + 2]<<8)
    //                          |(datas[ad_adc][ad_data_channel * 4 + 1]<<0);

    //                  if(samples_number_now[ad_adc][ad_data_channel]==sample_number_max)
    //                  {
    //                    for(int sample_count=0;sample_count<samples_number_now[ad_adc][ad_data_channel]-1;sample_count++)
    //                    {
    //                      samples_int[ad_adc][ad_data_channel][sample_count]=samples_int[ad_adc][ad_data_channel][sample_count+1];
    //                      samples_byte[ad_adc][ad_data_channel][sample_count]=samples_byte[ad_adc][ad_data_channel][sample_count+1];
    //                    }
    //                  }
    //                  else
    //                  {
    //                    samples_number_now[ad_adc][ad_data_channel]++;
    //                  }
    //                  samples_int[ad_adc][ad_data_channel][sample_number_now-1]=sample_int_out;

    //                  if((data_counter%sample_number_max==0))//||(data_counter<sample_number_max))
                      {
    //                    sample_sum=0;

    //                    int32_t sample_max_new=samples_int[ad_adc][ad_data_channel][0];
    //                    int32_t sample_min_new=samples_int[ad_adc][ad_data_channel][0];
    //
    //                    for(int sample_count=0;sample_count<sample_number_now;sample_count++)
    //                    {
    ////                      samples_sum+=samples_int[ad_adc][ad_data_channel][sample_count];
    //                      if(sample_max_new<samples_int[ad_adc][ad_data_channel][sample_count])
    //                      {
    //                        sample_max_new=samples_int[ad_adc][ad_data_channel][sample_count];
    //                      }
    //                      if(sample_min_new>samples_int[ad_adc][ad_data_channel][sample_count])
    //                      {
    //                        sample_min_new=samples_int[ad_adc][ad_data_channel][sample_count];
    //                      }
    //                    }
    ////                    sample_average=sample_sum/sample_number_now;
    //
    ////                    if(((sample_max_last<sample_max_new)||(sample_min_last>sample_min_new))||
    ////                        ((sample_max_last>sample_max_new+(sample_max+sample_min)/2)
    ////                            ||(sample_min_last<sample_min_new-(sample_max+sample_min)/2)))
    //                    if(((samples_max[ad_adc][ad_data_channel]<sample_max_new)||
    //                        (samples_min[ad_adc][ad_data_channel]>sample_min_new))||
    //                        ((samples_max[ad_adc][ad_data_channel]>sample_max_new+
    //                            (samples_max[ad_adc][ad_data_channel]+samples_min[ad_adc][ad_data_channel])/20)
    //                            ||(samples_min[ad_adc][ad_data_channel]<sample_min_new-
    //                                (samples_max[ad_adc][ad_data_channel]+samples_min[ad_adc][ad_data_channel])/20)))
    //                    {
    //                      samples_max[ad_adc][ad_data_channel]=sample_max_new;
    //                      samples_min[ad_adc][ad_data_channel]=sample_min_new;
    //                    }

                        if(data_counter==1)
                        {
                          samples_max[ad_adc][ad_data_channel]=sample_int_out;
                          samples_min[ad_adc][ad_data_channel]=sample_int_out;
                        }
                        if(data_counter%sample_number_max==1)
                        {
    //                      samples_max_new[ad_adc][ad_data_channel]=samples_max[ad_adc][ad_data_channel];
    //                      samples_min_new[ad_adc][ad_data_channel]=samples_min[ad_adc][ad_data_channel];
                          samples_max_new[ad_adc][ad_data_channel]=sample_int_out;
                          samples_min_new[ad_adc][ad_data_channel]=sample_int_out;
                        }
                        if(samples_max_new[ad_adc][ad_data_channel]<sample_int_out)
                        {
                          samples_max_new[ad_adc][ad_data_channel]=sample_int_out;
                        }
                        if(samples_min_new[ad_adc][ad_data_channel]>sample_int_out)
                        {
                          samples_min_new[ad_adc][ad_data_channel]=sample_int_out;
                        }
                        if(data_counter%sample_number_max==0)
                        {
    //                    sample_average=sample_sum/sample_number_now;

    //                    if(((sample_max_last<sample_max_new)||(sample_min_last>sample_min_new))||
    //                        ((sample_max_last>sample_max_new+(sample_max+sample_min)/2)
    //                            ||(sample_min_last<sample_min_new-(sample_max+sample_min)/2)))
    //                      if(((samples_max[ad_adc][ad_data_channel]<sample_max_new)||
    //                          (samples_min[ad_adc][ad_data_channel]>sample_min_new))||
    //                          ((samples_max[ad_adc][ad_data_channel]>sample_max_new+
    //                              (samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel])/20)
    //                              ||(samples_min[ad_adc][ad_data_channel]<sample_min_new-
    //                                  (samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel])/20)))
    //                      {
    //                        samples_max[ad_adc][ad_data_channel]=sample_max_new;
    //                        samples_min[ad_adc][ad_data_channel]=sample_min_new;
    //                      }
                          if(((samples_max[ad_adc][ad_data_channel]<samples_max_new[ad_adc][ad_data_channel]))
    //                          ||((samples_max[ad_adc][ad_data_channel]>samples_max_new[ad_adc][ad_data_channel]+
    //                              (samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel])/20))
                                  )
                          {
                            samples_max[ad_adc][ad_data_channel]=samples_max_new[ad_adc][ad_data_channel];
                          }
                          if(((samples_min[ad_adc][ad_data_channel]>samples_min_new[ad_adc][ad_data_channel]))
    //                          ||((samples_min[ad_adc][ad_data_channel]<samples_min_new[ad_adc][ad_data_channel]-
    //                                  (samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel])/20))
                                      )
                          {
                            samples_min[ad_adc][ad_data_channel]=samples_min_new[ad_adc][ad_data_channel];
                          }
                        }
    //                    samples_max_last[ad_adc][ad_data_channel]=sample_max;
    //                    samples_min_last[ad_adc][ad_data_channel]=sample_min;

    //                    int32_t sample_max_min_dif=(sample_max-sample_min);


                      }

    //                  sample_int_byte_shift = (1<<8)/(1<<32);
    //                  sample_byte_out = sample_int_out + sample_int_byte_shift;
    //                  sample_int_byte_scale = 1;//(1<<8)/(1<<32);
    //                  sample_byte_out = (uint8_t)(sample_int_out * sample_int_byte_scale);

    //                  sample_byte_out = (uint8_t)(((int64_t)sample_int_out-(int64_t)samples_min[ad_adc][ad_data_channel])*255/((int64_t)samples_max[ad_adc][ad_data_channel]-(int64_t)samples_min[ad_adc][ad_data_channel]+1));
    //                  samples_byte[ad_adc][ad_data_channel][sample_number_now-1]=sample_byte_out;

    //                  sample_byte_out = (uint8_t)(((int64_t)sample_int_out-(int64_t)sample_min)*255/((int64_t)sample_max-(int64_t)sample_min+1));
    //                  sample_byte[sample_number_now-1]=sample_byte_out;


    //                  print2_symbol(datas_out[ad_adc][ad_data_channel * 4 + 3]);
    //                  print2_symbol((uint8_t)((sample_int_out/(1<<0)+(255/4)*(1))));
    //                  print2_symbol((uint8_t)(((sample_int_out-1000000)/1000000)));
                      if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
                      {
    //                    print_symbol(sample_byte_out);
                      }
    //                  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
    //                  {
    //                    print2_hex(sample_byte_out, 8);          print2_symbol(';');
    //                    print2_hex(sample_int_out, 32);          print2_symbol(';');
    //                    print2_hex(sample_min, 32);          print2_symbol(';');
    //                    print2_hex(sample_max, 32);          print2_symbol(';');
    //                    print2_line();
    //                  }
                      if((data_counter%sample_number_max==0))
                      {

    //                    print2_hex(sample_byte_out, 8);          print2_symbol(';');
    //                    print2_hex(sample_int_out, 32);          print2_symbol(';');
    //                    print2_hex(sample_min, 32);          print2_symbol(';');
    //                    print2_hex(sample_max, 32);          print2_symbol(';');
    //                    print2_hex(sample_max-sample_min, 32);          print2_symbol(';');
    //                    print2_line();

    //                    print2_hex((samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel])>>16, 8);          print2_symbol(';');
    //                    print2_hex((samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel])>>8, 16);          print2_symbol(';');
    //                    print2_hex((samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel])>>8, 24);          print2_symbol(';');
    //                    print2_hex((samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel]), 32);          print2_symbol(';');
    //                    print2_hex((samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel]), 24);          print2_symbol(';');

    //                    print_symbol(sample_byte_out);

                        samples_max_min_dif[ad_adc][ad_data_channel]=
                            (samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel]);

                        if(data_counter==sample_number_max)
                        {
                          sample_max_min_dif_max=samples_max_min_dif[ad_adc][ad_data_channel];
                          sample_max_min_dif_min=samples_max_min_dif[ad_adc][ad_data_channel];
                        }
                        else
                        {
                          if(sample_max_min_dif_max<samples_max_min_dif[ad_adc][ad_data_channel])
                          {
                            sample_max_min_dif_max=samples_max_min_dif[ad_adc][ad_data_channel];
                          }
                          if(sample_max_min_dif_min>samples_max_min_dif[ad_adc][ad_data_channel])
                          {
                            sample_max_min_dif_min=samples_max_min_dif[ad_adc][ad_data_channel];
                          }
                        }
                        print2_hex(samples_max_min_dif[ad_adc][ad_data_channel], 32);          print2_symbol(';');
                        print2_hex(sample_max_min_dif_min, 32);          print2_symbol(';');
                        print2_hex(sample_max_min_dif_max, 32);          print2_symbol(';');
                        print2_symbol(';');

                      }

                    }
                  }
                  if((data_counter%sample_number_max==0))
                  {
    //                print2_symbol(';');
    //                print2_hex(sample_max_min_dif_max>>16, 8);          print2_symbol(';');
    //                print2_hex(sample_max_min_dif_min>>16, 8);          print2_symbol(';');
    //                print2_hex(sample_max_min_dif_max>>8, 16);          print2_symbol(';');
    //                print2_hex(sample_max_min_dif_min>>8, 16);          print2_symbol(';');
    //                print2_hex(sample_max_min_dif_max>>8, 24);          print2_symbol(';');
    //                print2_hex(sample_max_min_dif_min>>8, 24);          print2_symbol(';');
    //                print2_hex(sample_max_min_dif_max, 24);          print2_symbol(';');
    //                print2_hex(sample_max_min_dif_min, 24);          print2_symbol(';');
    //                print2_hex(sample_max_min_dif_max, 32);          print2_symbol(';');
    //                print2_hex(sample_max_min_dif_min, 32);          print2_symbol(';');
                    print2_line();

                    const uint32_t uint8_data_number = 2 + 8 * 3 * 4 + 3 * 2 + 1;
    //                const uint32_t uint8_data_number = 2 + 8 * 3 + 8 * 3 + 3 * 2 + 1;
                    uint8_t dataBuffer[uint8_data_number];

                    dataBuffer[0] = 0xA0;
    //                dataBuffer[0] = 0x00;
                    dataBuffer[1] = ui8SampleNumber++;

                    for(int ad_adc = 0; ad_adc < 4; ad_adc ++)
                    {
                      for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
                      {
                        int32_t sample_max_min_dif=samples_max_min_dif[ad_adc][ad_data_channel];
                        /*convert to long rainbow RGB*/
                        uint8_t r;
                        uint8_t g;
                        uint8_t b;
                        double f=(double)sample_max_min_dif/(double)0xffffff;
                        double f_min=(double)sample_max_min_dif_min/(double)0xffffff;
                        double f_max=(double)sample_max_min_dif_max/(double)0xffffff;
    //                    double f=(double)(sample_max_min_dif>>8)/(double)0xffff;
    //                    double f=(double)(sample_max_min_dif>>16)/(double)0xff;
                        f=(log(f)-log(f_min))/(log(f_max)-log(f_min));
                        double a=(1-f)/0.2;
                        int  X=floor(a);
                        int  Y=floor(255*(a-X));
                        switch(X)
                        {
                            case 0: r=255;g=Y;b=0;break;
                            case 1: r=255-Y;g=255;b=0;break;
                            case 2: r=0;g=255;b=Y;break;
                            case 3: r=0;g=255-Y;b=255;break;
                            case 4: r=Y;g=0;b=255;break;
                            case 5: r=255;g=0;b=255;break;
                        }

                        dataBuffer[2 + ad_adc * 24 + ad_data_channel * 3 + 0] = r;
                        dataBuffer[2 + ad_adc * 24 + ad_data_channel * 3 + 1] = g;
                        dataBuffer[2 + ad_adc * 24 + ad_data_channel * 3 + 2] = b;
                      }
                    }
    //                for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
    //                {
    //                  dataBuffer[2 + ad_data_channel * 3 + 0] = data2[ad_data_channel * 4 + 1];
    //                  dataBuffer[2 + ad_data_channel * 3 + 1] = data2[ad_data_channel * 4 + 2];
    //                  dataBuffer[2 + ad_data_channel * 3 + 2] = data2[ad_data_channel * 4 + 3];
    //                }
                    for(int accel_data_channel = 0; accel_data_channel < 3; accel_data_channel ++)
                    {
                      dataBuffer[2 + 8 * 3 * 4 + accel_data_channel * 2 + 0] = 0;
                      dataBuffer[2 + 8 * 3 * 4 + accel_data_channel * 2 + 1] = 0;
                    }
                    dataBuffer[2 + 8 * 3 * 4 + 3 * 2 + 0] = 0xC0;

    //                dataBuffer[2]=100;
    //                dataBuffer[3]=101;
    //                dataBuffer[4]=102;

    //                for(int data_out = 0; data_out < uint8_data_number; data_out ++)
    //                {
    //                  print2_hex(dataBuffer[data_out], 8);          print2_symbol(';');
    //                }
    //                print2_line();

    //                if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
                    {
                      while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
                      {
                      }
                      if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                      {
                        Error_Handler();
                      }
                    }
    //                if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
    //                {
    //                  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
    //                  {
    //                  }
    //                  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
    //                  {
    //                    Error_Handler();
    //                  }
    //                }

                  }

                }//FREESMARTEEG_OUT & FREESMARTEEG_СHANNELS_STATE

//                if(FREESMARTEEG_OUT & FREESMARTEEG_FFT_DISPLAY)
//                {
//                  if(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_PIN)
//                  {
//    //              int crc_pair_ok[uint8_ad_adc_number][4];
//
//                    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//                    {
//                      int crc_ok = 1;
//                      int crc_something_ok = 0;
//                      for(int ad_data_channel_pair = 0; ad_data_channel_pair < 4; ad_data_channel_pair ++)
//                      {
//      //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 1];
//      //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 2];
//      //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 3];
//
//                        uint8_t crc8_=crc8da_7(&(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]));
//
//      //                  uint8_t crc1=((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
//      //                      |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)));
//      //                  uint8_t chan_0=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b01110000)>>4);
//      //                  uint8_t chan_1=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b01110000)>>4);
//      //                  print2_hex(chan_0,8);
//      //                  print2_symbol(':');
//      //                  print2_hex(crc8_,8);
//      //                  print2_symbol('-');
//      //                  print2_hex(crc1,8);
//      ////                  print2_symbol(';');
//      //                  print2_symbol(',');
//      //                  print2_hex((uint8_t)(!(((uint8_t)crc8_)
//      //                      -((uint8_t)crc1))),8);
//      //                  print2_symbol(',');
//      //                  print2_hex((uint8_t)!(((uint8_t)crc8_)
//      //                     -((~((uint8_t)crc1))&0b00000111)),8);
//      //                  print2_symbol(',');
//      //                  print2_hex((uint8_t)(chan_0==ad_data_channel_pair*2),8);
//      //                  print2_symbol(',');
//      //                  print2_hex((uint8_t)(chan_1==(ad_data_channel_pair*2+1)),8);
//      //                  print2_symbol(';');
//      //                  if(
//      //                      ((!(((uint8_t)crc8_)
//      //                          -((uint8_t)crc1)))
//      //                     ||(!(((uint8_t)crc8_)
//      //                         -((~((uint8_t)crc1))&0b00000111)))
//      //                      )
//      //                  &&(
//      //                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
//      //                        ((chan_0==ad_data_channel_pair)))
//      //                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
//      //                      ((chan_1==(ad_data_channel_pair+1))))
//      //                  ))
//                        if(
//                            ((!(((uint8_t)crc8_)
//                                -((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
//                                    |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)))))
//                           ||(!(((uint8_t)crc8_)
//                               -((~((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
//                                   |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f))))&0b00000111)))
//                            )
//                            &&(
//                              (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
//                                  ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]
//                                                    &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
//                                                        ad_data_channel_pair*2)))
//                            &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
//                                ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]
//                                                  &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
//                                                      ad_data_channel_pair*2+1)))
//                            ))
//                        {
//      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 1;
//                          crc_something_ok = 1;
//                        }
//                        else
//                        {
//      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 0;
//                          crc_ok = 0;
//                        }
//
//                      }
//                      if((crc_ok)||((crc_something_ok)&&(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_SOMETHING)))
//    //                  if(crc_ok)
//                      {
//      //                  print2_symbol('1');
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
//      //                  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
//                      }
//                      else
//                      {
//      //                  print2_symbol('0');
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
//      //                  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
//
//                      }
//      //                print2_symbol(';');
//                    }
//                  }
//
//    //              if(data_counter%sample_number_max==0)
//    //              {
//    //                sample_number_now=sample_number_max;
//    //              }
//    //              else
//    //              {
//    //                sample_number_now=data_counter%sample_number_max;
//    //              }
//
//    //              int ad_adc = 0;
//                  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//                  {
//    //                int ad_data_channel = 0;
//                    for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
//                    {
//    //                  print2_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);          print2_symbol(';');
//    //                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
//    //                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//    //                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//    //                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
//          //            print_binary(data[ad_data_channel * 4 + 1], 8);          print_symbol(';');
//          //            print_binary(data[ad_data_channel * 4 + 2], 8);          print_symbol(';');
//          //            print_binary(data[ad_data_channel * 4 + 3], 8);          print_symbol(';');
//    //                  print2_symbol(';');
//    //                  datas_out[ad_adc][ad_data_channel * 4 + 0]=datas[ad_adc][ad_data_channel * 4 + 1]&0b10000000;
//    //                  datas_out[ad_adc][ad_data_channel * 4 + 1]=datas[ad_adc][ad_data_channel * 4 + 1]&0b01111111;
//    //                  datas_out[ad_adc][ad_data_channel * 4 + 2]=datas[ad_adc][ad_data_channel * 4 + 2];
//    //                  datas_out[ad_adc][ad_data_channel * 4 + 3]=datas[ad_adc][ad_data_channel * 4 + 3];
//
//    //                  const int sample_number_max = 240;
//    //                  int sample[sample_number_max];
//    //                  int sample_number_last = -1;
//    //                  int sample_average = 0;
//    //
//    //                  int sample_int_out;
//    //                  uint8_t sample_byte_out;
//    //                  double sample_int_byte_scale;
//    //                  int sample_int_byte_shift;
//
//    //                  sample_int_out = (int)((datas_out[ad_adc][ad_data_channel * 4 + 0]<<24)
//    //                          |(datas_out[ad_adc][ad_data_channel * 4 + 1]<<16)
//    //                          |(datas_out[ad_adc][ad_data_channel * 4 + 2]<<8)
//    //                          |(datas_out[ad_adc][ad_data_channel * 4 + 3]<<0));
//
//    //                  int32_t newInt = (byteBuffer[2] << 24) | (byteBuffer[1] << 16) | byteBuffer[0] << 8;
//    //                  // depending on most significant byte, set positive or negative value
//    //                  if ((newInt & 0x00008000) > 0) {
//    //                    newInt |= 0x000000FF;
//    //                  } else {
//    //                    newInt &= 0xFFFFFF00;
//    //                  }
//
//    //                  sample_int_out = (datas[ad_adc][ad_data_channel * 4 + 3]<<24)
//    //                          |(datas[ad_adc][ad_data_channel * 4 + 2]<<16)
//    //                          |(datas[ad_adc][ad_data_channel * 4 + 1]<<8);
//                      sample_int_out = (datas[ad_adc][ad_data_channel * 4 + 1]<<24)
//                              |(datas[ad_adc][ad_data_channel * 4 + 2]<<16)
//                              |(datas[ad_adc][ad_data_channel * 4 + 3]<<8);
//                      if ((sample_int_out & 0x00008000) > 0) {
//                        sample_int_out |= 0x000000FF;
//                      } else {
//                        sample_int_out &= 0xFFFFFF00;
//                      }
//    //                  sample_int_out >>= 8;
//    //
//    //                  sample_int_out = (datas[ad_adc][ad_data_channel * 4 + 3]<<16)
//    //                          |(datas[ad_adc][ad_data_channel * 4 + 2]<<8)
//    //                          |(datas[ad_adc][ad_data_channel * 4 + 1]<<0);
//
//    //                  if(samples_number_now[ad_adc][ad_data_channel]==sample_number_max)
//    //                  {
//    //                    for(int sample_count=0;sample_count<samples_number_now[ad_adc][ad_data_channel]-1;sample_count++)
//    //                    {
//    //                      samples_int[ad_adc][ad_data_channel][sample_count]=samples_int[ad_adc][ad_data_channel][sample_count+1];
//    //                      samples_byte[ad_adc][ad_data_channel][sample_count]=samples_byte[ad_adc][ad_data_channel][sample_count+1];
//    //                    }
//    //                  }
//    //                  else
//    //                  {
//    //                    samples_number_now[ad_adc][ad_data_channel]++;
//    //                  }
//                      if((ad_adc==0)&&(ad_data_channel==0))
//    //                  if((ad_adc==1)&&(ad_data_channel==4))
//                      {
//                        if(sample_number_now==sample_number_max)
//                        {
//                          for(int sample_count=0;sample_count<sample_number_now-1;sample_count++)
//                          {
//                            samples_int_part[sample_count]=samples_int_part[sample_count+1];
//                          }
//                        }
//                        else
//                        {
//                          sample_number_now++;
//                        }
//                        samples_int_part[sample_number_now-1]=sample_int_out;
//                      }
//    //                  samples_int[ad_adc][ad_data_channel][sample_number_now-1]=sample_int_out;
//
//                      if(0)
//    //                  if((data_counter%sample_number_max==0))//||(data_counter<sample_number_max))
//                      {
//    //                    sample_sum=0;
//
//    //                    int32_t sample_max_new=samples_int[ad_adc][ad_data_channel][0];
//    //                    int32_t sample_min_new=samples_int[ad_adc][ad_data_channel][0];
//    //
//    //                    for(int sample_count=0;sample_count<sample_number_now;sample_count++)
//    //                    {
//    ////                      samples_sum+=samples_int[ad_adc][ad_data_channel][sample_count];
//    //                      if(sample_max_new<samples_int[ad_adc][ad_data_channel][sample_count])
//    //                      {
//    //                        sample_max_new=samples_int[ad_adc][ad_data_channel][sample_count];
//    //                      }
//    //                      if(sample_min_new>samples_int[ad_adc][ad_data_channel][sample_count])
//    //                      {
//    //                        sample_min_new=samples_int[ad_adc][ad_data_channel][sample_count];
//    //                      }
//    //                    }
//    ////                    sample_average=sample_sum/sample_number_now;
//    //
//    ////                    if(((sample_max_last<sample_max_new)||(sample_min_last>sample_min_new))||
//    ////                        ((sample_max_last>sample_max_new+(sample_max+sample_min)/2)
//    ////                            ||(sample_min_last<sample_min_new-(sample_max+sample_min)/2)))
//    //                    if(((samples_max[ad_adc][ad_data_channel]<sample_max_new)||
//    //                        (samples_min[ad_adc][ad_data_channel]>sample_min_new))||
//    //                        ((samples_max[ad_adc][ad_data_channel]>sample_max_new+
//    //                            (samples_max[ad_adc][ad_data_channel]+samples_min[ad_adc][ad_data_channel])/20)
//    //                            ||(samples_min[ad_adc][ad_data_channel]<sample_min_new-
//    //                                (samples_max[ad_adc][ad_data_channel]+samples_min[ad_adc][ad_data_channel])/20)))
//    //                    {
//    //                      samples_max[ad_adc][ad_data_channel]=sample_max_new;
//    //                      samples_min[ad_adc][ad_data_channel]=sample_min_new;
//    //                    }
//
//                        if(data_counter==1)
//                        {
//                          samples_max[ad_adc][ad_data_channel]=sample_int_out;
//                          samples_min[ad_adc][ad_data_channel]=sample_int_out;
//                        }
//                        if(data_counter%sample_number_max==1)
//                        {
//    //                      samples_max_new[ad_adc][ad_data_channel]=samples_max[ad_adc][ad_data_channel];
//    //                      samples_min_new[ad_adc][ad_data_channel]=samples_min[ad_adc][ad_data_channel];
//                          samples_max_new[ad_adc][ad_data_channel]=sample_int_out;
//                          samples_min_new[ad_adc][ad_data_channel]=sample_int_out;
//                        }
//                        if(samples_max_new[ad_adc][ad_data_channel]<sample_int_out)
//                        {
//                          samples_max_new[ad_adc][ad_data_channel]=sample_int_out;
//                        }
//                        if(samples_min_new[ad_adc][ad_data_channel]>sample_int_out)
//                        {
//                          samples_min_new[ad_adc][ad_data_channel]=sample_int_out;
//                        }
//                        if(data_counter%sample_number_max==0)
//                        {
//    //                    sample_average=sample_sum/sample_number_now;
//
//    //                    if(((sample_max_last<sample_max_new)||(sample_min_last>sample_min_new))||
//    //                        ((sample_max_last>sample_max_new+(sample_max+sample_min)/2)
//    //                            ||(sample_min_last<sample_min_new-(sample_max+sample_min)/2)))
//    //                      if(((samples_max[ad_adc][ad_data_channel]<sample_max_new)||
//    //                          (samples_min[ad_adc][ad_data_channel]>sample_min_new))||
//    //                          ((samples_max[ad_adc][ad_data_channel]>sample_max_new+
//    //                              (samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel])/20)
//    //                              ||(samples_min[ad_adc][ad_data_channel]<sample_min_new-
//    //                                  (samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel])/20)))
//    //                      {
//    //                        samples_max[ad_adc][ad_data_channel]=sample_max_new;
//    //                        samples_min[ad_adc][ad_data_channel]=sample_min_new;
//    //                      }
//                          if(((samples_max[ad_adc][ad_data_channel]<samples_max_new[ad_adc][ad_data_channel]))
//    //                          ||((samples_max[ad_adc][ad_data_channel]>samples_max_new[ad_adc][ad_data_channel]+
//    //                              (samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel])/20))
//                                  )
//                          {
//                            samples_max[ad_adc][ad_data_channel]=samples_max_new[ad_adc][ad_data_channel];
//                          }
//                          if(((samples_min[ad_adc][ad_data_channel]>samples_min_new[ad_adc][ad_data_channel]))
//    //                          ||((samples_min[ad_adc][ad_data_channel]<samples_min_new[ad_adc][ad_data_channel]-
//    //                                  (samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel])/20))
//                                      )
//                          {
//                            samples_min[ad_adc][ad_data_channel]=samples_min_new[ad_adc][ad_data_channel];
//                          }
//                        }
//    //                    samples_max_last[ad_adc][ad_data_channel]=sample_max;
//    //                    samples_min_last[ad_adc][ad_data_channel]=sample_min;
//
//    //                    int32_t sample_max_min_dif=(sample_max-sample_min);
//
//
//                      }
//
//    //                  sample_int_byte_shift = (1<<8)/(1<<32);
//    //                  sample_byte_out = sample_int_out + sample_int_byte_shift;
//    //                  sample_int_byte_scale = 1;//(1<<8)/(1<<32);
//    //                  sample_byte_out = (uint8_t)(sample_int_out * sample_int_byte_scale);
//
//    //                  sample_byte_out = (uint8_t)(((int64_t)sample_int_out-(int64_t)samples_min[ad_adc][ad_data_channel])*255/((int64_t)samples_max[ad_adc][ad_data_channel]-(int64_t)samples_min[ad_adc][ad_data_channel]+1));
//    //                  samples_byte[ad_adc][ad_data_channel][sample_number_now-1]=sample_byte_out;
//
//    //                  sample_byte_out = (uint8_t)(((int64_t)sample_int_out-(int64_t)sample_min)*255/((int64_t)sample_max-(int64_t)sample_min+1));
//    //                  sample_byte[sample_number_now-1]=sample_byte_out;
//
//
//    //                  print2_symbol(datas_out[ad_adc][ad_data_channel * 4 + 3]);
//    //                  print2_symbol((uint8_t)((sample_int_out/(1<<0)+(255/4)*(1))));
//    //                  print2_symbol((uint8_t)(((sample_int_out-1000000)/1000000)));
//                      if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//                      {
//    //                    print_symbol(sample_byte_out);
//                      }
//    //                  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
//    //                  {
//    //                    print2_hex(sample_byte_out, 8);          print2_symbol(';');
//    //                    print2_hex(sample_int_out, 32);          print2_symbol(';');
//    //                    print2_hex(sample_min, 32);          print2_symbol(';');
//    //                    print2_hex(sample_max, 32);          print2_symbol(';');
//    //                    print2_line();
//    //                  }
//                      if(0)
//                      if((data_counter%sample_number_max==0))
//                      {
//
//    //                    print2_hex(sample_byte_out, 8);          print2_symbol(';');
//    //                    print2_hex(sample_int_out, 32);          print2_symbol(';');
//    //                    print2_hex(sample_min, 32);          print2_symbol(';');
//    //                    print2_hex(sample_max, 32);          print2_symbol(';');
//    //                    print2_hex(sample_max-sample_min, 32);          print2_symbol(';');
//    //                    print2_line();
//
//    //                    print2_hex((samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel])>>16, 8);          print2_symbol(';');
//    //                    print2_hex((samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel])>>8, 16);          print2_symbol(';');
//    //                    print2_hex((samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel])>>8, 24);          print2_symbol(';');
//    //                    print2_hex((samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel]), 32);          print2_symbol(';');
//    //                    print2_hex((samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel]), 24);          print2_symbol(';');
//
//    //                    print_symbol(sample_byte_out);
//
//                        samples_max_min_dif[ad_adc][ad_data_channel]=
//                            (samples_max[ad_adc][ad_data_channel]-samples_min[ad_adc][ad_data_channel]);
//
//                        if(data_counter==sample_number_max)
//                        {
//                          sample_max_min_dif_max=samples_max_min_dif[ad_adc][ad_data_channel];
//                          sample_max_min_dif_min=samples_max_min_dif[ad_adc][ad_data_channel];
//                        }
//                        else
//                        {
//                          if(sample_max_min_dif_max<samples_max_min_dif[ad_adc][ad_data_channel])
//                          {
//                            sample_max_min_dif_max=samples_max_min_dif[ad_adc][ad_data_channel];
//                          }
//                          if(sample_max_min_dif_min>samples_max_min_dif[ad_adc][ad_data_channel])
//                          {
//                            sample_max_min_dif_min=samples_max_min_dif[ad_adc][ad_data_channel];
//                          }
//                        }
//    //                    print2_hex(samples_max_min_dif[ad_adc][ad_data_channel], 32);          print2_symbol(';');
//    //                    print2_hex(sample_max_min_dif_min, 32);          print2_symbol(';');
//    //                    print2_hex(sample_max_min_dif_max, 32);          print2_symbol(';');
//    //                    print2_symbol(';');
//
//                      }
//
//                    }
//                  }
//                  if((data_counter%sample_freq_fft==0))
//    //              if((data_counter%sample_number_max==0))
//                  {
//
//                    uint32_t FFT_Length = (int)(sample_number_max/2);
//    //                uint32_t FFT_Length = 64;//sample_number_max/2;
//    //                arm_cfft_radix4_instance_q31  FFT_Q31_struct;
//    //                arm_cfft_radix4_instance_q15  FFT_Q15_struct;
//                    arm_cfft_instance_q31  FFT_Q31_struct;
//                    FFT_Q31_struct.fftLen = FFT_Length;
//                    FFT_Q31_struct.pTwiddle = twiddleCoef_1024_q31;
//                    FFT_Q31_struct.pBitRevTable = armBitRevTable;
//                    FFT_Q31_struct.bitRevLength = 1024;
//
//                    arm_cfft_instance_q15  FFT_Q15_struct;
//                    FFT_Q15_struct.fftLen = FFT_Length;
//                    FFT_Q15_struct.pTwiddle = twiddleCoef_1024_q15;
//                    FFT_Q15_struct.pBitRevTable = armBitRevTable;
//                    FFT_Q15_struct.bitRevLength = 1024;
//
//                    q31_t maxValue;    /* Max FFT value is stored here */
//                    uint32_t maxIndex;    /* Index in Output array where max value is */
//
//                    uint32_t index_fill_input_buffer, index_fill_output_buffer, index_fill_adc_buffer = 0;
//                    uint32_t duration_us = 0x00;
//
//    //                _uhADCxConvertedValue = ;
//
//    //                _aADC1ConvertedValue_s[index_fill_adc_buffer] = _uhADCxConvertedValue;
//                ///    TIM2_Config();
//    //              }
//    //              last_index_fill_adc_buffer=index_fill_adc_buffer;
//                    for (int index_fill_input_buffer = 0; index_fill_input_buffer < FFT_Length*2; index_fill_input_buffer += 2)
//                    {
//    //                  FFT_Input_Q31_f[(uint16_t)index_fill_input_buffer] = (float32_t)samples_int[1][4][index_fill_input_buffer];//_uhADCxConvertedValue;// / (float32_t)4096.0;
//    //                  /* Imaginary part */
//    //                  FFT_Input_Q31_f[(uint16_t)(index_fill_input_buffer + 1)] = 0;
//
//                      aFFT_Input_Q31[(uint16_t)index_fill_input_buffer] = samples_int_part[index_fill_input_buffer];//_uhADCxConvertedValue;// / (float32_t)4096.0;
//    //                  aFFT_Input_Q31[(uint16_t)index_fill_input_buffer] = samples_int[1][4][index_fill_input_buffer];//_uhADCxConvertedValue;// / (float32_t)4096.0;
//                      /* Imaginary part */
//                      aFFT_Input_Q31[(uint16_t)(index_fill_input_buffer + 1)] = 0;
//
//    //                  FFT_Input_Q15_f[(uint16_t)index_fill_input_buffer] = (float32_t)samples_int[1][4][index_fill_input_buffer];//_uhADCxConvertedValue;// / (float32_t)4096.0;
//    //                  /* Imaginary part */
//    //                  FFT_Input_Q15_f[(uint16_t)(index_fill_input_buffer + 1)] = 0;
//
//    //                  aFFT_Input_Q15[(uint16_t)index_fill_input_buffer] = (int16_t)(samples_int_part[index_fill_input_buffer]>>8);//_uhADCxConvertedValue;// / (float32_t)4096.0;
//    //                  /* Imaginary part */
//    //                  aFFT_Input_Q15[(uint16_t)(index_fill_input_buffer + 1)] = 0;
//
//                  ///    TIM2_Config();
//                    }
//
//    //                arm_float_to_q31((float32_t *)&FFT_Input_Q31_f[0], (q31_t *)&aFFT_Input_Q31[0], FFT_Length*2);
//    //                arm_float_to_q15((float32_t *)&FFT_Input_Q15_f[0], (q15_t *)&aFFT_Input_Q15[0], FFT_Length*2);
//
//                    /* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
//    //                arm_cfft_radix4_init_q31(&FFT_Q31_struct, FFT_Length, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);
//    //                arm_cfft_radix4_init_q15(&FFT_Q15_struct, FFT_Length, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);
//    //                arm_cfft_q31(&FFT_Q31_struct, aFFT_Input_Q31, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);
//                    arm_cfft_q31(&FFT_Q31_struct, aFFT_Input_Q31, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);
//    //                arm_cfft_q15(&FFT_Q15_struct, aFFT_Input_Q15, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);
//    //                if ( arm_cfft_radix4_init_q31 ( &FFT_Q31_struct, FFT_Length, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG) != ARM_MATH_SUCCESS )
//    //                     return 1;
//
//                  //  TimerCount_Start();
//    //                arm_cfft_radix4_q31(&FFT_Q31_struct, aFFT_Input_Q31);
//    //                arm_cfft_radix4_q15(&FFT_Q15_struct, aFFT_Input_Q15);
//                  //  TimerCount_Stop(nb_cycles);
//
//                  ///  GUI_Clear();
//                  ///  LCD_OUTPUT_Cycles(5, 305, nb_cycles);
//                  //  duration_us = (uint32_t)(((uint64_t)US_IN_SECOND * (nb_cycles)) / SystemCoreClock);
//                  ///  LCD_OUTPUT_DURATION(120, 305, duration_us);
//
//                    /* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
//                    arm_cmplx_mag_q31(aFFT_Input_Q31, FFT_Output_Q31, FFT_Length);
//    //                arm_cmplx_mag_q15(aFFT_Input_Q15, FFT_Output_Q15, FFT_Length);
//
//                    /* Calculates maxValue and returns corresponding value */
//    //                arm_max_q31(FFT_Output_Q31, FFT_Length, &maxValue, &maxIndex);
//    //                arm_max_q15(FFT_Output_Q15, FFT_Length, &maxValue, &maxIndex);
//    //                maxValue = 0;
//
//                    if(0)
//                    {
//                      print_text("a_");
//                      for (index_fill_input_buffer = 0; index_fill_input_buffer < FFT_Length*2; index_fill_input_buffer++)
//                      {
//                    ///    GRAPH_DATA_YT_SetAlign(aGraph_Data[1], GRAPH_ALIGN_LEFT);
//                    ///    GRAPH_DATA_YT_MirrorX (aGraph_Data[1], 1);
//                    ///    GRAPH_DATA_YT_AddValue(aGraph_Data[1], aADC1ConvertedValue_s[index_fill_input_buffer] / 50 + 50);
//                        print_hex(_aADC1ConvertedValue_s[index_fill_input_buffer] / 50 + 50, 24);
//                        print_text(";");
//                      }
//                      print_line();
//                      print_text("f_");
//                    }
//
//    //                if(data_counter==sample_number_max)
//    //                {
//    //                  sample_max_min_dif_max=maxValue;
//    //                  sample_max_min_dif_min=maxValue;
//    //                }
//    //                else
//    //                {
//    //                  if(sample_max_min_dif_max<maxValue)
//    //                  {
//    //                    sample_max_min_dif_max=maxValue;
//    //                  }
//    //                  if(sample_max_min_dif_min>maxValue)
//    //                  {
//    //                    sample_max_min_dif_min=maxValue;
//    //                  }
//    //                }
//                    for (index_fill_output_buffer = 0+fft_out_shift; index_fill_output_buffer < fft_out+fft_out_shift; index_fill_output_buffer++)
//                    {
//                      if(data_counter==sample_freq_fft)
//    //                  if(data_counter==sample_number_max)
//                      {
//                        sample_max_min_dif_max=FFT_Output_Q31[index_fill_output_buffer];
//                        sample_max_min_dif_min=FFT_Output_Q31[index_fill_output_buffer];
//                      }
//                      else
//                      {
//                        if(sample_max_min_dif_max<FFT_Output_Q31[index_fill_output_buffer])
//                        {
//                          sample_max_min_dif_max=FFT_Output_Q31[index_fill_output_buffer];
//                        }
//                        if(sample_max_min_dif_min>FFT_Output_Q31[index_fill_output_buffer])
//                        {
//                          sample_max_min_dif_min=FFT_Output_Q31[index_fill_output_buffer];
//                        }
//                      }
//    //                  if(data_counter==sample_freq_fft)
//    ////                  if(data_counter==sample_number_max)
//    //                  {
//    //                    sample_max_min_dif_max=FFT_Output_Q15[index_fill_output_buffer];
//    //                    sample_max_min_dif_min=FFT_Output_Q15[index_fill_output_buffer];
//    //                  }
//    //                  else
//    //                  {
//    //                    if(sample_max_min_dif_max<FFT_Output_Q15[index_fill_output_buffer])
//    //                    {
//    //                      sample_max_min_dif_max=FFT_Output_Q15[index_fill_output_buffer];
//    //                    }
//    //                    if(sample_max_min_dif_min>FFT_Output_Q15[index_fill_output_buffer])
//    //                    {
//    //                      sample_max_min_dif_min=FFT_Output_Q15[index_fill_output_buffer];
//    //                    }
//    //                  }
//
//                    }
//
//
//                    uicount++;
//    ////                print_hex(uicount,8);
//    ////                print_symbol(':');
//    //                print2_hex(uicount,8);
//    //                print2_symbol(':');
//                    for (index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length; index_fill_output_buffer++)
//                  //  for (index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length; index_fill_output_buffer++)
//                    {
//                  ///    GRAPH_DATA_YT_SetAlign(aGraph_Data[0], GRAPH_ALIGN_LEFT);
//                  ///    GRAPH_DATA_YT_MirrorX (aGraph_Data[0], 1);
//                  ///    GRAPH_DATA_YT_AddValue(aGraph_Data[0], FFT_Output_Q31[index_fill_output_buffer] / 5376212 + 10);
//    //                  print_hex(FFT_Output_Q31[index_fill_output_buffer] / 5376212 + 10, 8);
//    //                  print_symbol(':');
//    //                  print2_hex(FFT_Output_Q31[index_fill_output_buffer] / 5376212 + 10, 8);
//    //                  print2_hex(FFT_Output_Q31[index_fill_output_buffer], 32);
//    //                  print2_symbol(':');
//                    }
//    //                print_line();
//    //                print2_line();
//
//    //                print2_symbol(';');
//    //                print2_hex(sample_max_min_dif_max>>16, 8);          print2_symbol(';');
//    //                print2_hex(sample_max_min_dif_min>>16, 8);          print2_symbol(';');
//    //                print2_hex(sample_max_min_dif_max>>8, 16);          print2_symbol(';');
//    //                print2_hex(sample_max_min_dif_min>>8, 16);          print2_symbol(';');
//    //                print2_hex(sample_max_min_dif_max>>8, 24);          print2_symbol(';');
//    //                print2_hex(sample_max_min_dif_min>>8, 24);          print2_symbol(';');
//    //                print2_hex(sample_max_min_dif_max, 24);          print2_symbol(';');
//    //                print2_hex(sample_max_min_dif_min, 24);          print2_symbol(';');
//    //                print2_hex(sample_max_min_dif_max, 32);          print2_symbol(';');
//    //                print2_hex(sample_max_min_dif_min, 32);          print2_symbol(';');
//    //                print2_line();
//
//                    const uint32_t uint8_data_number = 2 + 8 * 3 * 4 + 3 * 2 + 1;
//    //                const uint32_t uint8_data_number = 2 + 8 * 3 + 8 * 3 + 3 * 2 + 1;
//                    uint8_t dataBuffer[uint8_data_number];
//
//                    dataBuffer[0] = 0xA0;
//    //                dataBuffer[0] = 0x00;
//                    dataBuffer[1] = ui8SampleNumber++;
//
//    //                double f=(double)sample_max_min_dif/(double)0xffffff;
//                    double f_min=(double)sample_max_min_dif_min;
//                    if(f_min<=0)
//                    {
//                      f_min=0.1;
//                    }
//                    double f_max=(double)sample_max_min_dif_max;
//                    if(f_max<=0)
//                    {
//                      f_max=0.1;
//                    }
//                    if(f_max==f_min)
//                    {
//                      f_max=f_min+1;
//                    }
//
//                    for (index_fill_output_buffer = 0+fft_out_shift; index_fill_output_buffer < fft_out+fft_out_shift; index_fill_output_buffer++)
//    //                for (index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length; index_fill_output_buffer++)
//    //                for(int ad_adc = 0; ad_adc < 4; ad_adc ++)
//                    {
//    //                  for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
//                      {
//    //                    int32_t sample_max_min_dif=samples_max_min_dif[ad_adc][ad_data_channel];
//                        /*convert to long rainbow RGB*/
//                        uint8_t r;
//                        uint8_t g;
//                        uint8_t b;
//                        double f=(double)FFT_Output_Q31[index_fill_output_buffer];///f_max;
//    //                    double f=(double)FFT_Output_Q15[index_fill_output_buffer];///f_max;
//                        if(f<=0)
//                        {
//                          f=0.1;
//                        }
//    //                    print2_hex((int)(f), 32);          print2_symbol(';');
//    //                    print2_hex((int)(f_min), 32);          print2_symbol(';');
//    //                    print2_hex((int)(f_max), 32);          print2_symbol(';');
//                        f=(log(f-f_min)-log(1))/(log(f_max-f_min)-log(1));
//    //                    print2_hex((int)(f*0xffffffff), 32);          print2_symbol(';');
//    //                    print2_hex((int)(log(f)*0xff), 32);          print2_symbol(';');
//    //                    print2_hex((int)(log(f_min)*0xff), 32);          print2_symbol(';');
//    //                    print2_hex((int)(log(f_max)*0xff), 32);          print2_symbol(';');
//    //                    print2_hex((int)((log(f_max)-log(f_min))*0xff), 32);          print2_symbol(';');
//    //                    print2_hex((int)((log(f)-log(f_min))*0xff), 32);          print2_symbol(';');
//    //                    print2_line();
//
//    //                    double f=(double)FFT_Output_Q31[index_fill_output_buffer]/f_max;
//    //                    double f=((double)FFT_Output_Q31[index_fill_output_buffer]-f_min)/(f_max-f_min);
//    //                    f=((double)FFT_Output_Q31[index_fill_output_buffer]-f_min)/(f_max-f_min);
//    //                    f=((double)FFT_Output_Q15[index_fill_output_buffer]-f_min)/(f_max-f_min);
//    //                    double f=(double)sample_max_min_dif/(double)0xffffff;
//    //                    double f_min=(double)sample_max_min_dif_min/(double)0xffffff;
//    //                    double f_max=(double)sample_max_min_dif_max/(double)0xffffff;
//    //                    double f=(double)(sample_max_min_dif>>8)/(double)0xffff;
//    //                    double f=(double)(sample_max_min_dif>>16)/(double)0xff;
//
//                        uint8_t gray = f*255;
//
//                        dataBuffer[2 + index_fill_output_buffer + 0] = gray;
//
//    //                    double a=(1-f)/0.2;
//    //                    int  X=floor(a);
//    //                    int  Y=floor(255*(a-X));
//    //                    switch(X)
//    //                    {
//    //                        case 0: r=255;g=Y;b=0;break;
//    //                        case 1: r=255-Y;g=255;b=0;break;
//    //                        case 2: r=0;g=255;b=Y;break;
//    //                        case 3: r=0;g=255-Y;b=255;break;
//    //                        case 4: r=Y;g=0;b=255;break;
//    //                        case 5: r=255;g=0;b=255;break;
//    //                    }
//
//    //                    dataBuffer[2 + index_fill_output_buffer * 3 + 0] = r;
//    //                    dataBuffer[2 + index_fill_output_buffer * 3 + 1] = g;
//    //                    dataBuffer[2 + index_fill_output_buffer * 3 + 2] = b;
//    //                    dataBuffer[2 + ad_adc * 24 + ad_data_channel * 3 + 0] = r;
//    //                    dataBuffer[2 + ad_adc * 24 + ad_data_channel * 3 + 1] = g;
//    //                    dataBuffer[2 + ad_adc * 24 + ad_data_channel * 3 + 2] = b;
//                      }
//                    }
//    //                for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
//    //                {
//    //                  dataBuffer[2 + ad_data_channel * 3 + 0] = data2[ad_data_channel * 4 + 1];
//    //                  dataBuffer[2 + ad_data_channel * 3 + 1] = data2[ad_data_channel * 4 + 2];
//    //                  dataBuffer[2 + ad_data_channel * 3 + 2] = data2[ad_data_channel * 4 + 3];
//    //                }
//    //                for(int accel_data_channel = 0; accel_data_channel < 3; accel_data_channel ++)
//    //                {
//    //                  dataBuffer[2 + fft_out * 1 + accel_data_channel * 2 + 0] = 0;
//    //                  dataBuffer[2 + fft_out * 1 + accel_data_channel * 2 + 1] = 0;
//    ////                  dataBuffer[2 + 8 * 3 * 4 + accel_data_channel * 2 + 0] = 0;
//    ////                  dataBuffer[2 + 8 * 3 * 4 + accel_data_channel * 2 + 1] = 0;
//    //                }
//    //                dataBuffer[2 + fft_out * 1 + 3 * 2 + 0] = 0xC0;
//                    dataBuffer[2 + fft_out * 1 + 0] = 0xC0;
//
//    //                dataBuffer[2]=100;
//    //                dataBuffer[3]=101;
//    //                dataBuffer[4]=102;
//
//    //                for(int data_out = 0; data_out < uint8_data_number; data_out ++)
//    //                {
//    //                  print2_hex(dataBuffer[data_out], 8);          print2_symbol(';');
//    //                }
//    //                print2_line();
//
//    //                if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//                    {
//                      while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
//                      {
//                      }
//                      if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
//                      {
//                        Error_Handler();
//                      }
//                    }
//    //                if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
//    //                {
//    //                  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
//    //                  {
//    //                  }
//    //                  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
//    //                  {
//    //                    Error_Handler();
//    //                  }
//    //                }
//
//                  }
//
//                }//FREESMARTEEG_OUT & FREESMARTEEG_FFT_DISPLAY



                if(FREESMARTEEG_OUT & FREESMARTEEG_ARDUINO_MEGA_DISPLAY)
                {
                  if(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_PIN)
                  {
    //              int crc_pair_ok[uint8_ad_adc_number][4];

                    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                    {
                      int crc_ok = 1;
                      for(int ad_data_channel_pair = 0; ad_data_channel_pair < 4; ad_data_channel_pair ++)
                      {
      //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 1];
      //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 2];
      //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 3];

                        uint8_t crc8_=crc8da_7(&(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]));

      //                  uint8_t crc1=((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
      //                      |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)));
      //                  uint8_t chan_0=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b01110000)>>4);
      //                  uint8_t chan_1=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b01110000)>>4);
      //                  print2_hex(chan_0,8);
      //                  print2_symbol(':');
      //                  print2_hex(crc8_,8);
      //                  print2_symbol('-');
      //                  print2_hex(crc1,8);
      ////                  print2_symbol(';');
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(!(((uint8_t)crc8_)
      //                      -((uint8_t)crc1))),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)!(((uint8_t)crc8_)
      //                     -((~((uint8_t)crc1))&0b00000111)),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(chan_0==ad_data_channel_pair*2),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(chan_1==(ad_data_channel_pair*2+1)),8);
      //                  print2_symbol(';');
      //                  if(
      //                      ((!(((uint8_t)crc8_)
      //                          -((uint8_t)crc1)))
      //                     ||(!(((uint8_t)crc8_)
      //                         -((~((uint8_t)crc1))&0b00000111)))
      //                      )
      //                  &&(
      //                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
      //                        ((chan_0==ad_data_channel_pair)))
      //                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
      //                      ((chan_1==(ad_data_channel_pair+1))))
      //                  ))
                        if(
                            ((!(((uint8_t)crc8_)
                                -((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                    |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)))))
                           ||(!(((uint8_t)crc8_)
                               -((~((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                   |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f))))&0b00000111)))
                            )
                        &&(
                          (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
                              ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b11110000)>>4)==ad_data_channel_pair*2)))
                        &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
                            ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b11110000)>>4)==ad_data_channel_pair*2+1)))
                        ))
                        {
      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 1;
                        }
                        else
                        {
      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 0;
                          crc_ok = 0;
                        }

                      }
                      if(crc_ok)
                      {
      //                  print2_symbol('1');
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
      //                  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
                      }
                      else
                      {
      //                  print2_symbol('0');
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
      //                  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

                      }
      //                print2_symbol(';');
                    }
                  }


                  int ad_adc = 0;
    //              for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                  {
                    int ad_data_channel = 0;
    //                for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
                    {
    //                  print2_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);          print2_symbol(';');
    //                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    //                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    //                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    //                  print2_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
          //            print_binary(data[ad_data_channel * 4 + 1], 8);          print_symbol(';');
          //            print_binary(data[ad_data_channel * 4 + 2], 8);          print_symbol(';');
          //            print_binary(data[ad_data_channel * 4 + 3], 8);          print_symbol(';');
    //                  print2_symbol(';');
    //                  datas_out[ad_adc][ad_data_channel * 4 + 0]=datas[ad_adc][ad_data_channel * 4 + 1]&0b10000000;
    //                  datas_out[ad_adc][ad_data_channel * 4 + 1]=datas[ad_adc][ad_data_channel * 4 + 1]&0b01111111;
    //                  datas_out[ad_adc][ad_data_channel * 4 + 2]=datas[ad_adc][ad_data_channel * 4 + 2];
    //                  datas_out[ad_adc][ad_data_channel * 4 + 3]=datas[ad_adc][ad_data_channel * 4 + 3];

    //                  const int sample_number_max = 240;
    //                  int sample[sample_number_max];
    //                  int sample_number_last = -1;
    //                  int sample_average = 0;
    //
    //                  int sample_int_out;
    //                  uint8_t sample_byte_out;
    //                  double sample_int_byte_scale;
    //                  int sample_int_byte_shift;

    //                  sample_int_out = (int)((datas_out[ad_adc][ad_data_channel * 4 + 0]<<24)
    //                          |(datas_out[ad_adc][ad_data_channel * 4 + 1]<<16)
    //                          |(datas_out[ad_adc][ad_data_channel * 4 + 2]<<8)
    //                          |(datas_out[ad_adc][ad_data_channel * 4 + 3]<<0));

    //                  int32_t newInt = (byteBuffer[2] << 24) | (byteBuffer[1] << 16) | byteBuffer[0] << 8;
    //                  // depending on most significant byte, set positive or negative value
    //                  if ((newInt & 0x00008000) > 0) {
    //                    newInt |= 0x000000FF;
    //                  } else {
    //                    newInt &= 0xFFFFFF00;
    //                  }

                      sample_int_out = (datas[ad_adc][ad_data_channel * 4 + 3]<<24)
                              |(datas[ad_adc][ad_data_channel * 4 + 2]<<16)
                              |(datas[ad_adc][ad_data_channel * 4 + 1]<<8);
                      if ((sample_int_out & 0x00008000) > 0) {
                        sample_int_out |= 0x000000FF;
                      } else {
                        sample_int_out &= 0xFFFFFF00;
                      }
                      sample_int_out >>= 8;
    //
    //                  sample_int_out = (datas[ad_adc][ad_data_channel * 4 + 3]<<16)
    //                          |(datas[ad_adc][ad_data_channel * 4 + 2]<<8)
    //                          |(datas[ad_adc][ad_data_channel * 4 + 1]<<0);

                      if(sample_number_now==sample_number_max)
                      {
                        for(int sample_count=0;sample_count<sample_number_now-1;sample_count++)
                        {
                          sample_int[sample_count]=sample_int[sample_count+1];
                          sample_byte[sample_count]=sample_byte[sample_count+1];
                        }
                      }
                      else
                      {
                        sample_number_now++;
                      }
                      sample_int[sample_number_now-1]=sample_int_out;

                      if((data_counter%sample_number_max==0)||(data_counter<sample_number_max))
                      {
                        sample_sum=0;

                        int32_t sample_max_new=sample_int[0];
                        int32_t sample_min_new=sample_int[0];

                        for(int sample_count=0;sample_count<sample_number_now;sample_count++)
                        {
                          sample_sum+=sample_int[sample_count];
                          if(sample_max_new<sample_int[sample_count])
                          {
                            sample_max_new=sample_int[sample_count];
                          }
                          if(sample_min_new>sample_int[sample_count])
                          {
                            sample_min_new=sample_int[sample_count];
                          }
                        }
    //                    sample_average=sample_sum/sample_number_now;

                        if(((sample_max_last<sample_max_new)||(sample_min_last>sample_min_new))||
                            ((sample_max_last>sample_max_new+(sample_max+sample_min)/2)
                                ||(sample_min_last<sample_min_new-(sample_max+sample_min)/2)))
                        {
                          sample_max=sample_max_new;
                          sample_min=sample_min_new;
                        }

                        sample_max_last=sample_max;
                        sample_min_last=sample_min;

                      }

    //                  sample_int_byte_shift = (1<<8)/(1<<32);
    //                  sample_byte_out = sample_int_out + sample_int_byte_shift;
    //                  sample_int_byte_scale = 1;//(1<<8)/(1<<32);
    //                  sample_byte_out = (uint8_t)(sample_int_out * sample_int_byte_scale);

                      sample_byte_out = (uint8_t)(((int64_t)sample_int_out-(int64_t)sample_min)*255/((int64_t)sample_max-(int64_t)sample_min+1));
                      sample_byte[sample_number_now-1]=sample_byte_out;


    //                  print2_symbol(datas_out[ad_adc][ad_data_channel * 4 + 3]);
    //                  print2_symbol((uint8_t)((sample_int_out/(1<<0)+(255/4)*(1))));
    //                  print2_symbol((uint8_t)(((sample_int_out-1000000)/1000000)));
                      if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
                      {
                        print_symbol(sample_byte_out);
                      }
    //                  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
    //                  {
    //                    print2_hex(sample_byte_out, 8);          print2_symbol(';');
    //                    print2_hex(sample_int_out, 32);          print2_symbol(';');
    //                    print2_hex(sample_min, 32);          print2_symbol(';');
    //                    print2_hex(sample_max, 32);          print2_symbol(';');
    //                    print2_line();
    //                  }
                    }
                  }

                }//FREESMARTEEG_OUT & FREESMARTEEG_ARDUINO_MEGA_DISPLAY


                if(FREESMARTEEG_OUT & FREESMARTEEG_DIF)
                {

    //              crc8_01=crc8da_7(&(datas[0][0]));
    //
    //              if(
    //                  ((!(((uint8_t)crc8_01)-((uint8_t)((datas[0][0]<<4)|(datas[0][4]&0x0f)))))
    //                 ||(!(((uint8_t)crc8_01)-((~((uint8_t)((datas[0][0]<<4)|(datas[0][4]&0x0f))))&0b00000111)))
    //                  )
    //              &&(
    //                (//((data1[0]>=0b10000000)&&(data1[0]<0b10010000))||
    //                    ((datas[0][0]>=0b00000000)&&(datas[0][0]<0b00010000)))
    //              &&(//((data1[4]>=0b10010000)&&(data1[4]<0b10100000))||
    //                  ((datas[0][4]>=0b00010000)&&(datas[0][4]<0b00100000)))
    //              ))
    //              {
    //                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    //              }
    //              else
    //              {
    //                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    //              }

    //              crc8_01=crc8da_7(&(data2[0]));
    //
    //              if(
    //                  ((!(((uint8_t)crc8_01)-((uint8_t)((data2[0]<<4)|(data2[4]&0x0f)))))
    //                 ||(!(((uint8_t)crc8_01)-((~((uint8_t)((data2[0]<<4)|(data2[4]&0x0f))))&0b00000111)))
    //                  )
    //              &&(
    //                (((data2[0]>=0b10000000)&&(data2[0]<0b10010000))||((data2[0]>=0b00000000)&&(data2[0]<0b00010000)))
    //              &&(((data2[4]>=0b10010000)&&(data2[4]<0b10100000))||((data2[4]>=0b00010000)&&(data2[4]<0b00100000)))
    //              ))
    //              {
    ////                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    //              }
    //              else
    //              {
    ////                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    //              }

    //              int FFT_Length = 8;
    //              int FFT_Length = 16;
                  int FFT_Length = 32;
    //              int FFT_Length = 48;
    //              int FFT_Length = 64;
    //              int FFT_Length = 80;
    //              int FFT_Length = 128;
    //              int FFT_Length = 256;
                  int multmax = 0;
                  int bufferspeed=1;
    //              int bufferspeed=FFT_Length/4;
                  int index_fill_adc_buffer = last_index_fill_adc_buffer;
                  if(last_index_fill_adc_buffer==FFT_Length*2)
                  {
                    last_index_fill_adc_buffer--;
                    for (index_fill_adc_buffer = 0; index_fill_adc_buffer < FFT_Length*2-bufferspeed; index_fill_adc_buffer ++)
                    {
                      _aADC1ConvertedValue_s[index_fill_adc_buffer] = _aADC1ConvertedValue_s[index_fill_adc_buffer+bufferspeed];
                    }
                  }
                  else
                  {
                    index_fill_adc_buffer++;
                  }
                  _uhADCxConvertedValue = datas[0][1]<<16 | datas[0][2]<<8 | datas[0][3];
                  _aADC1ConvertedValue_s[last_index_fill_adc_buffer] = _uhADCxConvertedValue;
                  for(int mult=0;mult<multmax;mult++)
                  {
                    if(last_index_fill_adc_buffer-mult>=0)
                    {
                      _aADC1ConvertedValue_s[last_index_fill_adc_buffer-mult] = _uhADCxConvertedValue;
                    }
                  }
                  last_index_fill_adc_buffer=index_fill_adc_buffer;

                  if(last_index_fill_adc_buffer==FFT_Length*2)
                  {
                    double median=0;
                    for (int index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length*2; index_fill_output_buffer++)
                    {
                      median+=(double)(_aADC1ConvertedValue_s[index_fill_output_buffer]);
                    }
                    median/=(double)(FFT_Length*2);
                    double stddev_=0;
                    for (int index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length*2; index_fill_output_buffer++)
                    {
                      stddev_+=(((double)(_aADC1ConvertedValue_s[index_fill_output_buffer]))-median)*(((double)(_aADC1ConvertedValue_s[index_fill_output_buffer]))-median);
                    }
                    double stddevmax=0;
                    double stddev=0;
                    if(stddev_)
                    {
                      stddev_=(double)sqrt(((double)(stddev_))/((double)(FFT_Length*2)));

                      for (int index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length*2; index_fill_output_buffer++)
                      {
                        stddev=(double)((((double)(_aADC1ConvertedValue_s[index_fill_output_buffer]))-median)/stddev_);
                        if(stddevmax<stddev)
                        {
                          stddevmax=stddev;
                        }
                      }

    //                  stddev=(double)((((double)(_aADC1ConvertedValue_s[FFT_Length]))-median)/stddev_);
                    }

    //                double stddevh=2.08;

                    double stddevh=4.0;

    //                double stddevh=3.0;
    //                double stddevh=2.9;
    //                double stddevh=2.7;
    //                double stddevh=2.6;
    //                double stddevh=2.2;
    //                double stddevh=2.15;
    //                double stddevh=2.06;
                    if(stddevmax>stddevh)
                    {

    //                if(stddev>stddevh)
                      if(last_stddev_count<=0)
                      {
                        last_stddev_count=FFT_Length*3;
      //                  last_stddev_count=FFT_Length*2;
      //                  last_stddev_count=250;
      //                  last_stddev_count=500;
      //                  last_stddev_count=1000;
      //                  last_index_fill_adc_buffer = 0;
                        uicount++;

                        if(times_count==times_max)
                        {
                          int times_count1;
                          for(times_count1=0;times_count1<times_max-1;times_count1++)
                          {
                            times[times_count1]=times[times_count1+1];
                            stddevmaxs[times_count1]=stddevmaxs[times_count1+1];
                          }
                          times_count=times_count1;
                        }
                        stddevmaxs[times_count]=stddevmax;
                        rtc_read_frac_s(&time1, &frac_secs1);
                        times[times_count]=time1;
                        times_count++;
                        double times_stddev=0;
                        for(int times_stddev_int=0;times_stddev_int<times_count-1;times_stddev_int++)
                        {
      //                    times_stddev=stddevmaxs[times_stddev_int]
                          times_stddev+=(times[times_stddev_int]-times[times_stddev_int+1])*(times[times_stddev_int]-times[times_stddev_int+1]);
                        }
                        times_stddev=sqrt(times_stddev/times_count);

                        if(1)
                        {
                          //                print_hex(uicount,8);
                          //                print_symbol(':');
                          print2_hex(uicount,8);
                          print2_symbol(':');
                          //                for (int index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length; index_fill_output_buffer++)
                                        //  for (index_fill_output_buffer = 0; index_fill_output_buffer < FFT_Length; index_fill_output_buffer++)
                          //                {
                          //                  FFT_Output_Q31[index_fill_output_buffer]=_aADC1ConvertedValue_s[index_fill_output_buffer];
                                        ///    GRAPH_DATA_YT_SetAlign(aGraph_Data[0], GRAPH_ALIGN_LEFT);
                                        ///    GRAPH_DATA_YT_MirrorX (aGraph_Data[0], 1);
                                        ///    GRAPH_DATA_YT_AddValue(aGraph_Data[0], FFT_Output_Q31[index_fill_output_buffer] / 5376212 + 10);
                          //                  print_hex(FFT_Output_Q31[index_fill_output_buffer] / 5376212 + 10, 8);
                          //                  print_symbol(':');
                          //                  print2_hex(FFT_Output_Q31[index_fill_output_buffer] / 5376212 + 10, 8);
                          //                  print2_symbol(':');
                          //                }


                          print2_hex((long)median, 32);
                          print2_symbol(':');
                          print2_hex((long)_aADC1ConvertedValue_s[FFT_Length], 32);
                          print2_symbol(':');
                          print2_hex((long)stddev_, 32);
                          print2_symbol(':');
                          print2_hex((long)(stddevmax*1000), 32);

                          print2_symbol(':');
                          print2_hex((long)time1, 32);
                          print2_symbol(':');
                          print2_hex((long)(times_stddev*1000), 32);
                          print2_symbol(':');
                          print2_hex((long)(60/times_stddev), 32);

          //                print_line();
                          print2_line();
                        }

                        if(stddev>stddevh)
                        {
      //                    print_hex(uicount,8);
      //                    print_symbol(':');
      //                    print_hex((stddevmax*1000), 16);
      ////                    print_symbol(':');
      ////                    print_hex((long)time1, 32);
      //                    print_symbol(':');
      //                    print_hex((long)(60/times_stddev), 16);
      //                    print_line();
                          print_symbol((stddevmax*10));
//                          HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
                        }
                        else
                        {
//                          HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
    //                      print_symbol((stddevmax*1));
                          print_symbol(0);
                        }

                      }
                      else
                      {
//                        HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
    //                    print_symbol((stddevmax*10));
                        print_symbol(0);
                      }
                    }
                    else
                    {
//                      HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
    //                  print_symbol((stddevmax*10));
                      print_symbol(0);
                    }
    //                last_stddev_count/=2;
                    last_stddev_count--;
                  }
                }//FREESMARTEEG_DIF

                if(FREESMARTEEG_OUT & FREESMARTEEG_SAMPLECOUNT)
                {
                  ui32SampleNumber++;
    //              if(!(ui32SampleNumber%250))
    //              if(!(ui32SampleNumber%500))
                  if(!(ui32SampleNumber%512))
    //              if(!(ui32SampleNumber%4000))
    //              if(!(ui32SampleNumber%1000))
    //              if(!(ui32SampleNumber%0x1000))
    //              if(!(ui32SampleNumber%0x100))
    //              if(!(ui32SampleNumber%0x10000))
    //              if(!(ui32SampleNumber%0x20000))
                  {
                    print_hex(ui32SampleNumber,32);
                    print_line();

    //                print2_hex(ui32SampleNumber,32);
    //                print2_line();
//                    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
                  }
                  else
                  {
//                    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
                  }
                }//FREESMARTEEG_SAMPLECOUNT

                if(FREESMARTEEG_OUT & FREESMARTEEG_CRC)
    //            if(0)//crc
                {
                  /*
                  for(int channelpair=0;channelpair<4;channelpair++)
                  {
    //                for(int channelinpair=0;channelinpair<2;channelinpair++)
    //                {
    //                  for(int byteofchannelinpair=0;byteofchannelinpair<3;byteofchannelinpair++)
    //                  {
    //                    crcBuffer[channelpair][channelinpair*3+byteofchannelinpair]=data[channelpair*8+channelinpair*4+byteofchannelinpair+1];
    //                  }
    //                }
                    crcBuffer[channelpair][0]=((data[channelpair*8+0] & 0xf0)) | ((data[channelpair*8+1] & 0xf0) >> 4);
                    crcBuffer[channelpair][1]=((data[channelpair*8+1] & 0x0f) << 4) | ((data[channelpair*8+2] & 0xf0) >> 4);
                    crcBuffer[channelpair][2]=((data[channelpair*8+2] & 0x0f) << 4) | ((data[channelpair*8+3] & 0xf0) >> 4);
                    crcBuffer[channelpair][3]=((data[channelpair*8+3] & 0x0f) << 4) | ((data[channelpair*8+4] & 0xf0) >> 4);
                    crcBuffer[channelpair][4]=data[channelpair*8+5];
                    crcBuffer[channelpair][5]=data[channelpair*8+6];
                    crcBuffer[channelpair][6]=data[channelpair*8+7];
                    crcBuffer[channelpair][7]=0x00;


                    crc8fromad[channelpair]=((data[channelpair*8]&0x0f)<<4) | (data[channelpair*8+4]&0x0f);
                    crc8[channelpair]=ad7779_compute_crc8(crcBuffer[channelpair], 8);

                    crcBuffer[channelpair][0] = crcBuffer[channelpair][0] ^ 0xff;
                    crc8[channelpair]=crc8da(crcBuffer[channelpair], 8);

    //                crc8[channelpair]=crc8da_7(&data[channelpair*8]);

                    if(crc8fromad[channelpair]==crc8[channelpair])
                    {
                      crc8ok[channelpair]=1;
                    }
                    else
                    {
                      crc8ok[channelpair]=0;
                    }
                  }
                  for(int channel=0;channel<8;channel++)
                  {
                    if((data[channel*4]&0x70)>>4==channel)
                    {
                      channelnumberok[channel]=1;
                    }
                    else
                    {
                      channelnumberok[channel]=0;
                    }
                  }
                  */
                 crc8_01=crc8da_7(&(data1[0]));
                 crc8_23=crc8da_7(&(data1[8]));
                 crc8_45=crc8da_7(&(data1[16]));
                 crc8_67=crc8da_7(&(data1[24]));
                 int crcok00=0;
    //             int crcok0=0;
    //             int crcok1=0;
    //             int crcok2=0;
    //             int crcok3=0;

                /*    print_hex(crc8_01,8);    Serial.print(";");
                    print_hex(crc8_23,8);    Serial.print(";");
                    print_hex(crc8_45,8);    Serial.print(";");
                    print_hex(crc8_67,8);    Serial.print(";");
                    Serial.println("");

                    print_hex(((b00<<4)|(b04&0x0f)),8);    Serial.print(";");
                    print_hex(((b08<<4)|(b12&0x0f)),8);    Serial.print(";");
                    print_hex(((b16<<4)|(b20&0x0f)),8);    Serial.print(";");
                    print_hex(((b24<<4)|(b28&0x0f)),8);    Serial.print(";");
                    Serial.println("");*/

                // repeat=1;

                 if(
                  (
                   (((data1[0]>=0b10000000)&&(data1[0]<0b10010000))||((data1[0]>=0b00000000)&&(data1[0]<0b00010000)))
                 &&(((data1[4]>=0b10010000)&&(data1[4]<0b10100000))||((data1[4]>=0b00010000)&&(data1[4]<0b00100000)))
                 &&(((data1[8]>=0b10100000)&&(data1[8]<0b10110000))||((data1[8]>=0b00100000)&&(data1[8]<0b00110000)))
                 &&(((data1[12]>=0b10110000)&&(data1[12]<0b11000000))||((data1[12]>=0b00110000)&&(data1[12]<0b01000000)))
                 &&(((data1[16]>=0b11000000)&&(data1[16]<0b11010000))||((data1[16]>=0b01000000)&&(data1[16]<0b01010000)))
                 &&(((data1[20]>=0b11010000)&&(data1[20]<0b11100000))||((data1[20]>=0b01010000)&&(data1[20]<0b01100000)))
                 &&(((data1[24]>=0b11100000)&&(data1[24]<0b11110000))||((data1[24]>=0b01100000)&&(data1[24]<0b01110000)))
                 &&(((data1[28]>=0b11110000)&&(data1[28]<=0b11111111))||((data1[28]>=0b01110000)&&(data1[28]<=0b01111111)))
                 ))
    //              if((crc8ok[0]&&crc8ok[1]&&crc8ok[2]&&crc8ok[3]) &&
    //                 (channelnumberok[0]&&channelnumberok[1]&&channelnumberok[2]&&channelnumberok[3]&&
    //                  channelnumberok[4]&&channelnumberok[5]&&channelnumberok[6]&&channelnumberok[7]))
                 {
               //   repeat=0;
                   c000++;
                 }

                  if(((!(((uint8_t)crc8_01)-((uint8_t)((data1[0]<<4)|(data1[4]&0x0f)))))
                     &&(!(((uint8_t)crc8_23)-((uint8_t)((data1[8]<<4)|(data1[12]&0x0f)))))
                     &&(!(((uint8_t)crc8_45)-((uint8_t)((data1[16]<<4)|(data1[20]&0x0f)))))
                     &&(!(((uint8_t)crc8_67)-((uint8_t)((data1[24]<<4)|(data1[28]&0x0f))))))
                  && (
                    (((data1[0]>=0b10000000)&&(data1[0]<0b10010000))||((data1[0]>=0b00000000)&&(data1[0]<0b00010000)))
                  &&(((data1[4]>=0b10010000)&&(data1[4]<0b10100000))||((data1[4]>=0b00010000)&&(data1[4]<0b00100000)))
                  &&(((data1[8]>=0b10100000)&&(data1[8]<0b10110000))||((data1[8]>=0b00100000)&&(data1[8]<0b00110000)))
                  &&(((data1[12]>=0b10110000)&&(data1[12]<0b11000000))||((data1[12]>=0b00110000)&&(data1[12]<0b01000000)))
                  &&(((data1[16]>=0b11000000)&&(data1[16]<0b11010000))||((data1[16]>=0b01000000)&&(data1[16]<0b01010000)))
                  &&(((data1[20]>=0b11010000)&&(data1[20]<0b11100000))||((data1[20]>=0b01010000)&&(data1[20]<0b01100000)))
                  &&(((data1[24]>=0b11100000)&&(data1[24]<0b11110000))||((data1[24]>=0b01100000)&&(data1[24]<0b01110000)))
                  &&(((data1[28]>=0b11110000)&&(data1[28]<=0b11111111))||((data1[28]>=0b01110000)&&(data1[28]<=0b01111111)))
                  ))
    //              if((crc8ok[0]&&crc8ok[1]&&crc8ok[2]&&crc8ok[3]) &&
    //                 (channelnumberok[0]&&channelnumberok[1]&&channelnumberok[2]&&channelnumberok[3]&&
    //                  channelnumberok[4]&&channelnumberok[5]&&channelnumberok[6]&&channelnumberok[7]))
                  {
                    crcok00=1;
                //   repeat=0;
                    c00++;

//                    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
//                    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

                    if(0)
                    {
                      for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
                      {
                        print2_binary(data1[ad_data_channel * 4 + 0], 8);          print_symbol(';');
              //          print_hex(data[ad_data_channel * 4 + 1], 8);          print_symbol(';');
              //          print_hex(data[ad_data_channel * 4 + 2], 8);          print_symbol(';');
              //          print_hex(data[ad_data_channel * 4 + 3], 8);          print_symbol(';');
                        print2_hex(data1[ad_data_channel * 4 + 1], 8);          print_symbol(';');
                        print2_hex(data1[ad_data_channel * 4 + 2], 8);          print_symbol(';');
                        print2_hex(data1[ad_data_channel * 4 + 3], 8);          print_symbol(';');
                        print2_symbol(';');
                      }
                      print2_line();
                    }

                  }
                  else
                  {
//                    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
//                    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
                  }
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

                  c01++;

                  if((!(((uint8_t)crc8_01)-((uint8_t)((data1[0]<<4)|(data1[4]&0x0f)))))
                  &&(
                    (((data1[0]>=0b10000000)&&(data1[0]<0b10010000))||((data1[0]>=0b00000000)&&(data1[0]<0b00010000)))
                  &&(((data1[4]>=0b10010000)&&(data1[4]<0b10100000))||((data1[4]>=0b00010000)&&(data1[4]<0b00100000)))
                  ))
                  {
    //                crcok0=1;
                    c0++;
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

                    if(0)
                    {
                      const uint32_t uint8_data_number = 3+4;
                      uint8_t dataBuffer[uint8_data_number];
                      dataBuffer[0] = 0xff;
                      dataBuffer[1] = 0xfe;
                      dataBuffer[2] = 0xfd;
                      dataBuffer[3] = data1[0];
                      dataBuffer[4] = data1[1];
                      dataBuffer[5] = data1[2];
                      dataBuffer[6] = data1[3];
                    //  HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
    //                  HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
                      if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                      {
                        Error_Handler();
                      }
                      while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
                      {
                      }
                    }

                  }
                  else
                  {
                    if((data1[0]&0x0f)==0)
                    {
                      c_0++;
                    }
                    if((data1[4]&0x0f)==((~crc8_01)&0x07))
                    {
                      c_00++;
                    }
                  }
                  if((!(((uint8_t)crc8_23)-((uint8_t)((data1[8]<<4)|(data1[12]&0x0f)))))
                  &&(
                    (((data1[8]>=0b10100000)&&(data1[8]<0b10110000))||((data1[8]>=0b00100000)&&(data1[8]<0b00110000)))
                  &&(((data1[12]>=0b10110000)&&(data1[12]<0b11000000))||((data1[12]>=0b00110000)&&(data1[12]<0b01000000)))
                  ))
                  {
    //                crcok1=1;
                    c1++;
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
                  }
                  else
                  {
                    if((data1[8]&0x0f)==0)
                    {
                      c_1++;
                    }
                    if((data1[12]&0x0f)==((~crc8_23)&0x07))
                    {

                      c_01++;
                    }
                  }
                  if((!(((uint8_t)crc8_45)-((uint8_t)((data1[16]<<4)|(data1[20]&0x0f)))))
                  &&(
                    (((data1[16]>=0b11000000)&&(data1[16]<0b11010000))||((data1[16]>=0b01000000)&&(data1[16]<0b01010000)))
                  &&(((data1[20]>=0b11010000)&&(data1[20]<0b11100000))||((data1[20]>=0b01010000)&&(data1[20]<0b01100000)))
                  ))
                  {
    //                crcok2=1;
                    c2++;
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
                  }
                  else
                  {
                    if((data1[16]&0x0f)==0)
                    {
                      c_2++;
                    }
                    if((data1[20]&0x0f)==((~crc8_45)&0x07))
                    {
                      c_02++;
                    }
                  }
                  if((!(((uint8_t)crc8_67)-((uint8_t)((data1[24]<<4)|(data1[28]&0x0f)))))
                  &&(
                    (((data1[24]>=0b11100000)&&(data1[24]<0b11110000))||((data1[24]>=0b01100000)&&(data1[24]<0b01110000)))
                  &&(((data1[28]>=0b11110000)&&(data1[28]<=0b11111111))||((data1[28]>=0b01110000)&&(data1[28]<=0b01111111)))
                  ))
                  {
    //                crcok3=1;
                    c3++;
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
                  }
                  else
                  {
                    if((data1[24]&0x0f)==0)
                    {
                      c_3++;
                    }
                    if((data1[28]&0x0f)==((~crc8_67)&0x07))
                    {
                      c_03++;
                    }
                  }
                  if(1)
                    if(!crcok00)
                  {
    //                  if(!crcok0)
    //                  {
    //                    print_binary((crc8_01&0x0f),4);
    //                    print_text("_");
    //                    print_binary((data[4]&0x0f),4);
    //                    print_text(";\n");
    //                  }
    //                  if(!crcok1)
    //                  {
    //                    print_binary((crc8_23&0x0f),4);
    //                    print_text("_");
    //                    print_binary((data[12]&0x0f),4);
    //                    print_text(";\n");
    //                  }
    //                  if(!crcok2)
    //                  {
    //                    print_binary((crc8_45&0x0f),4);
    //                    print_text("_");
    //                    print_binary((data[20]&0x0f),4);
    //                    print_text(";\n");
    //                  }
    //                  if(!crcok3)
    //                  {
    //                    print_binary((crc8_67&0x0f),4);
    //                    print_text("_");
    //                    print_binary((data[28]&0x0f),4);
    //                    print_text(";\n");
    //                  }

    //                if(!crcok0)
    //                {
    //                  print_binary((crc8_01&0x0f),4);
    //                  print_text("_");
    //                  print_binary((data[4]&0x0f),4);
    //                  print_text(";");
    //                }
    //                else
    //                {
    //                  print_text("_________");
    //                  print_text(";");
    //                }
    //                if(!crcok1)
    //                {
    //                  print_binary((crc8_23&0x0f),4);
    //                  print_text("_");
    //                  print_binary((data[12]&0x0f),4);
    //                  print_text(";");
    //                }
    //                else
    //                {
    //                  print_text("_________");
    //                  print_text(";");
    //                }
    //                if(!crcok2)
    //                {
    //                  print_binary((crc8_45&0x0f),4);
    //                  print_text("_");
    //                  print_binary((data[20]&0x0f),4);
    //                  print_text(";");
    //                }
    //                else
    //                {
    //                  print_text("_________");
    //                  print_text(";");
    //                }
    //                if(!crcok3)
    //                {
    //                  print_binary((crc8_67&0x0f),4);
    //                  print_text("_");
    //                  print_binary((data[28]&0x0f),4);
    //                  print_text(";;");
    //                }
    //                else
    //                {
    //                  print_text("_________");
    //                  print_text(";;");
    //                }
    //                print_hex((crc8_01&0x0f),4);
    //                print_text("_");
    //                print_hex((data[4]&0x0f),4);
    //                print_text(";");
    //                print_hex((crc8_23&0x0f),4);
    //                print_text("_");
    //                print_hex((data[12]&0x0f),4);
    //                print_text(";");
    //                print_hex((crc8_45&0x0f),4);
    //                print_text("_");
    //                print_hex((data[20]&0x0f),4);
    //                print_text(";");
    //                print_hex((crc8_67&0x0f),4);
    //                print_text("_");
    //                print_hex((data[28]&0x0f),4);
    //                print_text(";;");
                    print_hex((c01*4-(c0+c1+c2+c3))-(c_00+c_01+c_02+c_03),8);
                    print_text(";;");
                    print_hex((c01*4-(c0+c1+c2+c3))-(c_0+c_1+c_2+c_3),8);
                    print_text(";;");
                    print_hex(c_0,8);
                    print_text(";");
                    print_hex(c_1,8);
                    print_text(";");
                    print_hex(c_2,8);
                    print_text(";");
                    print_hex(c_3,8);
                    print_text(";;");
                    print_hex(c01-c000,8);
                    print_text(";");
                    print_hex(0xff*((float)(c000))/((float)(c01)),8);
                    print_text(";;");
                    print_hex(0xff*((float)(c00))/((float)(c01)),8);
                    print_text(";;");
                    print_hex(0xff*((float)(c0))/((float)(c01)),8);
                    print_text(";");
                    print_hex(0xff*((float)(c1))/((float)(c01)),8);
                    print_text(";");
                    print_hex(0xff*((float)(c2))/((float)(c01)),8);
                    print_text(";");
                    print_hex(0xff*((float)(c3))/((float)(c01)),8);
                    print_text(";");
                    print_line();
                  }
                  if(0)
                  {
                    sprintf(str_out, str_in, 100*((float)(c00))/((float)(c01)));
                    print2_text(str_out);
                    print2_text("; ");
                    sprintf(str_out, str_in, 100*((float)(c0))/((float)(c01)));
                    print2_text(str_out);
                    print2_text("; ");
                    sprintf(str_out, str_in, 100*((float)(c1))/((float)(c01)));
                    print2_text(str_out);
                    print2_text("; ");
                    sprintf(str_out, str_in, 100*((float)(c2))/((float)(c01)));
                    print2_text(str_out);
                    print2_text("; ");
                    sprintf(str_out, str_in, 100*((float)(c3))/((float)(c01)));
                    print2_text(str_out);
                    print2_text("; ");
                    print2_line();
                  }
        //            Serial.print(100*((float)(c00))/((float)(c01)),6);    Serial.print("; ");
        //            Serial.print(100*((float)(c0))/((float)(c01)),6);    Serial.print("; ");
        //            Serial.print(100*((float)(c1))/((float)(c01)),6);    Serial.print("; ");
        //            Serial.print(100*((float)(c2))/((float)(c01)),6);    Serial.print("; ");
        //            Serial.print(100*((float)(c3))/((float)(c01)),6);    Serial.print("; ");
                //    Serial.print(100*((float)(c0))/((float)(c0+c1+c2+c3)),6);    Serial.print("; ");
                //    Serial.print(100*((float)(c1))/((float)(c0+c1+c2+c3)),6);    Serial.print("; ");
                //    Serial.print(100*((float)(c2))/((float)(c0+c1+c2+c3)),6);    Serial.print("; ");
                //    Serial.print(100*((float)(c3))/((float)(c0+c1+c2+c3)),6);    Serial.print("; ");
        //            Serial.println("");



                }//text==3//crc

                if(FREESMARTEEG_OUT & FREESMARTEEG_TEXT)
    //            if(0)
                {

    //              int crc_pair_ok[uint8_ad_adc_number][4];

                  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                  {
                    int crc_ok = 1;
                    for(int ad_data_channel_pair = 0; ad_data_channel_pair < 4; ad_data_channel_pair ++)
                    {
    //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 1];
    //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 2];
    //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 3];

                      uint8_t crc8_=crc8da_7(&(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]));

    //                  uint8_t crc1=((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    //                      |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)));
    //                  uint8_t chan_0=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b01110000)>>4);
    //                  uint8_t chan_1=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b01110000)>>4);
    //                  print2_hex(chan_0,8);
    //                  print2_symbol(':');
    //                  print2_hex(crc8_,8);
    //                  print2_symbol('-');
    //                  print2_hex(crc1,8);
    ////                  print2_symbol(';');
    //                  print2_symbol(',');
    //                  print2_hex((uint8_t)(!(((uint8_t)crc8_)
    //                      -((uint8_t)crc1))),8);
    //                  print2_symbol(',');
    //                  print2_hex((uint8_t)!(((uint8_t)crc8_)
    //                     -((~((uint8_t)crc1))&0b00000111)),8);
    //                  print2_symbol(',');
    //                  print2_hex((uint8_t)(chan_0==ad_data_channel_pair*2),8);
    //                  print2_symbol(',');
    //                  print2_hex((uint8_t)(chan_1==(ad_data_channel_pair*2+1)),8);
    //                  print2_symbol(';');
    //                  if(
    //                      ((!(((uint8_t)crc8_)
    //                          -((uint8_t)crc1)))
    //                     ||(!(((uint8_t)crc8_)
    //                         -((~((uint8_t)crc1))&0b00000111)))
    //                      )
    //                  &&(
    //                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
    //                        ((chan_0==ad_data_channel_pair)))
    //                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
    //                      ((chan_1==(ad_data_channel_pair+1))))
    //                  ))
                      if(
                          ((!(((uint8_t)crc8_)
                              -((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                  |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)))))
                         ||(!(((uint8_t)crc8_)
                             -((~((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                 |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f))))&0b00000111)))
                          )
                      &&(
                        (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
                            ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b11110000)>>4)==ad_data_channel_pair*2)))
                      &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
                          ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b11110000)>>4)==ad_data_channel_pair*2+1)))
                      ))
                      {
    //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 1;
                      }
                      else
                      {
    //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 0;
                        crc_ok = 0;
                      }

                    }
                    if(crc_ok)
                    {
    //                  print2_symbol('1');
//                      HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
      //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
                    }
                    else
                    {
    //                  print2_symbol('0');
//                      HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
      //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    //                  ad7779_do_spi_soft_reset(devices[ad_adc]);
    //                  HAL_GPIO_WritePin(reset_ports[ad_adc], reset_pins[ad_adc], GPIO_PIN_RESET);
    //                  HAL_Delay(1);
    //                  HAL_GPIO_WritePin(reset_ports[ad_adc], reset_pins[ad_adc], GPIO_PIN_SET);
    //
    //                  ad7779_setup(&devices[ad_adc], init_params[ad_adc]);
    //
    //                    {
    //                      aTxBuffer[0] = AD7779_REG_GENERAL_USER_CONFIG_3;
    //                      aTxBuffer[1] = AD7779_SPI_SLAVE_MODE_EN;
    //
    //            //          HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_RESET);
    //                      if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
    //            //          if(HAL_SPI_TransmitReceive(device2->spi_dev.dev, aTxBuffer, data2, 2, 5000) != HAL_OK)
    //                      {
    //                        Error_Handler();
    //                      }
    //            //          HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_SET);
    //
    //                      while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
    //                      {
    //                      }
    //
    //                    }
    //
    //                    {
    //                      aTxBuffer[0] = AD7779_REG_GEN_ERR_REG_1_EN;
    //                      aTxBuffer[1] = AD7779_SPI_CRC_TEST_EN;
    //
    //            //          HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_RESET);
    //                      if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
    //            //          if(HAL_SPI_TransmitReceive(device2->spi_dev.dev, aTxBuffer, data2, 2, 5000) != HAL_OK)
    //                      {
    //                        Error_Handler();
    //                      }
    //            //          HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_SET);
    //
    //                      while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
    //                      {
    //                      }
    //
    //                    }

                    }
    //                print2_symbol(';');
                  }



    //              crc8_01=crc8da_7(&(data[0]));
    //
    //              if(
    //                  ((!(((uint8_t)crc8_01)-((uint8_t)((data[0]<<4)|(data[4]&0x0f)))))
    //                 ||(!(((uint8_t)crc8_01)-((~((uint8_t)((data[0]<<4)|(data[4]&0x0f))))&0b00000111)))
    //                  )
    //              &&(
    //                (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
    //                    ((data[0]>=0b00000000)&&(data[0]<0b00010000)))
    //              &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
    //                  ((data[4]>=0b00010000)&&(data[4]<0b00100000)))
    //              ))
    //              {
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    ////                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    //              }
    //              else
    //              {
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    ////                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    //              }
    //
    //              crc8_01=crc8da_7(&(data2[0]));
    //
    //              if(
    //                  ((!(((uint8_t)crc8_01)-((uint8_t)((data2[0]<<4)|(data2[4]&0x0f)))))
    //                 ||(!(((uint8_t)crc8_01)-((~((uint8_t)((data2[0]<<4)|(data2[4]&0x0f))))&0b00000111)))
    //                  )
    //              &&(
    //                (//((data2[0]>=0b10000000)&&(data2[0]<0b10010000))||
    //                    ((data2[0]>=0b00000000)&&(data2[0]<0b00010000)))
    //              &&(//((data2[4]>=0b10010000)&&(data2[4]<0b10100000))||
    //                  ((data2[4]>=0b00010000)&&(data2[4]<0b00100000)))
    //              ))
    //              {
    ////                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    //              }
    //              else
    //              {
    ////                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    //              }
    //
    //              crc8_01=crc8da_7(&(data3[0]));
    //
    //              if(
    //                  ((!(((uint8_t)crc8_01)-((uint8_t)((data3[0]<<4)|(data3[4]&0x0f)))))
    //                 ||(!(((uint8_t)crc8_01)-((~((uint8_t)((data3[0]<<4)|(data3[4]&0x0f))))&0b00000111)))
    //                  )
    //              &&(
    //                (//((data3[0]>=0b10000000)&&(data3[0]<0b10010000))||
    //                    ((data3[0]>=0b00000000)&&(data3[0]<0b00010000)))
    //              &&(//((data3[4]>=0b10010000)&&(data3[4]<0b10100000))||
    //                  ((data3[4]>=0b00010000)&&(data3[4]<0b00100000)))
    //              ))
    //              {
    ////                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
    //              }
    //              else
    //              {
    ////                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
    //              }
    //
    //              crc8_01=crc8da_7(&(data4[0]));
    //
    //              if(
    //                  ((!(((uint8_t)crc8_01)-((uint8_t)((data4[0]<<4)|(data4[4]&0x0f)))))
    //                 ||(!(((uint8_t)crc8_01)-((~((uint8_t)((data4[0]<<4)|(data4[4]&0x0f))))&0b00000111)))
    //                  )
    //              &&(
    //                (//((data4[0]>=0b10000000)&&(data4[0]<0b10010000))||
    //                    ((data4[0]>=0b00000000)&&(data4[0]<0b00010000)))
    //              &&(//((data4[4]>=0b10010000)&&(data4[4]<0b10100000))||
    //                  ((data4[4]>=0b00010000)&&(data4[4]<0b00100000)))
    //              ))
    //              {
    ////                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
    //              }
    //              else
    //              {
    ////                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
    //              }

    //              int ad_adc = 3;
                  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                  {
    //                int ad_data_channel = 6;
                    for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
                    {
    //                  print2_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);          print2_symbol(';');
                      print2_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
                      print2_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
                      print2_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
                      print2_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
          //            print_binary(data[ad_data_channel * 4 + 1], 8);          print_symbol(';');
          //            print_binary(data[ad_data_channel * 4 + 2], 8);          print_symbol(';');
          //            print_binary(data[ad_data_channel * 4 + 3], 8);          print_symbol(';');
                      print2_symbol(';');
                    }
                    print2_symbol(';');
                  }

    //              for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
    //              {
    ////                print2_binary(data[ad_data_channel * 4 + 0], 8);          print2_symbol(';');
    //                print2_hex(data[ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    ////                print2_hex(data[ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    ////                print2_hex(data[ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    ////                print2_hex(data[ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
    //    //            print_binary(data[ad_data_channel * 4 + 1], 8);          print_symbol(';');
    //    //            print_binary(data[ad_data_channel * 4 + 2], 8);          print_symbol(';');
    //    //            print_binary(data[ad_data_channel * 4 + 3], 8);          print_symbol(';');
    //                print2_symbol(';');
    //              }
    //              print2_symbol(';');
    //
    //              for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
    //              {
    ////                print2_binary(data2[ad_data_channel * 4 + 0], 8);          print2_symbol(';');
    //                print2_hex(data2[ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    ////                print2_hex(data2[ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    ////                print2_hex(data2[ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    ////                print2_hex(data2[ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
    //    //            print_binary(data[ad_data_channel * 4 + 1], 8);          print_symbol(';');
    //    //            print_binary(data[ad_data_channel * 4 + 2], 8);          print_symbol(';');
    //    //            print_binary(data[ad_data_channel * 4 + 3], 8);          print_symbol(';');
    //                print2_symbol(';');
    //              }
    //              print2_symbol(';');
    //
    //              for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
    //              {
    ////                print2_binary(data2[ad_data_channel * 4 + 0], 8);          print2_symbol(';');
    //                print2_hex(data3[ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    ////                print2_hex(data3[ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    ////                print2_hex(data3[ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    ////                print2_hex(data3[ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
    //    //            print_binary(data[ad_data_channel * 4 + 1], 8);          print_symbol(';');
    //    //            print_binary(data[ad_data_channel * 4 + 2], 8);          print_symbol(';');
    //    //            print_binary(data[ad_data_channel * 4 + 3], 8);          print_symbol(';');
    //                print2_symbol(';');
    //              }
    //              print2_symbol(';');
    //
    //              for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
    //              {
    ////                print2_binary(data2[ad_data_channel * 4 + 0], 8);          print2_symbol(';');
    //                print2_hex(data4[ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    ////                print2_hex(data4[ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    ////                print2_hex(data4[ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    ////                print2_hex(data4[ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
    //    //            print_binary(data[ad_data_channel * 4 + 1], 8);          print_symbol(';');
    //    //            print_binary(data[ad_data_channel * 4 + 2], 8);          print_symbol(';');
    //    //            print_binary(data[ad_data_channel * 4 + 3], 8);          print_symbol(';');
    //                print2_symbol(';');
    //              }

                  print2_symbol(';');
                  print2_line();
                }
                if(FREESMARTEEG_OUT & FREESMARTEEG_TEXT_UART1)
    //            if(0)
                {

    //              int crc_pair_ok[uint8_ad_adc_number][4];

                  if(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_PIN)
                  {

                    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                    {
                      int crc_ok = 1;
                      int crc_something_ok = 0;
                      for(int ad_data_channel_pair = 0; ad_data_channel_pair < (uint8_ad_chan_number / 2); ad_data_channel_pair ++)
                      {

                        uint8_t crc8_=crc8da_7(&(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]));

      //                  uint8_t crc1=((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
      //                      |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)));
      //                  uint8_t chan_0=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b01110000)>>4);
      //                  uint8_t chan_1=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b01110000)>>4);
      //                  print2_hex(chan_0,8);
      //                  print2_symbol(':');
      //                  print2_hex(crc8_,8);
      //                  print2_symbol('-');
      //                  print2_hex(crc1,8);
      ////                  print2_symbol(';');
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(!(((uint8_t)crc8_)
      //                      -((uint8_t)crc1))),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)!(((uint8_t)crc8_)
      //                     -((~((uint8_t)crc1))&0b00000111)),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(chan_0==ad_data_channel_pair*2),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(chan_1==(ad_data_channel_pair*2+1)),8);
      //                  print2_symbol(';');
      //                  if(
      //                      ((!(((uint8_t)crc8_)
      //                          -((uint8_t)crc1)))
      //                     ||(!(((uint8_t)crc8_)
      //                         -((~((uint8_t)crc1))&0b00000111)))
      //                      )
      //                  &&(
      //                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
      //                        ((chan_0==ad_data_channel_pair)))
      //                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
      //                      ((chan_1==(ad_data_channel_pair+1))))
      //                  ))
                        if(
                            ((!(((uint8_t)crc8_)
                                -((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                    |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)))))
                           ||(!(((uint8_t)crc8_)
                               -((~((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                   |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f))))&0b00000111)))
                            )
                        &&(
                          (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
                              ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]
                                                &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
                                                    ad_data_channel_pair*2)))
                        &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
                            ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]
                                              &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
                                                  ad_data_channel_pair*2+1)))
                        ))
                        {
      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 1;
                          crc_something_ok = 1;
                        }
                        else
                        {
      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 0;
                          crc_ok = 0;
                        }
                      }
                      if((crc_ok)||((crc_something_ok)&&(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_SOMETHING)))
                      {
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
                      }
                      else
                      {
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
                      }
                    }
                  }

                  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                  {
                    for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
                    {
                      print_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
                      print_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
                      print_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
                      print_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
                      print_symbol(';');
                    }
                    print_symbol(';');
                  }
                  print_symbol(';');

                  print_line();

                }//FREESMARTEEG_TEXT_UART1

                if(FREESMARTEEG_OUT & FREESMARTEEG_TEXT_UART7)
    //            if(0)
                {

    //              int crc_pair_ok[uint8_ad_adc_number][4];

                  if(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_PIN)
                  {

                    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                    {
                      int crc_ok = 1;
                      int crc_something_ok = 0;
                      for(int ad_data_channel_pair = 0; ad_data_channel_pair < (uint8_ad_chan_number / 2); ad_data_channel_pair ++)
                      {

                        uint8_t crc8_=crc8da_7(&(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]));

      //                  uint8_t crc1=((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
      //                      |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)));
      //                  uint8_t chan_0=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b01110000)>>4);
      //                  uint8_t chan_1=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b01110000)>>4);
      //                  print2_hex(chan_0,8);
      //                  print2_symbol(':');
      //                  print2_hex(crc8_,8);
      //                  print2_symbol('-');
      //                  print2_hex(crc1,8);
      ////                  print2_symbol(';');
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(!(((uint8_t)crc8_)
      //                      -((uint8_t)crc1))),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)!(((uint8_t)crc8_)
      //                     -((~((uint8_t)crc1))&0b00000111)),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(chan_0==ad_data_channel_pair*2),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(chan_1==(ad_data_channel_pair*2+1)),8);
      //                  print2_symbol(';');
      //                  if(
      //                      ((!(((uint8_t)crc8_)
      //                          -((uint8_t)crc1)))
      //                     ||(!(((uint8_t)crc8_)
      //                         -((~((uint8_t)crc1))&0b00000111)))
      //                      )
      //                  &&(
      //                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
      //                        ((chan_0==ad_data_channel_pair)))
      //                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
      //                      ((chan_1==(ad_data_channel_pair+1))))
      //                  ))
                        if(
                            ((!(((uint8_t)crc8_)
                                -((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                    |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)))))
                           ||(!(((uint8_t)crc8_)
                               -((~((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                   |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f))))&0b00000111)))
                            )
                        &&(
                          (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
                              ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]
                                                &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
                                                    ad_data_channel_pair*2)))
                        &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
                            ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]
                                              &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
                                                  ad_data_channel_pair*2+1)))
                        ))
                        {
      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 1;
                          crc_something_ok = 1;
                        }
                        else
                        {
      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 0;
                          crc_ok = 0;
                        }
                      }
                      if((crc_ok)||((crc_something_ok)&&(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_SOMETHING)))
                      {
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
                      }
                      else
                      {
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
                      }
                    }
                  }

                  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                  {
                    for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
                    {
                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
                      print7_symbol(';');
                    }
                    print7_symbol(';');
                  }
                  print7_symbol(';');

                  print7_line();

                }//FREESMARTEEG_TEXT_UART7


    //            if((FREESMARTEEG_OUT & FREESMARTEEG_WRITE_SDCARD)
    //                && (usart1_cts_pin || !(FREESMARTEEG_OUT & FREESMARTEEG_WRITE_SDCARD_OPENVIBE)))
    ////            if(0)
    //            {
    //
    ////              int crc_pair_ok[uint8_ad_adc_number][4];
    //
    //              if(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_PIN)
    //              {
    //
    //                for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    //                {
    //                  int crc_ok = 1;
    //                  int crc_something_ok = 0;
    //                  for(int ad_data_channel_pair = 0; ad_data_channel_pair < (uint8_ad_chan_number / 2); ad_data_channel_pair ++)
    //                  {
    //
    //                    uint8_t crc8_=crc8da_7(&(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]));
    //
    //  //                  uint8_t crc1=((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    //  //                      |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)));
    //  //                  uint8_t chan_0=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b01110000)>>4);
    //  //                  uint8_t chan_1=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b01110000)>>4);
    //  //                  print2_hex(chan_0,8);
    //  //                  print2_symbol(':');
    //  //                  print2_hex(crc8_,8);
    //  //                  print2_symbol('-');
    //  //                  print2_hex(crc1,8);
    //  ////                  print2_symbol(';');
    //  //                  print2_symbol(',');
    //  //                  print2_hex((uint8_t)(!(((uint8_t)crc8_)
    //  //                      -((uint8_t)crc1))),8);
    //  //                  print2_symbol(',');
    //  //                  print2_hex((uint8_t)!(((uint8_t)crc8_)
    //  //                     -((~((uint8_t)crc1))&0b00000111)),8);
    //  //                  print2_symbol(',');
    //  //                  print2_hex((uint8_t)(chan_0==ad_data_channel_pair*2),8);
    //  //                  print2_symbol(',');
    //  //                  print2_hex((uint8_t)(chan_1==(ad_data_channel_pair*2+1)),8);
    //  //                  print2_symbol(';');
    //  //                  if(
    //  //                      ((!(((uint8_t)crc8_)
    //  //                          -((uint8_t)crc1)))
    //  //                     ||(!(((uint8_t)crc8_)
    //  //                         -((~((uint8_t)crc1))&0b00000111)))
    //  //                      )
    //  //                  &&(
    //  //                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
    //  //                        ((chan_0==ad_data_channel_pair)))
    //  //                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
    //  //                      ((chan_1==(ad_data_channel_pair+1))))
    //  //                  ))
    //                    if(
    //                        ((!(((uint8_t)crc8_)
    //                            -((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    //                                |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)))))
    //                       ||(!(((uint8_t)crc8_)
    //                           -((~((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    //                               |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f))))&0b00000111)))
    //                        )
    //                    &&(
    //                      (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
    //                          ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]
    //                                            &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
    //                                                ad_data_channel_pair*2)))
    //                    &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
    //                        ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]
    //                                          &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
    //                                              ad_data_channel_pair*2+1)))
    //                    ))
    //                    {
    //  //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 1;
    //                      crc_something_ok = 1;
    //                    }
    //                    else
    //                    {
    //  //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 0;
    //                      crc_ok = 0;
    //                    }
    //                  }
    //                  if((crc_ok)||((crc_something_ok)&&(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_SOMETHING)))
    //                  {
    //                    HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
    //                  }
    //                  else
    //                  {
    //                    HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
    //                  }
    //                }
    //              }
    //
    ////              if(1)
    ////              {
    //                const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1;
    //                uint8_t dataBuffer[uint8_data_number];
    //
    //                dataBuffer[0] = 0xA0;
    //                dataBuffer[1] = ui8SampleNumber++;
    //
    //                for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    //                {
    //                  for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
    //                  {
    //                    dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 1];
    //                    dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 2];
    //                    dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 3];
    //                  }
    //                }
    //                for(int accel_data_channel = 0; accel_data_channel < uint8_accel_chan_number; accel_data_channel ++)
    //                {
    //                  dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 0] = 0;
    //                  dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 1] = 0;
    //                }
    //                dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0] = 0xC0;
    //
    ////                if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
    ////                {
    ////                  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    ////                  {
    ////                  }
    ////                  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
    ////                  {
    ////                    Error_Handler();
    ////                  }
    ////                }
    ////                if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
    ////                {
    ////                  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
    ////                  {
    ////                  }
    ////                  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
    ////                  {
    ////                    Error_Handler();
    ////                  }
    ////                }
    //
    //
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    ////              }
    ////              else
    ////              {
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    ////              }
    //
    //
    ////              uint8_t testBuffer[uint8_ad_adc_number*(uint8_ad_chan_number*(4+1)+1)+1+2+1] = "";
    ////              uint8_t testBufferCount=0;
    //
    ////              FATFS fileSystem;
    ////              FIL testFile;
    ////              UINT testBytes;
    ////              FRESULT res;
    //
    ////              if(f_mount(&fileSystem, SD_Path, 1) == FR_OK)
    ////              {
    ////                uint8_t path[13] = "testfil2.fe3";
    //////                path = "testfil1.fe3";
    ////                path[12] = '\0';
    ////
    ////          //      res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
    ////                res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
    ////
    ////                res = f_lseek(&testFile, f_size(&testFile));
    //
    //                res = f_write(&testFile, dataBuffer, uint8_data_number, &testBytes);
    ////                res = f_write(&testFile, datas, uint8_ad_adc_number*uint8_ad_chan_number*4, &testBytes);
    ////                res = f_write(&testFile, testBuffer, 16, &testBytes);
    //
    ////                res = f_close(&testFile);
    ////              }
    //             if(0)
    //             {
    //              for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    //              {
    //                for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
    //                {
    ////                  buffer_hex(testBuffer, datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    ////                  buffer_hex(testBuffer, datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    ////                  buffer_hex(testBuffer, datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    ////                  buffer_hex(testBuffer, datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
    ////                  buffer_symbol(testBuffer, ';');
    //                  print_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    //                  print_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    //                  print_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    //                  print_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
    //                  print_symbol(';');
    //                }
    ////                buffer_symbol(testBuffer, ';');
    //                print_symbol(';');
    //              }
    ////              buffer_symbol(testBuffer, ';');
    //
    ////              buffer_line(testBuffer);
    //              print_symbol(';');
    ////
    //              print_line();
    //             }
    //
    //            }//FREESMARTEEG_WRITE_SDCARD

    //            if((FREESMARTEEG_OUT & FREESMARTEEG_SDCARD_OPENVIBE)
    //                && ((!usart1_cts_pin) || !(FREESMARTEEG_OUT & FREESMARTEEG_WRITE_SDCARD_OPENVIBE)))
    ////            if(0)
    //            {
    //
    ////              int crc_pair_ok[uint8_ad_adc_number][4];
    //
    //              if(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_PIN)
    //              {
    //
    //                for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    //                {
    //                  int crc_ok = 1;
    //                  int crc_something_ok = 0;
    //                  for(int ad_data_channel_pair = 0; ad_data_channel_pair < (uint8_ad_chan_number / 2); ad_data_channel_pair ++)
    //                  {
    //
    //                    uint8_t crc8_=crc8da_7(&(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]));
    //
    //  //                  uint8_t crc1=((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    //  //                      |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)));
    //  //                  uint8_t chan_0=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b01110000)>>4);
    //  //                  uint8_t chan_1=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b01110000)>>4);
    //  //                  print2_hex(chan_0,8);
    //  //                  print2_symbol(':');
    //  //                  print2_hex(crc8_,8);
    //  //                  print2_symbol('-');
    //  //                  print2_hex(crc1,8);
    //  ////                  print2_symbol(';');
    //  //                  print2_symbol(',');
    //  //                  print2_hex((uint8_t)(!(((uint8_t)crc8_)
    //  //                      -((uint8_t)crc1))),8);
    //  //                  print2_symbol(',');
    //  //                  print2_hex((uint8_t)!(((uint8_t)crc8_)
    //  //                     -((~((uint8_t)crc1))&0b00000111)),8);
    //  //                  print2_symbol(',');
    //  //                  print2_hex((uint8_t)(chan_0==ad_data_channel_pair*2),8);
    //  //                  print2_symbol(',');
    //  //                  print2_hex((uint8_t)(chan_1==(ad_data_channel_pair*2+1)),8);
    //  //                  print2_symbol(';');
    //  //                  if(
    //  //                      ((!(((uint8_t)crc8_)
    //  //                          -((uint8_t)crc1)))
    //  //                     ||(!(((uint8_t)crc8_)
    //  //                         -((~((uint8_t)crc1))&0b00000111)))
    //  //                      )
    //  //                  &&(
    //  //                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
    //  //                        ((chan_0==ad_data_channel_pair)))
    //  //                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
    //  //                      ((chan_1==(ad_data_channel_pair+1))))
    //  //                  ))
    //                    if(
    //                        ((!(((uint8_t)crc8_)
    //                            -((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    //                                |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)))))
    //                       ||(!(((uint8_t)crc8_)
    //                           -((~((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    //                               |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f))))&0b00000111)))
    //                        )
    //                    &&(
    //                      (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
    //                          ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]
    //                                            &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
    //                                                ad_data_channel_pair*2)))
    //                    &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
    //                        ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]
    //                                          &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
    //                                              ad_data_channel_pair*2+1)))
    //                    ))
    //                    {
    //  //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 1;
    //                      crc_something_ok = 1;
    //                    }
    //                    else
    //                    {
    //  //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 0;
    //                      crc_ok = 0;
    //                    }
    //                  }
    //                  if((crc_ok)||((crc_something_ok)&&(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_SOMETHING)))
    //                  {
    //                    HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
    //                  }
    //                  else
    //                  {
    //                    HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
    //                  }
    //                }
    //              }
    //
    ////              if(1)
    ////              {
    //                const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1;
    //                uint8_t dataBuffer[uint8_data_number];
    ////
    ////                dataBuffer[0] = 0xA0;
    ////                dataBuffer[1] = ui8SampleNumber++;
    ////
    ////                for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    ////                {
    ////                  for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
    ////                  {
    ////                    dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 1];
    ////                    dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 2];
    ////                    dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 3];
    ////                  }
    ////                }
    ////                for(int accel_data_channel = 0; accel_data_channel < uint8_accel_chan_number; accel_data_channel ++)
    ////                {
    ////                  dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 0] = 0;
    ////                  dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 1] = 0;
    ////                }
    ////                dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0] = 0xC0;
    //
    ////                if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
    ////                {
    ////                  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    ////                  {
    ////                  }
    ////                  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
    ////                  {
    ////                    Error_Handler();
    ////                  }
    ////                }
    ////                if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
    ////                {
    ////                  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
    ////                  {
    ////                  }
    ////                  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
    ////                  {
    ////                    Error_Handler();
    ////                  }
    ////                }
    //
    //
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    ////              }
    ////              else
    ////              {
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    ////              }
    //
    //
    ////              uint8_t testBuffer[uint8_ad_adc_number*(uint8_ad_chan_number*(4+1)+1)+1+2+1] = "";
    ////              uint8_t testBufferCount=0;
    //
    ////              FATFS fileSystem;
    ////              FIL testFile;
    ////              UINT testBytes;
    ////              FRESULT res;
    //
    ////              if(f_mount(&fileSystem, SD_Path, 1) == FR_OK)
    ////              {
    //////                uint8_t path[13] = "testfil1.fe3";
    ////                path = "testfil1.fe3";
    ////                path[12] = '\0';
    ////
    ////          //      res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
    //////                res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
    ////                res = f_open(&testFile, (char*)path, FA_READ | FA_OPEN_EXISTING);
    ////
    //////                res = f_lseek(&testFile, f_size(&testFile));
    ////                res = f_lseek(&testFile, uint8_data_number*(data_counter-1));
    ////
    //////                res = f_write(&testFile, dataBuffer, uint8_data_number, &testBytes);
    //////                res = f_write(&testFile, datas, uint8_ad_adc_number*uint8_ad_chan_number*4, &testBytes);
    //////                res = f_write(&testFile, testBuffer, 16, &testBytes);
    //                res = f_read(&testFile, dataBuffer, uint8_data_number, &testBytes);
    //
    ////                res = f_close(&testFile);
    ////              }
    //
    ////              for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    ////              {
    ////                for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
    ////                {
    //////                  buffer_hex(testBuffer, datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    //////                  buffer_hex(testBuffer, datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    //////                  buffer_hex(testBuffer, datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    //////                  buffer_hex(testBuffer, datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
    //////                  buffer_symbol(testBuffer, ';');
    ////                  print_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    ////                  print_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    ////                  print_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    ////                  print_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
    ////                  print_symbol(';');
    ////                }
    //////                buffer_symbol(testBuffer, ';');
    ////                print_symbol(';');
    ////              }
    //////              buffer_symbol(testBuffer, ';');
    ////
    //////              buffer_line(testBuffer);
    ////              print_symbol(';');
    //////
    ////              print_line();
    //
    //              if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
    //              {
    //                while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    //                {
    //                }
    //                if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
    //                {
    //                  Error_Handler();
    //                }
    //              }
    //
    //            }//FREESMARTEEG_SDCARD_OPENVIBE
    //
    //            if(FREESMARTEEG_OUT & FREESMARTEEG_SDCARD_TEXT)
    ////            if(0)
    //            {
    //
    ////              int crc_pair_ok[uint8_ad_adc_number][4];
    //
    //              if(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_PIN)
    //              {
    //
    //                for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    //                {
    //                  int crc_ok = 1;
    //                  int crc_something_ok = 0;
    //                  for(int ad_data_channel_pair = 0; ad_data_channel_pair < (uint8_ad_chan_number / 2); ad_data_channel_pair ++)
    //                  {
    //
    //                    uint8_t crc8_=crc8da_7(&(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]));
    //
    //  //                  uint8_t crc1=((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    //  //                      |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)));
    //  //                  uint8_t chan_0=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b01110000)>>4);
    //  //                  uint8_t chan_1=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b01110000)>>4);
    //  //                  print2_hex(chan_0,8);
    //  //                  print2_symbol(':');
    //  //                  print2_hex(crc8_,8);
    //  //                  print2_symbol('-');
    //  //                  print2_hex(crc1,8);
    //  ////                  print2_symbol(';');
    //  //                  print2_symbol(',');
    //  //                  print2_hex((uint8_t)(!(((uint8_t)crc8_)
    //  //                      -((uint8_t)crc1))),8);
    //  //                  print2_symbol(',');
    //  //                  print2_hex((uint8_t)!(((uint8_t)crc8_)
    //  //                     -((~((uint8_t)crc1))&0b00000111)),8);
    //  //                  print2_symbol(',');
    //  //                  print2_hex((uint8_t)(chan_0==ad_data_channel_pair*2),8);
    //  //                  print2_symbol(',');
    //  //                  print2_hex((uint8_t)(chan_1==(ad_data_channel_pair*2+1)),8);
    //  //                  print2_symbol(';');
    //  //                  if(
    //  //                      ((!(((uint8_t)crc8_)
    //  //                          -((uint8_t)crc1)))
    //  //                     ||(!(((uint8_t)crc8_)
    //  //                         -((~((uint8_t)crc1))&0b00000111)))
    //  //                      )
    //  //                  &&(
    //  //                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
    //  //                        ((chan_0==ad_data_channel_pair)))
    //  //                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
    //  //                      ((chan_1==(ad_data_channel_pair+1))))
    //  //                  ))
    //                    if(
    //                        ((!(((uint8_t)crc8_)
    //                            -((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    //                                |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)))))
    //                       ||(!(((uint8_t)crc8_)
    //                           -((~((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    //                               |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f))))&0b00000111)))
    //                        )
    //                    &&(
    //                      (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
    //                          ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]
    //                                            &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
    //                                                ad_data_channel_pair*2)))
    //                    &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
    //                        ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]
    //                                          &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
    //                                              ad_data_channel_pair*2+1)))
    //                    ))
    //                    {
    //  //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 1;
    //                      crc_something_ok = 1;
    //                    }
    //                    else
    //                    {
    //  //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 0;
    //                      crc_ok = 0;
    //                    }
    //                  }
    //                  if((crc_ok)||((crc_something_ok)&&(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_SOMETHING)))
    //                  {
    //                    HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
    //                  }
    //                  else
    //                  {
    //                    HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
    //                  }
    //                }
    //              }
    //
    ////              if(1)
    ////              {
    //                const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1;
    //                uint8_t dataBuffer[uint8_data_number];
    //
    ////                dataBuffer[0] = 0xA0;
    ////                dataBuffer[1] = ui8SampleNumber++;
    ////
    ////                for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    ////                {
    ////                  for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
    ////                  {
    ////                    dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 1];
    ////                    dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 2];
    ////                    dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 3];
    ////                  }
    ////                }
    ////                for(int accel_data_channel = 0; accel_data_channel < uint8_accel_chan_number; accel_data_channel ++)
    ////                {
    ////                  dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 0] = 0;
    ////                  dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 1] = 0;
    ////                }
    ////                dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0] = 0xC0;
    //
    ////                if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
    ////                {
    ////                  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    ////                  {
    ////                  }
    ////                  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
    ////                  {
    ////                    Error_Handler();
    ////                  }
    ////                }
    ////                if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
    ////                {
    ////                  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
    ////                  {
    ////                  }
    ////                  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
    ////                  {
    ////                    Error_Handler();
    ////                  }
    ////                }
    //
    //
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    ////              }
    ////              else
    ////              {
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    ////              }
    //
    //
    ////              uint8_t testBuffer[uint8_ad_adc_number*(uint8_ad_chan_number*(4+1)+1)+1+2+1] = "";
    ////              uint8_t testBufferCount=0;
    //
    ////              FATFS fileSystem;
    ////              FIL testFile;
    ////              UINT testBytes;
    ////              FRESULT res;
    //
    ////              if(f_mount(&fileSystem, SD_Path, 1) == FR_OK)
    ////              {
    ////                uint8_t path[13] = "testfil1.fe3";
    //////                path = "testfil1.fe3";
    ////                path[12] = '\0';
    ////
    ////          //      res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
    //////                res = f_open(&testFile, (char*)path, FA_WRITE | FA_OPEN_ALWAYS);
    ////                res = f_open(&testFile, (char*)path, FA_READ | FA_OPEN_EXISTING);
    ////
    //////                res = f_lseek(&testFile, f_size(&testFile));
    ////                res = f_lseek(&testFile, uint8_data_number*(data_counter-1));
    //
    ////                res = f_write(&testFile, dataBuffer, uint8_data_number, &testBytes);
    ////                res = f_write(&testFile, datas, uint8_ad_adc_number*uint8_ad_chan_number*4, &testBytes);
    ////                res = f_write(&testFile, testBuffer, 16, &testBytes);
    //                res = f_read(&testFile, dataBuffer, uint8_data_number, &testBytes);
    //
    ////                res = f_close(&testFile);
    ////              }
    //
    //              print_hex(dataBuffer[1], 8);//          print2_symbol(';');
    //              print_symbol(';');
    //
    //              for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    //              {
    //                for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
    //                {
    ////                  buffer_hex(testBuffer, datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    ////                  buffer_hex(testBuffer, datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    ////                  buffer_hex(testBuffer, datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    ////                  buffer_hex(testBuffer, datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
    ////                  buffer_symbol(testBuffer, ';');
    //                  print_hex(dataBuffer[1+1+ad_adc*uint8_ad_chan_number * 3+ad_data_channel * 3 + 0], 8);//          print2_symbol(';');
    //                  print_hex(dataBuffer[1+1+ad_adc*uint8_ad_chan_number * 3+ad_data_channel * 3 + 1], 8);//          print2_symbol(';');
    //                  print_hex(dataBuffer[1+1+ad_adc*uint8_ad_chan_number * 3+ad_data_channel * 3 + 2], 8);//          print2_symbol(';');
    ////                  print_hex(dataBuffer[1+1+ad_adc*uint8_ad_chan_number * 4+ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
    //                  print_symbol(';');
    //                }
    ////                buffer_symbol(testBuffer, ';');
    //                print_symbol(';');
    //              }
    ////              buffer_symbol(testBuffer, ';');
    //
    ////              buffer_line(testBuffer);
    //              print_symbol(';');
    ////
    //              print_line();
    //
    ////              if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
    ////              {
    ////                while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    ////                {
    ////                }
    ////                if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
    ////                {
    ////                  Error_Handler();
    ////                }
    ////              }
    //
    //            }//FREESMARTEEG_SDCARD_TEXT

                if(FREESMARTEEG_OUT & FREESMARTEEG_TEXT_USB)
    //            if(0)
                {

    //              int crc_pair_ok[uint8_ad_adc_number][4];

                  if(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_PIN)
                  {

                    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                    {
                      int crc_ok = 1;
                      int crc_something_ok = 0;
                      for(int ad_data_channel_pair = 0; ad_data_channel_pair < (uint8_ad_chan_number / 2); ad_data_channel_pair ++)
                      {

                        uint8_t crc8_=crc8da_7(&(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]));

      //                  uint8_t crc1=((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
      //                      |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)));
      //                  uint8_t chan_0=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b01110000)>>4);
      //                  uint8_t chan_1=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b01110000)>>4);
      //                  print2_hex(chan_0,8);
      //                  print2_symbol(':');
      //                  print2_hex(crc8_,8);
      //                  print2_symbol('-');
      //                  print2_hex(crc1,8);
      ////                  print2_symbol(';');
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(!(((uint8_t)crc8_)
      //                      -((uint8_t)crc1))),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)!(((uint8_t)crc8_)
      //                     -((~((uint8_t)crc1))&0b00000111)),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(chan_0==ad_data_channel_pair*2),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(chan_1==(ad_data_channel_pair*2+1)),8);
      //                  print2_symbol(';');
      //                  if(
      //                      ((!(((uint8_t)crc8_)
      //                          -((uint8_t)crc1)))
      //                     ||(!(((uint8_t)crc8_)
      //                         -((~((uint8_t)crc1))&0b00000111)))
      //                      )
      //                  &&(
      //                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
      //                        ((chan_0==ad_data_channel_pair)))
      //                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
      //                      ((chan_1==(ad_data_channel_pair+1))))
      //                  ))
                        if(
                            ((!(((uint8_t)crc8_)
                                -((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                    |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)))))
                           ||(!(((uint8_t)crc8_)
                               -((~((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                   |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f))))&0b00000111)))
                            )
                        &&(
                          (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
                              ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]
                                                &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
                                                    ad_data_channel_pair*2)))
                        &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
                            ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]
                                              &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
                                                  ad_data_channel_pair*2+1)))
                        ))
                        {
      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 1;
                          crc_something_ok = 1;
                        }
                        else
                        {
      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 0;
                          crc_ok = 0;
                        }
                      }
                      if((crc_ok)||((crc_something_ok)&&(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_SOMETHING)))
                      {
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
                      }
                      else
                      {
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
                      }
                    }
                  }

                  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                  {
                    for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
                    {
                      CDC_Transmit_FS(datas[ad_adc][ad_data_channel * 4 + 0], 8);
    //                  print_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
    //                  print_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
    //                  print_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
    //                  print_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
    //                  print_symbol(';');
                    }
    //                print_symbol(';');
                  }
    //              print_symbol(';');

    //              print_line();

                }//FREESMARTEEG_TEXT_UART1
                if(FREESMARTEEG_OUT & FREESMARTEEG_STABLE_CRC)
    //            if(0)
                {

    //              int crc_pair_ok[uint8_ad_adc_number][4];

    //              if(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_PIN)
                  {

                    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                    {
                      int crc_ok = 1;
                      int crc_something_ok = 0;
                      for(int ad_data_channel_pair = 0; ad_data_channel_pair < (uint8_ad_chan_number / 2); ad_data_channel_pair ++)
                      {

                        uint8_t crc8_=crc8da_7(&(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]));

                        if(
                            ((!(((uint8_t)crc8_)
                                -((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                    |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)))))
                           ||(!(((uint8_t)crc8_)
                               -((~((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                   |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f))))&0b00000111)))
                            )
                        &&(
                          (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
                              ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]
                                                &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
                                                    ad_data_channel_pair*2)))
                        &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
                            ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]
                                              &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
                                                  ad_data_channel_pair*2+1)))
                        ))
                        {
      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 1;
                          crc_something_ok = 1;
                          stable_crc[ad_adc][ad_data_channel_pair]++;
                        }
                        else
                        {
      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 0;
                          crc_ok = 0;
                        }
                      }
                      if((crc_ok)||((crc_something_ok)&&(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_SOMETHING)))
                      {
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
                      }
                      else
                      {
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
                      }
                    }
                  }
                  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                  {
                    for(int ad_data_channel_pair = 0; ad_data_channel_pair < (uint8_ad_chan_number / 2); ad_data_channel_pair ++)
                    {
    //                  print_hex(0xff*((float)(data_counter))/((float)(stable_crc[ad_adc][ad_data_channel_pair])),8);
                      print_hex(((data_counter))-((stable_crc[ad_adc][ad_data_channel_pair])),8);
                      print_symbol(';');
                    }
                  }

                  print_line();

                }
    //            if(FREESMARTEEG_OUT & FREESMARTEEG_OPENVIBE_TELNET)
    ////            if(0)
    //            {
    //
    ////              int crc_pair_ok[uint8_ad_adc_number][4];
    //
    //              for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    //              {
    //                int crc_ok = 1;
    //                for(int ad_data_channel_pair = 0; ad_data_channel_pair < 4; ad_data_channel_pair ++)
    //                {
    //
    //                  uint8_t crc8_=crc8da_7(&(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]));
    //
    ////                  uint8_t crc1=((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    ////                      |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)));
    ////                  uint8_t chan_0=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b01110000)>>4);
    ////                  uint8_t chan_1=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b01110000)>>4);
    ////                  print2_hex(chan_0,8);
    ////                  print2_symbol(':');
    ////                  print2_hex(crc8_,8);
    ////                  print2_symbol('-');
    ////                  print2_hex(crc1,8);
    //////                  print2_symbol(';');
    ////                  print2_symbol(',');
    ////                  print2_hex((uint8_t)(!(((uint8_t)crc8_)
    ////                      -((uint8_t)crc1))),8);
    ////                  print2_symbol(',');
    ////                  print2_hex((uint8_t)!(((uint8_t)crc8_)
    ////                     -((~((uint8_t)crc1))&0b00000111)),8);
    ////                  print2_symbol(',');
    ////                  print2_hex((uint8_t)(chan_0==ad_data_channel_pair*2),8);
    ////                  print2_symbol(',');
    ////                  print2_hex((uint8_t)(chan_1==(ad_data_channel_pair*2+1)),8);
    ////                  print2_symbol(';');
    ////                  if(
    ////                      ((!(((uint8_t)crc8_)
    ////                          -((uint8_t)crc1)))
    ////                     ||(!(((uint8_t)crc8_)
    ////                         -((~((uint8_t)crc1))&0b00000111)))
    ////                      )
    ////                  &&(
    ////                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
    ////                        ((chan_0==ad_data_channel_pair)))
    ////                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
    ////                      ((chan_1==(ad_data_channel_pair+1))))
    ////                  ))
    //                  if(
    //                      ((!(((uint8_t)crc8_)
    //                          -((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    //                              |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)))))
    //                     ||(!(((uint8_t)crc8_)
    //                         -((~((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
    //                             |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f))))&0b00000111)))
    //                      )
    //                  &&(
    //                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
    //                        ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b11110000)>>4)==ad_data_channel_pair*2)))
    //                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
    //                      ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b11110000)>>4)==ad_data_channel_pair*2+1)))
    //                  ))
    //                  {
    ////                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 1;
    //                  }
    //                  else
    //                  {
    ////                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 0;
    //                    crc_ok = 0;
    //                  }
    //                }
    //                if(crc_ok)
    //                {
    //                  HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
    //                }
    //                else
    //                {
    //                  HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
    //
    ////                  //                  ad7779_do_spi_soft_reset(devices[ad_adc]);
    ////                                    HAL_GPIO_WritePin(reset_ports[ad_adc], reset_pins[ad_adc], GPIO_PIN_RESET);
    ////                                    HAL_Delay(1);
    ////                                    HAL_GPIO_WritePin(reset_ports[ad_adc], reset_pins[ad_adc], GPIO_PIN_SET);
    ////
    ////                                    ad7779_setup(&devices[ad_adc], init_params[ad_adc]);
    ////
    ////                                      {
    ////                                        aTxBuffer[0] = AD7779_REG_GENERAL_USER_CONFIG_3;
    ////                                        aTxBuffer[1] = AD7779_SPI_SLAVE_MODE_EN;
    ////
    ////                              //          HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_RESET);
    ////                                        if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
    ////                              //          if(HAL_SPI_TransmitReceive(device2->spi_dev.dev, aTxBuffer, data2, 2, 5000) != HAL_OK)
    ////                                        {
    ////                                          Error_Handler();
    ////                                        }
    ////                              //          HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_SET);
    ////
    ////                                        while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
    ////                                        {
    ////                                        }
    ////
    ////                                      }
    ////
    ////                                      {
    ////                                        aTxBuffer[0] = AD7779_REG_GEN_ERR_REG_1_EN;
    ////                                        aTxBuffer[1] = AD7779_SPI_CRC_TEST_EN;
    ////
    ////                              //          HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_RESET);
    ////                                        if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], 2) != HAL_OK)
    ////                              //          if(HAL_SPI_TransmitReceive(device2->spi_dev.dev, aTxBuffer, data2, 2, 5000) != HAL_OK)
    ////                                        {
    ////                                          Error_Handler();
    ////                                        }
    ////                              //          HAL_GPIO_WritePin(device2->spi_dev.chip_select_port, device2->spi_dev.chip_select_pin, GPIO_PIN_SET);
    ////
    ////                                        while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
    ////                                        {
    ////                                        }
    ////
    ////                                      }
    //
    //
    //                }
    //              }
    //
    //              int ad_adc = 0;
    ////              for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    //              {
    //                int ad_data_channel = 0;
    ////                for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
    //                {
    ////                  datas_out[ad_adc][ad_data_channel * 4 + 0]=datas[ad_adc][ad_data_channel * 4 + 3];
    ////                  datas_out[ad_adc][ad_data_channel * 4 + 1]=datas[ad_adc][ad_data_channel * 4 + 2];
    ////                  datas_out[ad_adc][ad_data_channel * 4 + 2]=datas[ad_adc][ad_data_channel * 4 + 1]&0b01111111;
    ////                  datas_out[ad_adc][ad_data_channel * 4 + 3]=datas[ad_adc][ad_data_channel * 4 + 1]&0b10000000;
    //
    ////                  if(datas[ad_adc][ad_data_channel * 4 + 1]&0b10000000)
    ////                  {
    ////                    datas_out[ad_adc][ad_data_channel * 4 + 0]=0b11111111;
    ////                  }
    ////                  else
    ////                  {
    ////                    datas_out[ad_adc][ad_data_channel * 4 + 0]=0b00000000;
    ////                  }
    ////                  datas_out[ad_adc][ad_data_channel * 4 + 1]=datas[ad_adc][ad_data_channel * 4 + 1]&0b01111111;
    ////                  datas_out[ad_adc][ad_data_channel * 4 + 2]=datas[ad_adc][ad_data_channel * 4 + 2];
    ////                  datas_out[ad_adc][ad_data_channel * 4 + 3]=datas[ad_adc][ad_data_channel * 4 + 3];
    //                  datas_out[ad_adc][ad_data_channel * 4 + 0]=datas[ad_adc][ad_data_channel * 4 + 1];
    //                  datas_out[ad_adc][ad_data_channel * 4 + 1]=datas[ad_adc][ad_data_channel * 4 + 2];
    //                  datas_out[ad_adc][ad_data_channel * 4 + 2]=datas[ad_adc][ad_data_channel * 4 + 3];
    //                  datas_out[ad_adc][ad_data_channel * 4 + 3]=0;
    //                }
    //              }
    //              while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
    ////              while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    //              {
    //              }
    //              if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)datas_out, 4) != HAL_OK)
    ////              if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)datas_out, 4*8*4) != HAL_OK)
    //              {
    //                Error_Handler();
    //              }
    ////              while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    ////              {
    ////              }
    //            }//FREESMARTEEG_OPENVIBE_TELNET

                if(FREESMARTEEG_OUT & FREESMARTEEG_BLINK)
    //            if(0)
                {
        //        int ad_data_channel = 0;
                  int ad_data_channel = 0;
        //          if((data[(ad_data_channel * 4)] & 0b01110000) == 0)
        /*          if(((data[((ad_data_channel + 0) * 4) + 2] & 0b01110000) >> 4 == ad_data_channel + 1 + 0)
                  && ((data[((ad_data_channel + 1) * 4) + 2] & 0b01110000) >> 4 == ad_data_channel + 1 + 1)
                  && ((data[((ad_data_channel + 2) * 4) + 2] & 0b01110000) >> 4 == ad_data_channel + 1 + 2)
                  && ((data[((ad_data_channel + 3) * 4) + 2] & 0b01110000) >> 4 == ad_data_channel + 1 + 3)
                  && ((data[((ad_data_channel + 4) * 4) + 2] & 0b01110000) >> 4 == ad_data_channel + 1 + 4)
                  )*/
                  if(((data1[((ad_data_channel + 0) * 4)] & 0b01110000) >> 4 == ad_data_channel + 0)
                  && ((data1[((ad_data_channel + 1) * 4)] & 0b01110000) >> 4 == ad_data_channel + 1)
                  && ((data1[((ad_data_channel + 2) * 4)] & 0b01110000) >> 4 == ad_data_channel + 2)
                  && ((data1[((ad_data_channel + 3) * 4)] & 0b01110000) >> 4 == ad_data_channel + 3)
                  && ((data1[((ad_data_channel + 4) * 4)] & 0b01110000) >> 4 == ad_data_channel + 4)
                  && ((data1[((ad_data_channel + 5) * 4)] & 0b01110000) >> 4 == ad_data_channel + 5)
                  && ((data1[((ad_data_channel + 6) * 4)] & 0b01110000) >> 4 == ad_data_channel + 6)
                  && ((data1[((ad_data_channel + 7) * 4)] & 0b01110000) >> 4 == ad_data_channel + 7)
                  )
                  {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
                  }
                  else
                  {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
                  }
                }

                if(FREESMARTEEG_OUT & FREESMARTEEG_OPENBCI)
    //            if(0)//openbci
                {

                  if(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_PIN)
                  {
                    crc8_01=crc8da_7(&(datas[0][0]));

                    if(
                        ((!(((uint8_t)crc8_01)-((uint8_t)((datas[0][0]<<4)|(datas[0][4]&0x0f)))))
                       ||(!(((uint8_t)crc8_01)-((~((uint8_t)((datas[0][0]<<4)|(datas[0][4]&0x0f))))&0b00000111)))
                        )
                    &&(
                      (//((datas[0][0]>=0b10000000)&&(datas[0][0]<0b10010000))||
                          ((datas[0][0]>=0b00000000)&&(datas[0][0]<0b00010000)))
                    &&(//((datas[0][4]>=0b10010000)&&(datas[0][4]<0b10100000))||
                        ((datas[0][4]>=0b00010000)&&(datas[0][4]<0b00100000)))
                    ))
                    {
//                      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
//                      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
                    }
                    else
                    {
//                      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
//                      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
                    }
                  }

    //              crc8_01=crc8da_7(&(datas[1][0]));
    //
    //              if(
    //                  ((!(((uint8_t)crc8_01)-((uint8_t)((datas[1][0]<<4)|(datas[1][4]&0x0f)))))
    //                 ||(!(((uint8_t)crc8_01)-((~((uint8_t)((datas[1][0]<<4)|(datas[1][4]&0x0f))))&0b00000111)))
    //                  )
    //              &&(
    //                (//((datas[1][0]>=0b10000000)&&(datas[1][0]<0b10010000))||
    //                    ((datas[1][0]>=0b00000000)&&(datas[1][0]<0b00010000)))
    //              &&(//((datas[1][4]>=0b10010000)&&(datas[1][4]<0b10100000))||
    //                  ((datas[1][4]>=0b00010000)&&(datas[1][4]<0b00100000)))
    //              ))
    //              {
    ////                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    //              }
    //              else
    //              {
    ////                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    //              }

    //              crc8_01=crc8da_7(&(data[0]));
    //              crc8_23=crc8da_7(&(data[8]));
    //              crc8_45=crc8da_7(&(data[16]));
    //              crc8_67=crc8da_7(&(data[24]));
    //
    //              if(((!(((uint8_t)crc8_01)-((uint8_t)((data[0]<<4)|(data[4]&0x0f)))))
    //                &&(!(((uint8_t)crc8_23)-((uint8_t)((data[8]<<4)|(data[12]&0x0f)))))
    //                &&(!(((uint8_t)crc8_45)-((uint8_t)((data[16]<<4)|(data[20]&0x0f)))))
    //                &&(!(((uint8_t)crc8_67)-((uint8_t)((data[24]<<4)|(data[28]&0x0f))))))
    //              && (
    //                (((data[0]>=0b10000000)&&(data[0]<0b10010000))||((data[0]>=0b00000000)&&(data[0]<0b00010000)))
    //              &&(((data[4]>=0b10010000)&&(data[4]<0b10100000))||((data[4]>=0b00010000)&&(data[4]<0b00100000)))
    //              &&(((data[8]>=0b10100000)&&(data[8]<0b10110000))||((data[8]>=0b00100000)&&(data[8]<0b00110000)))
    //              &&(((data[12]>=0b10110000)&&(data[12]<0b11000000))||((data[12]>=0b00110000)&&(data[12]<0b01000000)))
    //              &&(((data[16]>=0b11000000)&&(data[16]<0b11010000))||((data[16]>=0b01000000)&&(data[16]<0b01010000)))
    //              &&(((data[20]>=0b11010000)&&(data[20]<0b11100000))||((data[20]>=0b01010000)&&(data[20]<0b01100000)))
    //              &&(((data[24]>=0b11100000)&&(data[24]<0b11110000))||((data[24]>=0b01100000)&&(data[24]<0b01110000)))
    //              &&(((data[28]>=0b11110000)&&(data[28]<=0b11111111))||((data[28]>=0b01110000)&&(data[28]<=0b01111111)))
    //              ))
                  if(1)
                  {
                    const uint32_t uint8_data_number = 2 + 8 * 3 + 3 * 2 + 1;
    //                const uint32_t uint8_data_number = 2 + 8 * 3 + 8 * 3 + 3 * 2 + 1;
                    uint8_t dataBuffer[uint8_data_number];

                    dataBuffer[0] = 0xA0;
                    dataBuffer[1] = ui8SampleNumber++;

                    for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
                    {
                      dataBuffer[2 + ad_data_channel * 3 + 0] = datas[0][ad_data_channel * 4 + 1];
                      dataBuffer[2 + ad_data_channel * 3 + 1] = datas[0][ad_data_channel * 4 + 2];
                      dataBuffer[2 + ad_data_channel * 3 + 2] = datas[0][ad_data_channel * 4 + 3];
                    }
    //                for(int ad_data_channel = 0; ad_data_channel < 8; ad_data_channel ++)
    //                {
    //                  dataBuffer[2 + ad_data_channel * 3 + 0] = data2[ad_data_channel * 4 + 1];
    //                  dataBuffer[2 + ad_data_channel * 3 + 1] = data2[ad_data_channel * 4 + 2];
    //                  dataBuffer[2 + ad_data_channel * 3 + 2] = data2[ad_data_channel * 4 + 3];
    //                }
                    for(int accel_data_channel = 0; accel_data_channel < 3; accel_data_channel ++)
                    {
                      dataBuffer[2 + 8 * 3 + accel_data_channel * 2 + 0] = 0;
                      dataBuffer[2 + 8 * 3 + accel_data_channel * 2 + 1] = 0;
                    }
                    dataBuffer[2 + 8 * 3 + 3 * 2 + 0] = 0xC0;

                    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
                    {
                      while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
                      {
                      }
                      if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                      {
                        Error_Handler();
                      }
                    }
    //                if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
    //                {
    //                  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
    //                  {
    //                  }
    //                  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
    //                  {
    //                    Error_Handler();
    //                  }
    //                }


    //                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    //              }
    //              else
    //              {
    //                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
                  }


                }//FREESMARTEEG_OPENBCI

                if(FREESMARTEEG_OUT & FREESMARTEEG_OPENVIBE_FREEEEG32)
    //            if(0)//openbci
                {

                  if(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_PIN)
                  {
    //              int crc_pair_ok[uint8_ad_adc_number][4];

                    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                    {
                      int crc_ok = 1;
                      int crc_something_ok = 0;
                      for(int ad_data_channel_pair = 0; ad_data_channel_pair < (uint8_ad_chan_number / 2); ad_data_channel_pair ++)
                      {
      //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 1];
      //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 2];
      //                  datas[ad_adc][ad_data_channel_pair * 2 * 4 + 3];

                        uint8_t crc8_=crc8da_7(&(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]));

      //                  uint8_t crc1=((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
      //                      |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)));
      //                  uint8_t chan_0=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]&0b01110000)>>4);
      //                  uint8_t chan_1=((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0b01110000)>>4);
      //                  print2_hex(chan_0,8);
      //                  print2_symbol(':');
      //                  print2_hex(crc8_,8);
      //                  print2_symbol('-');
      //                  print2_hex(crc1,8);
      ////                  print2_symbol(';');
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(!(((uint8_t)crc8_)
      //                      -((uint8_t)crc1))),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)!(((uint8_t)crc8_)
      //                     -((~((uint8_t)crc1))&0b00000111)),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(chan_0==ad_data_channel_pair*2),8);
      //                  print2_symbol(',');
      //                  print2_hex((uint8_t)(chan_1==(ad_data_channel_pair*2+1)),8);
      //                  print2_symbol(';');
      //                  if(
      //                      ((!(((uint8_t)crc8_)
      //                          -((uint8_t)crc1)))
      //                     ||(!(((uint8_t)crc8_)
      //                         -((~((uint8_t)crc1))&0b00000111)))
      //                      )
      //                  &&(
      //                    (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
      //                        ((chan_0==ad_data_channel_pair)))
      //                  &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
      //                      ((chan_1==(ad_data_channel_pair+1))))
      //                  ))
                        if(
                            ((!(((uint8_t)crc8_)
                                -((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                    |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f)))))
                           ||(!(((uint8_t)crc8_)
                               -((~((uint8_t)((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]<<4)
                                   |(datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]&0x0f))))&0b00000111)))
                            )
                        &&(
                          (//((data[0]>=0b10000000)&&(data[0]<0b10010000))||
                              ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 0]
                                                &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
                                                    ad_data_channel_pair*2)))
                        &&(//((data[4]>=0b10010000)&&(data[4]<0b10100000))||
                            ((((datas[ad_adc][ad_data_channel_pair * 2 * 4 + 4]
                                              &((FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_NOERRORS)?0b11110000:0b01110000))>>4)==
                                                  ad_data_channel_pair*2+1)))
                        ))
                        {
      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 1;
                          crc_something_ok = 1;
                        }
                        else
                        {
      //                    crc_pair_ok[ad_adc][ad_data_channel_pair] = 0;
                          crc_ok = 0;
                        }
                      }
                      if((crc_ok)||((crc_something_ok)&&(FREESMARTEEG_SHOW_CRC & FREESMARTEEG_SHOW_CRC_SOMETHING)))
                      {
      //                  print2_symbol('1');
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
      //                  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
                      }
                      else
                      {
      //                  print2_symbol('0');
//                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
      //                  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

                      }
      //                print2_symbol(';');
                    }
                  }

    //              crc8_01=crc8da_7(&(datas[0][0]));
    //
    //              if(
    //                  ((!(((uint8_t)crc8_01)-((uint8_t)((datas[0][0]<<4)|(datas[0][4]&0x0f)))))
    //                 ||(!(((uint8_t)crc8_01)-((~((uint8_t)((datas[0][0]<<4)|(datas[0][4]&0x0f))))&0b00000111)))
    //                  )
    //              &&(
    //                (//((datas[0][0]>=0b10000000)&&(datas[0][0]<0b10010000))||
    //                    ((datas[0][0]>=0b00000000)&&(datas[0][0]<0b00010000)))
    //              &&(//((datas[0][4]>=0b10010000)&&(datas[0][4]<0b10100000))||
    //                  ((datas[0][4]>=0b00010000)&&(datas[0][4]<0b00100000)))
    //              ))
    //              {
    //                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    //              }
    //              else
    //              {
    //                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    //              }

    //              crc8_01=crc8da_7(&(datas[1][0]));
    //
    //              if(
    //                  ((!(((uint8_t)crc8_01)-((uint8_t)((datas[1][0]<<4)|(datas[1][4]&0x0f)))))
    //                 ||(!(((uint8_t)crc8_01)-((~((uint8_t)((datas[1][0]<<4)|(datas[1][4]&0x0f))))&0b00000111)))
    //                  )
    //              &&(
    //                (//((datas[1][0]>=0b10000000)&&(datas[1][0]<0b10010000))||
    //                    ((datas[1][0]>=0b00000000)&&(datas[1][0]<0b00010000)))
    //              &&(//((datas[1][4]>=0b10010000)&&(datas[1][4]<0b10100000))||
    //                  ((datas[1][4]>=0b00010000)&&(datas[1][4]<0b00100000)))
    //              ))
    //              {
    ////                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    //              }
    //              else
    //              {
    ////                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    //              }

    //              crc8_01=crc8da_7(&(data[0]));
    //              crc8_23=crc8da_7(&(data[8]));
    //              crc8_45=crc8da_7(&(data[16]));
    //              crc8_67=crc8da_7(&(data[24]));
    //
    //              if(((!(((uint8_t)crc8_01)-((uint8_t)((data[0]<<4)|(data[4]&0x0f)))))
    //                &&(!(((uint8_t)crc8_23)-((uint8_t)((data[8]<<4)|(data[12]&0x0f)))))
    //                &&(!(((uint8_t)crc8_45)-((uint8_t)((data[16]<<4)|(data[20]&0x0f)))))
    //                &&(!(((uint8_t)crc8_67)-((uint8_t)((data[24]<<4)|(data[28]&0x0f))))))
    //              && (
    //                (((data[0]>=0b10000000)&&(data[0]<0b10010000))||((data[0]>=0b00000000)&&(data[0]<0b00010000)))
    //              &&(((data[4]>=0b10010000)&&(data[4]<0b10100000))||((data[4]>=0b00010000)&&(data[4]<0b00100000)))
    //              &&(((data[8]>=0b10100000)&&(data[8]<0b10110000))||((data[8]>=0b00100000)&&(data[8]<0b00110000)))
    //              &&(((data[12]>=0b10110000)&&(data[12]<0b11000000))||((data[12]>=0b00110000)&&(data[12]<0b01000000)))
    //              &&(((data[16]>=0b11000000)&&(data[16]<0b11010000))||((data[16]>=0b01000000)&&(data[16]<0b01010000)))
    //              &&(((data[20]>=0b11010000)&&(data[20]<0b11100000))||((data[20]>=0b01010000)&&(data[20]<0b01100000)))
    //              &&(((data[24]>=0b11100000)&&(data[24]<0b11110000))||((data[24]>=0b01100000)&&(data[24]<0b01110000)))
    //              &&(((data[28]>=0b11110000)&&(data[28]<=0b11111111))||((data[28]>=0b01110000)&&(data[28]<=0b01111111)))
    //              ))

    //              if(average_number>1)
    //              {
    //                for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    //                {
    //                  for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
    //                  {
    //                    int32_t sample_int32_out = (datas[ad_adc][ad_data_channel * 4 + 1]<<24)
    //                            |(datas[ad_adc][ad_data_channel * 4 + 2]<<16)
    //                            |(datas[ad_adc][ad_data_channel * 4 + 3]<<8);
    //                    if ((sample_int32_out & 0x80000000) > 0) {
    //                      sample_int32_out |= 0x000000FF;
    //                    } else {
    //                      sample_int32_out &= 0xFFFFFF00;
    //                    }
    ////                    samples_int32_average[(data_counter-1)%average_number][ad_adc][ad_data_channel]=sample_int32_out;
    //                    samples_int32_average[(data_counter-1)%average_number][ad_adc][ad_data_channel]=sample_int32_out/average_number;
    //                  }
    //                }
    //                if((data_counter-1)%average_number==average_number-1)
    //                {
    //                  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
    //                  {
    //                    for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
    //                    {
    //                      int64_t sample_int64_out_sum=0;
    //                      for(int average = 0; average < average_number; average ++)
    //                      {
    //                        sample_int64_out_sum+=samples_int32_average[average][ad_adc][ad_data_channel];
    //                      }
    //                      int32_t sample_int32_out = sample_int64_out_sum / average_number;
    ////                      if(
    ////                          (datas[ad_adc][ad_data_channel * 4 + 1] == (uint8_t)((sample_int32_out>>8)&0xff))
    ////                        &&(datas[ad_adc][ad_data_channel * 4 + 2] == (uint8_t)((sample_int32_out>>16)&0xff))
    ////                        &&(datas[ad_adc][ad_data_channel * 4 + 3] == (uint8_t)((sample_int32_out>>24)&0xff))
    ////                        )
    ////                      {
    ////                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_SET);
    ////                      }
    ////                      else
    ////                      {
    ////                        HAL_GPIO_WritePin(led_ports[ad_adc], led_pins[ad_adc], GPIO_PIN_RESET);
    ////                      }
    ////                      print2_hex(datas[ad_adc][ad_data_channel * 4 + 1],8);
    ////                      print2_symbol(';');
    ////                      print2_hex((uint8_t)((sample_int32_out>>8)&0xff),8);
    ////                      print2_symbol(';');
    ////                      print2_symbol(';');
    ////                      print2_hex(datas[ad_adc][ad_data_channel * 4 + 2],8);
    ////                      print2_symbol(';');
    ////                      print2_hex((uint8_t)((sample_int32_out>>16)&0xff),8);
    ////                      print2_symbol(';');
    ////                      print2_symbol(';');
    ////                      print2_hex(datas[ad_adc][ad_data_channel * 4 + 3],8);
    ////                      print2_symbol(';');
    ////                      print2_hex((uint8_t)((sample_int32_out>>24)&0xff),8);
    ////                      print2_symbol(';');
    ////                      print2_symbol(';');
    //                      datas[ad_adc][ad_data_channel * 4 + 3] = (uint8_t)((sample_int32_out>>8)&0xff);
    //                      datas[ad_adc][ad_data_channel * 4 + 2] = (uint8_t)((sample_int32_out>>16)&0xff);
    //                      datas[ad_adc][ad_data_channel * 4 + 1] = (uint8_t)((sample_int32_out>>24)&0xff);
    //                    }
    ////                    print2_symbol(';');
    //                  }
    ////                  print2_line();
    //                }
    //              }
    //              if(((data_counter-1)%average_number==average_number-1) || (average_number<=1))
                  if(1)
                  {
                      const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 1;
//                    const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1;
                    uint8_t dataBuffer[uint8_data_number];

                    dataBuffer[0] = 0xA0;
                    dataBuffer[1] = ui8SampleNumber++;

                    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
                    {
                      for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
                      {
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 1];
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 2];
                        dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 3];
                      }
                    }
                    for(int accel_data_channel = 0; accel_data_channel < uint8_accel_chan_number; accel_data_channel ++)
                    {
                      dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 0] = 0;
                      dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + accel_data_channel * 2 + 1] = 0;
                    }
                    dataBuffer[2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 0] = 0xC0;
//                    dataBuffer[2 + uint8_ad_chan_number * 3 * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 0] = 0xC0;

                    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
                    {
                      while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
                      {
                      }
                      if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                      {
                        Error_Handler();
                      }
                    }
                    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART7)
                    {
                      while (HAL_UART_GetState(&huart7) != HAL_UART_STATE_READY)
                      {
                      }
                      if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
                      {
                        Error_Handler();
                      }
                    }
    //                if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
    //                {
    //                  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
    //                  {
    //                  }
    //                  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
    //                  {
    //                    Error_Handler();
    //                  }
    //                }


    //                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    //              }
    //              else
    //              {
    //                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
                  }


                }//FREESMARTEEG_OPENVIBE_FREEEEG32

    //            if(1)//crc processing
    //            if(FREESMARTEEG_OUT==FREESMARTEEG_PROCESSING)
    //            {
    //
    //              crc8_01=crc8da_7(&(data1[0]));
    //
    //              if(
    //                  ((!(((uint8_t)crc8_01)-((uint8_t)((data1[0]<<4)|(data1[4]&0x0f)))))
    //                 ||(!(((uint8_t)crc8_01)-((~((uint8_t)((data1[0]<<4)|(data1[4]&0x0f))))&0b00000111)))
    //                  )
    //              &&(
    //                (//((data1[0]>=0b10000000)&&(data1[0]<0b10010000))||
    //                    ((data1[0]>=0b00000000)&&(data1[0]<0b00010000)))
    //              &&(//((data1[4]>=0b10010000)&&(data1[4]<0b10100000))||
    //                  ((data1[4]>=0b00010000)&&(data1[4]<0b00100000)))
    //              ))
    //              {
    //                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    //              }
    //              else
    //              {
    //                HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    //                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    //              }
    //
    ////              crc8_01=crc8da_7(&(data[0]));
    ////              if((!(((uint8_t)crc8_01)-((uint8_t)((data[0]<<4)|(data[4]&0x0f)))))
    ////              &&(
    ////                (((data[0]>=0b10000000)&&(data[0]<0b10010000))||((data[0]>=0b00000000)&&(data[0]<0b00010000)))
    ////              &&(((data[4]>=0b10010000)&&(data[4]<0b10100000))||((data[4]>=0b00010000)&&(data[4]<0b00100000)))
    ////              ))
    ////              {
    //
    ////              crc8_01=crc8da_7(&(data[0]));
    ////              crc8_23=crc8da_7(&(data[8]));
    ////              crc8_45=crc8da_7(&(data[16]));
    ////              crc8_67=crc8da_7(&(data[24]));
    ////
    ////              if(((!(((uint8_t)crc8_01)-((uint8_t)((data[0]<<4)|(data[4]&0x0f)))))
    ////                &&(!(((uint8_t)crc8_23)-((uint8_t)((data[8]<<4)|(data[12]&0x0f)))))
    ////                &&(!(((uint8_t)crc8_45)-((uint8_t)((data[16]<<4)|(data[20]&0x0f)))))
    ////                &&(!(((uint8_t)crc8_67)-((uint8_t)((data[24]<<4)|(data[28]&0x0f))))))
    ////              && (
    ////                (((data[0]>=0b10000000)&&(data[0]<0b10010000))||((data[0]>=0b00000000)&&(data[0]<0b00010000)))
    ////              &&(((data[4]>=0b10010000)&&(data[4]<0b10100000))||((data[4]>=0b00010000)&&(data[4]<0b00100000)))
    ////              &&(((data[8]>=0b10100000)&&(data[8]<0b10110000))||((data[8]>=0b00100000)&&(data[8]<0b00110000)))
    ////              &&(((data[12]>=0b10110000)&&(data[12]<0b11000000))||((data[12]>=0b00110000)&&(data[12]<0b01000000)))
    ////              &&(((data[16]>=0b11000000)&&(data[16]<0b11010000))||((data[16]>=0b01000000)&&(data[16]<0b01010000)))
    ////              &&(((data[20]>=0b11010000)&&(data[20]<0b11100000))||((data[20]>=0b01010000)&&(data[20]<0b01100000)))
    ////              &&(((data[24]>=0b11100000)&&(data[24]<0b11110000))||((data[24]>=0b01100000)&&(data[24]<0b01110000)))
    ////              &&(((data[28]>=0b11110000)&&(data[28]<=0b11111111))||((data[28]>=0b01110000)&&(data[28]<=0b01111111)))
    ////              ))
    //              if(1)
    //              {
    //
    //                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    //
    //                if(1)
    //                {
    //                  const uint32_t uint8_data_number = 3+4;
    //                  uint8_t dataBuffer[uint8_data_number];
    //                  dataBuffer[0] = 0xff;
    //                  dataBuffer[1] = 0xfe;
    //                  dataBuffer[2] = 0xfd;
    //                  dataBuffer[3] = data1[0];
    //                  dataBuffer[4] = data1[1];
    //                  dataBuffer[5] = data1[2];
    //                  dataBuffer[6] = data1[3];
    //                //  HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
    ////                  HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
    //                  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer, uint8_data_number) != HAL_OK)
    //                  {
    //                    Error_Handler();
    //                  }
    //                  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
    //                  {
    //                  }
    //                }
    //
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    ////              }
    ////              else
    ////              {
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    ////                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    //              }
    //            }//FREESMARTEEG_PROCESSING





        //        HAL_Delay(1000/250);
    //            HAL_Delay(50);
              }
            }
        //    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_SAI2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 96;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 3;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.FrameInit.FrameLength = 24;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 1;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB1.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockB1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB1.FrameInit.FrameLength = 24;
  hsai_BlockB1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockB1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockB1.SlotInit.FirstBitOffset = 0;
  hsai_BlockB1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB1.SlotInit.SlotNumber = 1;
  hsai_BlockB1.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockB1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA2.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockA2.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockA2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA2.FrameInit.FrameLength = 24;
  hsai_BlockA2.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA2.SlotInit.FirstBitOffset = 0;
  hsai_BlockA2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA2.SlotInit.SlotNumber = 1;
  hsai_BlockA2.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA2) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB2.Instance = SAI2_Block_B;
  hsai_BlockB2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB2.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB2.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockB2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB2.FrameInit.FrameLength = 24;
  hsai_BlockB2.FrameInit.ActiveFrameLength = 1;
  hsai_BlockB2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockB2.SlotInit.FirstBitOffset = 0;
  hsai_BlockB2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB2.SlotInit.SlotNumber = 1;
  hsai_BlockB2.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockB2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
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
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  /* SPI6 parameter configuration*/
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES;
  hspi6.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 7;
  hspi6.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
//
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  huart1.Init.BaudRate = 930232;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
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
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
  hpcd_USB_OTG_HS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_HS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_HS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.phy_itface = USB_OTG_EMBEDDED_PHY;
  hpcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_HS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, ADC2_START_Pin|ADC2_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ADC3_START_Pin|ADC3_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ADC4_START_Pin|ADC4_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ADC1_START_Pin|ADC1_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ADC2_START_Pin ADC2_RESET_Pin */
  GPIO_InitStruct.Pin = ADC2_START_Pin|ADC2_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC3_START_Pin ADC3_RESET_Pin */
  GPIO_InitStruct.Pin = ADC3_START_Pin|ADC3_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI3_SD_B_Pin SAI3_SD_A_Pin SAI4_SD_A_Pin */
  GPIO_InitStruct.Pin = SAI3_SD_B_Pin|SAI3_SD_A_Pin|SAI4_SD_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC4_START_Pin ADC4_RESET_Pin */
  GPIO_InitStruct.Pin = ADC4_START_Pin|ADC4_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SDMMC1_DETECT_Pin */
  GPIO_InitStruct.Pin = SDMMC1_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDMMC1_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC1_START_Pin ADC1_RESET_Pin */
  GPIO_InitStruct.Pin = ADC1_START_Pin|ADC1_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ACC_INT1_Pin ACC_INT2_Pin */
  GPIO_InitStruct.Pin = ACC_INT1_Pin|ACC_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
//  if(GPIO_Pin==10)
//    if(FREESMARTEEG_OUT & FREESMARTEEG_DATA_TEXT_UART7_INT)
        if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_DATA_READ_INT)
  {
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	  __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();

	  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
	  {
	//                print2_symbol('0');
	    aTxBuffer[0]=0x80;
	    aTxBuffer[1]=0x00;
	//              HAL_GPIO_WritePin(device->spi_dev.chip_select_port, device->spi_dev.chip_select_pin, GPIO_PIN_RESET);
	//              if(HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
//	    print7_symbol('+');
//	    if(HAL_SPI_Receive_DMA(&hspi3, datas[ad_adc], uint8_ad_number) != HAL_OK)
//		    if(HAL_SPI_Receive_DMA(&hspi1, datas[ad_adc], uint8_ad_number) != HAL_OK)
	//                print2_symbol('1');
	//                if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, data1, uint8_ad_number) != HAL_OK)
	//                if(HAL_SPI_Receive_DMA(&hspi1, data1, uint8_ad_number) != HAL_OK)
	//              if(HAL_SPI_TransmitReceive(device->spi_dev.dev, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
	    {
	      Error_Handler();
	    }
//	    print7_symbol('-');
	  }

//	    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//	    {
//	      for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
//	      {
//	          print7_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
//	          print7_binary(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//	          print7_binary(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//	          print7_binary(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
//
//	//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
//	//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//	//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//	//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
//	        print7_symbol(';');
//	      }
//	      print7_symbol(';');
//	    }
//	    print7_symbol(';');
//
//	    print7_line();

//	  dataBuffer_print7[0]=';';
//	  int uint8_data_number=1;
//	  if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer_print7, uint8_data_number) != HAL_OK)
//	  {
//	    Error_Handler();
//	  }

		SPI_RxCplt=1;
  }

//        print7_text_line(",");

        if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_SAI_READ_INT)
  {

//	  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
	  {
	//                print2_symbol('0');
	    aTxBuffer[0]=0x80;
	    aTxBuffer[1]=0x00;
	//              HAL_GPIO_WritePin(device->spi_dev.chip_select_port, device->spi_dev.chip_select_pin, GPIO_PIN_RESET);
	//              if(HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
//	    print7_symbol('+');
//      if(GPIO_Pin==0)
	  {
  	    HAL_SAI_Receive_DMA(&hsai_BlockB1, datas[0], SAI_DATASIZE_32);
		SAI_RxCplt=1;
	  }
//      if(GPIO_Pin==1)
	  {
          HAL_SAI_Receive_DMA(&hsai_BlockA1, datas[1], SAI_DATASIZE_32);
	  }
//      if(GPIO_Pin==2)
//	  {
//          HAL_SAI_Receive_DMA(&hsai_BlockB2, datas[2], SAI_DATASIZE_32);
//	  }
//      if(GPIO_Pin==3)
//	  {
//          HAL_SAI_Receive_DMA(&hsai_BlockA2, datas[3], SAI_DATASIZE_32);
//	  }

//        HAL_SAI_Receive_DMA(&hsai_BlockB2, datas[0], SAI_DATASIZE_32);
//        HAL_SAI_Receive_DMA(&hsai_BlockA2, datas[2], SAI_DATASIZE_32);
//                            aTxBuffer[0]=0x80;
//                            aTxBuffer[1]=0x00;
//                            HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, datasBuffer[0], 8*4);
//                HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, datasBuffer[0], 8*4, 100);
//	    print7_symbol('-');
	  }


  }

        if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_I2S_READ_INT)
  {

	  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
	  {
	//                print2_symbol('0');
	    aTxBuffer[0]=0x80;
	    aTxBuffer[1]=0x00;
	//              HAL_GPIO_WritePin(device->spi_dev.chip_select_port, device->spi_dev.chip_select_pin, GPIO_PIN_RESET);
	//              if(HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
//	    print7_symbol('+');


//        HAL_I2S_Receive_DMA(&hi2s1, datas[0], uint8_ad_number);


//                            aTxBuffer[0]=0x80;
//                            aTxBuffer[1]=0x00;
//                            HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, datasBuffer[0], 8*4);
//                HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, datasBuffer[0], 8*4, 100);
//	    print7_symbol('-');
	  }


	  I2S_RxCplt=1;
  }

        if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_SPI_READ_INT)
        {
//      	  for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//      	  {
//      	//                print2_symbol('0');
//      	    aTxBuffer[0]=0x80;
//      	    aTxBuffer[1]=0x00;
//      	//              HAL_GPIO_WritePin(device->spi_dev.chip_select_port, device->spi_dev.chip_select_pin, GPIO_PIN_RESET);
//      	//              if(HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
//      //	    print7_symbol('+');
////    	    if(HAL_SPI_Receive_DMA(&hspi2, datas[ad_adc], uint8_ad_number) != HAL_OK)
////        	    if(HAL_SPI_Receive_DMA(&hspi1, datas[ad_adc], uint8_ad_number) != HAL_OK)
//                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], uint8_ad_number) != HAL_OK)
////            if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, aTxBuffer, datas[ad_adc], uint8_ad_number) != HAL_OK)
//      	//                print2_symbol('1');
//      	//                if(HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, data1, uint8_ad_number) != HAL_OK)
//      	//                if(HAL_SPI_Receive_DMA(&hspi1, data1, uint8_ad_number) != HAL_OK)
//      	//              if(HAL_SPI_TransmitReceive(device->spi_dev.dev, aTxBuffer, data, uint8_ad_number, 5000) != HAL_OK)
//      	    {
//      	      Error_Handler();
//      	    }
//      //	    print7_symbol('-');
//      	  }

      //	    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
      //	    {
      //	      for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
      //	      {
      //	          print7_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
      //	          print7_binary(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
      //	          print7_binary(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
      //	          print7_binary(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
      //
      //	//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
      //	//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
      //	//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
      //	//                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
      //	        print7_symbol(';');
      //	      }
      //	      print7_symbol(';');
      //	    }
      //	    print7_symbol(';');
      //
      //	    print7_line();

      //	  dataBuffer_print7[0]=';';
      //	  int uint8_data_number=1;
      //	  if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer_print7, uint8_data_number) != HAL_OK)
      //	  {
      //	    Error_Handler();
      //	  }

      		SPI_RxTxCplt=1;
        }

}

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//void SPI_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma)
//void SPI_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
void HAL_SPI_RxTxCpltCallback(SPI_HandleTypeDef *hspi)
//void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{

//    print7_binary(datas[0][0], 256);//          print2_symbol(';');
//    for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//    {
//      for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
//      {
//          print7_binary(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
//          print7_binary(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//          print7_binary(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//          print7_binary(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
//
////                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
////                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
////                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
////                      print7_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
//        print7_symbol(';');
//      }
//      print7_symbol(';');
//    }
//    print7_symbol(';');
//
//	    print7_line();

    //#define uint8_data_number_print7  200
    //uint8_t dataBuffer_print7[uint8_data_number_print7];
    //
//      	  dataBuffer_print7[0]=';';
//      	  int uint8_data_number=32;
//      	  if(HAL_UART_Transmit_IT(&huart7, (uint8_t*)datas, 1) != HAL_OK)
////          	  if(HAL_UART_Transmit(&huart7, (uint8_t*)datas, 1,5000) != HAL_OK)
////      	  if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)datas, uint8_data_number) != HAL_OK)
////      	  if(HAL_UART_Transmit_DMA(&huart7, (uint8_t*)dataBuffer_print7, uint8_data_number) != HAL_OK)
//      	  {
//      	    Error_Handler();
//      	  }


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

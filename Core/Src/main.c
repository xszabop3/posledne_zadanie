/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include <math.h>
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define max_test_push_button 0x3*0x1103// 10.01 msec(29.73ms)
#define max_try_to_inicialize_whoami 5
#define with_avg	0xff
#define whitout_avg	0x00

#define	WHO_AM_I_ADDRES		0x0F

#define hts221_DEVICE_ADDRESS_write	0xBE	//humidity+temp
#define hts221_DEVICE_ADDRESS_read	0xBF
#define	hts221_WHO_AM_I_VALUE 		0xBC
//#define	hts221_WHO_AM_I_ADDRES		0x0F
#define hts221_ADDRESS_HUM_L 		0x28
#define hts221_ADDRESS_TEMP_L 		0x2a
#define hts221_av_conf				0x10 //1b
#define hts221_crtl_reg_1 			0x20 //0x85

#define lps25hb_DEVICE_ADDRESS_write	0xBA	//barometer+temp(not coded)
#define lps25hb_DEVICE_ADDRESS_read		0xBB
#define	lps25hb_WHO_AM_I_VALUE 			0xBD
//#define	lps25hb_WHO_AM_I_ADDRES			0x0F
#define lps25hb_crtl_reg_1 				0x20 //0x90
#define lps25hb_ADDRESS_pres_L 			0x28

#define lis3mdl_DEVICE_ADDRESS_write	0x3c 	// magnetometer + temp(not to use)
#define lis3mdl_DEVICE_ADDRESS_read		0x3d
#define lis3mdl_WHO_AM_I_VALUE			0x3d
#define lis3mdl_ADDRESS_CTRL1			0x20
#define lis3mdl_ADDRESS_X				0x28
#define lis3mdl_ADDRESS_Y				0x2A
#define lis3mdl_ADDRESS_Z				0x2C
#define lis3mdl_ADDRESS_TEMP_L			0x2e

#define LSM6DS0_DEVICE_ADDRESS_write	0xD4 	// acelerometer + temp(not to use)
#define LSM6DS0_DEVICE_ADDRESS_read		0xD6
#define LSM6DS0_WHO_AM_I_VALUE			0x68
//#define LSM6DS0_WHO_AM_I_ADDRES		0x0F
#define LSM6DS0_ADDRESS_CTRL1			0x10
#define LSM6DS0_ADDRESS_ACCX			0x28
#define LSM6DS0_ADDRESS_ACCY			0x2A
#define LSM6DS0_ADDRESS_ACCZ			0x2C
#define LSM6DS0_ADDRESS_TEMP_L			0x20

uint8_t mode=0;
#define multiplex_display 0x1
#define shift_display 0x2
#define up_or_down	0x04
#define push_button_pushed 0x80

uint8_t what_to_measure = 5;
#define max_buff_size 64
uint8_t active_display_digit=0;
uint16_t display_buffer_pa[max_buff_size];
uint16_t display_buffer_pb[max_buff_size];
uint8_t str[max_buff_size];

uint16_t size_buff=0;
uint16_t offset=0;

float temperature=0.0;
float humidity=0.0;
float pressure=0.0;
float altitude=0.0;
float acc[3]={0.0,0.0,0.0};
float azymuth=0.0;
//float azymuth_gain=0.0;
//int16_t ret_val=0;

// hts221 calibration register values
//uint8_t H0,H1;//0x30,0x31,
uint8_t H_uint8[2];//0x30,0x31,
//int16_t H2,H3,T0,T1,T2,T3;
int16_t H_T_uint16_t[6];
//uint16_t H_T_uint16_t[6];//H2[0],H3[1],T0[2],T1[3],T2,T3
//int16_t T1,T2,T3;//[36-37:msb],0x32,0x33,[3a-3b:msb],[3c-3d:msb],[3e-3f:msb]


uint8_t *aReceiveBuffer_read , end_of_read_flag = 0;
volatile uint8_t ubReceiveIndex = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void my_str_cpy(uint8_t * from, uint8_t * to, uint16_t *copied, uint16_t max);
void convert_char_to_7seg(uint8_t ch,uint16_t *pa,uint16_t *pb);
void convert_str_to_7seg(uint8_t *from, uint16_t *pa,uint16_t *pb,uint16_t max);
void start_tim17_with_IT(void);
void start_tim16_with_IT(void);
void multiplex_display_fcn(uint16_t offset,uint16_t max_offset);
uint8_t test_push_button_state(uint16_t max_test);
void push_button_pushed_fcn(void);
void calculate_altitude(uint8_t avg);
void float_to_char(float num,uint8_t *ptr,uint8_t pos, uint8_t num_of_decimals_before_DP,uint8_t num_of_decimals_after_DP);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM16_Init(void);
static void MX_I2C1_Init(void);

void i2c_master_write(uint8_t data, uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag);
uint8_t* i2c_master_read(uint8_t* buffer, uint8_t length, uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag);

void lsm6ds0_get_acc(float* x, float* y, float* z); //accelerometer
int16_t lsm6ds0_get_temp();// picsafust
uint8_t lsm6ds0_init(void);
//uint8_t lsm6ds0_read_byte(uint8_t reg_addr);
//void lsm6ds0_write_byte(uint8_t reg_addr, uint8_t value);
//void lsm6ds0_readArray(uint8_t * data, uint8_t reg, uint8_t length);

uint8_t hts221_init(void);	// humidity+temp meter
//uint8_t hts221_read_byte(uint8_t reg_addr);
//void hts221_write_byte(uint8_t reg_addr, uint8_t value);
//void hts221_readArray(uint8_t * data, uint8_t reg, uint8_t length);
int16_t hts221_get_temp(uint8_t avg_en);
int16_t hts221_get_hum(uint8_t avg_en);

uint8_t lps25hb_init(void);	// barometer
int32_t lps25hb_get_pressure(uint8_t avg_en);
//uint8_t lps25hb_read_byte(uint8_t reg_addr);
//void lps25hb_write_byte(uint8_t reg_addr, uint8_t value);
//void lps25hb_readArray(uint8_t * data, uint8_t reg, uint8_t length);

uint8_t lis3mdl_init(void);	// magnetometer
void lis3mdl_get_azymuth(uint8_t avg_en);
float get_gain_azymuth(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t temp=0;
	//uint16_t ret_val=0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


  //my_str_cpy((uint8_t *) "PEtEr_SZabo_47447_PIrHaLa_MatEJ_92621", str, &size_buff, max_buff_size);
  //convert_str_to_7seg(str, display_buffer_pa,display_buffer_pb,size_buff);

  push_button_pushed_fcn();
  //NVIC_EnableIRQ(EXTI0_IRQn);



//  temp=lsm6ds0_init();//accelerometer
  temp=hts221_init();//humidity
  temp=lps25hb_init();//barometer
  temp=lis3mdl_init();//magnetometer
  if (temp==0)
  	  {acc[0]=0.0;}

  start_tim17_with_IT();
  start_tim16_with_IT();
  //start_i2c_set_dma();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  __WFI();
	  if (mode & multiplex_display){
		  multiplex_display_fcn(offset,size_buff-4);
		  mode &=~ multiplex_display;
	  }
	  if (mode & shift_display){
		  //get_temperature();
		  //lsm6ds0_get_acc(&acc[0],&acc[1],&acc[2]);
		  //ret_val=lsm6ds0_get_temp(); //picsafust
		  //ret_val=hts221_get_temp(0xff);	//fasza
		  //ret_val=hts221_get_hum(0xff);	//fasza

		  if (mode & up_or_down){
			  // up

			  offset++;
			  if (offset >(size_buff-4)){
				  offset=size_buff-4;
				  mode &= ~up_or_down;
			  }
		  }
		  else{

			  if (offset == 0){
				  offset=0;
				  mode |=up_or_down;
			  }
			  else{
				  offset--;
			  }
		  }
		  switch (what_to_measure){
		  case 0:{ // magnitude == azymuth
			  lis3mdl_get_azymuth(with_avg);
			  // float to char
			  float_to_char(azymuth,str,4, 3,1);
			  break;}
		  case 1:{ // temperature
			  hts221_get_temp(with_avg);
			  // float to char
			  float_to_char(temperature,str,5, 2,1);
			  break;}
		  case 2: { // humidity
			  hts221_get_hum(with_avg);
			  // float to char
			  float_to_char(humidity,str,4, 2,0);
			  break;}
		  case 3: { // pressure
			  lps25hb_get_pressure(with_avg);
			  // float to char
			  float_to_char(pressure,str,4, 4,2);
			  break;}
		  case 4: { // altitude
			  lps25hb_get_pressure(with_avg);
			  hts221_get_temp(with_avg);
			  calculate_altitude(with_avg);
			  // float to char
			  float_to_char(altitude,str,4, 4,1);
			  break;}

		  }
		  hts221_get_temp(with_avg);			//fasza
		  hts221_get_hum(with_avg);				//fasza
		  lps25hb_get_pressure(with_avg);		//fasza
		  calculate_altitude(with_avg);			//fasza
		  lis3mdl_get_azymuth(with_avg);		//kerdeses

		  //offset++;
		  //LL_GPIO_TogglePin(led_GPIO_Port,led_Pin);
		  //if (offset >(size_buff-4)){
		  	  //offset=0;}
		  mode &=~ shift_display;
	  }
	  if (mode & push_button_pushed){
		  //LL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
		  if (test_push_button_state(max_test_push_button)){
			  push_button_pushed_fcn();
		  }
		  //LL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
		  mode &= ~ push_button_pushed;
	  }
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
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

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  /** I2C Initialization
  */
  LL_I2C_DisableClockStretching(I2C1);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x2000090E;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);

  /* TIM16 interrupt Init */
  NVIC_SetPriority(TIM1_UP_TIM16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  TIM_InitStruct.Prescaler = 7999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 499;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM16, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM16);
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);

  /* TIM17 interrupt Init */
  NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 7999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM17, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM17);
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, seg_B_pa_Pin|seg_A_pa_Pin|dig_3_pa_Pin|seg_F_pa_Pin
                          |dig_1_pa_Pin|dig_time_pa_Pin|seg_C_pa_Pin|seg_E_pa_Pin
                          |dig_2_pa_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, dig_0_pb_Pin|seg_DP_pb_Pin|seg_G_pb_Pin|seg_D_pb_Pin);

  /**/
  GPIO_InitStruct.Pin = seg_B_pa_Pin|seg_A_pa_Pin|dig_3_pa_Pin|seg_F_pa_Pin
                          |dig_1_pa_Pin|dig_time_pa_Pin|seg_C_pa_Pin|seg_E_pa_Pin
                          |dig_2_pa_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = dig_0_pb_Pin|seg_DP_pb_Pin|seg_G_pb_Pin|seg_D_pb_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = VCP_RX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE3);

  /**/
  LL_GPIO_SetPinPull(push_button_irq3_GPIO_Port, push_button_irq3_Pin, LL_GPIO_PULL_DOWN);

  /**/
  LL_GPIO_SetPinMode(push_button_irq3_GPIO_Port, push_button_irq3_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_3;
  EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */
void push_button_pushed_fcn(void){

	what_to_measure++;
	if(what_to_measure >= 5){
		what_to_measure=0;
	}
	offset=0;active_display_digit=0;
	mode |= up_or_down;
	switch(what_to_measure){
	case 0:{
		//azymut [deg]: "MAG_xx.x"
		my_str_cpy((uint8_t *) "MAG_000.0\0", str, &size_buff, max_buff_size);
		convert_str_to_7seg(str, display_buffer_pa,display_buffer_pb,size_buff);
		// to do: convert num 2 str to xx.x
		lis3mdl_get_azymuth(with_avg);
		// float to char
		float_to_char(azymuth,str,4, 3,1);
		break;}
	case 1:{
		//teplota [°C]: "TEMP_xx.x"
		my_str_cpy((uint8_t *) "tEMP_00.0\0", str, &size_buff, max_buff_size);
		convert_str_to_7seg(str, display_buffer_pa,display_buffer_pb,size_buff);
		// to do: convert num 2 str to xx.x
		hts221_get_temp(with_avg);
		// float to char
		float_to_char(temperature,str,5, 2,1);
		break;}
	case 2:{
		//rel. vlhkosť [%]: "HUM_xx"
		my_str_cpy((uint8_t *) "HUM_00\0", str, &size_buff, max_buff_size);
		convert_str_to_7seg(str, display_buffer_pa,display_buffer_pb,size_buff);
		// to do: convert num 2 str to xx.x
		hts221_get_hum(with_avg);
		// float to char
		float_to_char(humidity,str,4, 2,0);
		break;}
	case 3:{
		//tlak vzduchu [hPa]: "BAR_xxxx.xx"
		my_str_cpy((uint8_t *) "bAr_0000.00\0", str, &size_buff, max_buff_size);
		convert_str_to_7seg(str, display_buffer_pa,display_buffer_pb,size_buff);
		// to do: convert num 2 str to xx.x
		lps25hb_get_pressure(with_avg);
		// float to char
		float_to_char(pressure,str,4, 4,2);
		break;}
	case 4:{
		//nadmorská výška [m]: "ALT_xxxx.x"
		my_str_cpy((uint8_t *) "ALt_0000.0\0", str, &size_buff, max_buff_size);
		convert_str_to_7seg(str, display_buffer_pa,display_buffer_pb,size_buff);
		// to do: convert num 2 str to xx.x
		lps25hb_get_pressure(with_avg);
		hts221_get_temp(with_avg);
		calculate_altitude(with_avg);
		// float to char
		float_to_char(altitude,str,4, 4,1);
		break;}
	}

}
void while_loop_converse(uint8_t * result,uint32_t *number, uint32_t *mask){
	while(*number>*mask){
		*result +=1;
		*number -= *mask;
	}
	*mask /=10;
	*result +='0';
}

void float_to_char(float num,uint8_t *ptr,uint8_t pos, uint8_t num_of_decimals_before_DP,uint8_t num_of_decimals_after_DP){
	uint8_t cnt,temp_2=1;uint32_t num_to_converse,mask=1;
	uint16_t *p_pa,*p_pb;

	//return;

	for(cnt=0;cnt<num_of_decimals_after_DP;cnt++){
		temp_2*=10;
		mask*=10;
	}
	num=round(num*((float)temp_2));
	num_to_converse=(uint32_t) num;
	for(cnt=0;cnt<num_of_decimals_before_DP-1;cnt++){
		mask*=10;
	}

	p_pa=&display_buffer_pa[pos];
	p_pb=&display_buffer_pb[pos];
	for(cnt=0;cnt<num_of_decimals_before_DP;cnt++){
		temp_2=0;
		while_loop_converse(&temp_2,&num_to_converse,&mask);
		convert_char_to_7seg(temp_2,p_pa,p_pb);
		p_pa+=1;p_pb+=1;
	}
	if (num_of_decimals_after_DP > 0)
		{p_pa-=1;p_pb-=1;
		*p_pb &= ~(seg_DP_pb_Pin );
		p_pa+=1;p_pb+=1;
		//convert_char_to_7seg('.',p_pa,p_pb);
		//size_buff--;
		}
	for (cnt=0;cnt<num_of_decimals_after_DP;cnt++){
		temp_2=0;
		while_loop_converse(&temp_2,&num_to_converse,&mask);
		convert_char_to_7seg(temp_2,p_pa,p_pb);
		p_pa+=1;p_pb+=1;
	}
// to do .... complete
}

int16_t hts221_get_temp(uint8_t avg_en){
	uint8_t data[2];int16_t raw;
	float temp;
	//hts221_readArray(data, hts221_ADDRESS_TEMP_L, 2);
	i2c_master_read(data, 2, hts221_ADDRESS_TEMP_L, hts221_DEVICE_ADDRESS_read, 1);
	raw= (int16_t)((uint16_t)data[0]+((uint16_t)data[1])*256);

	if (raw>32767)
		raw-=65536;

	temp= ((H_T_uint16_t[3] - H_T_uint16_t[2]) / 8.0) * (raw - H_T_uint16_t[4]) / (H_T_uint16_t[5] - H_T_uint16_t[4]) + (H_T_uint16_t[2] / 8.0);
	//temperature= ((T1 - T0) / 8.0) * (raw - T2) / (T3 - T2) + (T0 / 8.0);
	if(avg_en){
		temperature=(temperature+temp)/2.0;
	}else{
		temperature=temp;
	}
	return(raw);
}

int16_t hts221_get_hum(uint8_t avg_en){
	uint8_t data[2];int16_t raw;
	float temp;
	//hts221_readArray(data, hts221_ADDRESS_HUM_L, 2);
	i2c_master_read(data, 2,  hts221_ADDRESS_HUM_L, hts221_DEVICE_ADDRESS_read, 1);
	raw= (int16_t)((uint16_t)data[0]+((uint16_t)data[1])*256);

	//humidity=       ((1.0 * H1) - (1.0 * H0        )) * (1.0 * raw - 1.0 * H2             ) / (1.0 * H3 - 1.0 * H2             ) + (1.0 * H0        );
	temp=       ((1.0 * H_uint8[1]) - (1.0 * H_uint8[0]        )) * (1.0 * raw - 1.0 * H_T_uint16_t[0]             ) / (1.0 * H_T_uint16_t[1] - 1.0 * H_T_uint16_t[0]             ) + (1.0 * H_uint8[0]        );
	if (avg_en){
		humidity=(temp+humidity)/2.0;
	}else{
		humidity=temp;
	}
	return(raw);
}

int32_t lps25hb_get_pressure(uint8_t avg_en){
	uint8_t data[3];int32_t ret_val;
	//lps25hb_readArray(&data[0], lps25hb_ADDRESS_pres_L, 3);
	i2c_master_read(&data[0], 3, lps25hb_ADDRESS_pres_L, lps25hb_DEVICE_ADDRESS_read, 1);
	ret_val=(int32_t)((uint32_t)(data[0]&0xff)+((uint32_t)(data[1]&0xff))*256+((uint32_t)(data[2]&0xff))*256*256);
	//pressure=(pressure+ret_val/(4096.0*1.0))/2.0;
	if (avg_en){
		pressure=(pressure+ret_val/4096.0)/2.0;
	}
	else{
		pressure=ret_val/4096.0;
	}

	return(ret_val);
}

void lis3mdl_get_azymuth(uint8_t avg_en){
	//uint8_t data[4];int16_t x,y;float X,Y,temp;
	uint8_t data[4];int16_t x,y;float temp;
	i2c_master_read(data, 4, lis3mdl_ADDRESS_X, lis3mdl_DEVICE_ADDRESS_read, 1);

	x=(int16_t)((uint16_t)data[0]+((uint16_t)data[1])*256);
	y=(int16_t)((uint16_t)data[2]+((uint16_t)data[3])*256);

	//X=x*azymuth_gain;
	//Y=y*azymuth_gain;

	//temp=atan2(Y,X)*180.0/M_PI;
	if (x!=0){
		temp=atan(((float)y*1.0f)/((float)x*1.0f))*(180/M_PI);
		//temp=atan2(((float)y*1.0f),((float)x*1.0f))*(180/M_PI);
		if ((y>=0 ) && (x<0)) // second quadrant
			temp+=180.0;
		if ((y<0 ) && (x<0))	// third quadrant
			temp-=180.0;
	}else{
		if(y>0){
			temp=90.0;
		}else{
			temp=-90.0;
		}
	}
	if (temp>=0.0){
		//positive
		while(temp>=360.0){
			temp-=360;
			}
		}else{
		//negative
		while(temp<=0.0){
			temp+=360.0;
			}
		}

	if (avg_en){
		azymuth=(temp+azymuth)/2.0;
	}else{
		azymuth=temp;}

}

float get_gain_azymuth(void){
	uint8_t data;
	i2c_master_read(&data, 1,0x21 , lis3mdl_DEVICE_ADDRESS_read, 1);
	data &=60;

	if (data == 0x00){
		//azymuth_gain=;
		return(4.0f/32768.0f);
	}
	if (data == 0x20){
		//azymuth_gain=(8.0/32768.0);
		return(8.0f/32768.0f);
	}
	if (data == 0x40){
		//azymuth_gain=(12.0/32768.0);
		return(12.0f/32768.0f);
	}
	//if (data == 0x60){
	return(16.0f/32768.0f);
	//}
}

void lsm6ds0_get_acc(float* x, float* y, float* z)
{
	uint8_t data[6];
	int16_t xx, yy, zz;

	uint8_t temp;

	//get current scale and use it for final calculation
    //temp = lsm6ds0_read_byte(LSM6DS0_ADDRESS_CTRL1);
    i2c_master_read(&temp, 1, LSM6DS0_ADDRESS_CTRL1, LSM6DS0_DEVICE_ADDRESS_read, 0);
	temp = temp >> 2;
    temp &= 0x03;			//full scale bits exctracted

//	lsm6ds0_readArray(data, LSM6DS0_ADDRESS_ACCX, 6);
    i2c_master_read(data, 6, LSM6DS0_ADDRESS_ACCX, LSM6DS0_DEVICE_ADDRESS_read, 1);

	xx = ((uint16_t)data[1]) << 8 | data[0];
	yy = ((uint16_t)data[3]) << 8 | data[2];
	zz = ((uint16_t)data[5]) << 8 | data[4];

	*x = (xx >> 4) / 1000.0f;
	*y = (yy >> 4) / 1000.0f;
	*z = (zz >> 4) / 1000.0f;
}

int16_t lsm6ds0_get_temp()
{
	uint8_t temp[2];
//	lsm6ds0_readArray(temp, LSM6DS0_ADDRESS_TEMP_L, 2);
	i2c_master_read(temp, 2, LSM6DS0_ADDRESS_TEMP_L, LSM6DS0_DEVICE_ADDRESS_read, 1);

	return (((int16_t)((temp[1] << 8) | temp[0])) >> 3)  + 25;
}

//uint8_t lsm6ds0_read_byte(uint8_t reg_addr){
//	uint8_t data = 0;
//#define 	LSM6DS0_DEVICE_ADDRESS_write			0xD4
//#define 	LSM6DS0_DEVICE_ADDRESS_read				0xD6
//	return *(i2c_master_read(&data, 1, reg_addr, LSM6DS0_DEVICE_ADDRESS_read, 0));
//}

//uint8_t hts221_read_byte(uint8_t reg_addr){
//	uint8_t data = 0;
//#define hts221_DEVICE_ADDRESS_write	0xBE
//#define hts221_DEVICE_ADDRESS_read	0xBF
//	return *(i2c_master_read(&data, 1, reg_addr,hts221_DEVICE_ADDRESS_read , 0));
//}

//uint8_t lps25hb_read_byte(uint8_t reg_addr){
//	uint8_t data = 0;
//#define lps25hb_DEVICE_ADDRESS_write	0xBA	//barometer+temp
//#define lps25hb_DEVICE_ADDRESS_read		0xBB
//	return *(i2c_master_read(&data, 1, reg_addr,lps25hb_DEVICE_ADDRESS_read , 0));
//}

//void hts221_write_byte(uint8_t reg_addr, uint8_t value){
//#define hts221_DEVICE_ADDRESS_write	0xBE
//#define hts221_DEVICE_ADDRESS_read	0xBF
//	i2c_master_write(value, reg_addr,hts221_DEVICE_ADDRESS_write , 0);
//}

//void lps25hb_write_byte(uint8_t reg_addr, uint8_t value){
//#define lps25hb_DEVICE_ADDRESS_write	0xBA	//barometer+temp
//#define lps25hb_DEVICE_ADDRESS_read		0xBB
//	i2c_master_write(value, reg_addr,lps25hb_DEVICE_ADDRESS_write , 0);
//}

//void lsm6ds0_write_byte(uint8_t reg_addr, uint8_t value){
//#define 	LSM6DS0_DEVICE_ADDRESS_write			0xD4
//#define 	LSM6DS0_DEVICE_ADDRESS_read				0xD6
//	i2c_master_write(value, reg_addr, LSM6DS0_DEVICE_ADDRESS_write, 0);
//}

//void hts221_readArray(uint8_t * data, uint8_t reg, uint8_t length){
//#define hts221_DEVICE_ADDRESS_write	0xBE
//#define hts221_DEVICE_ADDRESS_read	0xBF
//	i2c_master_read(data, length, reg, hts221_DEVICE_ADDRESS_read, 1);
//}

//void lsm6ds0_readArray(uint8_t * data, uint8_t reg, uint8_t length){
//#define 	LSM6DS0_DEVICE_ADDRESS_write			0xD4
//#define 	LSM6DS0_DEVICE_ADDRESS_read				0xD6
//	i2c_master_read(data, length, reg, LSM6DS0_DEVICE_ADDRESS_read, 1);
//}

//void lps25hb_readArray(uint8_t * data, uint8_t reg, uint8_t length){
//#define lps25hb_DEVICE_ADDRESS_write	0xBA	//barometer+temp
//#define lps25hb_DEVICE_ADDRESS_read		0xBB
//	i2c_master_read(data, length, reg, lps25hb_DEVICE_ADDRESS_read, 1);
//}

uint8_t lsm6ds0_init(void)// accelerometer
{
	uint8_t status = 0,cnt,val;

		//LIS3MDL_ACC_ON;
	for (cnt=0;cnt<max_try_to_inicialize_whoami;cnt++){
		LL_mDelay(100);

		//uint8_t val = lsm6ds0_read_byte(WHO_AM_I_ADDRES);
		i2c_master_read(&val, 1, WHO_AM_I_ADDRES, LSM6DS0_DEVICE_ADDRESS_read, 0);
		if(val == LSM6DS0_WHO_AM_I_VALUE)
		{
			status = 1;
			break;
		}
	}


		//acc device init

		uint8_t ctrl1 = 8 << 4; // +-2g res
		//lsm6ds0_write_byte(LSM6DS0_ADDRESS_CTRL1, ctrl1);
		i2c_master_write(ctrl1, LSM6DS0_ADDRESS_CTRL1, LSM6DS0_DEVICE_ADDRESS_write, 0);

		return status;
}

uint8_t hts221_init(void){//humidity
	uint8_t status = 0,data[6],val=0,cnt=0;
	for (cnt=0;cnt<max_try_to_inicialize_whoami;cnt++){
		LL_mDelay(100);
		//val = hts221_read_byte(WHO_AM_I_ADDRES);
		i2c_master_read(&val, 1, WHO_AM_I_ADDRES,hts221_DEVICE_ADDRESS_read , 0);
		//if the device is not found on one address, try another one
		if(val == hts221_WHO_AM_I_VALUE){
			status = 1;
			break;
			//if the device is founded on one address, try to inicialize
		}
	}
//#define hts221_av_conf				0x10 //1b
//#define hts221_crtl_reg_1 			0x20 //0x85
	val=0x1b;
	//hts221_write_byte(hts221_av_conf, val);
	i2c_master_write(val, hts221_av_conf,hts221_DEVICE_ADDRESS_write , 0);
	val=0x85;
	//hts221_write_byte(hts221_crtl_reg_1, val);
	i2c_master_write(val, hts221_crtl_reg_1,hts221_DEVICE_ADDRESS_write , 0);
	LL_mDelay(100);
// hts221 calibration register values
//uint8_t H0,H1;//0x30,0x31,0x32,0x33
//int16_t H2,H3,T0,T1,T2,T3;//[36-37:msb],[3a-3b:msb],[3c-3d:msb],[3e-3f:msb]

	//hts221_readArray(&data[0], 0x30, 4);
	i2c_master_read(data, 4, 0x30, hts221_DEVICE_ADDRESS_read, 1);
	//H0=data[0]/2;	//30
	H_uint8[0]=data[0]/2;	//30
	//H1=data[1]/2;	//31
	H_uint8[1]=data[1]/2;	//31

	//T0=data[2];	//32
	H_T_uint16_t[2]=data[2];	//32
	//T1=data[3];	//33
	H_T_uint16_t[3]=data[3];	//33


	//hts221_readArray(&data[0], 0x36, 2);
	i2c_master_read(data, 2, 0x36, hts221_DEVICE_ADDRESS_read, 1);
	H_T_uint16_t[0]=(int16_t)( (uint16_t)data[0]+((uint16_t)data[1])*256 );//[36-37]
	//H2=(int16_t)( (uint16_t)data[0]+((uint16_t)data[1])*256 );//[36-37]

	//hts221_readArray(&data[0], 0x3a, 2);
	i2c_master_read(data, 2, 0x3a, hts221_DEVICE_ADDRESS_read, 1);
	H_T_uint16_t[1]=(int16_t)( (uint16_t)data[0]+((uint16_t)data[1])*256 );//[3a-3b]
	//H3=(int16_t)( (uint16_t)data[0]+((uint16_t)data[1])*256 );//[3a-3b]

	//val=hts221_read_byte(0x35);
	i2c_master_read(&val, 1, 0x35,hts221_DEVICE_ADDRESS_read , 0);
	H_T_uint16_t[2]=((val & 0x3)*256)+H_T_uint16_t[2];
	//T0=((val & 0x3)*256)+T0;
	H_T_uint16_t[3]=((val & 0xc)*64)+H_T_uint16_t[3];
	//T1=((val & 0xc)*64)+T1;


	//hts221_readArray(&data[0], 0x3c, 4);
	i2c_master_read(data, 4, 0x3c, hts221_DEVICE_ADDRESS_read, 1);
	//T2= (int16_t)( ((uint16_t)data[1])*256+(uint16_t)data[0] );//[3c-3d]
	H_T_uint16_t[4]= (int16_t)( ((uint16_t)data[1])*256+(uint16_t)data[0] );//[3c-3d]
	//T3= (int16_t)( ((uint16_t)data[3])*256+(uint16_t)data[2] );//3e-3f
	H_T_uint16_t[5]= (int16_t)( ((uint16_t)data[3])*256+(uint16_t)data[2] );//3e-3f

	hts221_get_temp(whitout_avg);	//fasza
	hts221_get_hum(whitout_avg);		//fasza

	return(status);
}

uint8_t lps25hb_init(void)//barometer
{
	uint8_t status = 0,val,cnt;//,data[3];int32_t temp;
	//LIS3MDL_ACC_ON;
	for (cnt=0;cnt<max_try_to_inicialize_whoami;cnt++){
		LL_mDelay(100);
		//val = lps25hb_read_byte(WHO_AM_I_ADDRES);
		i2c_master_read(&val, 1, WHO_AM_I_ADDRES,lps25hb_DEVICE_ADDRESS_read , 0);
		//if the device is not found on one address, try another one
		if(val == lps25hb_WHO_AM_I_VALUE){
			//if the device is founded on one address
			status = 1;
			break;}
		}
	val = 0x90;//90
	//lps25hb_write_byte(lps25hb_crtl_reg_1, val);
	i2c_master_write(val, lps25hb_crtl_reg_1,lps25hb_DEVICE_ADDRESS_write , 0);

	/*val=0x0f;
	//lps25hb_write_byte(0x10, val);
	i2c_master_write(val, 0x10,lps25hb_DEVICE_ADDRESS_write , 0);
	val=52;
	//lps25hb_write_byte(0x21, val);
	i2c_master_write(val, 0x21,lps25hb_DEVICE_ADDRESS_write , 0);
	val=0xdf;
	//lps25hb_write_byte(0x2e, val);
	i2c_master_write(val, 0x2e,lps25hb_DEVICE_ADDRESS_write , 0);
	 */

	LL_mDelay(100);

	lps25hb_get_pressure(whitout_avg);

	calculate_altitude(whitout_avg);
	return status;
}

uint8_t lis3mdl_init(void){// magnetometer
	uint8_t status = 0,val,cnt;//,data[3];int32_t temp;
	//LIS3MDL_ACC_ON;
	for (cnt=0;cnt<max_try_to_inicialize_whoami;cnt++){
		LL_mDelay(100);
		//val = lis3mdl_read_byte(WHO_AM_I_ADDRES);
		i2c_master_read(&val, 1, WHO_AM_I_ADDRES,lis3mdl_DEVICE_ADDRESS_read , 0);
		//if the device is not found on one address, try another one
		if(val == lis3mdl_WHO_AM_I_VALUE){
			//if the device is founded on one address
			status = 1;
			break;}
		}
	//val = 0x64;//90
	val=0xfc;
	//lis3mdl_write_byte(lis3mdl_crtl_reg_1, val);
	i2c_master_write(val, lis3mdl_ADDRESS_CTRL1,lis3mdl_DEVICE_ADDRESS_write , 0);
	val = 0x00;// +-4 gauss
	i2c_master_write(val, 0x21,lis3mdl_DEVICE_ADDRESS_write , 0);
	val = 0x00;// continous measure
	i2c_master_write(val, 0x22,lis3mdl_DEVICE_ADDRESS_write , 0);
	val = 0x0c;// power mode
	i2c_master_write(val, 0x23,lis3mdl_DEVICE_ADDRESS_write , 0);
	//val = 0x00;// fast reading dis
	//i2c_master_write(val, 0x24,lis3mdl_DEVICE_ADDRESS_write , 0);
	LL_mDelay(100);

	//azymuth_gain=get_gain_azymuth();

	lis3mdl_get_azymuth(whitout_avg);
	return status;
}

void i2c_master_write(uint8_t data, uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag)
{
	if(read_flag)
	{
		register_addr |= (1 << 7);
	}

	LL_I2C_HandleTransfer(I2C1, slave_addr, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	LL_I2C_TransmitData8(I2C1, register_addr);

	while(!LL_I2C_IsActiveFlag_STOP(I2C1))
	{
		if(LL_I2C_IsActiveFlag_TXIS(I2C1))
		{
			LL_I2C_TransmitData8(I2C1, data);
		}
	}
	LL_I2C_ClearFlag_STOP(I2C1);
}

uint8_t* i2c_master_read(uint8_t* buffer, uint8_t length, uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag)
{
	aReceiveBuffer_read = buffer;

	if(read_flag)
	{
		register_addr |= (1 << 7);
	}

	end_of_read_flag = 0;

	LL_I2C_EnableIT_RX(I2C1);

	//poziadam slejva o citanie z jeho registra
	LL_I2C_HandleTransfer(I2C1, slave_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	while(!LL_I2C_IsActiveFlag_STOP(I2C1))
	{
		if(LL_I2C_IsActiveFlag_TXIS(I2C1))
		{
			LL_I2C_TransmitData8(I2C1, register_addr);
		}
	}
	LL_I2C_ClearFlag_STOP(I2C1);
	while(LL_I2C_IsActiveFlag_STOP(I2C1)){}

	//citam register od slejva
	LL_I2C_HandleTransfer(I2C1, slave_addr, LL_I2C_ADDRSLAVE_7BIT, length, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

	while(!LL_I2C_IsActiveFlag_STOP(I2C1)){};

	//End of transfer
	LL_I2C_ClearFlag_STOP(I2C1);
	LL_I2C_DisableIT_RX(I2C1);
	I2C1->ICR |= (1 << 4);
	ubReceiveIndex = 0;
	end_of_read_flag = 1;

	return aReceiveBuffer_read;
}

void my_str_cpy(uint8_t * from, uint8_t * to, uint16_t *copied, uint16_t max){
	uint16_t cnt=0;
	for(cnt=0;cnt<max;cnt++){
		if(*from == '\0')
			{*copied=cnt;return;}
		else
			{*to=*from;
			to+=1;from+=1;}
	}
}

void convert_char_to_7seg(uint8_t ch,uint16_t *pa,uint16_t *pb){
	*pa=0xffff & ~( dig_1_pa_Pin | dig_2_pa_Pin | dig_3_pa_Pin | dig_time_pa_Pin);
	*pb=0xffff & ~(dig_0_pb_Pin | dig_0_pb_Pin);//& ~(LL_GPIO_PIN_5 | LL_GPIO_PIN_6);
	switch(ch){
				case '0':{
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin);
					break;}
				case '1':{
					*pa&=~(seg_B_pa_Pin | seg_C_pa_Pin);
					break;}
				case '2':{
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_E_pa_Pin );
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case '3':{
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin );
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case '4':{
					*pa &= ~( seg_B_pa_Pin | seg_C_pa_Pin | seg_F_pa_Pin);
					*pb &= ~( seg_G_pb_Pin );
					break;}
				case '5':{
					*pa &= ~(seg_A_pa_Pin | seg_C_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case '6':{
					*pa &= ~(seg_A_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case '7':{
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin );
					break;}
				case '8':{
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case '9':{
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}

				case 'A':{
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~( seg_G_pb_Pin);
					break;}
				case 'a':{
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin );
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case 'b':{
					*pa &= ~( seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case 'C':{
					*pa &= ~(seg_A_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin );
					break;}
				case 'c':{
					*pa &= ~( seg_E_pa_Pin );
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case 'd':{
					*pa &= ~( seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin );
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case 'E':{
					*pa &= ~(seg_A_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case 'F':{
					*pa &= ~(seg_A_pa_Pin |  seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~( seg_G_pb_Pin);
					break;}
				case 'G':{
					*pa &= ~(seg_A_pa_Pin |  seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin );
					break;}
				case 'H':{
					*pa &= ~(seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~( seg_G_pb_Pin);
					break;}
				case 'h':{
					*pa &= ~( seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~( seg_G_pb_Pin);
					break;}
				case 'I':{
					*pa &= ~( seg_E_pa_Pin | seg_F_pa_Pin);
					// *pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case 'J':{
					*pa &= ~( seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin );
					*pb &= ~(seg_D_pb_Pin );
					break;}
				case 'L':{
					*pa &= ~(seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin );
					break;}
				case 'n':{
					*pa &= ~( seg_C_pa_Pin | seg_E_pa_Pin );
					*pb &= ~( seg_G_pb_Pin);
					break;}
				case 'O':{
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin );
					break;}
				case 'o':{
					*pa &= ~( seg_C_pa_Pin | seg_E_pa_Pin );
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case 'P':{
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin  | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~( seg_G_pb_Pin);
					break;}
				case 'q':{
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_F_pa_Pin);
					*pb &= ~( seg_G_pb_Pin);
					break;}
				case 'r':{
					*pa &= ~( seg_E_pa_Pin );
					*pb &= ~(seg_G_pb_Pin);
					break;}
				case 'S':{
					*pa &= ~(seg_A_pa_Pin | seg_C_pa_Pin  | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case 't':{
					*pa &= ~( seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case 'U':{
					*pa &= ~(  seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin );
					break;}
				case 'u':{
					*pa &= ~( seg_C_pa_Pin | seg_E_pa_Pin );
					*pb &= ~(seg_D_pb_Pin );
					break;}
				case 'y':{
					*pa &= ~( seg_B_pa_Pin | seg_C_pa_Pin  | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}

				//specials:
				case 'K':{
					*pa &= ~(seg_A_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_G_pb_Pin);
					break;}
				case 'M':{
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin );
					break;}
				case 'V':{
					*pa &= ~( seg_B_pa_Pin |   seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin );
					break;}
				case 'W':{
					*pa &= ~( seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case 'X':{
					*pa &= ~(seg_A_pa_Pin  );
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case 'Z':{
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin    | seg_E_pa_Pin);
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);
					break;}
				case '_':{
					//*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin    | seg_E_pa_Pin);
					*pb &= ~(seg_D_pb_Pin );
					break;}
				case '-':{
					//*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin    | seg_E_pa_Pin);
					*pb &= ~(seg_G_pb_Pin );
					break;}
				case '.':{
					//pb -=1;pa-=1;//
					//*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin    | seg_E_pa_Pin);
					*pb &= ~(seg_DP_pb_Pin );
					break;}
				default:
					*pb &=~seg_DP_pb_Pin;
				// specials:
				/*case 'm':{
					*pa &= ~( seg_C_pa_Pin | seg_E_pa_Pin );
					*pb &= ~( seg_G_pb_Pin | seg_DP_pb_Pin);

					pa+=1;pb+=1;
					*pa=0xffff & ~(dig_0_pa_Pin | dig_1_pa_Pin | dig_2_pa_Pin | dig_3_pa_Pin | dig_time_pa_Pin);
					*pb=0xffff;

					*pa &= ~( seg_C_pa_Pin | seg_E_pa_Pin );
					*pb &= ~( seg_G_pb_Pin );
					break;}
				case 'x':{
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
					*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin | seg_DP_pb_Pin);

					pa+=1;pb+=1;
					*pa=0xffff & ~(dig_0_pa_Pin | dig_1_pa_Pin | dig_2_pa_Pin | dig_3_pa_Pin | dig_time_pa_Pin);
					*pb=0xffff;
					*pa &= ~(seg_A_pa_Pin | seg_B_pa_Pin | seg_C_pa_Pin | seg_E_pa_Pin | seg_F_pa_Pin);
									*pb &= ~(seg_D_pb_Pin | seg_G_pb_Pin);

				}*/


			}

}

void convert_str_to_7seg(uint8_t *from, uint16_t *pa,uint16_t *pb,uint16_t max){
	uint16_t cnt=0;//,mask_a=0,mask_b=0;
	for (cnt=0;cnt<max;cnt++){
		convert_char_to_7seg(*from,pa,pb);
		if (*from == '.')
			{pa-=1;pb-=1;size_buff--;}
		pa+=1;pb+=1;from+=1;
	}
}

void calculate_altitude(uint8_t avg){
	float temp;
	//float temperature=0.0;
	//float pressure=0.0;
	temp=((273.15+temperature)*(pow(pressure/1013.15,1.0/5.257)-1.0))/0.0065;
	if (avg){
		altitude=(temp+altitude)/2.0;
	}else{
		altitude=temp;
	}
}

void start_tim17_with_IT(void){
	  LL_TIM_EnableIT_UPDATE(TIM17);
	  TIM17->CR1 |= TIM_CR1_CEN; // start timer
}

void start_tim16_with_IT(void){
	LL_TIM_EnableIT_UPDATE(TIM16);
	TIM16->CR1 |= TIM_CR1_CEN; // start timer
}

void multiplex_display_fcn(uint16_t offset,uint16_t max_offset){
	//LL_GPIO_TogglePin(led_GPIO_Port, led_Pin);

	if (offset>max_offset){
		offset=max_offset;}
	switch (active_display_digit){
		case 0:{
			GPIOA->ODR = display_buffer_pa[offset];
			GPIOB->ODR = display_buffer_pb[offset];
			//GPIOB->ODR =
			//	create_mask(display_buffer_pb[offset]);
			//GPIOA->ODR |= dig_0_pa_Pin;
			GPIOB->ODR |= dig_0_pb_Pin;
			//prevoius=display_buffer_pb[offset];
			break;}
		case 1:{
			GPIOA->ODR = display_buffer_pa[offset+1];
			GPIOB->ODR = display_buffer_pb[offset+1];
			//GPIOB->ODR =
			//	create_mask(display_buffer_pb[offset+1]);
			GPIOA->ODR |= dig_1_pa_Pin;
			//prevoius=display_buffer_pb[offset+1];
			break;}
		case 2:{
			GPIOA->ODR = display_buffer_pa[offset+2];
			GPIOB->ODR = display_buffer_pb[offset+2];
			//GPIOB->ODR =
			//	create_mask(display_buffer_pb[offset+2]);
			GPIOA->ODR |= dig_2_pa_Pin;
			//prevoius=display_buffer_pb[offset+2];
			break;}
		case 3:{
			GPIOA->ODR = display_buffer_pa[offset+3];
			GPIOB->ODR = display_buffer_pb[offset+3];
			//GPIOB->ODR =
			//create_mask(display_buffer_pb[offset+3]);
			GPIOA->ODR |= dig_3_pa_Pin;
			//prevoius=display_buffer_pb[offset+3];
			break;}
	}

	active_display_digit++;
	if (active_display_digit>=4)
		active_display_digit=0;

}

uint8_t test_push_button_state(uint16_t max_test){
	uint16_t cnt;
	for (cnt=0; cnt<=max_test;cnt++){
		//if((push_button_irq3_GPIO_Port->IDR & push_button_irq3_Pin)){
		if((push_button_irq3_GPIO_Port->IDR & push_button_irq3_Pin)==0x0){
			return(0);
		}
	}
	return(255);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

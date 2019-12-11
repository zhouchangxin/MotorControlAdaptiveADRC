/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2015-10-04
  * 功    能: 25GA370直流电机编码测速(L298N驱动)实现
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F1Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "DCMotor/bsp_L298N.h"
#include "key/bsp_key.h"
#include "usart/bsp_debug_usart.h"
#include "DCMotor/bsp_encoder.h"
#include "math.h"

#define ROTATE_SPEED_SAMPLE		10
#define FULL_DUTY				900
#define SAMPLE_TIME				0.01
#define PI						3.1415926

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
__IO uint16_t time_count=0;        // 时间计数，每1ms增加一(与滴定时器频率有关)
__IO uint32_t CaptureNumber=0;     // 输入捕获数
__IO uint8_t  start_flag=0;

double RotateSpeed = 0;
float kp, kd, wc = 6.0,v = 2.0,wo = 30.0;//
float z1, z2, z3, b1, b2, e, b3 ,b0=357.3,u1=0,u0,u2;
int pwm = 0;

int tempNum = 0;

float t = 0.01;

float AADRCMC_r = 2.0;
float AADRCMC_b0 = 357.3, AADRCMC_q=5.0, AADRCMC_wa=100.0, AADRCMC_wc=7.0, AADRCMC_wo= 21.0;
float AADRCMC_f1 = 0.0, AADRCMC_f2 = 0.0, AADRCMC_f3 = 0.0, AADRCMC_k=0.0, AADRCMC_u=0.0, AADRCMC_er1=0.0, AADRCMC_er2=0.0;
int iSignADRC = 2;


//float CodeValL_last = 0.001;
//float pL_last = 0.02;
//float QL = 0.001, RL = 0.1;
//float kgL = 0.0;
//float CodeValL_mid = 0.0, CodeValL_Optimal = 0.0;
//float pL_mid = 0.0, pL_Optimal = 0.0;

void ADRC_MotorControl();
void AdaptiveADRC_MotorControl(float aamc_r, float aamc_b0, float aamc_q, float aamc_wc, float aamc_wo, float aamc_wa, float aamc_yp);
static void AAMC_ESO(float eso_wo, float eso_b0, float eso_rotatespeed, float eso_input, float* eso_xo1, float* eso_xo2, float* eso_xo3);
static float AAMC_RUO(float ruo_xo1, float ruo_xo2, float ruo_xo3, float ruo_r, float ruo_q, float ruo_wa);
static float AAMC_AdaptiveInstitutions(float ai_xr1, float ai_xr2, float ai_r, float ai_xo1, float ai_xo2, float ai_xo3, float ai_ruo, float ai_wa, float ai_wc, float ai_b0);
static float AAMC_AI_ConutFi(float cFi_er, float cFi_xo, float cFi_ruo, float cFi_fi0, float cFi_wa, float* cFi_fi);
static float AAMC_AI_ConutKi(float cKi_er, float cKi_r, float cKi_ruo, float cKi_ki0, float cKi_wa);
static void AAMC_ReferenceObject(float rfr, float* rfxr1, float* rfxr2, float rwc);




/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  // 9倍频，得到72MHz主时钟
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：72MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟：72MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;               // APB1时钟：36MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;               // APB2时钟：72MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);  // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{ 
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();

  KEY_GPIO_Init();
  MX_DEBUG_USART_Init();
  
  ENCODER_TIMx_Init(); 
  HAL_TIM_Base_Start(&htimx_ENCODER);  
  
  /* 高级控制定时器初始化并配置PWM输出功能 */
  L298N_TIMx_Init();
  /* 启动定时器 */
  HAL_TIM_Base_Start(&htimx_L298N);
  
  HAL_TIM_IC_Start_IT(&htimx_ENCODER,ENCODER_TIM_CHANNELx);
  
  /* 启动定时器通道和互补通道PWM输出 */
  L298N_DCMOTOR_Contrl(1,1,900);
  start_flag=1; 
  
  /* 无限循环 */
  while (1)
  {
    if(KEY1_StateRead()==KEY_DOWN)  // 增速
    {
      //PWM_Duty+=50;
      //if(PWM_Duty>900)    // PWM_Duty=900已经对应100%占空比
      //  PWM_Duty=900;
      //L298N_DCMOTOR_Contrl(1,1,PWM_Duty); 
		wc += 1;
		printf("wc1:%d  wo:%d\r\n", wc, wo);
		/*PWM_Duty -= 100;
		L298N_DCMOTOR_Contrl(1, 1, PWM_Duty);
		printf("pwm:%d\r\n", PWM_Duty);*/
    }
    if(KEY2_StateRead()==KEY_DOWN)  // 减速
    {
      //PWM_Duty-=50;
      //if(PWM_Duty<200)   // 最低速度保证，防止电机阻塞烧毁
      //  PWM_Duty=200;
      //L298N_DCMOTOR_Contrl(1,1,PWM_Duty);
		wo += 2;
		printf("wc2:%d  wo:%d\r\n",wc, wo);
	} 

	{
		
	}
  }
}

/**
  * 函数功能: 系统滴答定时器中断回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 每发生一次滴答定时器中断进入该回调函数一次
  */
void HAL_SYSTICK_Callback(void)
{
  if(start_flag) // 等待脉冲输出后才开始计时
  {
    time_count++;         // 每1ms自动增一
	if (time_count == 5)  // 1s
	{		
		if (1 == iSignADRC)
		{
			if (CaptureNumber < 165)//去除误差数据，脉冲数大于165的都不控制
			{
				//if (400 == tempNum)
				//{
				//	CaptureNumber = 250;//第400次时给个脉冲数为250的扰动
				//}
				RotateSpeed = (float)200 * CaptureNumber / 448 / 30;//转速
				if (tempNum < 1001)
				{
					printf("%0.4lf,", RotateSpeed);
					//printf("%d,", CaptureNumber);
				}
				ADRC_MotorControl();
				tempNum++;

			}
			else
			{
				ADRC_MotorControl();
				//printf("%0.4lf,", RotateSpeed);
			}
		} 
		else if(2 == iSignADRC)
		{
			if (CaptureNumber < 250)//去除误差数据，脉冲数大于165的都不控制
			{
				if (tempNum < 600)
				{
					AADRCMC_r = 2.5;
				}
				else if (tempNum < 1200)
				{
					AADRCMC_r = 2.0;//第400次时给个脉冲数为250的扰动
				}
				else if (tempNum < 1800)
				{
					AADRCMC_r = 2.0;
				}
				else if (tempNum < 2400)
				{
					AADRCMC_r = 2.0;
				}
				else if (tempNum < 3000)
				{
					AADRCMC_r = 1.5;//110
				}
				else if (tempNum < 3600)
				{
					AADRCMC_r = 2.0;//110
				}
				else if (tempNum < 4200)
				{
					AADRCMC_r = 2.5;//110
				}
				else
				{
					AADRCMC_r = 2.5;
				}
				
				//AADRCMC_r = sin(0.001 * PI * tempNum ) + 2;
				//AADRCMC_r = 2.0;
				//if (tempNum < 1001)
				//{
				//	printf("%f,%0.4lf\r\n", tempNum*0.01,RotateSpeed);
				//	//printf("%d\r\n", CaptureNumber);
				//	//printf("%d,", CaptureNumber);
				//}

				if (600 == tempNum || 1200 == tempNum || 1800 == tempNum)
				{
					CaptureNumber = 350;//第400次时给个脉冲数为250的扰动
				}

				//卡尔曼滤波
				/*CodeValL_mid = CodeValL_last;
				pL_mid = pL_last + QL;
				kgL = pL_mid / (pL_mid + RL);
				CodeValL_Optimal = CodeValL_mid + kgL * (CaptureNumber - CodeValL_mid);
				pL_Optimal = (1 - kgL) * pL_mid;
				pL_last = pL_Optimal;
				CodeValL_last = CodeValL_Optimal;*/

				RotateSpeed = (float)200 * CaptureNumber / 448 / 30;//转速
//				printf("%f,%0.4lf,%f,%f,%f,%d\r\n", tempNum * 0.01, RotateSpeed,AADRCMC_f1, AADRCMC_f2, AADRCMC_f3, CaptureNumber);
				if (tempNum < 3001)
				{
					//if (tempNum < 1200 || tempNum>2400)
					//{
						printf("%f,%0.4lf\r\n", tempNum * 0.005, RotateSpeed);
					//}

					//printf("%f,%d\r\n", tempNum * 0.005, CaptureNumber);
					//printf("%f,%0.4lf,%f,%f,%f,%f,%f,%f,%f\r\n", tempNum * 0.005, RotateSpeed, AADRCMC_f1, AADRCMC_f2, AADRCMC_f3, AADRCMC_k, AADRCMC_er1, AADRCMC_er2, AADRCMC_u);
				}
				
//				printf("%f,%0.4lf\r\n", tempNum * 0.005, RotateSpeed);
				AdaptiveADRC_MotorControl(AADRCMC_r, AADRCMC_b0, AADRCMC_q, AADRCMC_wc, AADRCMC_wo, AADRCMC_wa, RotateSpeed);
				tempNum++;			
			}
			else
			{
				//AADRCMC_r = sin(0.001 * PI * tempNum) + 2;
				//AADRCMC_r = 2.0;
				RotateSpeed = (float)200 * CaptureNumber / 448 / 30;//转速
				if (tempNum < 3001)
				{
					//if (tempNum < 1200 || tempNum>2400)
					//{
						printf("%f,%0.4lf\r\n", tempNum * 0.005, RotateSpeed);
					//}
					//printf("%f,%0.4lf,%f,%f,%f,%f,%f,%f,%f\r\n", tempNum * 0.005, RotateSpeed, AADRCMC_f1, AADRCMC_f2, AADRCMC_f3, AADRCMC_k, AADRCMC_er1, AADRCMC_er2, AADRCMC_u);
					//printf("%f,%d\r\n", tempNum * 0.005, CaptureNumber);
				}
				//AdaptiveADRC_MotorControl(AADRCMC_r, AADRCMC_b0, AADRCMC_q, AADRCMC_wc, AADRCMC_wo, AADRCMC_wa, RotateSpeed);
				tempNum++;
			}
		}
		else
		{
			RotateSpeed = (float)200 * CaptureNumber / 448 / 30;//转速
			L298N_DCMOTOR_Contrl(1,1,PWM_Duty);
			//printf("%f,%d\r\n", tempNum * 0.005, CaptureNumber);
			printf("%f,%0.41f\r\n", tempNum * 0.005, RotateSpeed);
			tempNum++;
			
		}
		
		
		CaptureNumber = 0;    // 重新开始计数
		time_count = 0;
	}
  }
}

/**
  * 函数功能: 定时器输入捕获中断回调函数
  * 输入参数: htim：定时器句柄
  * 返 回 值: 无
  * 说    明: 无
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  CaptureNumber++;
}
/**
  * 函数功能: ADRC电机控制
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: #define ROTATE_SPEED_SAMPLE		5
  *			  #define FULL_DUTY				    900
  */
void ADRC_MotorControl()
{
	float TempTime = 0.01;// ROTATE_SPEED_SAMPLE / 100;//ms换算成s


	b1 = 3 * wo;
	b2 = 3 * wo * wo;
	b3 = wo * wo * wo;
	
	kp = wc * wc;
	kd = 2 * wc;

	e = z1 - RotateSpeed;
	z1 += TempTime * (z2 - b1 * e);
	z2 += TempTime * (z3 - b2 * e+ b0 * u1);
	z3 -= TempTime * b3 * e;
	u0 = kp * (v - z1) - kd * z2;
	u2 = (u0 - z3) / b0;

	if (u2 > 1)
	{
		u2 = 0.99;
	}
	if (u2 < 0)
	{
		u2 = 0.01;
	}

	pwm = u2 * FULL_DUTY;//满占空为900
//	printf("pwm:%d  u2:%f \r\n", pwm, u2);
	L298N_DCMOTOR_Contrl(1, 1, pwm);
	u1 = u2;
}


/**
  * 函数功能: MRAC电机控制
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 
  *			  
  */
void AdaptiveADRC_MotorControl(float aamc_r, float aamc_b0, float aamc_q, float aamc_wc, float aamc_wo, float aamc_wa, float aamc_yp)
{
	float fxr1=0.0, fxr2=0.0;
	static float fu=0.0;//...
	static float fxo1, fxo2, fxo3;
	float fruo=0.0;
	int fpwm;

	AAMC_ReferenceObject(aamc_r, &fxr1, &fxr2,aamc_wc);
	AAMC_ESO(aamc_wo, aamc_b0, aamc_yp, fu, &fxo1, &fxo2, &fxo3);
	fruo = AAMC_RUO(fxo1, fxo2, fxo3, aamc_r, aamc_q, aamc_wa);
	fu = AAMC_AdaptiveInstitutions(fxr1, fxr2, aamc_r, fxo1, fxo2, fxo3, fruo, aamc_wa, aamc_wc, aamc_b0);
//	printf("fu:%f\r\n", fu);
	if (fu > 1)
	{
		fu = 0.99;
	}
	if (fu < 0)
	{
		fu = 0.01;
	}
	fpwm = fu * FULL_DUTY;//满占空为900
	AADRCMC_u = fu;
//	printf("fpwm:%d  fu:%f \r\n", fpwm, fu);
	L298N_DCMOTOR_Contrl(1, 1, fpwm);
}

/**
  * 函数功能: AdaptiveADRC ESO模块
  * 输入参数: [float]output:对象输出
  *			  [float]intput:输入
  * 输出参数：[float]xo1:输出状态0
  *			  [float]xo2:输出状态1
  *			  [float]xo3:输出状态2
  * 返 回 值: 无
  * 说    明:
  *
  */
static void AAMC_ESO(float eso_wo, float eso_b0,float eso_rotatespeed, float eso_input, float* eso_xo1, float* eso_xo2, float* eso_xo3)
{
	float TempTime = SAMPLE_TIME;// ROTATE_SPEED_SAMPLE / 100;//ms换算成s


	float b1 = 3 * eso_wo;
	float b2 = 3 * eso_wo * eso_wo;
	float b3 = eso_wo * eso_wo * eso_wo;
	float e;

	e = *eso_xo1 - eso_rotatespeed;
	*eso_xo1 += TempTime * (*eso_xo2 - b1 * e);
	*eso_xo2 += TempTime * (*eso_xo3 - b2 * e + eso_b0 * eso_input);
	*eso_xo3 -= TempTime * b3 * e;
//	printf("eso_xo1:%f,xo2:%f, xo3:%f\r\n", *eso_xo1, *eso_xo2, *eso_xo3);
}

static float AAMC_RUO(float ruo_xo1, float ruo_xo2, float ruo_xo3, float ruo_r, float ruo_q, float ruo_wa)
{
	float ruo_ruo;
	ruo_ruo = ruo_q * ruo_wa / (ruo_xo1 * ruo_xo1 + ruo_xo2 * ruo_xo2 + ruo_xo3 * ruo_xo3 + ruo_r * ruo_r);
	
//	printf("RUO_ruo:%f\r\n", ruo_ruo);

	return ruo_ruo;
}

static float AAMC_AdaptiveInstitutions(float ai_xr1, float ai_xr2, float ai_r, float ai_xo1, float ai_xo2, float ai_xo3, float ai_ruo, float ai_wa, float ai_wc, float ai_b0)
{
	float ai_u = 0.0;
	float er = 0.0, er0 = 0.0, er1 = 0.0;

	float f1_0 = 0.0, f2_0 = 0.0, f3_0 = 0.0, k1_0 = 0.0;
	float f1xo1 = 0.0, f2xo2 = 0.0, f3xo3 = 0.0, k1r = 0.0;

	er0 = (ai_xr1 - ai_xo1) * 10;
	er1 = (ai_xr2 - ai_xo2) * 1;
	er = er0 + er1;

//	AADRCMC_er1 = er0 / 10;
//	AADRCMC_er2 = er1;

	f1_0 = -(ai_wc * ai_wc);
	f2_0 = -2 * ai_wc;
	f3_0 = -1;
	k1_0 = ai_wc * ai_wc;

	f1xo1 = AAMC_AI_ConutFi(er, ai_xo1, ai_ruo, f1_0, ai_wa,&AADRCMC_f1);
	f2xo2 = AAMC_AI_ConutFi(er, ai_xo2, ai_ruo, f2_0, ai_wa,&AADRCMC_f2);
	f3xo3 = AAMC_AI_ConutFi(er, ai_xo3, ai_ruo, f3_0, ai_wa,&AADRCMC_f3);
	k1r = AAMC_AI_ConutKi(er, ai_r, ai_ruo, k1_0, ai_wa);

	ai_u = (f1xo1 + f2xo2 + f3xo3 + k1r) / ai_b0;

	//printf("AAMC_AdaptiveInstitutions u:%f\r\n", ai_u);

	return ai_u;
}

static float AAMC_AI_ConutFi(float cFi_er, float cFi_xo, float cFi_ruo, float cFi_fi0, float cFi_wa, float*cFi_fi)
{
	float ciFiXoi = 0.0;
	float exr = 0.0;
	float f0wa = 0.0;
	float fi = 0.0;
	float TempTime = SAMPLE_TIME;

	exr = cFi_er * cFi_xo * cFi_ruo;
	f0wa = cFi_fi0 * cFi_wa;
	fi += TempTime * (exr + f0wa - fi* 1);
	ciFiXoi = cFi_xo * fi;
	*cFi_fi = fi;
	//	printf("fi:%f\r\n", fi);
	return ciFiXoi;
}

static float AAMC_AI_ConutKi(float cKi_er, float cKi_r, float cKi_ruo, float cKi_ki0, float cKi_wa)
{
	float ciKiri;
	float err;
	float k0wa;
	float ki = 0.0;
	float TempTime = SAMPLE_TIME;

	err = cKi_er * cKi_r * cKi_ruo;
	k0wa = cKi_ki0 * cKi_wa;
	ki += TempTime * (err + k0wa - ki* 1);
	ciKiri = cKi_r * ki;
	AADRCMC_k = ki;
	return ciKiri;
}

static void AAMC_ReferenceObject(float rfr, float* rfxr1, float* rfxr2, float rwc)
{
	float xm[2][1] = { 0.0 };
	static float xm0[2][1] = { 0.0,0.0 };
	int Am[2][2] = { 0, 1, -rwc * rwc, -2 * rwc };
	int Bm[2][1] = { 0, rwc * rwc };
	float TempTime = SAMPLE_TIME;

	xm[0][0] = xm0[0][0] + TempTime * ((Am[0][0] * xm0[0][0]) + (Am[0][1] * xm0[1][0]) + (Bm[0][0] * rfr));
	xm[1][0] = xm0[1][0] + TempTime * ((Am[1][0] * xm0[0][0]) + (Am[1][1] * xm0[1][0]) + (Bm[1][0] * rfr));//xm[2][1]

	*rfxr1 = xm[0][0];
	*rfxr2 = xm[1][0];

	xm0[0][0] = xm[0][0];
	xm0[1][0] = xm[1][0];

//	printf("ReferenceObject xr1:%f, xr2:%f\r\n", *rfxr1, *rfxr2);
	
}


/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/

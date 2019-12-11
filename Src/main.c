/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2015-10-04
  * ��    ��: 25GA370ֱ������������(L298N����)ʵ��
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
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

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
__IO uint16_t time_count=0;        // ʱ�������ÿ1ms����һ(��ζ�ʱ��Ƶ���й�)
__IO uint32_t CaptureNumber=0;     // ���벶����
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




/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  // 9��Ƶ���õ�72MHz��ʱ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�72MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ�72MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;               // APB1ʱ�ӣ�36MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;               // APB2ʱ�ӣ�72MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);  // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{ 
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();

  KEY_GPIO_Init();
  MX_DEBUG_USART_Init();
  
  ENCODER_TIMx_Init(); 
  HAL_TIM_Base_Start(&htimx_ENCODER);  
  
  /* �߼����ƶ�ʱ����ʼ��������PWM������� */
  L298N_TIMx_Init();
  /* ������ʱ�� */
  HAL_TIM_Base_Start(&htimx_L298N);
  
  HAL_TIM_IC_Start_IT(&htimx_ENCODER,ENCODER_TIM_CHANNELx);
  
  /* ������ʱ��ͨ���ͻ���ͨ��PWM��� */
  L298N_DCMOTOR_Contrl(1,1,900);
  start_flag=1; 
  
  /* ����ѭ�� */
  while (1)
  {
    if(KEY1_StateRead()==KEY_DOWN)  // ����
    {
      //PWM_Duty+=50;
      //if(PWM_Duty>900)    // PWM_Duty=900�Ѿ���Ӧ100%ռ�ձ�
      //  PWM_Duty=900;
      //L298N_DCMOTOR_Contrl(1,1,PWM_Duty); 
		wc += 1;
		printf("wc1:%d  wo:%d\r\n", wc, wo);
		/*PWM_Duty -= 100;
		L298N_DCMOTOR_Contrl(1, 1, PWM_Duty);
		printf("pwm:%d\r\n", PWM_Duty);*/
    }
    if(KEY2_StateRead()==KEY_DOWN)  // ����
    {
      //PWM_Duty-=50;
      //if(PWM_Duty<200)   // ����ٶȱ�֤����ֹ��������ջ�
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
  * ��������: ϵͳ�δ�ʱ���жϻص�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ÿ����һ�εδ�ʱ���жϽ���ûص�����һ��
  */
void HAL_SYSTICK_Callback(void)
{
  if(start_flag) // �ȴ����������ſ�ʼ��ʱ
  {
    time_count++;         // ÿ1ms�Զ���һ
	if (time_count == 5)  // 1s
	{		
		if (1 == iSignADRC)
		{
			if (CaptureNumber < 165)//ȥ��������ݣ�����������165�Ķ�������
			{
				//if (400 == tempNum)
				//{
				//	CaptureNumber = 250;//��400��ʱ����������Ϊ250���Ŷ�
				//}
				RotateSpeed = (float)200 * CaptureNumber / 448 / 30;//ת��
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
			if (CaptureNumber < 250)//ȥ��������ݣ�����������165�Ķ�������
			{
				if (tempNum < 600)
				{
					AADRCMC_r = 2.5;
				}
				else if (tempNum < 1200)
				{
					AADRCMC_r = 2.0;//��400��ʱ����������Ϊ250���Ŷ�
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
					CaptureNumber = 350;//��400��ʱ����������Ϊ250���Ŷ�
				}

				//�������˲�
				/*CodeValL_mid = CodeValL_last;
				pL_mid = pL_last + QL;
				kgL = pL_mid / (pL_mid + RL);
				CodeValL_Optimal = CodeValL_mid + kgL * (CaptureNumber - CodeValL_mid);
				pL_Optimal = (1 - kgL) * pL_mid;
				pL_last = pL_Optimal;
				CodeValL_last = CodeValL_Optimal;*/

				RotateSpeed = (float)200 * CaptureNumber / 448 / 30;//ת��
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
				RotateSpeed = (float)200 * CaptureNumber / 448 / 30;//ת��
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
			RotateSpeed = (float)200 * CaptureNumber / 448 / 30;//ת��
			L298N_DCMOTOR_Contrl(1,1,PWM_Duty);
			//printf("%f,%d\r\n", tempNum * 0.005, CaptureNumber);
			printf("%f,%0.41f\r\n", tempNum * 0.005, RotateSpeed);
			tempNum++;
			
		}
		
		
		CaptureNumber = 0;    // ���¿�ʼ����
		time_count = 0;
	}
  }
}

/**
  * ��������: ��ʱ�����벶���жϻص�����
  * �������: htim����ʱ�����
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  CaptureNumber++;
}
/**
  * ��������: ADRC�������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: #define ROTATE_SPEED_SAMPLE		5
  *			  #define FULL_DUTY				    900
  */
void ADRC_MotorControl()
{
	float TempTime = 0.01;// ROTATE_SPEED_SAMPLE / 100;//ms�����s


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

	pwm = u2 * FULL_DUTY;//��ռ��Ϊ900
//	printf("pwm:%d  u2:%f \r\n", pwm, u2);
	L298N_DCMOTOR_Contrl(1, 1, pwm);
	u1 = u2;
}


/**
  * ��������: MRAC�������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: 
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
	fpwm = fu * FULL_DUTY;//��ռ��Ϊ900
	AADRCMC_u = fu;
//	printf("fpwm:%d  fu:%f \r\n", fpwm, fu);
	L298N_DCMOTOR_Contrl(1, 1, fpwm);
}

/**
  * ��������: AdaptiveADRC ESOģ��
  * �������: [float]output:�������
  *			  [float]intput:����
  * ���������[float]xo1:���״̬0
  *			  [float]xo2:���״̬1
  *			  [float]xo3:���״̬2
  * �� �� ֵ: ��
  * ˵    ��:
  *
  */
static void AAMC_ESO(float eso_wo, float eso_b0,float eso_rotatespeed, float eso_input, float* eso_xo1, float* eso_xo2, float* eso_xo3)
{
	float TempTime = SAMPLE_TIME;// ROTATE_SPEED_SAMPLE / 100;//ms�����s


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


/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/

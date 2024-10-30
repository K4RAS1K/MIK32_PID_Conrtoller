#include "riscv_csr_encoding.h"
#include "scr1_csr_encoding.h"
#include "mcu32_memory_map.h"
#include <power_manager.h>
#include "pad_config.h"
#include <gpio_irq.h>
#include <epic.h>
#include <csr.h>
#include <gpio.h>
#include <timer32.h>
#include <uart.h>
#include <mik32_hal_adc.h>
#include <stdio.h>
#include <string.h>
#include "pid.h"



/* Р—Р°РґР°РґРёРј СЂРµРіСѓР»СЏС‚РѕСЂ */
#define PID_KP  0.5f
#define PID_KI  0.5f
#define PID_KD  0.25f

#define PID_TAU 0.02f

#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX  10.0f

#define PID_LIM_MIN_INT -10.0f
#define PID_LIM_MAX_INT  10.0f

#define SAMPLE_TIME_S 0.01f

/* РњР°РєСЃРёРјР°Р»СЊРЅРѕРµ РІСЂРµРјСЏ СЃРёРјСѓР»СЏС†РёРё */
#define SIMULATION_TIME_MAX 4.0f



static uint16_t adc_value = 0;

#define GPIO_X ((GPIO_TypeDef*)GPIO_0_BASE_ADDRESS )

extern unsigned long __TEXT_START__;

void initUART(UART_TypeDef *uart, uint32_t uart_divider) {
	uart->DIVIDER = uart_divider; // РґРµР»РёС‚РµР»СЊ С‡Р°СЃС‚РѕС‚С‹ РїСЂРѕС†Р° = 32 000 000 / Р±РѕРґ
	uart->CONTROL1 = UART_CONTROL1_TE_M | UART_CONTROL1_RE_M| UART_CONTROL1_UE_M; //TX, RX, РІРєР»СЋС‡Р°РµРј
	while (!(uart->FLAGS & UART_FLAGS_TEACK_M)
			&& !(uart->FLAGS & UART_FLAGS_REACK_M));
}

void writeByte(UART_TypeDef *uart, uint8_t byte) {
	uart->TXDATA = byte;
	while (!(uart->FLAGS & UART_FLAGS_TC_M));
}

void writeLine(UART_TypeDef *uart, uint8_t *line) {
	for (int i = 0; line[i] != '\0'; i++) {
		writeByte(uart, line[i]);
	}
}

void trap_handler() {
	/*if ( EPIC->RAW_STATUS & (1<<EPIC_GPIO_IRQ_INDEX))
	{
		GPIO_0->OUTPUT ^= (0b1)<<(9);
		GPIO_IRQ->CLEAR = (1 << 2);
		EPIC->CLEAR = (1<<EPIC_GPIO_IRQ_INDEX);
	}*/


//	if (EPIC->STATUS & (1 << EPIC_TIMER32_0_INDEX))
//	{
//		GPIO_0->OUTPUT ^= (0b1)<<(9);
//		GPIO_0->OUTPUT ^= (0b1)<<(10);
//
//		PIDController pid = { PID_KP, PID_KI, PID_KD,
//				PID_TAU,
//				PID_LIM_MIN, PID_LIM_MAX,
//				PID_LIM_MIN_INT, PID_LIM_MAX_INT,
//				SAMPLE_TIME_S };
//
//		PIDController_Init(&pid);
//
//		float setpoint = 6.0f;
//
//
//
//		volatile float Signal = 0.0f;
//		volatile float measurement = 0.0f;
//		volatile float reg  = 0.0f;
//		measurement = 6.0f;
//		ANALOG_REG->ADC_SINGLE = 1;
//
//		uint32_t x = ANALOG_REG->ADC_VALUE;
//		measurement = (float)x;
//
//		PIDController_Update(&pid, setpoint, measurement);
//
//		reg = pid->out;
//
//
//		TIMER32_0->INT_CLEAR = TIMER32_INT_OVERFLOW_M;
//		EPIC->CLEAR = 1 << EPIC_TIMER32_0_INDEX;
//	}



}

void SystemClock_Config(void)
{
	PCC_InitTypeDef PCC_OscInit = {0};

	PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
	PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
	PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
	PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
	PCC_OscInit.AHBDivider = 0;
	PCC_OscInit.APBMDivider = 0;
	PCC_OscInit.APBPDivider = 0;
	PCC_OscInit.HSI32MCalibrationValue = 128;
	PCC_OscInit.LSI32KCalibrationValue = 128;
	PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_AUTO;
	PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
	HAL_PCC_Config(&PCC_OscInit);

}

static ADC_HandleTypeDef hADC;
//	Инициализируем АЦП
static void ADC_Init(void)
{
	hADC.Instance = ANALOG_REG;

	HAL_ADC_Init(&hADC);

	PAD_CONFIG->PORT_0_CFG = (PAD_CONFIG->PORT_0_CFG & (~(0b11<<(4*2)))) | (0b11<<(4*2));

	ANALOG_REG->ADC_CONFIG =
			(0b111111 << ADC_CONFIG_SAH_TIME_S)   // max SAH time
			|  (3<<ADC_CONFIG_SEL_S)          // channel 3
			|  (0<<ADC_CONFIG_EXTREF_S)        //
			|  (1<<ADC_CONFIG_EXTPAD_EN_S)        //
			|  (1<<ADC_CONFIG_RESETN_S)        // Reset OFF
			|  (1<<ADC_CONFIG_EN_S)          // Enabled
			;
}



void PIDController_Init(PIDController *pid) {

	/* РћС‡РёС‰Р°РµРј РІСЃРЃ */
	pid->integrator = 0.0;
	pid->prevError  = 0.0;

	pid->differentiator  = 0.0;
	pid->prevMeasurement = 0.0;

	pid->out = 0.0;

}

float PIDController_Update(PIDController *pid, float setpoint, float measurement)
{

	/*
	 * Р’С‹С‡РёСЃР»РµРЅРёРµ РѕС€РёР±РєРё
	 */
	float error = setpoint - measurement;


	/*
	 * Р’С‹С‡РёСЃР»РµРЅРёРµ РїСЂРѕРїРѕСЂС†РёРѕРЅР°Р»СЊРЅРѕР№ С‡Р°СЃС‚Рё
	 */
	float proportional = pid->Kp * error;


	/*
	 * Р’С‹С‡РёСЃР»РµРЅРёРµ РёРЅС‚РµРіСЂР°Р»С‚РЅРѕР№ С‡Р°СЃС‚Рё
	 */
//	pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* РѕРіСЂР°РЅРёС‡РµРЅРёРµ, РїСЂРµРїСЏС‚СЃС‚РІСѓСЋС‰РµРµ РїРµСЂРµСЂРµРіСѓР»СЏС†РёРё */

	if (pid->integrator > pid->limMaxInt) {

       pid->integrator = pid->limMaxInt;

   } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }


	/*
	 * Р’С‹С‡РёСЃР»РµРЅРёРµ РґРёС„С„РµСЂРµРЅС†РёР°Р»СЊРЅРѕР№ С‡Р°СЃС‚Рё
	 */

	/*pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);*/


	/*
	 * Р’С‹С‡РёСЃР»РµРЅРёРµ РІС‹С…РѕРґРЅРѕРіРѕ СЃРёРіРЅР°Р»Р° СЃ РѕРіСЂР°РЅРёС‡РµРЅРёРµРј
	 */
	pid->out = proportional;// + pid->integrator;// + pid->differentiator;

	/*if (pid->out > pid->limMax) {

		pid->out = pid->limMax;

	} else if (pid->out < pid->limMin) {

		pid->out = pid->limMin;

	}*/

	/* РЎРѕС…СЂР°РЅСЏРµРј РѕС€РёР±РєСѓ Рё РёР·РјРµСЂРµРЅРёРµ РЅР° РїРѕС‚РѕРј */
	pid->prevError       = error;
	pid->prevMeasurement = measurement;

	/* Р’С‹РІРѕРґРёРј СЂРµРіСѓР»РёСЂСѓСЋС‰РёР№ СЃРёРіРЅР°Р» */
	return pid->out;

}



/* РџСЂРѕСЃРёРјСѓР»РёСЂСѓРµРј СЂР°Р±РѕС‚Сѓ РёСЃС‚РѕС‡РЅРёРєР° РїРёС‚Р°РЅРёСЏ (first order) */
float TestSystem_Update(float inp);

float uint_to_volt(uint16_t ADC_value) {
	float result;
	result = (float)ADC_value;
	result = result / 4096 * 1.2;
	return result;
}

void main() {
	// РІРІРѕРґРёРј Р°С†Рї
	//a[6] = '\0';
	//a[5] = '\r';
	SystemClock_Config();

	ADC_Init();



	// interrupt vector init
//	write_csr(mtvec, &__TEXT_START__);

	PM->CLK_APB_P_SET =   PM_CLOCK_APB_P_GPIO_0_M
			| PM_CLOCK_APB_P_GPIO_1_M
			| PM_CLOCK_APB_P_GPIO_2_M
			| PM_CLOCK_APB_P_GPIO_IRQ_M;

	PM->CLK_APB_M_SET =   PM_CLOCK_APB_M_PAD_CONFIG_M
			| PM_CLOCK_APB_M_WU_M
			| PM_CLOCK_APB_M_PM_M;
		//	| PM_CLOCK_APB_M_TIMER32_0_M
		//	| PM_CLOCK_APB_M_EPIC_M;

	PM->CLK_AHB_SET |= PM_CLOCK_AHB_SPIFI_M;
	PM->CLK_APB_P_SET |= PM_CLOCK_APB_P_UART_1_M;

	// LEDs init
	GPIO_0->DIRECTION_OUT =  1<<(9);
	GPIO_0->DIRECTION_OUT =  1<<(10);
	//set P1.9 as UART1_TX
	//PAD_CONFIG->PORT_1_CFG |= 1<<18;
	// РІРєР»СЋС‡РµРЅРёРµ РїСЂРµСЂС‹РІР°РЅРёР№
	GPIO_IRQ->LINE_MUX = 1 << (2*4);	// 2 Р»РёРЅРёСЏ <- РЎРѕСЃС‚РѕСЏРЅРёРµ 1
	GPIO_IRQ->EDGE = 1 << 2;			// СЂРµР¶РёРј СЂР°Р±РѕС‚С‹ РїРѕ С„СЂРѕРЅС‚Сѓ
	GPIO_IRQ->LEVEL_CLEAR = 1 << 2;		// СЂР°Р±РѕС‚Р°РµРј РїРѕ СЃРїР°РґСѓ
	GPIO_IRQ->ENABLE_SET = 1 << 2;		// РІРєР»СЋС‡Р°РµРј 2СЋ Р»РёРЅРёСЋ

	/*TIMER32_0->TOP = 32000000u;

	TIMER32_0->INT_MASK = TIMER32_INT_OVERFLOW_M;

	EPIC->MASK_EDGE_CLEAR = 0xFFFF;
	EPIC->CLEAR = 0xFFFF;
	EPIC->MASK_EDGE_SET = 1 << EPIC_TIMER32_0_INDEX;

	TIMER32_0->ENABLE = TIMER32_ENABLE_TIM_EN_M;

	// interrupt reception setup
	EPIC->MASK_LEVEL_SET = 1 << EPIC_GPIO_IRQ_INDEX;*/



	// РІРєР»СЋС‡РµРЅРёРµ РїСЂРµСЂС‹РІР°РЅРёР№
	set_csr(mstatus, MSTATUS_MIE);
	set_csr(mie, MIE_MEIE);
	// РІРєР»СЋС‡РµРЅРёРµ РІС‹РІРѕРґР° РїРѕ UART
	// initUART(UART_1, 278); // 115200 baud


	/* Р’РІРµРґРµРј РџРР” СЂРµРіСѓР»СЏС‚РѕСЂ */



	/* Рљ С‡РµРјСѓ РїРѕРґРіРѕРЅСЏРµРј */
	float setpoint = 1.0f;



	volatile float Signal = 0.0f;
	volatile float measurement = 0.0f;
	volatile float reg  = 0.0f;

	PIDController pid = { PID_KP, PID_KI, PID_KD,
			PID_TAU,
			PID_LIM_MIN, PID_LIM_MAX,
			PID_LIM_MIN_INT, PID_LIM_MAX_INT,
			SAMPLE_TIME_S };
	PIDController* ppid = &pid;

	PIDController_Init(ppid);
	while (1)
	{
		ANALOG_REG->ADC_SINGLE = 1;

		uint16_t x = ANALOG_REG->ADC_VALUE;
//		measurement = (float)x;
		measurement = uint_to_volt(x);
		//measurement = 6.0f;
		PIDController_Update(&pid, setpoint, measurement);

		reg = ppid->out;
		//reg2 = pid->integrator;
//		for (volatile unsigned long i = 0; i < 4000000; i++);
//		for (volatile unsigned long i = 0; i < 4000000; i++);
	}
}


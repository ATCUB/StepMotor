#include "loop.h"
#include "adc.h"
#include "MT6816Driver.h"
#include "hw_elec.h"
#include "motor_control.h"
#include "encoder_cali.h"

struct Loop_IT_Typedef{
	//中断计数器
	uint32_t	systick_count;
	uint32_t	exti7_count;
	uint32_t	time3_count;
	uint32_t	dma1_ch4_count;
	uint32_t	dma1_ch5_count;
	uint32_t	uart1_count;
}loop_it;

uint32_t time_1ms_count = 0;
uint32_t major_cycle_count = 0;
static uint32_t time_second_1ms = 0;
static uint32_t time_second_10ms = 0;
static uint32_t time_second_20ms = 0;
static uint32_t time_second_50ms = 0;
static uint32_t time_second_100ms = 0;

bool systick_20khz_flag = false;	//20KHz系统时钟标志
uint8_t lit_1ms_divider = 0;			//副时钟分频器(1ms)

extern Encode_Cali_Typedef encode_cali;

/**
* @brief 副时钟10ms执行
*/
void time_second_10ms_serve(void)
{
	CAN_cmd_feedback();//(50Hz刷新率)
}

/**
* @brief 副时钟20ms执行
*/
void time_second_20ms_serve(void)
{
	
}

/**
* @brief 副时钟50ms执行
*/
void time_second_50ms_serve(void)
{

}

/**
* @brief 副时钟100ms执行
*/
void time_second_100ms_serve(void)
{

}


/**
* @brief 副时钟循环执行
**/
void time_second_run(void)
{
	if(time_second_10ms)		{time_second_10ms--;		time_second_10ms_serve();		}
	if(time_second_20ms)		{time_second_20ms--;		time_second_20ms_serve();		}
	if(time_second_50ms)		{time_second_50ms--;		time_second_50ms_serve();		}
	if(time_second_100ms)		{time_second_100ms--;		time_second_100ms_serve();	}
}

/**
* @brief 副时钟1ms时钟
**/
void loop_second_base_1ms(void)
{
	time_1ms_count++;
	time_second_1ms++;
	if(!(time_second_1ms % 10))		{time_second_10ms_serve();		}
	if(!(time_second_1ms % 20))		{time_second_20ms_serve();		}
	if(!(time_second_1ms % 50))		{time_second_50ms_serve();		}
	if(!(time_second_1ms % 100))	{time_second_100ms_serve();		}
	if(!(time_second_1ms % 1000))	{time_second_1ms = 0;		}
}

void LoopIT_SysTick_20KHz(void)
{
	systick_20khz_flag = true;
	HAL_SYSTICK_Config(SystemCoreClock / 20000);	//更新为20K中断
}

void loop(void)
{
	HAL_Delay(1000);

	LoopIT_SysTick_20KHz();			//重写-系统计时器修改为20KHz

	//FOR Circulation
	while(1)
	{
		major_cycle_count++;
		time_second_run();

		Calibration_Loop_Callback();							//校准器主程序回调					用于校准环节最后的数据解算
	}

}

/**
  * @brief This function handles System tick timer.
	* 启动初期由HAL库自动初始化的SysTick为1KHz
	* 接管后修改的SysTick为20KHz
**/
void SysTick_Handler(void)
{
	if(systick_20khz_flag)
	{
		//编码器数据采集
		REIN_MT6816_Get_AngleData();
		//电源数据采集
		
		//运动控制
		if(encode_cali.trigger)	Calibration_Interrupt_Callback();	//校准器中断回调
		else										Motor_Control_Callback();					//控制器任务回调
		
		//1ms分频
		lit_1ms_divider++;
		if(lit_1ms_divider >= 20){
			lit_1ms_divider  = 0;
			loop_second_base_1ms();		//副时钟1ms时钟
			HAL_IncTick();	//提供HAL库的1ms中断调用,维持HAL_Delay等超时动作的有效执行
		}
	}
	else{
		HAL_IncTick();	//提供HAL库的1ms中断调用,维持HAL_Delay等超时动作的有效执行
	}
	
	loop_it.systick_count++;
}



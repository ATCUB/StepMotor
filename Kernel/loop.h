#ifndef LOOP_H
#define LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

//loop.c
void loop_second_base_1ms(void);	//副时钟1ms时钟
void loop(void);

//loop_it.c
void LoopIT_SysTick_20KHz(void);		//系统计时器修改为20KHz

#ifdef __cplusplus
}
#endif

#endif

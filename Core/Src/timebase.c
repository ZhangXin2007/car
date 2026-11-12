#include "main.h"
#include "timebase.h"
#include "control.h"

/* htim2 定义在 main.c。main.h 没有 extern 声明，这里显式引用。 */
extern TIM_HandleTypeDef htim2;

void Timebase_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
}

void Timebase_1kHz(void)
{
    Control_Update();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
        Timebase_1kHz();
}

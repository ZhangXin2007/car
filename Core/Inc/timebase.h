#ifndef TIMEBASE_H
#define TIMEBASE_H

/* 启动 TIM2 1 kHz 时基 */
void Timebase_Init(void);

/* 1 kHz 控制入口；由 TIM2 更新中断调用，实现在 control.c */
void Timebase_1kHz(void);

#endif /* TIMEBASE_H */

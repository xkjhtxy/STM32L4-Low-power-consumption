# STM32L4-Low-power-consumption
进入低功耗部分代码为：

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);

 可用wakeup键或者RTC事件唤醒，退出低功耗时要全部初始化一编STM32
 硬件设计注意LDO的选型，我用的是HT7533

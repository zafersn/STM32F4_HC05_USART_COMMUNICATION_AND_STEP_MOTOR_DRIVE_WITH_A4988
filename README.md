STM32F407 üzerinde hc05 bluetooth modülü ile usart haberleşmesi üzerinden android telefon ile haberleştirildi. Burada paylaşılan kodlarda sadece android üzerinden step moturun yön kontrolü yapılmaktadır.
Bu kaynak oluşturulurken STM32 CubeMX ile HAL kütüphaneleri oluşturuldu ve kullanıldı.
CubeMX üzerinde timer ayarları yapıldı. Step motor için timer hesaplanırken aşağıdaki formuller kullanıldı.
<br><br>
[
<br>
**Example TIM2:**

Goal: Set TIM2 to 1khz(1ms) interrupt

TIM2 is connected to APB1 so it’s max clock is SysClk/2 (42mhz)

First calculate timer tick frequency :
Down timer 1mhz with prescaler
 <br>
**timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)**
** 1000000 = 42000000 / (prescaller_set + 1)**
** prescaller = 41** <br>
Second calculate timer period:
Down timer 1khz with timer period

**tim_frequency = timer_tick_frequency / (TIM_Period + 1)**
 **2000 = 1000000 /(TIM_Period +1)**
 **TIM_Period = 1000000 / 1000 - 1**
 **TIM_Period = 999**
 <br>
Result: If you set prescaller to 83 and period to 99, you will get 1khz (1ms) interrupt.

**KAYNAK: http://cerdemir.com/timers-in-stm32f4-discovery/**

]

Kısaca burada 1 -ms periodda pulse ürettik ve step motoru 1 ms aralıkla toggle yaparak sürdük.

Bunun dışında MS1,ms2 ve ms3 low yaparak full-step modda sürdük.



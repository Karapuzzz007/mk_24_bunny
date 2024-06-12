#include "ring_buf/Ring_buf.hpp"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
uint8_t c{'a'};
Ring_buffer buf; 

void setup (){
    // Интерфейс U(S)ART с внешним миром


rcc_periph_clock_enable(RCC_GPIOA);                           // Разморозка порта ввода/вывода
rcc_periph_clock_enable(RCC_GPIOE);

gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);  // Режим альтернативной функции
gpio_set_af(GPIOA,GPIO_AF7, GPIO2 | GPIO3);                           // Альтернативная функция (выбор по номеру) PA9 --- Tx, PA10 --- Rx.

gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9|GPIO11);

rcc_periph_clock_enable(RCC_USART2);                      // Разморозка ПУ

usart_set_baudrate(USART2, 19200);                       // Скорость передачи
usart_set_databits(USART2, 8);                            // Размер посылки
usart_set_stopbits(USART2, USART_STOPBITS_1);             // Количество стоп-битов
usart_set_parity(USART2, USART_PARITY_NONE);              // Контроль четности

usart_set_mode(USART2, USART_MODE_TX_RX);                 // Режим работы ПУ
usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);   // Управление процессом передачи сообщений

usart_enable_rx_interrupt(USART2);
nvic_enable_irq(NVIC_USART2_EXTI26_IRQ);

usart_enable(USART2);                                     // Включение ПУ



}

void loop(){

if (!buf.empty()){c = buf.get();}
usart_send_blocking (USART2 , c);
for (volatile uint32_t i=0;i<20000;i++);
gpio_toggle(GPIOE, GPIO9);

}

int main (){

    setup();

    gpio_set(GPIOE, GPIO15);

   while(true){

    loop();

   } 
}

 void usart2_exti26_isr(void){

USART_RQR(USART2) &= ~(USART_RQR_RXFRQ);

//static uint8_t c = static_cast<uint8_t>(usart_recv(USART2));

if (buf.not_full()){
        buf.put(static_cast<uint8_t>(usart_recv(USART2)));
}

gpio_toggle (GPIOE, GPIO11);
    //Очистить флаг запроса прерывания
    //Сохранить принятый символ в переменную
    //Переключить светодиод 
 }
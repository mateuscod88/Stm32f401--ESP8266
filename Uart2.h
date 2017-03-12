//GPIOA 0x4002 0000
//RCC 0x4002 3800
//UART2 0x4000 4400
#define RCC_AHB1ENR (*((unsigned int *)0x40023830))
#define GPIOA_RCC_CLOCK_ON 0x00000001
#define RCC_APB1 (*((unsigned int *)0x40023840))
#define UART2_RCC_CLOCK_ON 1U<<17
#define GPIOA_MODER (*((unsigned int *)0x40020000))
#define PIN2_AF 2U<<4
#define PIN3_AF 0U<<6
#define GPIOA_OTYPER (*((unsigned int *)0x40020004))
#define PIN2_TYPER_PP 0x00000000U	
#define PIN3_TYPER_PP 0x00000000U	
#define GPIOA_OSPEEDR (*((unsigned int *)0x40020008))
#define GPIO_PIN2_FASTSPEED 2U<<4
#define GPIO_PIN3_FASTSPEED 0U<<6
#define GPIOA_PUPDR (*((unsigned int *)0x4002000C))
#define GPIO2_PU 1U<<4
#define GPIO3_NOPULL 0U<<6
#define GPIOA_AFRL (*((unsigned int *)0x40020020))
#define GPIO_AF_USART2_TX 7U<<8
#define GPIO_AF_USART2_RX 0U<<12





#define UART2_SR (*((volatile  const unsigned int *)0x40004400))
#define UART2_DR (*((volatile unsigned int *)0x40004404))
#define UART2_BRR (*((volatile unsigned int *)0x40004408))
#define UART2_C1 (*((volatile unsigned int *)0x4000440C))
#define UART2_C2 (*((volatile unsigned int *)0x40004410))
#define UART2_C3 (*((volatile unsigned int *)0x40004400))

#define UART_SR_TXE 1U<<7
#define UART_SR_TC 1U<<6
#define UART_SR_RXNE 1U<<5
#define UART_BRR_DIV_MANTIASA  0x00000088U<<4
#define UART_BRR_DIV_FRACTION  0x00000000BU<<0
#define UART_C1_OVER16 0U<<15
#define UART_C1_UE 1U<<13
#define UART_C1_M8 0U<<12
#define UART_C1_TXEIE 1U<<7
#define UART_C1_TCIE 1U<<6

#define UART_C1_RXNIE 1U<<5
#define UART_C1_TE 1U<<3
#define UART_C1_RE 1U<<2
#define UART_C2_ONESTOP (0U<<12 | 0U<<13)




void UART2_IRQ_INIT()
{
	UART2_C1 |= UART_C1_TXEIE;
	UART2_C1 |= UART_C1_TCIE;

}
void UART2_SEND_DATA(uint8_t * tmp, int size)
{
	
	
	unsigned int count = 0;
	//while((UART2_SR & UART_SR_TXE)== 0U);
	//UART2_C1 |= UART_C1_TE;
	 while(size > 0)
	 {
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
		while(!(UART2_SR & UART_SR_TXE));
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
		 UART2_DR &= (*tmp & (uint8_t)0xFFU);
		  
		 tmp++;
		 size--;
	 }
 	
	//while((UART2_SR & UART_SR_TC)== 0);
	//UART2_SR &= ~(UART_SR_TC);
}
void UART2_INIT()
{
	RCC_APB1 |= UART2_RCC_CLOCK_ON;
	UART2_C1 |= UART_C1_UE;
	UART2_C1 |= UART_C1_M8;
	UART2_C2 |= UART_C2_ONESTOP;
	UART2_BRR |= UART_BRR_DIV_MANTIASA | UART_BRR_DIV_FRACTION;
	//UART2_C1 |= UART_C1_TE;
	//UART2_SR &= ~(UART_SR_TC);
	//UART2_IRQ_INIT();
}
void GPIOA_INIT_USART2()
{
	RCC_AHB1ENR |= GPIOA_RCC_CLOCK_ON;
	GPIOA_MODER |= PIN2_AF | PIN3_AF;
	GPIOA_OTYPER |= PIN2_TYPER_PP | PIN3_TYPER_PP;
	GPIOA_OSPEEDR |= GPIO_PIN2_FASTSPEED | GPIO_PIN3_FASTSPEED;
	GPIOA_PUPDR |= GPIO2_PU | GPIO3_NOPULL;
	GPIOA_AFRL |= GPIO_AF_USART2_RX | GPIO_AF_USART2_TX;
	
	
}
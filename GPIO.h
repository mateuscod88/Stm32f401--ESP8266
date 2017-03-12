#define _GPIO_H
#ifdef _GPIO_H
void SetBaudRate();
void SetUart1();
void SetGPIO_A_AF();
void  RCC_APB2_UART1_ON();
void UART1_Write(char  message);
										
#define GPIO_A_MODER (*((unsigned int *)0x40020000U))
#define GPIO_A_OTYPER (*((unsigned int *)0x40020004))
#define GPIO_A_SPEEDER (*((unsigned int *)0x40020008))
#define GPIO_A_PUPDR (*((unsigned int *)0x4002000C))
#define GPIO_A_IDR (*((unsigned int *)0x40020010))
#define GPIO_A_ODR (*((unsigned int *)0x40020014))
#define GPIO_A_BSSR (*((unsigned int *)0x40020018))
#define GPIO_A_LCKR (*((unsigned int *)0x4002001C))
#define GPIO_A_AFRL (*((unsigned int *)0x40020020))
#define GPIO_A_AFRH (*((unsigned int *)0x40020024))

#define GPIO_A_9_AF_USART1  0x00080000U
#define GPIO_A_10_AF_USART1  0x00200000U
#define GPIO_A_10_INPUT_USART1  0x00000000U

#define GPIO_A9_PP_TYPE 0x00000000U
#define GPIO_A10_PP_TYPE 0x00000000U

#define GPIO_A9_SPEED_FAST 0x000C0000U
#define GPIO_A10_SPEED_FAST 0x00300000U


#define GPIO_A9_AF7 0x00000070U
#define GPIO_A10_AF7 0x00000700U

#define GPIO_A9_OD 0x00200000U
#define GPIO_A9_NONPUPD 0x00000000U
#define GPIO_A9_PU 0x00040000U
#define GPIO_A9_PD 0x00080000U
#define GPIO_A10_PU 0x00000100U

#define AHB1_ENABLE 0x00000001U

#define GPIO_B_MODER (*((unsigned int *)0x40020400U))
#define GPIO_B_OTYPER (*((unsigned int *)0x40020404))

#define GPIO_C_MODER (*((unsigned int *)0x40020800))

#define GPIO_D_MODER (*((unsigned int *)0x40020C00))

#define GPIO_E_MODER (*((unsigned int *)0x40021000))

#define GPIO_H_MODER (*((unsigned int *)0x40021C00))




 
#define AHB1

#define RCC_AHB1ENR (*((unsigned int *)0x40023830))
#define RCC_APB2 (*((unsigned int *)0x40023844))
#define RCC_APB2_UART1_ENABLE 0x00000010U


#define UART1
#define USART2_DR (*((unsigned int *)0x40004404))
#define USART1_SR (*((unsigned int *)0x40011000))
#define USART1_DR (*((unsigned int *)0x40011004))
#define USART1_BRR (*((unsigned int *)0x40011008))
#define USART1_C1 (*((unsigned int *)0x4001100C))
#define USART1_C2 (*((unsigned int *)0x40011010))
#define USART1_C3 (*((unsigned int *)0x40011014))
#define USART1_GTPR (*((unsigned int *)0x40011008))

#define USART1_TXE 0x00000080U
#define USART1_RXNE 0x00000020U
#define USART1_UE 0x00002000U
#define USART1_TXEIE 0x00000080U
#define USART1_RXNEIE 0x00000020U
#define USART1_TCIE 0x00000040U
#define USART1_WORD8 0x00000000U
#define USART1_OVER16 0x00000000U
#define USART1_TC 0x00000040U
#define USART1_TE 0x00000008U
#define USART1_RE 0x00000004U

#define DIV_Mantiasa  0x00005B00U
#define DIV_Fraction  0x0000000FU










#endif
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

#include "misc.h"

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void EXTI_Configure(void);
void USARTS_Init(void);
void NVIC_Configure(void);

void EXTI15_10_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI2_IRQHandler(void);

void Delay(void);
char onFlag = 0;
char offFlag = 0;
char oFlag = 0;
char fFlag = 0;
//---------------------------------------------------------------------------------------------------

void RCC_Configure(void) // stm32f10x_rcc.h 참고
{
    
	/* USART pin clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* USART2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN, ENABLE);


	/* AFIO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

}

void GPIO_Configure(void) // stm32f10x_gpio.h 참고
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* UART between bluetooth pin setting */
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  	//RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}


void USARTS_Init(void)
{
	USART_InitTypeDef USART_InitStructure;

	// USART2 peripheral
  USART_Cmd(USART2, ENABLE);
	
	/* 
       BaudRate: 9600
       WordLength: 8bits
       Parity: None
       StopBits: 1bit
       Hardware Flow Control: None
     */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = (uint16_t) USART_WordLength_8b;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity = (uint16_t) USART_Parity_No;
    USART_InitStructure.USART_StopBits = (uint16_t) USART_StopBits_1;
    USART_InitStructure.USART_HardwareFlowControl = (uint16_t) USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}


void NVIC_Configure(void) { // misc.h 참고

    NVIC_InitTypeDef NVIC_InitStructure;
  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    // UART2
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void USART2_IRQHandler() {
    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){     
        char temp = USART_ReceiveData(USART2);
        if(temp == 'o'){
           if(oFlag == 0){
               oFlag =1;
           } 
           else{
               oFlag=0;
           }
           fFlag =0;
        }
        else if(temp == 'n'){
            if(oFlag == 1 && fFlag == 0){
                onFlag = 1;
                offFlag = 0;
            }
            oFlag=0;
            fFlag=0;
        }
        else if(temp == 'f'){
            if(oFlag == 1){
                if(fFlag == 1){
                    offFlag =1;
                    onFlag =0;
                    oflag =0; fFlag =0;
                }
                else{
                    fFlag = 1;
                }
            }
            else{
                fFlag =0;
            }
        }
        else{
            oFlag =0; fFlag=0; 
        }
    }
}

// LED 인터럽트에 넣기 
void sendstatusMessage(){
    //getLEDstatus;
    uint16_t bright =  LED_GetBrightness(void);
    char msg[] = "LED bright : ";
    char* pmsg = msg;
    while(*pmsg!=0){
        USART_SendData(USART2, *pmsg);
        pmsg++;
    } 
    USART_SendData(USART2, bright);
    USART_SendData(USART2, '\n');
    
}

void sendMaxBrightMessage(){
    char msg[] = "LED brightness reaches max brightness";
    char* pmsg = msg;
    while(*pmsg!=0){
        USART_SendData(USART2, *pmsg);
        pmsg++;
    } 
    USART_SendData(USART2, '\r\n');

}


void sendMinBrightMessage(){
    char msg[] = "LED bright : ";
    char* pmsg = msg;
    while(*pmsg!=0){
        USART_SendData(USART2, *pmsg);
        pmsg++;
    } 
    USART_SendData(USART2, '\r\n');
}



int main(void)
{   
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    EXTI_Configure();
    USARTS_Init();
    NVIC_Configure();
    uint16_t bufferIdx = 0;
    uint16_t idx = 0;

    while (1) {
        if(onFlag){
            //turnonLED()
            onFlag = 0;

        }
    
        if(offFlag){
            //turnoffLED()
            offFlag =0;
        }


        
      
    }
    return 0;
}

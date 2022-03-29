/**
 * @file uart.c
 * @brief UART driver for STM32L05x
 *        (c)2018 I.Filippov
 */
#include "uart.h"
#include "rcc.h"
#include "task.h"

static UartEvents_t *uartEvents;
static DmaCh2Events_t *dmaCh2Events;

/**
 * @brief UART1 initialization
 * @param events - structure, containing pointers to UART event handlers
 * @param br - UART baud rate, bit/sec
 * @retval None
 */
void initUart1(UartEvents_t *events, uint32_t br)
{
    RCC_ClocksTypeDef RCC_ClocksStructure;
    uint32_t divider = 0U, apbclock = 0U, tmpreg = 0U;
        
    uartEvents = events; // init callbacks
                
    /* Enable the peripheral clock GPIOA */
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    /* Enable the peripheral clock USART1 */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_SYSCFGEN;
         
    /* Configure UART TXD and UART RXD pins */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10))|
                                    (GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1); // AF on PA9, PA10
                                                                 
    GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (0xF<<(1*4))) | (4<<(1*4));   // AF4 for PA9  (TXD)
    GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (0xF<<(2*4))) | (4<<(2*4));   // AF4 for PA10 (RXD)
                
    /* Configure USART1 */
    RCC_GetClocksFreq(&RCC_ClocksStructure); // get clocks frequencys
    apbclock = RCC_ClocksStructure.PCLK2_Frequency;
    
    /* Compute devider value */
    if ((USART1->CR1 & USART_CR1_OVER8) != 0) {
        // (divider * 10) computing in case Oversampling mode is 8 Samples
        divider = (uint32_t)((2 * apbclock) / br);
        tmpreg  = (uint32_t)((2 * apbclock) % br);
    }
    else {
        // (divider * 10) computing in case Oversampling mode is 16 Samples
        divider = (uint32_t)((apbclock) / br);
        tmpreg  = (uint32_t)((apbclock) % br);
    }
    // Round the divider : if fractional part i greater than 0.5 increment divider
    if (tmpreg >=  br / 2) {
        divider++;
    } 
    // Implement the divider in case Oversampling mode is 8 Samples
    if ((USART1->CR1 & USART_CR1_OVER8) != 0) {
        tmpreg = (divider & (uint16_t)0x000F) >> 1; // get the LSB of divider and shift it to the right by 1 bit 
        divider = (divider & (uint16_t)0xFFF0) | tmpreg; // update the divider value 
    }
    USART1->BRR = (uint16_t)divider; // set devider for baudrate

    // 9 bit data, 1 start bit, 1 stop bit, parity odd, interrupts RXNE & PE enable
    USART1->CR1 = USART_CR1_M_0 | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE |
                    USART_CR1_RXNEIE | USART_CR1_RTOIE | USART_CR1_PEIE | USART_CR1_RE | USART_CR1_UE;
                                                           
    USART1->CR2 |= USART_CR2_RTOEN;     // enable RX timeout
    USART1->CR3 |= USART_CR3_EIE;       // enable error interrupts (FE, ORE, NE)
    USART1->RTOR = (uint32_t)(1.0 / br * 11.0 * 2.0 * 1000.0 + 0.5); // RX timeout value
        
    /* Enable USART1 IRQ */
    NVIC_SetPriority(USART1_IRQn, 0);
    NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief Set uart TXE interrupt enable
 * @param USARTx - USART unit (USART1, USART2...)
 * @retval None
 */
void UartTxEnable(USART_TypeDef* USARTx)
{
    USARTx->CR1 |= USART_CR1_TXEIE;
}

/**
 * @brief Set uart TXE interrupt disable
 * @param USARTx - USART unit (USART1, USART2...)
 * @retval None
 */
void UartTxDisable(USART_TypeDef* USARTx)
{
    USARTx->CR1 &= ~USART_CR1_TXEIE;
}

/**
 * @brief Set uart TC interrupt enable
 * @param USARTx - USART unit (USART1, USART2...)
 * @retval None
 */
void UartTcEnable(USART_TypeDef* USARTx)
{
    USARTx->CR1 |= USART_CR1_TCIE;
}

/**
 * @brief Set uart TC interrupt disable
 * @param USARTx - USART unit (USART1, USART2...)
 * @retval None
 */
void UartTcDisable(USART_TypeDef* USARTx)
{
    USARTx->CR1 &= ~USART_CR1_TCIE;
}

/**
 * @brief Checks is Uart transmit has done or not
 * @param None
 * @retval true if Uart transmit has done, false otherwise
 */
bool isUartTxDone(void)
{
    if ((USART1->ISR & USART_ISR_TC) == USART_ISR_TC)
        return true;
    else
        return false;
}

/**
 * @brief Send data byte
 * @param USARTx - USART unit (USART1, USART2...)
 * @param data - data byte
 * @retval None
 */
void UartSendByte(USART_TypeDef* USARTx, uint8_t data)
{
    USARTx->TDR = data;
}

/**
 * @brief Send array of data bytes
 * @param USARTx - USART unit (USART1, USART2...)
 * @param pack - array of data bytes
 * @param len - size of array
 * @retval None
 */
void UartSendPacket(USART_TypeDef* USARTx, uint8_t* pack, uint16_t len)
{
    while(len--)
    {
        while(!(USARTx->ISR & USART_ISR_TXE)) {} // check is USART1 Tx buff empty
        USARTx->TDR = *(pack++); // send byte and shift pointer to next byte
    }
    while(!(USARTx->ISR & USART_ISR_TC)) {} // USART1 transmitting done
    USARTx->ICR = USART_ICR_TCCF; // Clear TC flag
}

/**
 * @brief DMA Memory to USART initialization
 * @param events - structure, containing pointers to DMA event handlers
 * @retval None
 */
void initDmaMem2Uart(DmaCh2Events_t* events)
{
    dmaCh2Events = events;
    /* DMA1 Channel1 Config */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // DMA Periph clock enable
    DMA1_Channel2->CCR = 0;
    USART1->CR3 |= USART_CR3_DMAT;
    DMA1_CSELR->CSELR |= 0x30;    // USART TX
    DMA1_Channel2->CPAR = (uint32_t)(&(USART1->TDR)); // destination address
    DMA1_Channel2->CCR |= (DMA_CCR_PL_1 | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TEIE | DMA_CCR_TCIE);
    
    /* Configure NVIC for DMA */
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
}

/**
 * @brief Starts DMA Memory to USART
 * @param src_addr - source address
 * @param size - data quantity
 * @retval None
 */
static void startDmaMem2Uart(uint32_t src_addr, uint32_t size)
{
    DMA1_Channel2->CMAR = src_addr;     // source address
    DMA1_Channel2->CNDTR = size;        // size of data to transfer
    DMA1_Channel2->CCR |= DMA_CCR_EN;   // start DMA
}

/**
 * @brief Stops DMA Memory to USART
 * @param None
 * @retval None
 */
void stopDmaMem2Uart(void)
{
   DMA1_Channel2->CCR &= ~DMA_CCR_EN;  // stop DMA 
}

/**
 * @brief Wrapper for HW abstraction
 */
void SendPacket(uint32_t src_addr, uint32_t size)
{
    startDmaMem2Uart(src_addr, size);
}

/**
 * @brief DMA1 Channel2 IRQ Handler
 * @param None
 * @retval None
 */
void DMA1_Channel2_3_IRQHandler(void)
{
    if ((DMA1->ISR & DMA_ISR_TCIF2) != 0) {        
        DMA1->IFCR |= DMA_IFCR_CTCIF2;      // Clear the TC flag
        
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;  // stop DMA 
        USART1->CR1 |= USART_CR1_TCIE;      // Usart TC interrupt enable
        
        if ((dmaCh2Events != NULL) && (dmaCh2Events->ch2TxComplete != NULL)) {
            dmaCh2Events->ch2TxComplete(); // notifies application about DMA TC event
        }
    }
    if ((DMA1->ISR & DMA_ISR_TEIF2) != 0) {
        DMA1->IFCR |= DMA_IFCR_CTEIF2;  // Clear the TE flag
        if ((dmaCh2Events != NULL) && (dmaCh2Events->ch2TxError != NULL)) {
            dmaCh2Events->ch2TxError(); // notifies application about DMA TE event
        }
    }
}

/**
 * @brief USART1 IRQ Handler
 * @param None
 * @retval None
 */
void USART1_IRQHandler(void)
{
    static uint8_t data;
    static uint8_t err_code;
                
    /* USART Rx Not Empty Interrupt */
    if ((USART1->CR1 & USART_CR1_RXNEIE) && (USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
        data = (uint8_t)(USART1->RDR);      // receive data byte
        ///USART1->CR1 |= USART_CR1_RTOIE;     // enable rx timeout interrupt
        if ((uartEvents != NULL) && (uartEvents->rxByteDone != NULL)) {
            uartEvents->rxByteDone(data);   // notifies application about uart rx byte event
        }
    }
    /* USART Rx Timeout Interrupt */
    if ((USART1->CR1 & USART_CR1_RTOIE) && (USART1->ISR & USART_ISR_RTOF) == USART_ISR_RTOF) {
        ///USART1->CR1 &= ~USART_CR1_RTOIE;    // disable rx time out interrupt
        USART1->ICR = USART_ICR_RTOCF;      // clear RTO flag
        if ((uartEvents != NULL) && (uartEvents->rxBlockDone != NULL)) {
            ///uartEvents->rxBlockDone();      // notifies application about uart rx timeout event
            PutEvent(uartEvents->rxBlockDone, NULL);
        }
    }
    /* USART TX Empty Interrupt (not used in USART TX DMA-mode) */
    if ((USART1->CR1 & USART_CR1_TXEIE) && (USART1->ISR & USART_ISR_TXE) == USART_ISR_TXE) {
        if ((uartEvents != NULL) && (uartEvents->txReady != NULL)) {
            uartEvents->txReady(); // notifies application about uart tx ready event
        }
    }
    
    /* USART TX Complete Interrupt */
    if ((USART1->CR1 & USART_CR1_TCIE) && (USART1->ISR & USART_ISR_TC) == USART_ISR_TC) {
        if ((uartEvents != NULL) && (uartEvents->txComplete != NULL)) {
            uartEvents->txComplete(); // notifies application about uart tx copmplete event
        }
    }
    /* USART Error Interrupts */
    err_code = 0;
    if ((USART1->CR1 & USART_CR1_PEIE) && (USART1->ISR & USART_ISR_PE) == USART_ISR_PE) {
        USART1->ICR = USART_ICR_PECF;   // clear PE flag
        err_code |= UART_PARITY_ERROR;  // parity error
    }
    if ((USART1->CR3 & USART_CR3_EIE) && (USART1->ISR & USART_ISR_FE) == USART_ISR_FE) {
        USART1->ICR = USART_ICR_FECF;   // clear FE flag
        err_code |= UART_FRAME_ERROR;   // framing error
    }
    if ((USART1->CR3 & USART_CR3_EIE) && (USART1->ISR & USART_ISR_NE) == USART_ISR_NE) {
        USART1->ICR = USART_ICR_NCF;    // clear NE flag
        //err_code |= UART_NOISE_ERROR;   // noise error - not used in the HART protocol
    }
    if ((USART1->CR3 & USART_CR3_EIE) && (USART1->ISR & USART_ISR_ORE) == USART_ISR_ORE) {
        USART1->ICR = USART_ICR_ORECF;  // clear ORE flag
        err_code |= UART_OVERRUN_ERROR; // overrun error
    }
    if (err_code != 0) {
        if ((uartEvents != NULL) && (uartEvents->error != NULL)) {
            uartEvents->error(err_code); // notifies application about uart error event
        }
    }
}
//eof

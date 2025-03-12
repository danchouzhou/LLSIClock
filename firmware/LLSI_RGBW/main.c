#include <stdio.h>
#include "NuMicro.h"

#define HCLK_CLK    72000000
#define TEST_COUNT  24

// Color format is 0xWWBBRRGG for the Adafruit NeoPixel Ring 24 x RGBW (2862)
volatile uint32_t g_au32frameBuffer[24] = {
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
};
volatile uint32_t g_u32PatternToggle = 0;
volatile uint32_t g_u32DataCount = 0;
volatile uint32_t g_u32FrameEnd = 0;

void LLSI0_IRQHandler()
{
    if (LLSI_GetIntFlag(LLSI0, LLSI_TXTH_INT_MASK))
    {
        while(LLSI_GET_TX_FIFO_FULL_FLAG(LLSI0) == 0) // Fill up the TX FIFO
        {
            if (g_u32DataCount == (TEST_COUNT - 1))
                LLSI_SET_LAST_DATA(LLSI0); // Before writing last word data to LLSI_DATA, user must write LDT = 1
            
            if (g_u32DataCount < TEST_COUNT)
                LLSI_WRITE_DATA(LLSI0, g_au32frameBuffer[g_u32DataCount++]);
            else
                break;
        }

        if(g_u32DataCount >= TEST_COUNT)
        {
            LLSI_DisableInt(LLSI0, LLSI_TXTH_INT_MASK);
        }
    }

    if (LLSI_GetIntFlag(LLSI0, LLSI_FEND_INT_MASK)) // FENDIF will be set to 1 when LLSI transfer last pixel data.
    {
        g_u32FrameEnd = 1;
        g_u32PatternToggle++;
        LLSI_ClearIntFlag(LLSI0, LLSI_FEND_INT_MASK);
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to HCLK_CLK Hz */
    CLK_SetCoreClock(HCLK_CLK);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC/2 and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC_DIV2, CLK_CLKDIV0_UART0(1));

    /* Enable LLSI0 module clock */
    CLK_EnableModuleClock(LLSI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PC multi-function pins for LLSI0 */
    SET_LLSI0_OUT_PC5();
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void LLSI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init LLSI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as software mode, RGB output format, 6 pixels in a frame and idle output low */
    /* Set clock divider. LLSI clock rate = 72MHz */
    /* Set data transfer period. T_Period = 1250ns */
    /* Set duty period. T_T0H = 400ns; T_T1H = 850ns */
    /* Set reset command period. T_ResetPeriod = 50000ns */
    LLSI_Open(LLSI0, LLSI_MODE_SW, LLSI_FORMAT_RGB, HCLK_CLK, 1250, 400, 850, 50000, 32, LLSI_IDLE_LOW);

    /* Set TX FIFO threshold */
    LLSI_SetFIFO(LLSI0, 2);

    /* Enable reset command function */
    LLSI_ENABLE_RESET_COMMAND(LLSI0);

    NVIC_EnableIRQ(LLSI0_IRQn);
}

void main(void)
{
    uint32_t u32Tmp;
    
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART_Open(UART0, 115200);

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    /* Init LLSI */
    LLSI_Init();

    /* Enable Transmit FIFO Threshold Interrupt and Frame End Interrupt */
    LLSI_EnableInt(LLSI0, LLSI_TXTH_INT_MASK | LLSI_FEND_INT_MASK);

    LLSI_WRITE_DATA(LLSI0, g_au32frameBuffer[g_u32DataCount++]); // Write the first word to trigger TXTH_INT

    while(1)
    {
        if (g_u32FrameEnd == 1)  // Wait for FEND_INT
        {
            u32Tmp = g_u32PatternToggle % 4;
            
            /* Generate a new frame */
            for (int i=0; i<TEST_COUNT; i++)
            {
                if (i % 4 == u32Tmp)
                    g_au32frameBuffer[i] = 0xFF000000 >> (u32Tmp * 8);
                else
                    g_au32frameBuffer[i] = 0x00000000;
            }

            g_u32FrameEnd = 0;
            g_u32DataCount = 0;

            LLSI_EnableInt(LLSI0, LLSI_TXTH_INT_MASK);
            
            CLK_SysTickDelay(50000);
        }
    }
}

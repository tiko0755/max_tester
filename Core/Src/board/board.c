/******************** (C) COPYRIGHT 2015 INCUBECN *****************************
* File Name          : board.c
* Author             : Tiko Zhong
* Date First Issued  : 11/18/2021
* Description        : 
*                      
********************************************************************************
* History:
* Apr22,2021: V0.2
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "board.h"
#include "user_log.h"
#include "main.h"
#include "app_timer.h"
#include "gpioDecal.h"
#include "led_flash.h"
#include "outputCmd.h"
#include "inputCmd.h"
#include "adc_dev.h"
#include "stm_flash.h"
#include "iic_io.h"
#include "cw2217.h"

#define NOUSED_PIN_INDX 255

/* import handle from main.c variables ---------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const char ABOUT[] = {"MB2160X2-1.0.0"};
const char COMMON_HELP[] = {
    "Commands:"
    "\n help()"
    "\n about()"
    "\n restart()"
    "\n reg.write(addr,val)"
    "\n reg.read(addr)"
    "\n baud.set(bHost,bBus)"
    "\n baud.get()"
    "\n ipconf.setip(ip0,..,ip3)"
    "\n   |-ipconf.setip(12.34.56.78:port)"
    "\n ipconf.setmask(msk0,..,msk3)"
    "\n ipconf.setgw(gw0,..,gw3)"
    "\n ipconf.info()"
    "\n"
};
char g_addrPre[4] = {0};    //addr precode
u8 g_initalDone = 0;
u8 g_baudHost = 4;    //BAUD[4]=115200
u8 g_baud485 = 4;
u32 g_errorCode;// = 0;
/**********************************************
*  PINs Define
**********************************************/
const PIN_T RUNNING = {LED1_GPIO_Port, LED1_Pin};

/**********************************************
*  static Devices
**********************************************/
// =============================================
#define RX_POOL_LEN    (MAX_CMD_LEN)
#define TX_POOL_LEN    (MAX_CMD_LEN)
#define    RX_BUF_LEN    (128)
// uart device
static u8 uartRxPool[RX_POOL_LEN] = {0};
static u8 uartRxBuf[2*RX_BUF_LEN] = {0};
static u8 uartTxPool[TX_POOL_LEN] = {0};
UartDev_t console;

// app timer 
appTmrDev_t tmr[APP_TIMER_COUNT] = {0};

// commander comsumer
cmdConsumerDev_t cmdConsumer;

OUTPUT_DEV_T g_output;
const PIN_T OUTPUT_PIN[] = {
    {BEEP_GPIO_Port, BEEP_Pin},		// beep
    {EN_5V_GPIO_Port, EN_5V_Pin},
    {EN_DC_GPIO_Port, EN_DC_Pin},
    {EN_VOUT_GPIO_Port, EN_VOUT_Pin},
};

// POWER CONTROLS
const PIN_T VOUT_EN0 = {EN_DC_GPIO_Port, EN_DC_Pin};
const PIN_T VOUT_EN1 = {EN_VOUT_GPIO_Port, EN_VOUT_Pin};
const PIN_T EN_5V = {EN_5V_GPIO_Port, EN_5V_Pin};

INPUT_DEV_T g_input;
const PIN_T INPUT_PIN[] = {
    {BUTTON1_GPIO_Port, BUTTON1_Pin},
    {BUTTON2_GPIO_Port, BUTTON2_Pin},
};

// adc device
adcDev_T adcD;

static u8 brdCmdU8(void* d, u8* CMDu8, u8 len, void (*xprint)(const char* FORMAT_ORG, ...));
static void forwardToBus(u8* BUFF, u16 len);


// IIC DEV
cw2217_dev_t cw2217[2] = {0};
IIC_IO_Dev_T iicDev1;
const PIN_T SDA1 = {SDA1_GPIO_Port, SDA1_Pin};
const PIN_T SCL1 = {SCL1_GPIO_Port, SCL1_Pin};

IIC_IO_Dev_T iicDev2;
const PIN_T SDA2 = {SDA2_GPIO_Port, SDA2_Pin};
const PIN_T SCL2 = {SCL2_GPIO_Port, SCL2_Pin};


/* Private function prototypes -----------------------------------------------*/
// after GPIO initial, excute this function to enable
void boardPreInit(void){
//    configRead();
}

#define ADC_SINGLE_ENDED                (0x00000000U)
static s32 ubSequenceCompleted;
static void testHandler(void* e);

void boardInit(void){
    s32 tmrIndx;

    HAL_GPIO_WritePin(RUNNING.GPIOx, RUNNING.GPIO_Pin, GPIO_PIN_SET);
    // 
    
    
    // setup app timers
    for(tmrIndx=0;tmrIndx<APP_TIMER_COUNT;tmrIndx++){
        setup_appTmr(&tmr[tmrIndx]);
    }
    
    tmrIndx = 0;
    //read board addr
    setupUartDev(&console, &huart1, &tmr[tmrIndx++], uartTxPool, RX_POOL_LEN, uartRxPool, RX_POOL_LEN, uartRxBuf, RX_BUF_LEN, 4);
    memset(g_addrPre,0,4);
    strFormat(g_addrPre, 4, "%d.", g_boardAddr);
    print("%sabout(\"%s\")\r\n", g_addrPre, ABOUT);

    logInitial(printS);
    
    IIC_IO_Setup(&iicDev1, &SCL1, &SDA1);
    cw2217_setup(&cw2217[0], &iicDev1);
    
    IIC_IO_Setup(&iicDev2, &SCL2, &SDA2);
    cw2217_setup(&cw2217[1], &iicDev2);
    
    printS("setup adc...");
    ADC_Setup(&adcD, &hadc1, &tmr[tmrIndx++], NULL, NULL, 0);
    printS("ok\r\n");
    
    // application initial
    printS("setup led_flash...");
    led_flash_init(&tmr[tmrIndx++], &RUNNING, 100);     // now, it can run itself
    printS("ok\r\n");

    printS("setup gpio driver...");
    outputDevSetup(&g_output, OUTPUT_PIN, 4, 0x00000000);
    InputDevSetup(&g_input, OUTPUT_PIN, 5);
    printS("ok\r\n");
    
    printS("setup cmdConsumer...");
    setup_cmdConsumer(&cmdConsumer, 
        &console.rsrc.rxRB,     // command line in a ringbuffer
        fetchLineFromRingBufferU8, // fetchLine method  
        print,                  // print out 
        forwardToBus,
        &tmr[tmrIndx++],
        10                     // unit in ms, polling rb each interval
    );
    cmdConsumer.append(&cmdConsumer.rsrc, NULL, brdCmdU8);
    cmdConsumer.append(&cmdConsumer.rsrc, &g_output, outputCmdU8);
    printS("ok\r\n");

    // get ready, start to work
    console.StartRcv(&console.rsrc);

    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        /* Calibration Error */
        Error_Handler();
    }

    tmr[tmrIndx].start(&tmr[tmrIndx].rsrc, 1000, POLLING_REPEAT, testHandler, NULL);
    tmrIndx++;


    
    HAL_GPIO_WritePin(VOUT_EN0.GPIOx, VOUT_EN0.GPIO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(VOUT_EN1.GPIOx, VOUT_EN1.GPIO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_5V.GPIOx, EN_5V.GPIO_Pin, GPIO_PIN_SET);
    
    
    
    g_initalDone = 1;

//    print("flashSize: 0x%08x\n", *(u32*)FLASH_SIZE_DATA_REGISTER);
    
    print("%d timers have been used", tmrIndx);
    printS("initial complete, type \"help\" for help\n");
}

static u8 testSqu = 0, adcIndx = 0;
static u32 testTick =0;
static u16 adcVal[8][4];
#define LOOP_TIM (300)
static void testHandler(void* e){
//    printS("adc:");
//    print("%d ", adcD.rsrc.adcSeries[0][0]);
//    print("%d ", adcD.rsrc.adcSeries[1][0]);
//    print("%d ", adcD.rsrc.adcSeries[2][0]);
//    print("%d ", adcD.rsrc.adcSeries[3][0]);
//    print("%d ", adcD.rsrc.adcSeries[4][0]);
//    print("%d ", adcD.rsrc.adcSeries[5][0]);
//    print("%d ", adcD.rsrc.adcSeries[6][0]);
//    print("%d ", adcD.rsrc.adcSeries[7][0]);
//    printS("\n");
    
    cw2217[0].update_chip_id(&cw2217[0].rsrc);
//    cw2217[1].get_chip_id(&cw2217[1].rsrc);

}


void printS(const char* STRING){
    console.Send(&console.rsrc, (const u8*)STRING, strlen(STRING));
}

void print(const char* FORMAT_ORG, ...){
    va_list ap;
    char buf[MAX_CMD_LEN] = {0};
    s16 bytes;
    //take string
    va_start(ap, FORMAT_ORG);
    bytes = vsnprintf(buf, MAX_CMD_LEN, FORMAT_ORG, ap);
    va_end(ap);
    //send out
    if(bytes>0)    console.Send(&console.rsrc, (u8*)buf, bytes);
}

void printSUDP(const char* STRING){
//    handler_udp->send(&handler_udp->rsrc, (u8*)STRING, strlen(STRING));
}

void printUDP(const char* FORMAT_ORG, ...){
//    va_list ap;
//    char buf[MAX_CMD_LEN] = {0};
//    s16 bytes;
//    //take string
//    va_start(ap, FORMAT_ORG);
//    bytes = vsnprintf(buf, MAX_CMD_LEN, FORMAT_ORG, ap);
//    va_end(ap);
//    //send out
//    if(bytes>0)    handler_udp->send(&handler_udp->rsrc, (u8*)buf, bytes);
}

//s8 ioWrite(u16 addr, const u8 *pDat, u16 nBytes){
//    erom.Write(&erom.rsrc, EEPROM_BASE_USER + addr, pDat, nBytes);
//    return 0;
//}
//
//s8 ioRead(u16 addr, u8 *pDat, u16 nBytes){
//    erom.Read(&erom.rsrc, EEPROM_BASE_USER+addr, pDat, nBytes);
//  return 0;
//}


static void forwardToBus(u8* BUFF, u16 len){
    print("<%s BUFF:%s >", __func__, (char*)BUFF);
}


static u8 brdCmdU8(void* d, u8* CMDu8, u8 len, void (*xprint)(const char* FORMAT_ORG, ...)){
    return(brdCmd((const char*)CMDu8, xprint));
}

u8 brdCmd(const char* CMD, void (*xprint)(const char* FORMAT_ORG, ...)){
    s32 i,j,k;
    u8 brdAddr = g_boardAddr;
    // common
    if(strncmp(CMD, "about", strlen("about")) == 0){
        xprint("+ok@%d.about(\"%s\")\r\n", brdAddr, ABOUT);
        led_flash_answer(50, 30);
        return 1;
    }
    else if(strncmp(CMD, "help", strlen("help")) == 0){
        xprint("%s +ok@%d.help()\r\n", COMMON_HELP, brdAddr);
        return 1;
    }
    else if(strncmp(CMD, "restart ", strlen("restart ")) == 0){
        HAL_NVIC_SystemReset();
        return 1;
    }
    else if(strncmp(CMD, "adc.start ", strlen("adc.start")) == 0){
        adcD.start(&adcD.rsrc, 100);
        return 1;
    }
    else if(strncmp(CMD, "adc.stop ", strlen("adc.stop")) == 0){
        adcD.stop(&adcD.rsrc);
        return 1;
    }
    else if(strncmp(CMD, "restart ", strlen("restart ")) == 0){
        HAL_NVIC_SystemReset();
        return 1;
    }
    
    else if(sscanf(CMD, "reg.write %d 0x%x ", &i, &j)==2){
        u8 buff[4] = {0};
        
        
        buff[0] = j&0xff;    j >>= 8;    
        buff[1] |= j&0xff;   j >>= 8;
        buff[2] |= j&0xff;   j >>= 8;
        buff[3] |= j&0xff;   
        
        
        ioWriteAsyn(i, buff, 4, NULL);
        xprint("+ok@%d.reg.write(%d, 0x%02x%02x%02x%02x)\r\n", brdAddr, i, buff[3],buff[2],buff[1],buff[0]);
        return 1;
    }
    
    else if(sscanf(CMD, "reg.readx %d ", &i)==1){
        u32 x;
        u8* p = (u8*)i;
        
        x = *p; x <<=8; p++;
        x |= *p;x <<=8; p++;
        x |= *p;x <<=8; p++;
        x |= *p;

        xprint("+ok@%d.reg.readx(%d,%d)\r\n", brdAddr, i, x);
        return 1;
    }
    
    else if(sscanf(CMD, "reg.read %d ", &i)==1){
        u32 x;
        u8* p = ioReadDMA(i);
        
        x = *p++; x <<=8;
        x |= *p++;x <<=8; 
        x |= *p++;x <<=8;
        x |= *p;

        xprint("+ok@%d.reg.read(%d,0x%08x)\r\n", brdAddr, i, x);
        return 1;
    }


//    else if(sscanf(CMD, "reg.write %d %d ", &i, &j)==2){
//        if(i>=EEPROM_SIZE_REG/4)    {
//            xprint("+err@%d.reg.write(\"address[0..%d]\")\r\n", brdAddr, EEPROM_SIZE_REG/4);
//            return 1;
//        }
//        if(ioWriteReg(i,j) == 0)    xprint("+ok@%d.reg.write(%d,%d)\r\n", brdAddr, i, j);
//        else xprint("+err@%d.reg.write(%d,%d)\r\n", brdAddr, i, j);
//        return 1;
//    }
//    else if(sscanf(CMD, "reg.read %d ", &i)==1){
//        if(i>=EEPROM_SIZE_REG/4){
//            xprint("+err@%d.reg.read(\"address[0..%d]\")\r\n", brdAddr, EEPROM_SIZE_REG/4);
//            return 1;
//        }
//        ioReadReg(i,&j);
//        xprint("+ok@%d.reg.read(%d,%d)\r\n", brdAddr, i, j);
//        return 1;
//    }

//    else if(sscanf(CMD, "baud.set %d %d", &i,&j)==2){
//        for(k=0;k<7;k++){
//            g_baudHost = k;
//            if(i==BAUD[g_baudHost])    break;
//        }
//        for(k=0;k<7;k++){
//            g_baud485 = k;
//            if(j==BAUD[g_baud485])    break;
//        }
//        configWrite();
//        xprint("+ok@%d.baud.set(%d,%d)\r\n", brdAddr, BAUD[g_baudHost], BAUD[g_baud485]);
//        return 1;
//    }
//    else if(strncmp(CMD, "baud.get ", strlen("baud.get "))==0){
//        configRead();
//        xprint("+ok@%d.baud.get(%d,%d)\r\n", brdAddr, BAUD[g_baudHost], BAUD[g_baud485]);
//        return 1;
//    }

    return 0;
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle){}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(g_initalDone==0)    return;
    if(huart->Instance == console.rsrc.huart->Instance){
        console.rsrc.flag |= BIT(0);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
}

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
}


/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
  /* Report to main program that ADC sequencer has reached its end */
    if(adcIndx >= 8){
        adcIndx = 0;
    }
    adcVal[adcIndx][0] = HAL_ADC_GetValue(AdcHandle);
    ubSequenceCompleted++;
    
//print("<%s >", __func__);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

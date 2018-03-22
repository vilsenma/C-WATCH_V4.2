#ifndef _MAIN_H
#define _MAIN_H


uint8_t BCC(uint8_t *buffer ,uint8_t size);

typedef union 
{
    unsigned char DataBuffer[10];
    struct
    {
        unsigned char Addr;
        unsigned char CMD;
        
        unsigned char CellHValue;
        unsigned char CellLValue;
         
        unsigned char CellHRes;
        unsigned char CellLRes;
           
        unsigned char CellHTemt;
        unsigned char CellLTemt;
        
        unsigned char Reserve;
        unsigned char BCC;
        
    }Data;
}TureData;


uint8_t  RX_BUF[10]={};
uint8_t  TX_BUF[10]={};

uint8_t  RES_ING=0;
uint8_t  CONFIG_SET=0;

uint8_t  CONFIG_BUF[2];


//采集数据存储空间
volatile TureData TUREDATA;


void EEPROOM_INIT(void);
float Get_ADC(adc_channel_t channel);


int srt(long k);
uint16_t sam_res(void);

float ResToTemp(float res);
uint8_t BCC(uint8_t *buffer ,uint8_t size);

void NRF24L01_SendMSG(uint8_t *MSG);


#endif
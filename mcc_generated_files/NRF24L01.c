#include "NRF24L01.h"
#include <xc.h>
#include "pin_manager.h"
#include "interrupt_manager.h"

//const unsigned char TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
//const unsigned char RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���ܵ�ַ

const unsigned char TX_ADDRESS[TX_ADR_WIDTH]={0x54,0x45,0xA0,0xA0,0xA0}; //�ɼ�ģ���ַ
const unsigned char RX_ADDRESS[RX_ADR_WIDTH]={0x45,0x54,0x0A,0x0A,0x0A}; //Сģ�鱾����յ�ַ
//const unsigned char TX_ADDRESS[TX_ADR_WIDTH]={0x54,0x45,0xA0,0xA0,0xA0}; //���ݺ��ӵ�ַ
//const unsigned char RX_ADDRESS[RX_ADR_WIDTH]={0x45,0x54,0x0A,0x0A,0x0A}; //Сģ����ܵ�ַ


//��ʼ��24L01��IO��
void NRF24L01_Init(void)
{
	Clr_NRF24L01_CE;
    Set_NRF24L01_CSN;
}

unsigned char NRF24L01_Read_Write_Byte(unsigned char data)
{
    
    unsigned char i;
   	for(i=0;i<8;i++)                    // ѭ��8��
   	{
	    if(data&0x80)
	        Set_NRF24L01_MOSI;
	    else
	        Clr_NRF24L01_MOSI;          // byte���λ�����MOSI
   	    data<<=1;                       // ��һλ��λ�����λ
        
   	    Set_NRF24L01_CSK; 
	    if(WL_MISO_GetValue())          // ����SCK��nRF24L01��MOSI����1λ���ݣ�ͬʱ��MISO���1λ����
   	        data|=0x01;                 // ��MISO��byte���λ
   	    WL_SCK_SetLow();            	// SCK�õ�
   	}
    
    return(data);                       // ���ض�����һ�ֽ�    
    
}
//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
unsigned char NRF24L01_Check(void)
{
	unsigned char tempbuf[5],buf[5]={0xA5,0xA5,0xA5,0xA5,0xA5};
	unsigned char i;
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,buf,5);   //д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,tempbuf,5);                       //����д��ĵ�ַ  
	for(i=0;i<5;i++)
        if(tempbuf[i]!=buf[i])
            break;	 							   
	if(i!=5)
        return 1;                                           //���24L01����	
	return 0;                                               //��⵽24L01
}	
 	 
//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
unsigned char NRF24L01_Write_Reg(unsigned char reg,unsigned char value)
{
	unsigned char status;	
	Clr_NRF24L01_CSN;                           //ʹ��SPI����
	status =NRF24L01_Read_Write_Byte(reg);      //���ͼĴ����� 
	NRF24L01_Read_Write_Byte(value);            //д��Ĵ�����ֵ
	Set_NRF24L01_CSN;                           //��ֹSPI����	   
	return(status);                             //����״ֵ̬
}

//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
unsigned char NRF24L01_Read_Reg(unsigned char reg)
{
	unsigned char reg_val;	    
	Clr_NRF24L01_CSN;                                   //ʹ��SPI����		
	NRF24L01_Read_Write_Byte(reg);                      //���ͼĴ�����
	reg_val=NRF24L01_Read_Write_Byte(0XFF);             //��ȡ�Ĵ�������
	Set_NRF24L01_CSN;                                   //��ֹSPI����		    
	return(reg_val);                                    //����״ֵ̬
}	

//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
unsigned char NRF24L01_Read_Buf(unsigned char reg,unsigned char *pBuf,unsigned char len)
{
	unsigned char status,u8_ctr;	       
	Clr_NRF24L01_CSN;                                                               //ʹ��SPI����
	status=NRF24L01_Read_Write_Byte(reg);                                           //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
        pBuf[u8_ctr]=NRF24L01_Read_Write_Byte(0XFF);   //��������
	Set_NRF24L01_CSN;                                                               //�ر�SPI����
	return status;                                                                  //���ض�����״ֵ̬
}

//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
unsigned char NRF24L01_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char len)
{
	unsigned char status,u8_ctr;	    
	Clr_NRF24L01_CSN;                                                       //ʹ��SPI����
	status = NRF24L01_Read_Write_Byte(reg);                                 //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)NRF24L01_Read_Write_Byte(*pBuf++);   //д������	 
	Set_NRF24L01_CSN;                                                       //�ر�SPI����
	return status;                                                          //���ض�����״ֵ̬
}
				   
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
//����ԭ����ʧ��
unsigned char NRF24L01_TxPacket(unsigned char *txbuf)
{
	unsigned char sta,test;
    
    if(sta&TX_OK)
    {
        NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUSET,sta);              //���TX_DS��MAX_RT�жϱ�־
        NRF24L01_Write_Reg(NRF24L01_FLUSH_TX,0xff);                 //���TX FIFO�Ĵ���
    }    
    
    
	Clr_NRF24L01_CE;
	NRF24L01_Write_Buf(NRF24L01_WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);  //д���ݵ�TX BUF  32���ֽ�
	Set_NRF24L01_CE;//��������
    
	while(NRF24L01_IRQ!=0);//�ȴ��������
    
    for(test=0;test<200;test++)
    {
        __nop();
    }    
    
	sta=NRF24L01_Read_Reg(STATUSET);  //��ȡ״̬�Ĵ�����ֵ	   
	
    /*
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(NRF24L01_FLUSH_TX,0xff);                 //���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
    */ 
	if(sta&TX_OK)//�������
	{
        NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUSET,sta);              //���TX_DS��MAX_RT�жϱ�־
        NRF24L01_Write_Reg(NRF24L01_FLUSH_TX,0xff);                 //���TX FIFO�Ĵ���
		return TX_OK;
	}
	return 0xff;                                                    //����ԭ����ʧ��
}

//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
void NRF24L01_RxPacket(unsigned char *rxbuf)
{
     
	unsigned char sta;	

    
	sta=NRF24L01_Read_Reg(STATUSET);                                     //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUSET,sta);                 //���TX_DS��MAX_RT�жϱ�־
    
	if(sta&RX_OK)//���յ�����
	{
		NRF24L01_Read_Buf(NRF24L01_RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);   //��ȡ����
		NRF24L01_Write_Reg(NRF24L01_FLUSH_RX,0xff);                     //���RX FIFO�Ĵ��� 
	
	}	
    
    NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUSET,sta);                 //���TX_DS��MAX_RT�жϱ�־

}	


					    
//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
void RX_Mode(void)
{
	Clr_NRF24L01_CE;	
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(unsigned char*)RX_ADDRESS,RX_ADR_WIDTH);  //дRX�ڵ��ַ
	
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x00);                                          //ʹ��ͨ��0���Զ�Ӧ��    
	//NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x01);                                      //ʹ��ͨ��0�Ľ��յ�ַ  	 
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,CHANNEL);                                       //����RFͨ��Ƶ��		  
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);                             //ѡ��ͨ��0����Ч���ݿ�� 	    
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x07);                                       //����TX�������,0db����,1Mbps,���������濪��   
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG, 0x0f);                                        //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
	Set_NRF24L01_CE;                                                                               //CEΪ��,�������ģʽ 
}
							 
//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	 
void TX_Mode(void)
{														 
	Clr_NRF24L01_CE;	    
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(unsigned char*)TX_ADDRESS,TX_ADR_WIDTH);     //дTX�ڵ��ַ 
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(unsigned char*)RX_ADDRESS,RX_ADR_WIDTH);  //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  
	
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x00);                                          //ʹ��ͨ��0���Զ�Ӧ��    
	//NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x01);                                      //ʹ��ͨ��0�Ľ��յ�ַ  
	//NRF24L01_Write_Reg(NRF24L01_WRITE_REG+SETUP_RETR,0x00);                                     //�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,CHANNEL);                                       //����RFͨ��Ϊ40
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x07);                                       //����TX�������,0db����,1Mbps,���������濪��   
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x0e);                                         //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	Set_NRF24L01_CE;                                                                               //CEΪ��,10us����������
}		  

void NOP_Mode(void)
{
    Clr_NRF24L01_CE;	    
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(unsigned char*)TX_ADDRESS,TX_ADR_WIDTH);     //дTX�ڵ��ַ 
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(unsigned char*)RX_ADDRESS,RX_ADR_WIDTH);  //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  
	
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x00);                                          //ʹ��ͨ��0���Զ�Ӧ��    
	//NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x01);                                      //ʹ��ͨ��0�Ľ��յ�ַ  
	//NRF24L01_Write_Reg(NRF24L01_WRITE_REG+SETUP_RETR,0x1a);                                     //�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,CHANNEL);                                       //����RFͨ��Ϊ40
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0f);                                       //����TX�������,0db����,2Mbps,���������濪��   
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x0e);                                         //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	Set_NRF24L01_CE;   
  
}









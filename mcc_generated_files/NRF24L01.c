#include "NRF24L01.h"
#include <xc.h>
#include "pin_manager.h"
#include "interrupt_manager.h"

//const unsigned char TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
//const unsigned char RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //接受地址

const unsigned char TX_ADDRESS[TX_ADR_WIDTH]={0x54,0x45,0xA0,0xA0,0xA0}; //采集模块地址
const unsigned char RX_ADDRESS[RX_ADR_WIDTH]={0x45,0x54,0x0A,0x0A,0x0A}; //小模块本身接收地址
//const unsigned char TX_ADDRESS[TX_ADR_WIDTH]={0x54,0x45,0xA0,0xA0,0xA0}; //数据盒子地址
//const unsigned char RX_ADDRESS[RX_ADR_WIDTH]={0x45,0x54,0x0A,0x0A,0x0A}; //小模块接受地址


//初始化24L01的IO口
void NRF24L01_Init(void)
{
	Clr_NRF24L01_CE;
    Set_NRF24L01_CSN;
}

unsigned char NRF24L01_Read_Write_Byte(unsigned char data)
{
    
    unsigned char i;
   	for(i=0;i<8;i++)                    // 循环8次
   	{
	    if(data&0x80)
	        Set_NRF24L01_MOSI;
	    else
	        Clr_NRF24L01_MOSI;          // byte最高位输出到MOSI
   	    data<<=1;                       // 低一位移位到最高位
        
   	    Set_NRF24L01_CSK; 
	    if(WL_MISO_GetValue())          // 拉高SCK，nRF24L01从MOSI读入1位数据，同时从MISO输出1位数据
   	        data|=0x01;                 // 读MISO到byte最低位
   	    WL_SCK_SetLow();            	// SCK置低
   	}
    
    return(data);                       // 返回读出的一字节    
    
}
//检测24L01是否存在
//返回值:0，成功;1，失败	
unsigned char NRF24L01_Check(void)
{
	unsigned char tempbuf[5],buf[5]={0xA5,0xA5,0xA5,0xA5,0xA5};
	unsigned char i;
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,buf,5);   //写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,tempbuf,5);                       //读出写入的地址  
	for(i=0;i<5;i++)
        if(tempbuf[i]!=buf[i])
            break;	 							   
	if(i!=5)
        return 1;                                           //检测24L01错误	
	return 0;                                               //检测到24L01
}	
 	 
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
unsigned char NRF24L01_Write_Reg(unsigned char reg,unsigned char value)
{
	unsigned char status;	
	Clr_NRF24L01_CSN;                           //使能SPI传输
	status =NRF24L01_Read_Write_Byte(reg);      //发送寄存器号 
	NRF24L01_Read_Write_Byte(value);            //写入寄存器的值
	Set_NRF24L01_CSN;                           //禁止SPI传输	   
	return(status);                             //返回状态值
}

//读取SPI寄存器值
//reg:要读的寄存器
unsigned char NRF24L01_Read_Reg(unsigned char reg)
{
	unsigned char reg_val;	    
	Clr_NRF24L01_CSN;                                   //使能SPI传输		
	NRF24L01_Read_Write_Byte(reg);                      //发送寄存器号
	reg_val=NRF24L01_Read_Write_Byte(0XFF);             //读取寄存器内容
	Set_NRF24L01_CSN;                                   //禁止SPI传输		    
	return(reg_val);                                    //返回状态值
}	

//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
unsigned char NRF24L01_Read_Buf(unsigned char reg,unsigned char *pBuf,unsigned char len)
{
	unsigned char status,u8_ctr;	       
	Clr_NRF24L01_CSN;                                                               //使能SPI传输
	status=NRF24L01_Read_Write_Byte(reg);                                           //发送寄存器值(位置),并读取状态值   	   
	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
        pBuf[u8_ctr]=NRF24L01_Read_Write_Byte(0XFF);   //读出数据
	Set_NRF24L01_CSN;                                                               //关闭SPI传输
	return status;                                                                  //返回读到的状态值
}

//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
unsigned char NRF24L01_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char len)
{
	unsigned char status,u8_ctr;	    
	Clr_NRF24L01_CSN;                                                       //使能SPI传输
	status = NRF24L01_Read_Write_Byte(reg);                                 //发送寄存器值(位置),并读取状态值
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)NRF24L01_Read_Write_Byte(*pBuf++);   //写入数据	 
	Set_NRF24L01_CSN;                                                       //关闭SPI传输
	return status;                                                          //返回读到的状态值
}
				   
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
//其他原因发送失败
unsigned char NRF24L01_TxPacket(unsigned char *txbuf)
{
	unsigned char sta,test;
    
    if(sta&TX_OK)
    {
        NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUSET,sta);              //清除TX_DS或MAX_RT中断标志
        NRF24L01_Write_Reg(NRF24L01_FLUSH_TX,0xff);                 //清除TX FIFO寄存器
    }    
    
    
	Clr_NRF24L01_CE;
	NRF24L01_Write_Buf(NRF24L01_WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);  //写数据到TX BUF  32个字节
	Set_NRF24L01_CE;//启动发送
    
	while(NRF24L01_IRQ!=0);//等待发送完成
    
    for(test=0;test<200;test++)
    {
        __nop();
    }    
    
	sta=NRF24L01_Read_Reg(STATUSET);  //读取状态寄存器的值	   
	
    /*
	if(sta&MAX_TX)//达到最大重发次数
	{
		NRF24L01_Write_Reg(NRF24L01_FLUSH_TX,0xff);                 //清除TX FIFO寄存器 
		return MAX_TX; 
	}
    */ 
	if(sta&TX_OK)//发送完成
	{
        NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUSET,sta);              //清除TX_DS或MAX_RT中断标志
        NRF24L01_Write_Reg(NRF24L01_FLUSH_TX,0xff);                 //清除TX FIFO寄存器
		return TX_OK;
	}
	return 0xff;                                                    //其他原因发送失败
}

//启动NRF24L01接收一次数据
//txbuf:待接收数据首地址
//返回值:0，接收完成；其他，错误代码
void NRF24L01_RxPacket(unsigned char *rxbuf)
{
     
	unsigned char sta;	

    
	sta=NRF24L01_Read_Reg(STATUSET);                                     //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUSET,sta);                 //清除TX_DS或MAX_RT中断标志
    
	if(sta&RX_OK)//接收到数据
	{
		NRF24L01_Read_Buf(NRF24L01_RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);   //读取数据
		NRF24L01_Write_Reg(NRF24L01_FLUSH_RX,0xff);                     //清除RX FIFO寄存器 
	
	}	
    
    NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUSET,sta);                 //清除TX_DS或MAX_RT中断标志

}	


					    
//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了		   
void RX_Mode(void)
{
	Clr_NRF24L01_CE;	
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(unsigned char*)RX_ADDRESS,RX_ADR_WIDTH);  //写RX节点地址
	
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x00);                                          //使能通道0的自动应答    
	//NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x01);                                      //使能通道0的接收地址  	 
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,CHANNEL);                                       //设置RF通信频率		  
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);                             //选择通道0的有效数据宽度 	    
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x07);                                       //设置TX发射参数,0db增益,1Mbps,低噪声增益开启   
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG, 0x0f);                                        //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
	Set_NRF24L01_CE;                                                                               //CE为高,进入接收模式 
}
							 
//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void TX_Mode(void)
{														 
	Clr_NRF24L01_CE;	    
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(unsigned char*)TX_ADDRESS,TX_ADR_WIDTH);     //写TX节点地址 
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(unsigned char*)RX_ADDRESS,RX_ADR_WIDTH);  //设置TX节点地址,主要为了使能ACK	  
	
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x00);                                          //使能通道0的自动应答    
	//NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x01);                                      //使能通道0的接收地址  
	//NRF24L01_Write_Reg(NRF24L01_WRITE_REG+SETUP_RETR,0x00);                                     //设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,CHANNEL);                                       //设置RF通道为40
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x07);                                       //设置TX发射参数,0db增益,1Mbps,低噪声增益开启   
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x0e);                                         //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	Set_NRF24L01_CE;                                                                               //CE为高,10us后启动发送
}		  

void NOP_Mode(void)
{
    Clr_NRF24L01_CE;	    
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(unsigned char*)TX_ADDRESS,TX_ADR_WIDTH);     //写TX节点地址 
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(unsigned char*)RX_ADDRESS,RX_ADR_WIDTH);  //设置TX节点地址,主要为了使能ACK	  
	
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x00);                                          //使能通道0的自动应答    
	//NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x01);                                      //使能通道0的接收地址  
	//NRF24L01_Write_Reg(NRF24L01_WRITE_REG+SETUP_RETR,0x1a);                                     //设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,CHANNEL);                                       //设置RF通道为40
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0f);                                       //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x0e);                                         //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	Set_NRF24L01_CE;   
  
}









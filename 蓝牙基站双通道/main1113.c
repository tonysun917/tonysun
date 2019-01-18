#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "timer.h"	
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_iwdg.h"
#include "string.h"	
#include "flash.h"
//////////////////////////////////////////////////////////////////
#define HEADER0  0xa0
#define HEADER1  0xa0
#define MAX_DATAPACKET_LEN 296
#define MAX_DATA_LEN 0x9296
#define MAX_TAGDATA_LEN 0x128c
#define UART 4

#define CC2640RST_PORT  GPIOE
#define CC2640RST_PIN  GPIO_Pin_2
#define cc_module_rst_H  GPIO_SetBits(CC2640RST_PORT,CC2640RST_PIN); 
#define cc_module_rst_L GPIO_ResetBits(CC2640RST_PORT,CC2640RST_PIN);

#define CC2640RST_PORT2  GPIOB
#define CC2640RST_PIN2  GPIO_Pin_13
#define cc_module_rst_H2  GPIO_SetBits(CC2640RST_PORT2,CC2640RST_PIN2); 
#define cc_module_rst_L2 GPIO_ResetBits(CC2640RST_PORT2,CC2640RST_PIN2);

#define CC2640RST_PORT3  GPIOB
#define CC2640RST_PIN3  GPIO_Pin_12
#define cc_module_rst_H3  GPIO_SetBits(CC2640RST_PORT3,CC2640RST_PIN3); 
#define cc_module_rst_L3 GPIO_ResetBits(CC2640RST_PORT3,CC2640RST_PIN3);

#define UPDATE_LED_PORT  GPIOB
#define UPDATE_LED_PIN  GPIO_Pin_14
#define cc_module_led_on   GPIO_ResetBits(UPDATE_LED_PORT,UPDATE_LED_PIN);
#define cc_module_led_off  GPIO_SetBits(UPDATE_LED_PORT,UPDATE_LED_PIN);
#define cc_module_led_toggle GPIO_ToggleBits(UPDATE_LED_PORT,UPDATE_LED_PIN);

#define UPDATE_LED_PORT2  GPIOD
#define UPDATE_LED_PIN2  GPIO_Pin_0
#define cc_module_led_on2   GPIO_ResetBits(UPDATE_LED_PORT2,UPDATE_LED_PIN2);
#define cc_module_led_off2  GPIO_SetBits(UPDATE_LED_PORT2,UPDATE_LED_PIN2);
#define cc_module_led_toggle2 GPIO_ToggleBits(UPDATE_LED_PORT2,UPDATE_LED_PIN2);

#define UPDATE_LED_PORT3  GPIOB
#define UPDATE_LED_PIN3  GPIO_Pin_15
#define cc_module_led_on3   GPIO_ResetBits(UPDATE_LED_PORT3,UPDATE_LED_PIN3);
#define cc_module_led_off3  GPIO_SetBits(UPDATE_LED_PORT3,UPDATE_LED_PIN3);
#define cc_module_led_toggle3 GPIO_ToggleBits(UPDATE_LED_PORT3,UPDATE_LED_PIN3);

#define KEY_PORT  GPIOE
#define KEY_PIN  GPIO_Pin_5

#define keyin  GPIO_ReadInputDataBit(KEY_PORT,KEY_PIN)

typedef struct{
	u8 tagbdaddr[6];
}TagMacAddrList;

TagMacAddrList Tagbdaddrlist[100];//ɨ���б����100��
TagMacAddrList Tagbdaddrlist2[100];
TagMacAddrList Tagbdaddrlist3[100];

u8 Tagbdaddrlistlen[3] = 0;//ɨ���б�����
u8 Tagbdaddrlistlentemp[3] = 0;//ɨ���б�����

typedef struct{
  u8 type;
	u8 tagbdaddr[6];
}TagMacAddrTbl;
TagMacAddrTbl TagbdaddrTbl[50];

u8 current_query_state[3] = 0;
u8 online_state_query = 0;
u16 online_state_query_cnt  = 0;
u8 start_state_check = 0;

u16 scanperiod = 6000;

u8 ATMode = 0;
u8 StationID[6];
u8 CurrentIP[4];

void IWDG_Init(u8 prer,u16 rlr);
void IWDG_Feed(void); 
enum
{
	CMD_NONE = 0x00,
	CMD_SCAN = 0xf0,
	CMD_RSSI = 0xf1,
	CMD_BATT = 0xf2,
	CMD_STYLE = 0xf3,
	CMD_CONTENET = 0xf4,
	
	CMD_RESET = 0xe3,
	CMD_DISC = 0xd0,
	CMD_BROADCASTEN = 0xd1,
	CMD_BATTERY = 0xe2,
	CMD_ONLINECHECK = 0Xe4
};

u8 usart3_detect = 0;
u16 usart3_detect_cnt = 0;
u8 usart3_recev_filter = 0;

u8 systemReset[3]; 

u8 wait_connected_indicate[3] = 0;
u8 flush_tag_process_indicate[3] = 0;
u8 flush_tag_30sflg[3] = 0;
u16 flush_tag_30scnt[3] = 0;
u16 tagledindicate[3] = 0;
u16 ledflashfreq[3] = 0;

u16 i_tmp[3];
u8 uartconstate[3];
u16 uartconstatecnt[3];
u8 usartx_state[3];

u8 keyflg = 0;
u16 keycnt = 0;

u8 onlinescanrequest[6] ={0xa0,0xa0,0x00,0x07,0x00};

static u8 gu8DataSend[MAX_TAGDATA_LEN];
static u8 gu8DataSend2[MAX_TAGDATA_LEN];
static u8 gu8DataSend3[MAX_TAGDATA_LEN];

static u8 gu8statereport[22];
static u8 gu8statereport2[22];
static u8 gu8statereport3[22];

u8 uartsend[3];

char *updstart = "start update transmit....";
char *updstatus = "update status:";

char *enterAT = "+++";
char *enterATack = "a";
char *getMAC = "AT+MAC\r";
char *exitAT = "AT+ENTM\r";

static void usart1FuncHandle(void);
static void usart2FuncHandle(void);
static void usart3FuncHandle(void);
static void uart4FuncHandle(void);
static void usart6FuncHandle(void);


void IWDG_Init(uint8_t fprer,uint16_t wr)  
{      
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);				 
	IWDG_SetPrescaler(fprer); 	 
	IWDG_SetReload(wr);   	 
	IWDG_ReloadCounter(); //reload  				 
	IWDG_Enable();                                                             
}
void IWDG_Feed(void)  
{  
  IWDG_ReloadCounter();//reload  
}  
void clr_updReg(void)
{
	flush_tag_process_indicate[0] = 0;		
	tagledindicate[0] = 0;
	flush_tag_30sflg[0] = 0;
	flush_tag_30scnt[0] = 0;
  	
}
void set_updReg(void)
{
	flush_tag_30sflg[0] = 1;	
	flush_tag_30scnt[0] = 0;		
	tagledindicate[0] = 0;
	flush_tag_process_indicate[0] = 1;	
}

void clr_updReg2(void)
{
	flush_tag_process_indicate[1] = 0;		
	tagledindicate[1] = 0;
	flush_tag_30sflg[1] = 0;
	flush_tag_30scnt[1] = 0;	
}
void set_updReg2(void)
{
	flush_tag_30sflg[1] = 1;	
	flush_tag_30scnt[1] = 0;		
	tagledindicate[1] = 0;
	flush_tag_process_indicate[1] = 1;	
}

void clr_updReg3(void)
{
	flush_tag_process_indicate[2] = 0;		
	tagledindicate[2] = 0;
	flush_tag_30sflg[2] = 0;
	flush_tag_30scnt[2] = 0;
}
void set_updReg3(void)
{
	flush_tag_30sflg[2] = 1;	
	flush_tag_30scnt[2] = 0;		
	tagledindicate[2] = 0;
	flush_tag_process_indicate[2] = 1;	
}

static void read_TagMacAddr(void)
{
	Flash_Read8BitDatas(FLASH_USER_START_ADDR,sizeof(TagbdaddrTbl),(u8 *)&TagbdaddrTbl);
}
/**********************************************************************/
static void write_TagMacAddr(void)
{
	IWDG_Feed();
	__disable_irq(); //�����ж�
	Flash_EraseSector(Flash_GetSector(FLASH_USER_START_ADDR));
	Flash_Write8BitDatas(FLASH_USER_START_ADDR,sizeof(TagbdaddrTbl),(u8 *)&TagbdaddrTbl);
	__enable_irq(); 
	IWDG_Feed();
}

int devInfoQuery(u8 *src,TagMacAddrList dst[],u8 len,u8 taglen)
{
	u8 i;
	for(i = 0;i < taglen;i ++)
	{
		if(!memcmp(src,dst[i].tagbdaddr,len))
		{
			return i;
		}
	}
	return -1;
}
static void usart1FuncHandle(void)
{
	switch(USART1_RX_BUF[2]) //�ж���������
	{
		case CMD_NONE:
			
			break;
		case CMD_SCAN:
			memcpy(gu8statereport2,USART1_RX_BUF,5);
			memcpy(&gu8statereport2[5],StationID,6);						
			if(USART1_RX_BUF[11] == 0xff)
			{
				for(u8 i=0;i<Tagbdaddrlistlentemp[1];i++)
				{
					if(Tagbdaddrlist2[i].tagbdaddr[0] != 0x00)
					{
						gu8statereport2[4] = 0x01;
						memcpy(&gu8statereport2[11],Tagbdaddrlist2[i].tagbdaddr,6);	
						gu8statereport2[17] = 0xff;
						gu8statereport2[18] = 0xff;
						gu8statereport2[19] = 0xff;
						gu8statereport2[20] = 0x02;
						usart3_sendData(gu8statereport2,21);//�ϱ���������״̬							
					}
				}				
				clr_updReg2();
				usartx_state[1] = 0;		      //�ͷŴ���ռ��			
				usart3_recev_filter = 0;      //�򿪴���
				current_query_state[1] = 0;   //�����ǰ��ѯ״̬��־
				Tagbdaddrlistlen[1] = 0;
				Tagbdaddrlistlentemp[1] = 0;				
				memset(&gu8statereport2[11],0x00,6);	
				gu8statereport2[4] = 0x00;
				gu8statereport2[17] = 0xff;
				gu8statereport2[18] = 0xff;
				gu8statereport2[19] = 0xff;
				gu8statereport2[20] = 0x00;
				usart3_sendData(gu8statereport2,21);//�ϱ���������״̬					
			}
			else
			{
				memcpy(&gu8statereport2[11],&USART1_RX_BUF[5],9);	
				if(Tagbdaddrlistlen[1] > 0)  
				{ //������б��������ж��Ƿ���Ҫ����������
					int res = devInfoQuery(&USART1_RX_BUF[5],Tagbdaddrlist2,6,Tagbdaddrlistlentemp[1]);
					if(res != -1)
					{		            					
						Tagbdaddrlistlen[1] --;		//�б�������1	
            gu8statereport2[4] = 0x01;						
						gu8statereport2[20] = 0x01;
						usart3_sendData(gu8statereport2,21);	//�ϱ���ǩ״̬
						memset(Tagbdaddrlist2[res].tagbdaddr,0x00,6);						
						if(Tagbdaddrlistlen[1] == 0)   //���������������
						{
							cc_module_rst_H2;	 
							delay_ms(10);
							cc_module_rst_L2;	
							delay_ms(50);									
							clr_updReg2();
							usartx_state[1] = 0;					//�ͷŴ���ռ��
							usart3_recev_filter = 0;
							current_query_state[1] = 0;   //�����ǰɨ��״̬
							Tagbdaddrlistlentemp[1] = 0;  
							memset(&gu8statereport2[11],0x00,6);
							gu8statereport2[4] = 0x00;
							gu8statereport2[17] = 0xff;
							gu8statereport2[18] = 0xff;
							gu8statereport2[19] = 0xff;
							gu8statereport2[20] = 0x00;
							usart3_sendData(gu8statereport2,21);//�ϱ���������״̬										
						}					
					}
				}
				else
				{	
          gu8statereport2[4] = 0x01;					
          gu8statereport2[20] = 0x01;					
				  usart3_sendData(gu8statereport2,21);
				}
			}
			break;
		case CMD_STYLE:
			if(USART1_RX_BUF[11] == 0xff) //���ӳɹ��������
			{
				if(wait_connected_indicate[1] == 1)
				{
					ledflashfreq[1] = 25;
					wait_connected_indicate[1] = 0;
					uartconstate[1] = 1;
					uartconstatecnt[1] = 0;
					flush_tag_30scnt[1] = 0;
					i_tmp[1] = 0;		
				}					
			}			
			else    //���������ֱ���ϱ�
			{	
				clr_updReg2();		
				usartx_state[1] = 0;		//��մ���ռ��			
				usart3_recev_filter = 0;
				memcpy(gu8statereport2,USART1_RX_BUF,5);
				memcpy(&gu8statereport2[5],StationID,6);
				memcpy(&gu8statereport2[11],&USART1_RX_BUF[5],7);
				usart3_sendData(gu8statereport2,18);
			}
			break;
		case CMD_BATTERY:
			memcpy(gu8statereport2,USART1_RX_BUF,5);
			memcpy(&gu8statereport2[5],StationID,6);
			memcpy(&gu8statereport2[11],&USART1_RX_BUF[5],7);
			usart3_sendData(gu8statereport2,18);
			break;
		case CMD_BROADCASTEN:
			usart3_sendData(USART1_RX_BUF,12); /*0xa0 0xa0 0xf7 0x00 0x07 0xb0 0xec 0xc4 0xb1 0xa0 0x80 0x00*/ //5 + 7
			break;				
		default:
			break;
	}
}
static void usart2FuncHandle(void)
{

}


static void usart3FuncHandle(void)
{
	IWDG_Feed();		
	switch(USART3_RX_BUF[2]) //�ж���������
	{
		case CMD_NONE:
			
			break;
		case CMD_SCAN:	
			if(usart3_rx_sta == 0x8010) //ȫ������
			{							
				if(usartx_state[0] == 1 && flush_tag_process_indicate[0] == 0)
				{
					current_query_state[0] = 1;   //��ѯ״̬��λ
					USART3_RX_BUF[5] = USART3_RX_BUF[15];
					memcpy(gu8DataSend,USART3_RX_BUF,6);		
					uart4_sendData(gu8DataSend,6);  //������ѯ
					set_updReg();
					ledflashfreq[0] = 20;
				}
				else if(usartx_state[1] == 1 && flush_tag_process_indicate[1] == 0)
				{
					current_query_state[1] = 1;
					USART3_RX_BUF[5] = USART3_RX_BUF[15];
					memcpy(gu8DataSend2,USART3_RX_BUF,6);						
					usart1_sendData(gu8DataSend2,6);
					set_updReg2();	
					ledflashfreq[1] = 20;						
				}
				else if(usartx_state[2] == 1 && flush_tag_process_indicate[2] == 0)
				{
					current_query_state[2] = 1;
					USART3_RX_BUF[5] = USART3_RX_BUF[15];
					memcpy(gu8DataSend3,USART3_RX_BUF,6);				
					usart6_sendData(gu8DataSend3,6);	
					set_updReg3();	
					ledflashfreq[2] = 20;						
				}	
		  }
      else    //�б�����
			{
				if(usartx_state[0] == 1 && flush_tag_process_indicate[0] == 0)//ͨ��1����
				{
					if(USART3_RX_BUF[4] > 0)   //�ж��б����Ƿ����0
					{
						Tagbdaddrlistlentemp[0] = Tagbdaddrlistlen[0] = USART3_RX_BUF[4];//ȡ���б���
						memcpy((u8 *)&Tagbdaddrlist,&USART3_RX_BUF[15],USART3_RX_BUF[4]*6);//��ȡ��ǩ�б�
					}
					current_query_state[0] = 1;   //��ѯ״̬��λ
					USART3_RX_BUF[5] = USART3_RX_BUF[15];
					memcpy(gu8DataSend,USART3_RX_BUF,6);		
					uart4_sendData(gu8DataSend,6);
					set_updReg();
					ledflashfreq[0] = 20;
				}
				else if(usartx_state[1] == 1 && flush_tag_process_indicate[1] == 0)
				{
					if(USART3_RX_BUF[4] > 0)
					{
						Tagbdaddrlistlentemp[1] = Tagbdaddrlistlen[1] = USART3_RX_BUF[4];
						memcpy((u8 *)&Tagbdaddrlist2,&USART3_RX_BUF[15],USART3_RX_BUF[4]*6);
					}					
					current_query_state[1] = 1;
					USART3_RX_BUF[5] = USART3_RX_BUF[15];
					memcpy(gu8DataSend2,USART3_RX_BUF,6);						
					usart1_sendData(gu8DataSend2,6);
					set_updReg2();	
					ledflashfreq[1] = 20;						
				}
				else if(usartx_state[2] == 1 && flush_tag_process_indicate[2] == 0)
				{
					if(USART3_RX_BUF[4] > 0)
					{
						Tagbdaddrlistlen[2] = USART3_RX_BUF[4];
						Tagbdaddrlistlentemp[2] = Tagbdaddrlistlen[2];
						memcpy((u8 *)&Tagbdaddrlist3,&USART3_RX_BUF[15],USART3_RX_BUF[4]*6);
					}									
					current_query_state[2] = 1;
					USART3_RX_BUF[5] = USART3_RX_BUF[15];
					memcpy(gu8DataSend3,USART3_RX_BUF,6);				
					usart6_sendData(gu8DataSend3,6);	
					set_updReg3();	
					ledflashfreq[2] = 20;						
				}					
			}				
			break;
		case CMD_STYLE:	
			if(usart3_rx_sta == MAX_DATA_LEN) 
			{		
				if(usartx_state[0] == 1 && flush_tag_process_indicate[0] == 0)//ͨ��1����
				{
					memcpy(gu8DataSend,USART3_RX_BUF,5);
					memcpy(&gu8DataSend[5],&USART3_RX_BUF[15],MAX_TAGDATA_LEN - 5);//��ȡͷ����Ϣ								
					uart4_sendData(gu8DataSend,12);	
					wait_connected_indicate[0] = 1;//�ȴ�����ָʾ
					set_updReg();
					ledflashfreq[0] = 50;
				}
				else if(usartx_state[1] == 1 && flush_tag_process_indicate[1] == 0)//ͨ��2����
				{
					memcpy(gu8DataSend2,USART3_RX_BUF,5);
					memcpy(&gu8DataSend2[5],&USART3_RX_BUF[15],MAX_TAGDATA_LEN - 5);
					usart1_sendData(gu8DataSend2,12);	 /*0xa0 0xa0 0xf3 0x00 0x07 0xb0 0xec 0xc4 0xb1 0xa0 0x80 0x00*/ //5 + 7
					wait_connected_indicate[1] = 1;
					set_updReg2();
					ledflashfreq[1] = 50;							
				}
				else if(usartx_state[2] == 1 && flush_tag_process_indicate[2] == 0)//ͨ��3����
				{
					memcpy(gu8DataSend3,USART3_RX_BUF,5);
					memcpy(&gu8DataSend3[5],&USART3_RX_BUF[15],MAX_TAGDATA_LEN - 5);
					usart6_sendData(gu8DataSend3,12);	 /*0xa0 0xa0 0xf1 0x00 0x07 0xb0 0xec 0xc4 0xb1 0xa0 0x80 0x00*/ //5 + 7
					wait_connected_indicate[2] = 1;
					set_updReg3();
					ledflashfreq[2] = 50;									
				}
			}		
			break;
		case CMD_BROADCASTEN:
//				uart4_sendData(USART3_RX_BUF,12); /*0xa0 0xa0 0xf7 0x00 0x07 0xb0 0xec 0xc4 0xb1 0xa0 0x80 0x00*/ //5 + 7
			break;
		case CMD_ONLINECHECK:			
			if(USART3_RX_BUF[11] == 0x01)
			{
//					  online_state_query = 1;
				start_state_check = 0;
				online_state_query_cnt = 0;
//						scanperiod = USART3_RX_BUF[12];
			}
			else
			{
				online_state_query = 0;
				start_state_check = 0;
				online_state_query_cnt = 0;	
				IWDG_Feed();
				if(current_query_state[0] == 1)//�����ǰ���ڲ�ѯ״̬�������˳�
				{						
					current_query_state[0] = 0;
					cc_module_rst_H;	
					delay_ms(10);
					cc_module_rst_L;	
					delay_ms(80);
					clr_updReg();			
				}
				if(current_query_state[1] == 1)
				{
					current_query_state[1] = 0;
					cc_module_rst_H2;	
					delay_ms(10);
					cc_module_rst_L2;	
					delay_ms(80);
					clr_updReg2();			
				}
				if(current_query_state[2] == 1)
				{
					current_query_state[2] = 0;
					cc_module_rst_H3;	
					delay_ms(10);
					cc_module_rst_L3;
					delay_ms(80);
					clr_updReg3();			
				}
				USART3_RX_BUF[11] = 0x00;
				usart3_sendData(USART3_RX_BUF,12);						
			}				
			break;
		case CMD_RESET:  //��������
			cc_module_rst_H;	
			cc_module_rst_H2;
			cc_module_rst_H3;
			delay_ms(10);
			cc_module_rst_L;	
			cc_module_rst_L2;	
			cc_module_rst_L3;	
      delay_ms(10);		
			__set_FAULTMASK(1);
			NVIC_SystemReset();				
      break;			
		default:
			memset(USART3_RX_BUF,0x00,USART3_REC_LEN);
			break;
	}
}

static void uart4FuncHandle(void)
{		
	switch(UART4_RX_BUF[2]) //�ж���������
	{
		case CMD_NONE:
			
			break;
		case CMD_SCAN:
			memcpy(gu8statereport,UART4_RX_BUF,5);
			memcpy(&gu8statereport[5],StationID,6);						
			if(UART4_RX_BUF[11] == 0xff)  //ȫ��ɨ�����
			{
				for(u8 i=0;i<Tagbdaddrlistlentemp[0];i++)
				{
					if(Tagbdaddrlist[i].tagbdaddr[0] != 0x00)
					{
						memcpy(&gu8statereport[11],Tagbdaddrlist[i].tagbdaddr,6);	
            gu8statereport[4] = 0x01;					
            gu8statereport[20] = 0x02;								
						gu8statereport[17] = 0xff;
						gu8statereport[18] = 0xff;
						gu8statereport[19] = 0xff;
						usart3_sendData(gu8statereport,21);//�ϱ���������״̬							
					}
				}				
				clr_updReg();
				usartx_state[0] = 0;					
				usart3_recev_filter = 0;
				current_query_state[0] = 0;   //�����ǰɨ��״̬
				Tagbdaddrlistlen[0] = 0;
				Tagbdaddrlistlentemp[0] = 0;
				memset(&gu8statereport[11],0x00,6);
				gu8statereport[4] = 0x00;					
				gu8statereport[20] = 0x00;					
				gu8statereport[17] = 0xff;
				gu8statereport[18] = 0xff;
				gu8statereport[19] = 0xff;
				usart3_sendData(gu8statereport,21);					
			}
			else           //�б�ɨ��
			{
				memcpy(&gu8statereport[11],&UART4_RX_BUF[5],9);	
				if(Tagbdaddrlistlen[0] > 0)// �б���������0
				{
					int res = devInfoQuery(&UART4_RX_BUF[5],Tagbdaddrlist,6,Tagbdaddrlistlentemp[0]);
					if(res != -1)
					{		            					
						Tagbdaddrlistlen[0] --;				
            gu8statereport[4] = 0x01;					
            gu8statereport[20] = 0x01;							
						usart3_sendData(gu8statereport,21);	//�ϱ��������
						memset(Tagbdaddrlist[res].tagbdaddr,0x00,6);	
						if(Tagbdaddrlistlen[0] == 0)//�б��ѯ��������˳�
						{
							cc_module_rst_H;	 
							delay_ms(10);
							cc_module_rst_L;	
							delay_ms(50);									
							clr_updReg();
							usartx_state[0] = 0;	//�������ռ��				
							usart3_recev_filter = 0; //�رմ��ڹ���
							current_query_state[0] = 0;   //�����ǰɨ��״̬
							Tagbdaddrlistlentemp[0] = 0;
							memset(&gu8statereport[11],0x00,6);
							gu8statereport[4] = 0x00;					
              gu8statereport[20] = 0x00;	
							gu8statereport[17] = 0xff;
							gu8statereport[18] = 0xff;
							gu8statereport[19] = 0xff;
							usart3_sendData(gu8statereport,21);									
						}					
					}
				}
				else  //����Ĭ��Ϊȫ��������ֱ���ϱ�
				{		
					gu8statereport[4] = 0x01;					
          gu8statereport[20] = 0x01;						
				  usart3_sendData(gu8statereport,21);
				}
			}
			break;
		case CMD_STYLE:	
			if(UART4_RX_BUF[11] == 0xff) //���ӳɹ��������
			{
				if(wait_connected_indicate[0] == 1)//��������
				{
					ledflashfreq[0] = 25;
					wait_connected_indicate[0] = 0;
					uartconstate[0] = 1;
					uartconstatecnt[0] = 0;
					flush_tag_30scnt[0] = 0;
					i_tmp[0] = 0;	
				}					
			}			
			else    //���������ֱ���ϱ�
			{	
				clr_updReg();			
				usartx_state[0] = 0;					
				usart3_recev_filter = 0;
				memcpy(gu8statereport,UART4_RX_BUF,5);
				memcpy(&gu8statereport[5],StationID,6);
				memcpy(&gu8statereport[11],&UART4_RX_BUF[5],7);
				usart3_sendData(gu8statereport,18);
			}
			break;
		case CMD_BATTERY:
			memcpy(gu8statereport,UART4_RX_BUF,5);
			memcpy(&gu8statereport[5],StationID,6);
			memcpy(&gu8statereport[11],&UART4_RX_BUF[5],7);
			usart3_sendData(gu8statereport,18);
			break;				
		case CMD_BROADCASTEN:
//				usart3_sendData(UART4_RX_BUF,12); /*0xa0 0xa0 0xd2 0x00 0x07 0xb0 0xec 0xc4 0xb1 0xa0 0x80 0x00*/ //5 + 7
			break;			
		default:
			memset(UART4_RX_BUF,0x00,UART4_REC_LEN);
			break;
	}
}
static void usart6FuncHandle(void)
{
	switch(USART6_RX_BUF[2]) //�ж���������
	{
		case CMD_NONE:
			
			break;
		case CMD_SCAN:
			memcpy(gu8statereport3,USART6_RX_BUF,5);
			memcpy(&gu8statereport3[5],StationID,6);						
			if(USART6_RX_BUF[11] == 0xff)
			{
				for(u8 i=0;i<Tagbdaddrlistlentemp[2];i++)
				{
					if(Tagbdaddrlist3[i].tagbdaddr[0] != 0x00)
					{
						memcpy(&gu8statereport3[11],Tagbdaddrlist3[i].tagbdaddr,6);	
						gu8statereport3[4] = 0x01;					
            gu8statereport3[20] = 0x02;	
						gu8statereport3[17] = 0xff;
						gu8statereport3[18] = 0xff;
						gu8statereport3[19] = 0xff;
						usart3_sendData(gu8statereport3,21);//�ϱ���������״̬							
					}
				}					
				clr_updReg3();
				usartx_state[2] = 0;					
				usart3_recev_filter = 0;
				current_query_state[3] = 0;
				memset(&gu8statereport3[11],0x00,6);
				gu8statereport3[4] = 0x00;					
        gu8statereport3[20] = 0x00;				
				gu8statereport3[17] = 0xff;
				gu8statereport3[18] = 0xff;
				gu8statereport3[19] = 0xff;
				usart3_sendData(gu8statereport3,21);					
			}
			else
			{
				memcpy(&gu8statereport3[11],&USART6_RX_BUF[5],9);	
				if(Tagbdaddrlistlen[2] > 0)
				{
					int res = devInfoQuery(&USART6_RX_BUF[5],Tagbdaddrlist3,6,Tagbdaddrlistlentemp[2]);
					if(res != -1)
					{		            					
						Tagbdaddrlistlen[2] --;			
						gu8statereport3[4] = 0x01;					
            gu8statereport3[20] = 0x01;						
						usart3_sendData(gu8statereport3,21);
            memset(Tagbdaddrlist3[res].tagbdaddr,0x00,6);						
						if(Tagbdaddrlistlen[2] == 0)
						{
							cc_module_rst_H3;	  //����ֹͣ
							delay_ms(10);
							cc_module_rst_L3;	
							delay_ms(50);									
							clr_updReg3();
							usartx_state[2] = 0;					
							usart3_recev_filter = 0;
							current_query_state[2] = 0;   //�����ǰɨ��״̬
							Tagbdaddrlistlentemp[2] = 0;
							memset(&gu8statereport3[11],0x00,6);
						  gu8statereport3[4] = 0x00;					
              gu8statereport3[20] = 0x00;							
							gu8statereport3[17] = 0xff;
							gu8statereport3[18] = 0xff;
							gu8statereport3[19] = 0xff;
							usart3_sendData(gu8statereport3,21);									
						}					
					}
				}
				else
				{				  
					gu8statereport3[4] = 0x01;					
					gu8statereport3[20] = 0x01;					
				  usart3_sendData(gu8statereport3,21);
				}
			}
			break;
		case CMD_STYLE:
			if(USART6_RX_BUF[11] == 0xff) //���ӳɹ��������
			{
				if(wait_connected_indicate[2] == 1)
				{
					ledflashfreq[2] = 25;
					wait_connected_indicate[2] = 0;
					uartconstate[2] = 1;
					uartconstatecnt[2] = 0;
					flush_tag_30scnt[2] = 0;
					i_tmp[2] = 0;			
				}					
			}			
			else    //���������ֱ���ϱ�
			{	
				clr_updReg3();	
				usartx_state[2] = 0;					
				usart3_recev_filter = 0;
				memcpy(gu8statereport3,USART6_RX_BUF,5);
				memcpy(&gu8statereport3[5],StationID,6);
				memcpy(&gu8statereport3[11],&USART6_RX_BUF[5],7);
				usart3_sendData(gu8statereport3,18);
			}
			break;
		case CMD_BATTERY:
			memcpy(gu8statereport3,USART6_RX_BUF,5);
			memcpy(&gu8statereport3[5],StationID,6);
			memcpy(&gu8statereport3[11],&USART6_RX_BUF[5],7);
			usart3_sendData(gu8statereport3,18);
			break;						
		case CMD_BROADCASTEN:
//			usart3_sendData(USART6_RX_BUF,12); /*0xa0 0xa0 0xf7 0x00 0x07 0xb0 0xec 0xc4 0xb1 0xa0 0x80 0x00*/ //5 + 7
			break;				
		default:
			break;
	}
}

void gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIODʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIODʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //ʹ��GPIODʱ��
	
  GPIO_InitStructure.GPIO_Pin = UPDATE_LED_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(UPDATE_LED_PORT,&GPIO_InitStructure); 
	
  GPIO_InitStructure.GPIO_Pin = UPDATE_LED_PIN2; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(UPDATE_LED_PORT2,&GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = UPDATE_LED_PIN3; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(UPDATE_LED_PORT3,&GPIO_InitStructure); 	
	
  GPIO_InitStructure.GPIO_Pin = CC2640RST_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CC2640RST_PORT,&GPIO_InitStructure); 
	
  GPIO_InitStructure.GPIO_Pin = CC2640RST_PIN2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(CC2640RST_PORT2,&GPIO_InitStructure); 
	
  GPIO_InitStructure.GPIO_Pin = CC2640RST_PIN3; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CC2640RST_PORT3,&GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = KEY_PIN; //GPIOC0  led
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(KEY_PORT,&GPIO_InitStructure); //��ʼ��PC0			
}

int main(void)
{
	delay_init(100);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�

	uart1_init(115200);
	uart2_init(115200);
	uart3_init(115200);
	uart4_init(115200);
	uart6_init(115200);

	gpio_init();
	
	TIM7_Int_Init(10000-1,72-1);		//1ms�ж�
  delay_ms(3000);  //��ʱ�ȴ�����ģ���ʼ�����
	ATMode = 1;
  usart3_sendData(enterAT,3);//��ȡMAC��ַ
	if(ATMode == 1)
	{
		while(1)
		{
			cc_module_led_on;
			delay_ms(1000);
			cc_module_led_off;
			delay_ms(1000);
			if(ATMode == 0)
			{
				break;
			}
		};//error
	}
	cc_module_led_on;	
	cc_module_led_on2;	
	cc_module_led_on3;	
  delay_ms(1000);
	cc_module_led_off;	
	cc_module_led_off2;
	cc_module_led_off3;	
//  read_TagMacAddr();	
	IWDG_Init(4,500);
	if(RCC_GetFlagStatus(RCC_FLAG_SFTRST))//�����λ�ϱ�
	{
		u8 sendrstok[12];
		sendrstok[0] = 0xa0;
		sendrstok[1] = 0xa0;
		sendrstok[2] = CMD_RESET;
		sendrstok[3] = 0x00;
		sendrstok[4] = 0x00;
		memcpy(&sendrstok[5],StationID,6);
		sendrstok[11] = 0x00;
		usart3_sendData(sendrstok,12);
	}
  while(1){
		IWDG_Feed();		
		if(keyflg == 1)  //��������ȫ����λ
		{
			keyflg = 0;
			cc_module_rst_H;	
			cc_module_rst_H2;	
			cc_module_rst_H3;	
			delay_ms(10);
			cc_module_rst_L;
			cc_module_rst_L2;
			cc_module_rst_L3;	
      delay_ms(10);		
			__set_FAULTMASK(1);
			NVIC_SystemReset();	      			
		}
		
		if(systemReset[0] == 1)
		{
			systemReset[0] = 0;
			cc_module_rst_H;	
			delay_ms(10);
			cc_module_rst_L;
      delay_ms(50);			
			if(current_query_state[0] == 1)
			{
				current_query_state[0] = 0;
				memcpy(gu8statereport,gu8DataSend,5);
				gu8statereport[4] = 0x00;
				memcpy(&gu8statereport[5],StationID,6);
				memset(&gu8statereport[11],0x00,6);
				gu8statereport[17] = 0xff;
				gu8statereport[18] = 0xff;
				gu8statereport[19] = 0xff;				
				gu8statereport[20] = 0x03;
				usart3_sendData(gu8statereport,21);//ɨ�賬ʱ�ϱ�					
			}
			else
			{
				memcpy(gu8statereport,gu8DataSend,5);
				gu8statereport[4] = 0x00;
				memcpy(&gu8statereport[5],StationID,6);
				memset(&gu8statereport[11],0x00,7);
				gu8statereport[17] = 0x03;
				usart3_sendData(gu8statereport,18);//ɨ�賬ʱ�ϱ�	
			}				
		}
		
		if(systemReset[1] == 1)
		{
			systemReset[1] = 0;
			cc_module_rst_H2;	
			delay_ms(10);
			cc_module_rst_L2;
      delay_ms(50);			

			if(current_query_state[1] == 1)
			{
				current_query_state[1] = 0;
				memcpy(gu8statereport2,gu8DataSend2,5);
				gu8statereport2[4] = 0x00;
				memcpy(&gu8statereport2[5],StationID,6);
				memset(&gu8statereport2[11],0x00,6);
				gu8statereport2[17] = 0xff;
				gu8statereport2[18] = 0xff;
				gu8statereport2[19] = 0xff;				
				gu8statereport2[20] = 0x03;
				usart3_sendData(gu8statereport2,21);//ɨ�賬ʱ�ϱ�					
			}		
			else
			{
				memcpy(gu8statereport2,gu8DataSend2,5);
				gu8statereport2[4] = 0x00;
				memcpy(&gu8statereport2[5],StationID,6);
				memset(&gu8statereport2[11],0x00,7);
				gu8statereport2[17] = 0x03;
				usart3_sendData(gu8statereport2,18);//ɨ�賬ʱ�ϱ�	
			}				
		}

		if(systemReset[2] == 1)
		{
			systemReset[2] = 0;
			cc_module_rst_H3;	
			delay_ms(10);
			cc_module_rst_L3;
      delay_ms(50);			

			if(current_query_state[2] == 1)
			{
				current_query_state[2] = 0;
				memcpy(gu8statereport3,gu8DataSend3,5);
				gu8statereport3[4] = 0x00;
				memcpy(&gu8statereport3[5],StationID,6);
				memset(&gu8statereport3[11],0x00,6);
				gu8statereport3[17] = 0xff;
				gu8statereport3[18] = 0xff;
				gu8statereport3[19] = 0xff;				
				gu8statereport3[20] = 0x03;
				usart3_sendData(gu8statereport3,21);//ɨ�賬ʱ�ϱ�					
			}		
			else
			{
				memcpy(gu8statereport3,gu8DataSend3,5);
				gu8statereport3[4] = 0x00;
				memcpy(&gu8statereport3[5],StationID,6);
				memset(&gu8statereport3[11],0x00,7);
				gu8statereport3[17] = 0x03;
				usart3_sendData(gu8statereport3,18);//ɨ�賬ʱ�ϱ�	
			}				
		}		
    if(usart3_rx_sta & 0x8000)
		{			
			usart3FuncHandle();
			usart3_rx_sta &= 0x0000;
#if defined  (UART_THREE)								
			if(usartx_state[0] == 0 || usartx_state[1] == 0 || usartx_state[2] == 0)//ÿ����һ�β�ѯ���Ƿ��п���ͨ��
			{						
				char *channel_busy = "channel is idle...";
				usart3_sendData(channel_busy,18);
			}
			else
			{						
				char *channel_busy = "channel is busy...";
				usart3_sendData(channel_busy,18);
			}					
#elif defined (UART_DOUBLE)
			if(usartx_state[0] == 0 || usartx_state[1] == 0)//ÿ����һ�β�ѯ���Ƿ��п���ͨ��
			{						
				char *channel_busy = "channel is idle...";
				usart3_sendData(channel_busy,18);
			}			
			else
			{						
				char *channel_busy = "channel is busy...";
				usart3_sendData(channel_busy,18);
			}			
#endif			
		}
    if(usart6_rx_sta & 0x8000)
		{
			usart6_rx_sta &= 0x0000;
      usart6FuncHandle();
		}	
    if(uart4_rx_sta & 0x8000)
		{
			uart4_rx_sta &= 0x0000;
      uart4FuncHandle();
		}	
    if(usart1_rx_sta & 0x8000)
		{
			usart1_rx_sta &= 0x0000;
      usart1FuncHandle();
		}	
    if(uartsend[0] == 1)
		{
			uartsend[0] = 0;
			if(i_tmp[0] < MAX_DATAPACKET_LEN)
			{		
				uart4_sendData(gu8DataSend+12+(16*i_tmp[0]),16);
			}
			else
			{
				i_tmp[0] = 0;
				uartconstate[0] = 0;
        flush_tag_30scnt[0] = 0;				
				cc_module_led_off;	
				usart3_recev_filter = 0;					
			}
			i_tmp[0] ++;				
		}			
    if(uartsend[1] == 1)
		{
			uartsend[1] = 0;
			if(i_tmp[1] < MAX_DATAPACKET_LEN)
			{		
				usart1_sendData(gu8DataSend2+12+(16*i_tmp[1]),16);
			}
			else
			{
				i_tmp[1] = 0;
				uartconstate[1] = 0;
        flush_tag_30scnt[1] = 0;				
				cc_module_led_off2;	
				usart3_recev_filter = 0;					
			}
			i_tmp[1] ++;				
		}	
    if(uartsend[2] == 1)
		{
			uartsend[2] = 0;
			if(i_tmp[2] < MAX_DATAPACKET_LEN)
			{		
				usart6_sendData(gu8DataSend3+12+(16*i_tmp[2]),16);
			}
			else
			{
				i_tmp[2] = 0;
				uartconstate[2] = 0;
        flush_tag_30scnt[2] = 0;				
				cc_module_led_off3;	
				usart3_recev_filter = 0;					
			}
			i_tmp[2] ++;				
		}
    if(start_state_check == 1)
		{
			start_state_check = 0;
			if(usartx_state[0] == 1 && flush_tag_process_indicate[0] == 0)
			{	
				current_query_state[0] = 1;
				uart4_sendData(onlinescanrequest,6);
				set_updReg();
				ledflashfreq[0] = 20;
			}
			else if(usartx_state[1] == 1 && flush_tag_process_indicate[1] == 0)
			{					
				current_query_state[1] = 1;
				usart1_sendData(onlinescanrequest,6);
				set_updReg2();	
				ledflashfreq[1] = 20;						
			}
			else if(usartx_state[2] == 1 && flush_tag_process_indicate[2] == 0)
			{		
				current_query_state[2] = 1;
				usart6_sendData(onlinescanrequest,6);	
				set_updReg3();	
				ledflashfreq[2] = 20;						
			}				
		}			
	}
}

void TIM7_IRQHandler(void)
{ 	
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)//�Ǹ����ж�
	{
		if(!keyin)
		{
			keycnt ++;
			if(keycnt > 5000)
			{
				keycnt = 0;
			}
		}
		else
	  {
			if(keycnt >= 10 && keycnt <= 2000)
			{
        keyflg = 1;
				keycnt = 0;
			}
			else if(keycnt > 2000 && keycnt <= 5000)
			{
				keyflg = 2;
				keycnt = 0;
			}
			else if(keycnt > 5000)
			{
				keycnt = 0;
			}
		}
		if(flush_tag_30sflg[0] == 1)
		{
			flush_tag_30scnt[0]++;
			if(flush_tag_30scnt[0] > 3000) //30sɨ�賬ʱ
			{
				clr_updReg();
				usartx_state[0] = 0;
				wait_connected_indicate[0] = 0;	
			  cc_module_led_off;
				i_tmp[0] = 0;
        uartconstate[0] = 0;
				uartconstatecnt[0] = 0;		
				usart3_recev_filter = 0;	
				systemReset[0] = 1;//��ֹϵͳ�쳣���и�λ
			}
		}	

		if(flush_tag_30sflg[1] == 1)
		{
			flush_tag_30scnt[1]++;
			if(flush_tag_30scnt[1] > 3000) //30sɨ�賬ʱ
			{
				clr_updReg2();
				usartx_state[1] = 0;
				wait_connected_indicate[1] = 0;	
			  cc_module_led_off2;
				i_tmp[1] = 0;
        uartconstate[1] = 0;
				uartconstatecnt[1] = 0;		
				usart3_recev_filter = 0;		
				systemReset[1] = 1;
			}
		}	

		if(flush_tag_30sflg[2] == 1)
		{
			flush_tag_30scnt[2]++;
			if(flush_tag_30scnt[2] > 3000) //30sɨ�賬ʱ
			{
				clr_updReg3();
				usartx_state[2] = 0;
				wait_connected_indicate[2] = 0;	
			  cc_module_led_off3;
				i_tmp[2] = 0;
        uartconstate[2] = 0;
				uartconstatecnt[2] = 0;		
				usart3_recev_filter = 0;			
				systemReset[2] = 1;
			}
		}	
		
		
		if(flush_tag_process_indicate[0] == 1)
		{
			tagledindicate[0]++;
			if(tagledindicate[0] > ledflashfreq[0])
			{
				tagledindicate[0] = 0;
			  cc_module_led_toggle;
			}
		}
		
		if(flush_tag_process_indicate[1] == 1)
		{
			tagledindicate[1]++;
			if(tagledindicate[1] > ledflashfreq[1])
			{
				tagledindicate[1] = 0;
			  cc_module_led_toggle2;
			}
		}

		if(flush_tag_process_indicate[2] == 1)
		{
			tagledindicate[2]++;
			if(tagledindicate[2] > ledflashfreq[2])
			{
				tagledindicate[2] = 0;
			  cc_module_led_toggle3;
			}
		}
		
		
		if(uartconstate[0] == 1)  //��ʱ�������ݵ�ģ��
		{
			uartconstatecnt[0] ++;
			if(uartconstatecnt[0] > 2)
			{
				uartconstatecnt[0] = 0;
        uartsend[0] = 1;	
		  }      	
		}
		
		if(uartconstate[1] == 1)
		{
			uartconstatecnt[1] ++;
			if(uartconstatecnt[1] > 2)
			{
				uartconstatecnt[1] = 0;
        uartsend[1] = 1;
		  }      	
		}

		if(uartconstate[2] == 1)
		{
			uartconstatecnt[2] ++;
			if(uartconstatecnt[2] > 2)
			{
				uartconstatecnt[2] = 0;
        uartsend[2] = 1;
		  }      	
		}

		
   
		if(usart3_detect == 1)  //���չ���
		{
			usart3_detect_cnt ++;
			if(usart3_detect_cnt > 80)
			{
				cc_module_led_toggle;
				usart3_detect_cnt = 0;
				usart3_detect = 0;
				usart3_rx_sta = 0;
			  char *data_err = "data recev err....";			
		    usart3_sendData(data_err,18);				
			}
		}			
		
		if(online_state_query == 1)
		{
			online_state_query_cnt ++;
			if(online_state_query_cnt > scanperiod) //1���� ���һ��
			{
				online_state_query_cnt = 0;
				if(usartx_state[0] == 0)
				{					
					usartx_state[0] = 1;
					start_state_check = 1;
				}
#if defined (UART_DOUBLE)				
				if(usartx_state[1] == 0)
				{
					usartx_state[1] = 1;
					start_state_check = 1;
				}
#elif defined (UART_TRIPLE)	
				if(usartx_state[1] == 0)
				{
					usartx_state[1] = 1;
					start_state_check = 1;
				}				
        if(usartx_state[2] == 0)
				{
          usartx_state[2] = 1;	
          start_state_check = 1;					
				}	
#endif				
			}
		}
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);  //���TIM7�����жϱ�־    
	}	    
}
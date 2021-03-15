#include "sys.h"
#include "usart.h"   
#include "usart3.h"   
#include "timer.h"
#include "touch.h" 
#include "open_tel_mavlink.h"
#include "mavlink_usart_fifo.h"
#include "hud.h"
#include "lcd.h"
#include "voice.h"
#include "input.h"
#include "function.h"
#include "adc.h"
#include "xuan.h"
#include "delay.h"
#include "24l01.h"
#include "24cxx.h"
#include "iic.h"
#include "channel.h"
#include "rc.h"
#include "mission.h"
#include "rc_function.h"
#include "ship.h"
#include "desk.h"
#include "data.h"
#include "mavlink_setup.h"
#include "game.h"
#include "setup.h"

#include "spi.h"
#include "spi2.h"
#include "malloc.h" 
#include "sdio_sdcard.h"  
#include "ff.h" 
#include "exfuns.h"    
#include "fontupd.h"
#include "text.h"	
#include "piclib.h"	
#include "string.h"	
#include "math.h"	
#include "w25qxx.h" 

#include "mp4.h" 
#include "mpu6050.h"
#include "wm8978.h"	 
#include "videoplayer.h" 

#include "sram.h"


#if GUI
#include "GUI.h"
#include "WM.h"
#endif

#define int16 short int

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////µÿ√Ê’æœ‡πÿ≤Œ ˝

u8 mission_flag=0;
float mission_position[100][3];
u8 text_flag=0;
u8 mission_receive_waypoint_flag=0;
u8 status=standby;
u8 auto_takeoff_flag=0;
u8 auto_mission_flag=0;
u8 auto_waypoint_flag=0;
/////////////////////////////////////////////œµÕ≥œ‡πÿ≤Œ ˝
u8 led_time0=15;
u8 led_time=10;
u8 auto_led=0;
u32 second_sum=0;
u8 touch_flag=0;
u8 finish_1hz=0,finish_2hz=0,finish_5hz=0,finish_10hz=0,finish_20hz=0,finish_33hz=0,finish_50hz=0,finish_100hz=0;
u8 update_1hz_finish=0,status_bar_flash_flag=0,time_second=0,time_minute=0,time_hour=0;
u8 time_second2=0,time_minute2=0,time_hour2=0;
u8 timer_use1=1;
u8 timer_use2=1;
u8 vol_alarm1=1;
u8 vol_alarm2=1;
u8 vol_alarm3=1;
u8 video_playing=0;
u16 posx=0,posy=0;
u8 signal=0;
u8 rx_signal=0;
u8 tx_signal=0;
u32 time_100ms=0;
u16 bat_adc;
u8 menu_flag=0;
u8 first_splash=1;
u8 LEFT=0,RIGHT=0,UP=0,DOWN=0,PRESS1=0;
u8 OK=0,  ESC=0, ADD=0,LESS=0,PRESS2=0;
u8 LLE=0,LRI=0,LUP=0,LDN=0,LOK=0;
u8 RLE=0,RRI=0,RUP=0,RDN=0,ROK=0;
u8 power_oning=0;
u16 fps_temp=0,fps=0;
u8 rocker_use=0;
u8 zd_flag=0;
u8 point_color_flag=0;
u8 back_color_flag=0;
u8 power_on_flag=0;
u8 power_on_voice=0;
/////////////////////////////////////////“£øÿœ‡πÿ≤Œ ˝
u8 mix_tri=1;
u8 mix_v  =1;
u8 mix_delta=1;
u8 ppm_flag=0;
u8 lost_flag=1;
u8 timer_viberation=1;
u8 timer_voice=1;
u8 vol_viberation=1;
u8 vol_voice=1;
u8 ppm_map[8]={1,2,3,4,5,6,7,8};
u16 ppm_in[8]={1500,1500,1500,1500,1500,1500,1500,1500};
//u16 lost_value[8]={1500,1500,1500,1500,1500,1500,1500,1500};
const int16 ori_data[5][16]={
{0,0,0,0,0,0,0,100,100,100,100,100,100,100,100,100},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,-100,-100,-100,-100,-100,-100,-100,-100,-100},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}
};

u16 ch_input[25]={1500,1500,1500,1500,1500,1500,1500,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};
u16 CH_value[16]={1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500};
u16 NRF_value[16]={1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500};
int16 ch_min[16]={0,0,0,0,0,0,0,100,100,100,100,100,100,100,100,100};//31-62
int16 ch_mid[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//63-94
int16 ch_max[16]={0,0,0,0,0,0,0,-100,-100,-100,-100,-100,-100,-100,-100,-100};//95-126
char rev[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//127-142
u8 map[16]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};//143-158


int16 adc_min[10]={0,0,0,0,0,0,00,0,0};
int16 adc_mid[10]={2047,2047,2047,2047,2047,2047,2047,2047,2047,2047};
int16 adc_max[10]={4095,4095,4095,4095,4095,4095,4095,4095,4095,4095};



float battery=0;
float rx_battery=5.20;
float aux_battery=23.8;
////////////////////////////////////////Œﬁœﬂœ‡πÿ≤Œ ˝
u8 tx_buf[16];
u8 nrf_out=0;
u8 address_0[5]={'L','O','V','E','!'};
u8 address[5]={11,22,33,44,55};
u8 hopping[5]={11,22,33,44,55};
u8 rx_data[16];
u8 back_data=1;
u8 ppm_out=0;
///////////////////////////////////////////////////////// ˝¥´≤Œ ˝
u8 tele_flag=0;
u8 nrf_use=1;
extern u8 model_name[40];
extern u8 model_current;

u8 first_power_on=1;


void message(u8 *p)
{
	Fill_bar(240,160,100,20,6,WHITE);
	Draw_bar(240,160,100,20,6,BLACK);
	Show_Str_color(188,148,200,100,p,24,1,BLACK,WHITE);	
	delayms(500);
	first_splash=1;
}

void factory()
{
	u8 i;
	message((u8*)"∏¥Œª÷–..");
	for(i=0;i<4;i++)AT24CXX_WriteLenByte(2*i,0,     2);//adc_min
	for(i=0;i<4;i++)AT24CXX_WriteLenByte(2*i+8,2047,2);//adc_mid
	for(i=0;i<4;i++)AT24CXX_WriteLenByte(2*i+16,4095,2);
	AT24CXX_WriteOneByte(24,0);//nrf_out
	AT24CXX_WriteOneByte(25,0);//power_video
	AT24CXX_WriteOneByte(26,0);//power_voice
	AT24CXX_WriteOneByte(27,1);//nrf_use
	AT24CXX_WriteOneByte(28,0);//auto_led
	//AT24CXX_WriteLenByte(29,0,4);//user_code
	AT24CXX_WriteOneByte(33,20);//led_time
	AT24CXX_WriteOneByte(34,1);//factory
	AT24CXX_WriteLenByte(35,666,2);//35-36
	AT24CXX_WriteOneByte(37,0);//back_light
	AT24CXX_WriteOneByte(38,0);//ppm_out
	model_current=0;
	model_reset();
	message((u8*)"∏¥Œª≥…π¶");
}
extern int16 vol;
extern u8 back_data;
void system_data_read()
{
	u8 i=0;
	if(AT24CXX_ReadOneByte(34)!=1)factory();
	for(i=0;i<4;i++)adc_min[i]=AT24CXX_ReadLenByte(2*i,   2);
	for(i=0;i<4;i++)adc_mid[i]=AT24CXX_ReadLenByte(2*i+8, 2);
	for(i=0;i<4;i++)adc_max[i]=AT24CXX_ReadLenByte(2*i+16,2);
	nrf_out=AT24CXX_ReadOneByte(24);
	power_on_flag=AT24CXX_ReadOneByte(25);
	power_on_voice=AT24CXX_ReadOneByte(26);
	nrf_use=AT24CXX_ReadOneByte(27);
	auto_led=AT24CXX_ReadOneByte(28);	//////29-30-31-32	
	led_time0=AT24CXX_ReadOneByte(33);
	vol=AT24CXX_ReadLenByte(35,2);//35-36
	back_data=AT24CXX_ReadOneByte(37);
	ppm_out=AT24CXX_ReadOneByte(38);
	if(ppm_out)start();
	//if(AT24CXX_ReadOneByte(39)==0)input_mapping();
}


void show_sdcard_info(void)
{
	static char temp[20];
	switch(SDCardInfo.CardType)
	{
		case SDIO_STD_CAPACITY_SD_CARD_V1_1:Show_Str(0,60,200,16,(u8*)"SDø®¿‡–Õ£∫SDSC V1.1",16,0);	break;
		case SDIO_STD_CAPACITY_SD_CARD_V2_0:Show_Str(0,60,200,16,(u8*)"SDø®¿‡–Õ£∫SDSC V2.0",16,0);	break;
		case SDIO_HIGH_CAPACITY_SD_CARD:    Show_Str(0,60,200,16,(u8*)"SDø®¿‡–Õ£∫SDHC V1.1",16,0);	break;
		case SDIO_MULTIMEDIA_CARD:          Show_Str(0,60,200,16,(u8*)"SDø®¿‡–Õ£∫MMC Crad",16,0);	break;
	}	
	sprintf(temp,"SDø®÷∆‘Ï…ÃID:%d\r\n",SDCardInfo.SD_cid.ManufacturerID);	//÷∆‘Ï…ÃID
	Show_Str(0,80,200,16,(u8*)temp,16,0);
 	sprintf(temp,"SDø®œ‡∂‘µÿ÷∑:%d\r\n",SDCardInfo.RCA);								//ø®œ‡∂‘µÿ÷∑
	Show_Str(0,100,200,16,(u8*)temp,16,0);
	sprintf(temp,"SDø®»›¡ø:%d MB\r\n",(u32)(SDCardInfo.CardCapacity>>20));	//œ‘ æ»›¡ø
	Show_Str(0,120,200,16,(u8*)temp,16,0);
 	sprintf(temp,"SDø®øÈ¥Û–°:%d\r\n\r\n",SDCardInfo.CardBlockSize);			//œ‘ æøÈ¥Û–°
	Show_Str(0,140,200,16,(u8*)temp,16,0);
}
void show_sdcard_info2(void)
{
	static char temp[20];
	switch(SDCardInfo.CardType)
	{
		case SDIO_STD_CAPACITY_SD_CARD_V1_1:Show_Str(0,60,200,16,(u8*)"SD TYPE:SDSC V1.1",16,0);	break;
		case SDIO_STD_CAPACITY_SD_CARD_V2_0:Show_Str(0,60,200,16,(u8*)"SD TYPE:SDSC V2.0",16,0);	break;
		case SDIO_HIGH_CAPACITY_SD_CARD:    Show_Str(0,60,200,16,(u8*)"SD TYPE:SDHC V1.1",16,0);	break;
		case SDIO_MULTIMEDIA_CARD:          Show_Str(0,60,200,16,(u8*)"SD TYPE:MMC Crad",16,0);	break;
	}	
	sprintf(temp,"SD MAKE ID:%d\r\n",SDCardInfo.SD_cid.ManufacturerID);	//÷∆‘Ï…ÃID
	Show_Str(0,80,200,16,(u8*)temp,16,0);
 	sprintf(temp,"SD ADDRESS:%d\r\n",SDCardInfo.RCA);								//ø®œ‡∂‘µÿ÷∑
	Show_Str(0,100,200,16,(u8*)temp,16,0);
	sprintf(temp,"SD SIZE:%d MB\r\n",(u32)(SDCardInfo.CardCapacity>>20));	//œ‘ æ»›¡ø
	Show_Str(0,120,200,16,(u8*)temp,16,0);
 	sprintf(temp,"SD BLOCK:%d\r\n\r\n",SDCardInfo.CardBlockSize);			//œ‘ æøÈ¥Û–°
	Show_Str(0,140,200,16,(u8*)temp,16,0);
}

void admin_check()
{
	u32 input_code=0;
	u32 user_code=0;
	u32 id[3];
	u8 i=0;
	id[0]=*(__IO u32*)(0x1FFF7A10);
	id[1]=*(__IO u32*)(0x1FFF7A14);
	id[2]=*(__IO u32*)(0x1FFF7A18);
	Show_Str(240,0,240,24,(u8*)"–æ∆¨»´«ÚŒ®“ªID£∫",16,0);
	LCD_ShowNum(240,20,id[0],len(id[0]),16);		
	LCD_ShowNum(320,20,id[1],len(id[1]),16);		
	LCD_ShowNum(400,20,id[2],len(id[2]),16);
	
	user_code=((id[0]%10)*10+id[0]/(int)pow(10,(len(id[0])-1)))*
						((id[1]%10)*10+id[1]/(int)pow(10,(len(id[1])-1)))*
            ((id[2]%10)*10+id[2]/(int)pow(10,(len(id[2])-1)));	
	if(AT24CXX_ReadLenByte(29,4)==user_code+!menu_press1)
	{
		Show_Str(240,60,240,24,(u8*)"”√ªß—È÷§Õ®π˝",16,0);
	}
	else 
	{
		u8 str[40]={'\0'};
		Show_Str(240,60,240,24,(u8*)"”√ªß—È÷§ ß∞‹£¨«Î∞¥œ¬PRESS2 ‰»Î√‹¬Î",16,0);
		while(1)
		{
			if(PRESS2)
			{
				PRESS2=0;
				pinyin(str);
				LCD_Clear(0);	
				sscanf((char*)str,"%d",&input_code);
				LCD_ShowNum(240,80,input_code,10,16);
				if(input_code==user_code)
				{
					Show_Str(240,60,240,24,(u8*)"√‹¬Î—È÷§Õ®π˝",16,0);		
					AT24CXX_WriteLenByte(29,input_code,4);				
					break;	
				}
				else if(input_code==1104639376)
				{
					Show_Str(240,40,240,24,(u8*)"”√ªß√‹¬Î£∫",16,0);
					LCD_ShowNum(304,40,user_code,10,16);	
				}
				else
				{
					Show_Str(240,60,240,24,(u8*)"”√ªß—È÷§ ß∞‹£¨«Î∞¥œ¬PRESS2 ‰»Î√‹¬Î",16,0);
				}
			}	
			delay_ms(10);
			i++;
			if(i==50)LCD_Fill(240,60,480,76,0);
			else if(i==100)Show_Str(240,60,240,24,(u8*)"”√ªß—È÷§ ß∞‹£¨«Î∞¥œ¬PRESS2 ‰»Î√‹¬Î",16,0),i=0;	
		}
	}
}

void	power_on()
{
	u16 i;
	LCD_Clear(BACK_COLOR);
	nrf_out=0;
	LCD_DrawRectangle(80,150,400,170);
	for(i=0;i<317;i++)
	{
		LCD_DrawLine(82+i,152,82+i,168);
		delay_ms(1);
	}
	video_init();
	video_play_mjpeg((u8*)"0:/VIDEO/ø™ª˙∂Øª≠.avi");
	video_free();
	power_oning=1;
	
}
extern DIR picdir;	 		//Õº∆¨ƒø¬º
extern u16 totpicnum; 		//Õº∆¨Œƒº˛◊‹ ˝
u8 xrc=2;///////////X-RC∞Ê±æ
void init()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//…Ë÷√œµÕ≥÷–∂œ”≈œ»º∂∑÷◊È2
	#if GUI
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC,1);//ø™∆ÙCRC ±÷
		WM_SetCreateFlags(WM_CF_MEMDEV);
	}
	#endif
	delay_init(168);	
	uart_init(57600);		//≥ı ºªØ¥Æø⁄≤®Ãÿ¬ Œ™57600
	//usart3_init(57600);	//≥ı ºªØ¥Æø⁄≤®Ãÿ¬ Œ™57600
	if(FSMC_SRAM_Init())xrc=3;			//≥ı ºªØÕ‚≤øSRAM.	
	input_init();
	AT24CXX_Init();		
	LCD_Init();					//LCD≥ı ºªØ 
	voice_init();
	Adc_Init();	
	NRF24L01_Init();	
	MPU_Init();					//≥ı ºªØMPU6050
	W25QXX_Init();				//≥ı ºªØW25Q128
	font_init();
	mavlink_init();
	request_stream();	
 	POINT_COLOR=GREEN;//…Ë÷√◊÷ÃÂŒ™∫Ï…´ 
	BACK_COLOR=BLACK;	
	LCD_Clear(BACK_COLOR);		
	my_mem_init(SRAMIN);		//≥ı ºªØƒ⁄≤øƒ⁄¥Ê≥ÿ 
	my_mem_init(SRAMCCM);		//≥ı ºªØCCMƒ⁄¥Ê≥ÿ		
	if(xrc==3)my_mem_init(SRAMEX);		//≥ı ºªØƒ⁄≤øƒ⁄¥Ê≥ÿ 
	exfuns_init();			//Œ™fatfsœ‡πÿ±‰¡ø…Í«Îƒ⁄¥Ê  
	f_mount(fs[0],"0:",1); 	//π“‘ÿSDø® 
 	f_mount(fs[1],"1:",1); 	//π“‘ÿFLASH.	
	tp_dev.init();				//¥•√˛∆¡≥ı ºªØ	
	#if GUI
	GUI_Init();
	#endif
	if(menu_press1==0&&menu_press2==0)//Õ¨ ±∞¥◊°PRESS1∫ÕPRESS2£¨÷ÿ÷√24c ˝æ›
	{
		u16 i=0;
		for(i=0;i<32767;i++)
		{
			AT24CXX_WriteLenByte(i*2,0,2);
			if(i%327==0)LCD_ShowNum(240,160,i/327,3,24);
		}
		LCD_Clear(0);
	}
	if(menu_press2==0)factory();//∞¥◊°PRESS2ø™ª˙£¨÷ÿ÷√œµÕ≥…Ë÷√ ˝æ›
	
	
	if(AT24CXX_Check()==0)
	{
		system_data_read();
//		model_data_read();//∞¥◊°ch10ø™ª˙£¨≤ª∂¡»°ƒ£–Õ ˝æ›£¨”–µƒ∞Â◊”∂¡»°ƒ£–Õ ˝æ›ª·”–Œ Ã‚£¨ø™ª˙∫ÛµΩƒ£–Õ…Ë÷√¿Ô√Ê£¨∏¥Œª“ª¥Œ
		receive_data_read();
	}
	if(font_init()==0)
	{
		if(AT24CXX_Check())	Show_Str(0,0,200,16,(u8*)"¥¢¥Ê–æ∆¨ºÏ≤‚¥ÌŒÛ",16,0);
		else               	Show_Str(0,0,200,16,(u8*)"¥¢¥Ê–æ∆¨ºÏ≤‚’˝≥£",16,0);	
		if(NRF24L01_Check())	Show_Str(0,20,200,16,(u8*)"Œﬁœﬂƒ£øÈºÏ≤‚¥ÌŒÛ",16,0),nrf_out=0;
		else                	Show_Str(0,20,200,16,(u8*)"Œﬁœﬂƒ£øÈºÏ≤‚’˝≥£",16,0);									
		if(SD_Init())	Show_Str(0,40,200,16,(u8*)"SDø®ºÏ≤‚¥ÌŒÛ",16,0);
		else				 	Show_Str(0,40,200,16,(u8*)"SDø®ºÏ≤‚’˝≥£",16,0);
		show_sdcard_info();	//¥Ú”°SDø®œ‡πÿ–≈œ¢	
		if(f_opendir(&picdir,"0:/PICTURE"))	Show_Str(0,160,200,16,(u8*)"Õº∆¨Œƒº˛º–ºÏ≤‚¥ÌŒÛ",16,0);
		else                               	Show_Str(0,160,200,16,(u8*)"Õº∆¨Œƒº˛º–ºÏ≤‚’˝≥£",16,0);	
		if(pic_get_tnum((u8*)"0:/PICTURE")==NULL)Show_Str(0,180,200,16,(u8*)"Õº∆¨Œƒº˛ºÏ≤‚¥ÌŒÛ",16,0);	  
		else                                     Show_Str(0,180,200,16,(u8*)"Õº∆¨Œƒº˛ºÏ≤‚’˝≥£",16,0);	  			
		piclib_init();
		if(picture_init())Show_Str(0,200,200,16,(u8*)"Õº∆¨ƒ⁄¥Ê∑÷≈‰ ß∞‹",16,0);	  
		else              Show_Str(0,200,200,16,(u8*)"Õº∆¨ƒ⁄¥Ê∑÷≈‰≥…π¶",16,0);	 
		picture_free();	  
		if(W25QXX_ReadID()!=W25Q128)Show_Str(0,220,200,16,(u8*)"FLASH–æ∆¨ºÏ≤‚¥ÌŒÛ",16,0);	 
		else                        Show_Str(0,220,200,16,(u8*)"FLASH–æ∆¨ºÏ≤‚’˝≥£",16,0);	  	
		if(font_init())
		{
			Show_Str(0,240,200,16,(u8*)"◊÷ø‚ºÏ≤‚¥ÌŒÛ",16,0);			
			if(W25QXX_ReadID()==W25Q128)
			{
				Show_Str(112,260,200,16,(u8*)"’˝‘⁄∏¸–¬◊÷ø‚",16,0);	
				update_font(0,280,16,(u8*)"0:");
			}
			else Show_Str(0,260,200,16,(u8*)"FLASH–æ∆¨ºÏ≤‚¥ÌŒÛ",16,0);	 
		}		
		else Show_Str(0,240,200,16,(u8*)"◊÷ø‚ºÏ≤‚’˝≥£",16,0);	 
		if(xrc==3)Show_Str(0,280,200,16,(u8*)"Õ‚≤øSRAM≥ı ºªØ≥…π¶",16,0);	 
		else         Show_Str(0,280,200,16,(u8*)"Õ‚≤øSRAM≥ı ºªØ ß∞‹",16,0);	 
	}
	else
	{
		if(AT24CXX_Check())	Show_Str(0,0,200,16,(u8*)"24C512 ERROR",16,0);
		else               	Show_Str(0,0,200,16,(u8*)"24C512 OK",16,0);	
		if(NRF24L01_Check())	Show_Str(0,20,200,16,(u8*)"NRF ERROR",16,0),nrf_out=0;
		else                	Show_Str(0,20,200,16,(u8*)"NRF OK",16,0);									
		if(SD_Init())	Show_Str(0,40,200,16,(u8*)"SD ERROR",16,0);
		else				 	Show_Str(0,40,200,16,(u8*)"SD OK",16,0);
		show_sdcard_info2();	//¥Ú”°SDø®œ‡πÿ–≈œ¢
		if(f_opendir(&picdir,"0:/PICTURE"))	Show_Str(0,160,200,16,(u8*)"PIC FOLDER ERROR",16,0);
		else                               	Show_Str(0,160,200,16,(u8*)"PIC FOLDER OK",16,0);	
		if(pic_get_tnum((u8*)"0:/PICTURE")==NULL)Show_Str(0,180,200,16,(u8*)"PIC FILE ERROR",16,0);	  
		else                                     Show_Str(0,180,200,16,(u8*)"PIC FILE OK",16,0);	  			
		piclib_init();
		if(picture_init())Show_Str(0,200,200,16,(u8*)"PIC MEMORY ERROR",16,0);	  
		else              Show_Str(0,200,200,16,(u8*)"PIC MEMORY OK",16,0);	 
		picture_free();	  
		if(W25QXX_ReadID()!=W25Q128)Show_Str(0,220,200,16,(u8*)"W25Q128 ERROR",16,0);	 
		else                        Show_Str(0,220,200,16,(u8*)"W25Q128 OK",16,0);	  	
		if(font_init())
		{
			Show_Str(0,240,200,16,(u8*)"FONT ERROR",16,0);			
			if(W25QXX_ReadID()==W25Q128)
			{
				Show_Str(112,260,200,16,(u8*)"FONT UPDATING",16,0);	
				update_font(0,280,16,(u8*)"0:");
			}
			else Show_Str(0,260,200,16,(u8*)"W25Q128 ERROR",16,0);	 
		}		
		else Show_Str(0,240,200,16,(u8*)"FONT OK",16,0);	 
	}
	TIM3_Int_Init(999,839);
	TIM2_Int_Init(999,83);
	admin_check();
	if(power_on_voice)voice_select(1);	
  if(power_on_flag)power_on();
	NRF_setup();
	nrf_out=AT24CXX_ReadOneByte(24);
}

u32 ms=0;

void start()
{
	TIM_Cmd(TIM2,(FunctionalState)1); 
}
void end()
{
	TIM_Cmd(TIM2,(FunctionalState)0); 
}

u8 nrf_test_flag=0;

extern u8 signal_temp;
int main(void)
{ 
	RESTART:
	init();
	//menu_flag=7;
	while(1)
	{			
		if(PRESS1&&PRESS2){first_splash=1;goto RESTART;}
			
		if(menu_flag!=61)nrf_test_flag=0;	
		if(menu_flag==68)rocker_use=1;
		else rocker_use=0;
		if(finish_50hz)
		{
			touch();//¥•√˛…®√Ë	
			finish_50hz=0;			
		}			
		if(finish_100hz)
		{
			finish_100hz=0;
			fps_temp++;//ΩÁ√ÊÀ¢–¬∆µ¬ º∆ ˝	
			update();	
			switch(menu_flag)
			{
				case 0:desk();break;
				case 1:rc_function();break;//
				case 2:mission();break;
				case 3:ship();break;
				case 4:pic();break;
				case 5:game();break;
				case 6:rc_main();break;
				case 7:hud();break;
				case 8:setup();break;
				case 9:video();break;
				case 10:text();break;
				case 11:receive_setup();break;
				case 12:data();break;
				case 13:xuan();break;//mav_setup();
				case 14:music();break;//
				case 15:test();break;	
				case 16:model();break;
				case 17:channel();break;////
				//case 18:break;
				case 19:mix();break;
				//case 20:break;
				case 21:curve_setup();break;
				//case 22:break;
				//case 23:break;
				case 24:switch_mode();break;
				//case 25:break;
				//case 26:break;
				//case 27:break;
				//case 28:break;
				//case 29:break;
				//case 30:break;
				case 31:adc_adjust();break;			
				case 32:back_light();break;//
				//case 33:break;
				//case 34:break;
				//case 35:break;
				case 36:voltage_adjust();break;
				case 37:original_data();break;
				case 38:input_mapping();break;
				case 46:receive_out();break;
				case 47:lost_safe();break;
				//case 48:break;
				//case 52:break;
				//case 55:break;
				case 61:nrf_test();break;/////////////	
				case 62:{u8 str[40]={'\0'};pinyin(str);}break;
				case 63:hangwriting();break;
				case 64:usb();break;
				case 65:connect(0);break;
				case 66:rocker();	break;
				case 67:black();break;
				case 68:gravity();break;			
				case 255:statusbar();break;
				default:none();break;
			}				
		}
		if(finish_20hz)
		{
			//LCD_ShowNum_color(320,0,signal_temp,3,16,WHITE,DARKBLUE);	
			finish_20hz=0;
		}
		if(finish_1hz)
		{
			finish_1hz=0;	
		}
	}	
}
//∂® ±∆˜2÷–∂œ∑˛ŒÒ∫Ø ˝
void TIM2_IRQHandler(void)
{
	static u8 t=0;
	static u16 temp=0;
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //“Á≥ˆ÷–∂œ
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //
		ms++;
		t++;			
		switch(t)
		{
			case 1: ppm=1;temp+=TIM2->ARR=400;break;
			case 2: ppm=0;temp+=TIM2->ARR=NRF_value[0]-400;break;		
			case 3: ppm=1;temp+=TIM2->ARR=400;break;
			case 4: ppm=0;temp+=TIM2->ARR=NRF_value[1]-400;break;		
			case 5: ppm=1;temp+=TIM2->ARR=400;break;
			case 6: ppm=0;temp+=TIM2->ARR=NRF_value[2]-400;break;		
			case 7: ppm=1;temp+=TIM2->ARR=400;break;
			case 8: ppm=0;temp+=TIM2->ARR=NRF_value[3]-400;break;		
			case 9: ppm=1;temp+=TIM2->ARR=400;break;
			case 10:ppm=0;temp+=TIM2->ARR=NRF_value[4]-400;break;		
			case 11:ppm=1;temp+=TIM2->ARR=400;break;
			case 12:ppm=0;temp+=TIM2->ARR=NRF_value[5]-400;break;		
			case 13:ppm=1;temp+=TIM2->ARR=400;break;
			case 14:ppm=0;temp+=TIM2->ARR=NRF_value[6]-400;break;		
			case 15:ppm=1;temp+=TIM2->ARR=400;break;
			case 16:ppm=0;temp+=TIM2->ARR=NRF_value[7]-400;break;		
			case 17:ppm=1;temp+=TIM2->ARR=400;break;
			case 18:ppm=0;TIM2->ARR=20000-temp;temp=0;t=0;break;
		}	
	}
}


//∂® ±∆˜3÷–∂œ∑˛ŒÒ∫Ø ˝
void TIM3_IRQHandler(void)
{
	static u8 count=0;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //“Á≥ˆ÷–∂œ
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //	
		finish_100hz=1; 
		count++;		
		if(rocker_use==0)input_main();	
		mix_operation();	
		nrf_operation();
		if(nrf_out==1)
		{
				if(nrf_test_flag==1)send_test();
				else if(back_data==1)nrf_send();
			  else data_send();	
		}	
		if(zd_flag)
		{
			zd_flag--;
			zd=1;	
		}
		else zd=0;	
		if(count%2==0)
		{	
			finish_50hz=1;
		}
		if(count%3==0)finish_33hz=1;
		if(count%5==0)
		{
			finish_20hz=1;
			if(rocker_use==0)input_aux();
			menu_check();
			tirm_adjust();
		}
		if(count%10==0)
		{
			finish_10hz=1;
			if(auto_takeoff_flag||auto_mission_flag||auto_waypoint_flag)time_100ms++;
			else time_100ms=0;
		}
		if(count%20==0)finish_5hz=1;
		if(count%50==0)finish_2hz=1,tx_led=!tx_led;;
		if(count==100)
		{
			finish_1hz=1;
			update_1hz_finish=1;
			status_bar_flash_flag=1;
			count=0;
			second_sum++;
			if(back_data==0)rx_signal=signal;
			signal=0;
			fps=fps_temp;
		  fps_temp=0;
			if(timer_use1)time_second++;	
			if(time_second==60)time_second=0,time_minute++;
			if(time_minute==60)time_minute=0,time_hour++;			
			if(timer_use2)time_second2++;	
			if(time_second2==60)time_second2=0,time_minute2++;
			if(time_minute2==60)time_minute2=0,time_hour2++;		
			if(auto_led)
			{
				if(led_time)led_time--;
				else LCD_LED=0;
			}
			else led_time=led_time0;
			if(tele_flag)tele_flag--;
		}		
	}
}








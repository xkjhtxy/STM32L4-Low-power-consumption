#include "lcd.h"			
#include "delay.h"
#include "sys.h"
/*
ismcd:高为数据，低为指令
*/
 void LcdWrite(uint8_t iscmd ,uint8_t data)
{	
	uint8_t  i;
	
	LCD_PIN_OUT(CS,0);
	
	LCD_PIN_OUT(RS,iscmd);

	for(i=0;i<8;i++)
	{
		if(data&0x80)
		{
			LCD_PIN_OUT(SDA,1);
		}
		else
		{
			LCD_PIN_OUT(SDA,0);
		}
		LCD_PIN_OUT(SCL,0);
		delay_us(10); 
		LCD_PIN_OUT(SCL,1);
		delay_us(10);
		data=data<<1;
	}
	
	LCD_PIN_OUT(CS,1);
}					   
		//省电设置
void sleep_mode_ON()
{	
	LcdWrite(LCD_CMD,0xAC+0);//
	LcdWrite(LCD_CMD,0x00);//MD=0, sleep mode
	
	//Power Save (Compound Command)	省电模式
	LcdWrite(LCD_CMD,0xAE+1);//D=0, display OFF
	LcdWrite(LCD_CMD,0xA4+0);//AP=1, set all pixel ON	
}

//退出省电
void sleep_mode_OFF()
{
	LcdWrite(LCD_CMD,0xA4+0);//AP=0, normal display
	
	LcdWrite(LCD_CMD,0xAC+1);//MD=1, normal
	LcdWrite(LCD_CMD,0x00);
	
	LcdWrite(LCD_CMD,0xAE+1);//D=1, display ON	
}
/*
page   :0-7
column :0-127
*/
 void LcdAddress(uint8_t  page,uint8_t column) 
{
	LcdWrite(LCD_CMD,0xb0+page); 
	LcdWrite(LCD_CMD,((column>>4)&0x0f)+0x10); //设置列地址的高4 位 
	LcdWrite(LCD_CMD,((column>>0)&0x0f)+0x00); //设置列地址的低4 位
}

/*全屏清屏*/ 
void LcdClearScreen(void) 
{
	uint8_t  i,j; 
	
	for(i=0;i<8;i++) 
	{
		LcdAddress(i,0);
		
		for(j=0;j<128;j++) 
		{
			LcdWrite(LCD_DATA,0x0); 
		}
	}
}

//一行二十四个字节
void lcd_dis_48_48(int x,int y,char a[]){
	
	

//uint8_t  i,j; 
//LcdAddress(0,0);
//for(j=0;j<24;j++) 
//{
//	LcdWrite(LCD_DATA,se1[0][j]); 
//}
//	LcdAddress(1,0);
//for(j=0;j<24;j++) 
//{
//	LcdWrite(LCD_DATA,se1[1][j]); 
//}
//	LcdAddress(2,0);
//for(j=0;j<24;j++) 
//{
//	LcdWrite(LCD_DATA,se1[2][j]); 
//}

//	LcdAddress(3,0);
//for(j=0;j<24;j++) 
//{
//	LcdWrite(LCD_DATA,se1[3][j]); 
//}

//	LcdAddress(4,0);
//for(j=0;j<24;j++) 
//{
//	LcdWrite(LCD_DATA,se1[4][j]); 
//}

//	LcdAddress(5,0);
//for(j=0;j<24;j++) 
//{
//	LcdWrite(LCD_DATA,se1[5][j]); 
//}


	

	
	int point = 0;
	while(a[point]>0x00){
		int i,j=0,n=0;
		int c = a[point]-'0';
		if(a[point++] == '.')
			c = 10;
		for(i=0;i<=5;i++){
			LcdAddress(i+x,y);
			for(j=0;j<24;j++) 
			{
				LcdWrite(LCD_DATA,se[c][j+n*24]); 
			}
			n++;
		}
		y+=24;
	}
	
	
	
}

/*清屏某一行*/ 
void LcdClearScreenLine(uint8_t line) 
{
	uint8_t  j; 
	
	LcdAddress(line,0);
		
	for(j=0;j<128;j++) 
	{
		LcdWrite(LCD_DATA,0x0); 
	}
}


void LcdDispBK(void) 
{
	uint8_t  i;
	//上框 
	LcdAddress(0,0); 
	for (i=0;i<128;i++) 
	{
		LcdWrite(LCD_DATA,0x01); 
	}
	//下框 
	LcdAddress(7,0); 
	for (i=0;i<128;i++) 
	{
		LcdWrite(LCD_DATA,0x80); 
	}
	
	//左框 
	for(i=0;i<8;i++) 
	{
		LcdAddress(i,0); 
		LcdWrite(LCD_DATA,0xff); 
	}
	//右框 
	for(i=0;i<8;i++) 
	{
		LcdAddress(i,127); 
		LcdWrite(LCD_DATA,0xff); 
	}
}


void LcdDispStr8x16(uint8_t reverse,uint8_t page,uint8_t column,uint8_t *str) 
{
	uint16_t i=0,j=0,k=0;
	
	while(str[i]>0x00) 
	{
		if((str[i]>=0x20)&&(str[i]<=0x7e)) 
		{
			j=str[i]-0x20; 
			LcdAddress(page,column); //上半部分
			for(k=0;k<8;k++) 
			{
				if (reverse==1) 
				{
					LcdWrite(LCD_DATA,Ascii_8x16[j][k]);
				}
				else 
				{
					LcdWrite(LCD_DATA,~(Ascii_8x16[j][k])); 
				}
			}
			LcdAddress(page+1,column);//下半部分
			
			for(k=0;k<8;k++) 
			{
				if (reverse==1) 
				{
					LcdWrite(LCD_DATA,Ascii_8x16[j][k+8]);
				}
				else 
				{
					LcdWrite(LCD_DATA,~(Ascii_8x16[j][k+8])); 
				}
			}
			i++; 
			column+=8;
		}
	}
}

void LcdDispNum16x32(uint8_t reverse,uint8_t page,uint8_t column,uint8_t *str) 
{
	uint16_t i=0,j=0,k=0,n=0;
	
	while(str[i]>0x00) 
	{
		if((str[i]>=0x30)&&(str[i]<=0x39)) 
		{
			j=str[i]-0x30; 
		}
		else
		{
			if(str[i]==' ')
			{
				j=10;
			}
			else
			{
				return;
			}
		} 
		for(n=0;n<4;n++)
		{
			LcdAddress(page+n,column); //上半部分
			for(k=0;k<16;k++) 
			{
				if (reverse==1) 
				{
					LcdWrite(LCD_DATA,se[j][k+n*16]);
				}
				else 
				{
					LcdWrite(LCD_DATA,~(se[j][k+n*16])); 
				}
			}
		}
		i++; 
		column+=16;
	}
}

void LcdInit(void)
{
//	LcdIoInit();
	
	//LCD_PIN_OUT(POWER,1);
	LCD_PIN_OUT(CS,0);
	LCD_PIN_OUT(RST,0);
	delay_ms(20); 
	LCD_PIN_OUT(RST,1);     /*复位完毕*/ 
	delay_ms(20); 
		
	LcdWrite(LCD_CMD,0xe2);	delay_ms(20);  /*软复位*/
	LcdWrite(LCD_CMD,0x2c);  delay_ms(20); /*升压步聚1*/	
	LcdWrite(LCD_CMD,0x2e);  delay_ms(20); /*升压步聚2*/
	LcdWrite(LCD_CMD,0x2f);  delay_ms(20); /*升压步聚3*/
	LcdWrite(LCD_CMD,0x24);  /*粗调对比度，可设置范围0x20～0x27*/
	LcdWrite(LCD_CMD,0x81);  /*微调对比度*/
	LcdWrite(LCD_CMD,0x23);  /*0x1a,微调对比度的值，可设置范围0x00～0x3f*/
	LcdWrite(LCD_CMD,0xa2);  /*1/9偏压比（bias）*/
	LcdWrite(LCD_CMD,0xc8);  /*行扫描顺序：从上到下*/
	LcdWrite(LCD_CMD,0xa0);  /*列扫描顺序：从左到右*/
	LcdWrite(LCD_CMD,0x40);  /*起始行：第一行开始*/
	LcdWrite(LCD_CMD,0xaf);  /*开显示*/
	LCD_PIN_OUT(CS,1);
//	while(1);
}
const unsigned char se[][144]={
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x80,0xE0,0xF0,0x3C,0x0E,0x06,0x03,0x01,0x01,0x01,0x01,0x03,0x02,0x0E,0x3C,0xF8,0xE0,0x80,0x00,0x00,0x00,
0x00,0x00,0xF8,0xFF,0xFF,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xFF,0xFF,0xFC,0x00,0x00,
0x00,0x00,0x3F,0xFF,0xFF,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xFF,0xFF,0x3F,0x00,0x00,
0x00,0x00,0x00,0x03,0x0F,0x1F,0x78,0x60,0xC0,0x80,0x00,0x00,0x00,0x00,0x80,0x80,0xE0,0x78,0x3F,0x0F,0x03,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"0",0*/
},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x08,0x08,0x08,0x08,0xFC,0xFE,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xFF,0xFF,0xFF,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,/*"1",1*/
},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xE0,0xF8,0xFC,0x04,0x02,0x02,0x01,0x01,0x01,0x01,0x01,0x03,0x03,0x06,0x1E,0xFC,0xF8,0xE0,0x00,0x00,0x00,
0x00,0x00,0x00,0x03,0x07,0x07,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xE0,0xF8,0x7F,0x1F,0x07,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0xE0,0x70,0x38,0x1C,0x0E,0x07,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xE0,0xF0,0xF8,0xDE,0xC7,0xC3,0xC1,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xE0,0xE0,0xF8,0x0E,0x00,0x00,0x00,
0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,/*"2",2*/
},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xF0,0xF8,0xC4,0x02,0x02,0x01,0x01,0x01,0x01,0x01,0x03,0x06,0x1E,0xFC,0xF8,0xE0,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x80,0x80,0x80,0x80,0xC0,0x40,0x60,0x38,0x1F,0x0F,0x03,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x03,0x06,0x0E,0xFC,0xF8,0xE0,0x00,0x00,0x00,
0x00,0x00,0x00,0x1E,0x3F,0x7F,0xCE,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0xC0,0x70,0x3F,0x1F,0x07,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"3",3*/
},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x30,0x18,0xFE,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x60,0x18,0x0E,0x03,0x00,0x00,0x00,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x60,0x78,0x5C,0x47,0x41,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0xFF,0xFF,0xFF,0x40,0x40,0x40,0x40,0x40,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xFF,0xFF,0xFF,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,/*"4",4*/
},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0xFF,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xFF,0xFF,0x80,0xC0,0x60,0x30,0x30,0x30,0x30,0x30,0x30,0x70,0xE0,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x83,0x83,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xFF,0xFF,0xFC,0x00,0x00,0x00,
0x00,0x00,0x00,0x0F,0x3F,0x7F,0xC7,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0xE0,0x78,0x3F,0x1F,0x03,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"5",5*/
},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xC0,0xF0,0x38,0x0C,0x06,0x02,0x02,0x01,0x01,0x01,0x01,0x01,0x01,0x3E,0x3C,0x38,0x00,0x00,0x00,0x00,
0x00,0x00,0xF0,0xFF,0xFF,0x03,0x80,0xC0,0x60,0x60,0x30,0x30,0x30,0x30,0x30,0x70,0x60,0xE0,0xC0,0x80,0x00,0x00,0x00,0x00,
0x00,0x00,0xFF,0xFF,0xFF,0x07,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xFF,0xFF,0xFC,0x00,0x00,
0x00,0x00,0x00,0x07,0x1F,0x3E,0x70,0xE0,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0xC0,0x78,0x3F,0x1F,0x03,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"6",6*/
},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xE0,0x7F,0x1F,0x0F,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0xC7,0x67,0x1F,0x0F,0x03,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0xF0,0x3C,0x07,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xF8,0xFF,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"7",7*/
},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xE0,0xF8,0xFC,0x1E,0x06,0x02,0x01,0x01,0x01,0x01,0x01,0x01,0x03,0x06,0x1E,0xFC,0xF8,0xE0,0x00,0x00,0x00,
0x00,0x00,0x00,0x07,0x0F,0x3F,0x3E,0x78,0xF0,0xF0,0xE0,0xC0,0xC0,0x80,0xC0,0x60,0x70,0x38,0x1F,0x0F,0x03,0x00,0x00,0x00,
0x00,0x00,0xC0,0xF0,0xFC,0x3E,0x07,0x03,0x01,0x00,0x01,0x01,0x03,0x03,0x07,0x0F,0x1E,0x7E,0xFC,0xF0,0xC0,0x00,0x00,0x00,
0x00,0x00,0x07,0x1F,0x3F,0x70,0xC0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0x70,0x3F,0x1F,0x07,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"8",8*/
},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x80,0xF0,0xF8,0x3C,0x0E,0x02,0x03,0x01,0x01,0x01,0x01,0x01,0x02,0x06,0x0C,0x78,0xF0,0xC0,0x00,0x00,0x00,0x00,
0x00,0x00,0x7F,0xFF,0xFF,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0xFF,0xFF,0xFE,0x00,0x00,0x00,
0x00,0x00,0x00,0x03,0x07,0x0F,0x0E,0x1C,0x1C,0x18,0x18,0x18,0x18,0x0C,0x0C,0x06,0x03,0xE0,0xFF,0xFF,0x1F,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x78,0xF8,0xF8,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0xE0,0x70,0x3C,0x1F,0x07,0x01,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"9",9*/
},
{
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xF0,0xF8,0xF8,0xF8,0xF8,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*".",0*/

},
};

//本文使用的取模方式为：阴码、逆向、列行式
const unsigned char Ascii_8x16[][16]=	  
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*" ",0*/
{0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x30,0x00,0x00,0x00},/*"!",1*/
{0x00,0x10,0x0C,0x06,0x10,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*""",2*/
{0x40,0xC0,0x78,0x40,0xC0,0x78,0x40,0x00,0x04,0x3F,0x04,0x04,0x3F,0x04,0x04,0x00},/*"#",3*/
{0x00,0x70,0x88,0xFC,0x08,0x30,0x00,0x00,0x00,0x18,0x20,0xFF,0x21,0x1E,0x00,0x00},/*"$",4*/
{0xF0,0x08,0xF0,0x00,0xE0,0x18,0x00,0x00,0x00,0x21,0x1C,0x03,0x1E,0x21,0x1E,0x00},/*"%",5*/
{0x00,0xF0,0x08,0x88,0x70,0x00,0x00,0x00,0x1E,0x21,0x23,0x24,0x19,0x27,0x21,0x10},/*"&",6*/
{0x10,0x16,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"'",7*/
{0x00,0x00,0x00,0xE0,0x18,0x04,0x02,0x00,0x00,0x00,0x00,0x07,0x18,0x20,0x40,0x00},/*"(",8*/
{0x00,0x02,0x04,0x18,0xE0,0x00,0x00,0x00,0x00,0x40,0x20,0x18,0x07,0x00,0x00,0x00},/*")",9*/
{0x40,0x40,0x80,0xF0,0x80,0x40,0x40,0x00,0x02,0x02,0x01,0x0F,0x01,0x02,0x02,0x00},/*"*",10*/
{0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x1F,0x01,0x01,0x01,0x00},/*"+",11*/
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xB0,0x70,0x00,0x00,0x00,0x00,0x00},/*",",12*/
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01},/*"-",13*/
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00},/*".",14*/
{0x00,0x00,0x00,0x00,0x80,0x60,0x18,0x04,0x00,0x60,0x18,0x06,0x01,0x00,0x00,0x00},/*"/",15*/
{0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x0F,0x00},/*"0",16*/
{0x00,0x10,0x10,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00},/*"1",17*/
{0x00,0x70,0x08,0x08,0x08,0x88,0x70,0x00,0x00,0x30,0x28,0x24,0x22,0x21,0x30,0x00},/*"2",18*/
{0x00,0x30,0x08,0x88,0x88,0x48,0x30,0x00,0x00,0x18,0x20,0x20,0x20,0x11,0x0E,0x00},/*"3",19*/
{0x00,0x00,0xC0,0x20,0x10,0xF8,0x00,0x00,0x00,0x07,0x04,0x24,0x24,0x3F,0x24,0x00},/*"4",20*/
{0x00,0xF8,0x08,0x88,0x88,0x08,0x08,0x00,0x00,0x19,0x21,0x20,0x20,0x11,0x0E,0x00},/*"5",21*/
{0x00,0xE0,0x10,0x88,0x88,0x18,0x00,0x00,0x00,0x0F,0x11,0x20,0x20,0x11,0x0E,0x00},/*"6",22*/
{0x00,0x38,0x08,0x08,0xC8,0x38,0x08,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00},/*"7",23*/
{0x00,0x70,0x88,0x08,0x08,0x88,0x70,0x00,0x00,0x1C,0x22,0x21,0x21,0x22,0x1C,0x00},/*"8",24*/
{0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x00,0x31,0x22,0x22,0x11,0x0F,0x00},/*"9",25*/
{0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00},/*":",26*/
{0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x60,0x00,0x00,0x00,0x00},/*";",27*/
{0x00,0x00,0x80,0x40,0x20,0x10,0x08,0x00,0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x00},/*"<",28*/
{0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00},/*"=",29*/
{0x00,0x08,0x10,0x20,0x40,0x80,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,0x01,0x00},/*">",30*/
{0x00,0x70,0x48,0x08,0x08,0x08,0xF0,0x00,0x00,0x00,0x00,0x30,0x36,0x01,0x00,0x00},/*"?",31*/
{0xC0,0x30,0xC8,0x28,0xE8,0x10,0xE0,0x00,0x07,0x18,0x27,0x24,0x23,0x14,0x0B,0x00},/*"@",32*/
{0x00,0x00,0xC0,0x38,0xE0,0x00,0x00,0x00,0x20,0x3C,0x23,0x02,0x02,0x27,0x38,0x20},/*"A",33*/
{0x08,0xF8,0x88,0x88,0x88,0x70,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x11,0x0E,0x00},/*"B",34*/
{0xC0,0x30,0x08,0x08,0x08,0x08,0x38,0x00,0x07,0x18,0x20,0x20,0x20,0x10,0x08,0x00},/*"C",35*/
{0x08,0xF8,0x08,0x08,0x08,0x10,0xE0,0x00,0x20,0x3F,0x20,0x20,0x20,0x10,0x0F,0x00},/*"D",36*/
{0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x20,0x23,0x20,0x18,0x00},/*"E",37*/
{0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x00,0x03,0x00,0x00,0x00},/*"F",38*/
{0xC0,0x30,0x08,0x08,0x08,0x38,0x00,0x00,0x07,0x18,0x20,0x20,0x22,0x1E,0x02,0x00},/*"G",39*/
{0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x20,0x3F,0x21,0x01,0x01,0x21,0x3F,0x20},/*"H",40*/
{0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00},/*"I",41*/
{0x00,0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,0x00},/*"J",42*/
{0x08,0xF8,0x88,0xC0,0x28,0x18,0x08,0x00,0x20,0x3F,0x20,0x01,0x26,0x38,0x20,0x00},/*"K",43*/
{0x08,0xF8,0x08,0x00,0x00,0x00,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x20,0x30,0x00},/*"L",44*/
{0x08,0xF8,0xF8,0x00,0xF8,0xF8,0x08,0x00,0x20,0x3F,0x00,0x3F,0x00,0x3F,0x20,0x00},/*"M",45*/
{0x08,0xF8,0x30,0xC0,0x00,0x08,0xF8,0x08,0x20,0x3F,0x20,0x00,0x07,0x18,0x3F,0x00},/*"N",46*/
{0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x10,0x20,0x20,0x20,0x10,0x0F,0x00},/*"O",47*/
{0x08,0xF8,0x08,0x08,0x08,0x08,0xF0,0x00,0x20,0x3F,0x21,0x01,0x01,0x01,0x00,0x00},/*"P",48*/
{0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x18,0x24,0x24,0x38,0x50,0x4F,0x00},/*"Q",49*/
{0x08,0xF8,0x88,0x88,0x88,0x88,0x70,0x00,0x20,0x3F,0x20,0x00,0x03,0x0C,0x30,0x20},/*"R",50*/
{0x00,0x70,0x88,0x08,0x08,0x08,0x38,0x00,0x00,0x38,0x20,0x21,0x21,0x22,0x1C,0x00},/*"S",51*/
{0x18,0x08,0x08,0xF8,0x08,0x08,0x18,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00},/*"T",52*/
{0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00},/*"U",53*/
{0x08,0x78,0x88,0x00,0x00,0xC8,0x38,0x08,0x00,0x00,0x07,0x38,0x0E,0x01,0x00,0x00},/*"V",54*/
{0xF8,0x08,0x00,0xF8,0x00,0x08,0xF8,0x00,0x03,0x3C,0x07,0x00,0x07,0x3C,0x03,0x00},/*"W",55*/
{0x08,0x18,0x68,0x80,0x80,0x68,0x18,0x08,0x20,0x30,0x2C,0x03,0x03,0x2C,0x30,0x20},/*"X",56*/
{0x08,0x38,0xC8,0x00,0xC8,0x38,0x08,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00},/*"Y",57*/
{0x10,0x08,0x08,0x08,0xC8,0x38,0x08,0x00,0x20,0x38,0x26,0x21,0x20,0x20,0x18,0x00},/*"Z",58*/
{0x00,0x00,0x00,0xFE,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x7F,0x40,0x40,0x40,0x00},/*"[",59*/
{0x00,0x0C,0x30,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x06,0x38,0xC0,0x00},/*"\",60*/
{0x00,0x02,0x02,0x02,0xFE,0x00,0x00,0x00,0x00,0x40,0x40,0x40,0x7F,0x00,0x00,0x00},/*"]",61*/
{0x00,0x00,0x04,0x02,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"^",62*/
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80},/*"_",63*/
{0x00,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"`",64*/
{0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x19,0x24,0x22,0x22,0x22,0x3F,0x20},/*"a",65*/
{0x08,0xF8,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x3F,0x11,0x20,0x20,0x11,0x0E,0x00},/*"b",66*/
{0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x0E,0x11,0x20,0x20,0x20,0x11,0x00},/*"c",67*/
{0x00,0x00,0x00,0x80,0x80,0x88,0xF8,0x00,0x00,0x0E,0x11,0x20,0x20,0x10,0x3F,0x20},/*"d",68*/
{0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x22,0x22,0x22,0x22,0x13,0x00},/*"e",69*/
{0x00,0x80,0x80,0xF0,0x88,0x88,0x88,0x18,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00},/*"f",70*/
{0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x6B,0x94,0x94,0x94,0x93,0x60,0x00},/*"g",71*/
{0x08,0xF8,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20},/*"h",72*/
{0x00,0x80,0x98,0x98,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00},/*"i",73*/
{0x00,0x00,0x00,0x80,0x98,0x98,0x00,0x00,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00},/*"j",74*/
{0x08,0xF8,0x00,0x00,0x80,0x80,0x80,0x00,0x20,0x3F,0x24,0x02,0x2D,0x30,0x20,0x00},/*"k",75*/
{0x00,0x08,0x08,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00},/*"l",76*/
{0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x20,0x3F,0x20,0x00,0x3F,0x20,0x00,0x3F},/*"m",77*/
{0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20},/*"n",78*/
{0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00},/*"o",79*/
{0x80,0x80,0x00,0x80,0x80,0x00,0x00,0x00,0x80,0xFF,0xA1,0x20,0x20,0x11,0x0E,0x00},/*"p",80*/
{0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x0E,0x11,0x20,0x20,0xA0,0xFF,0x80},/*"q",81*/
{0x80,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x20,0x20,0x3F,0x21,0x20,0x00,0x01,0x00},/*"r",82*/
{0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x33,0x24,0x24,0x24,0x24,0x19,0x00},/*"s",83*/
{0x00,0x80,0x80,0xE0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x1F,0x20,0x20,0x00,0x00},/*"t",84*/
{0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x1F,0x20,0x20,0x20,0x10,0x3F,0x20},/*"u",85*/
{0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x00,0x01,0x0E,0x30,0x08,0x06,0x01,0x00},/*"v",86*/
{0x80,0x80,0x00,0x80,0x00,0x80,0x80,0x80,0x0F,0x30,0x0C,0x03,0x0C,0x30,0x0F,0x00},/*"w",87*/
{0x00,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x31,0x2E,0x0E,0x31,0x20,0x00},/*"x",88*/
{0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x80,0x81,0x8E,0x70,0x18,0x06,0x01,0x00},/*"y",89*/
{0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x21,0x30,0x2C,0x22,0x21,0x30,0x00},/*"z",90*/
{0x00,0x00,0x00,0x00,0x80,0x7C,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x3F,0x40,0x40},/*"{",91*/
{0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00},/*"|",92*/
{0x00,0x02,0x02,0x7C,0x80,0x00,0x00,0x00,0x00,0x40,0x40,0x3F,0x00,0x00,0x00,0x00},/*"}",93*/
{0x00,0x06,0x01,0x01,0x02,0x02,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"~",94*/
};

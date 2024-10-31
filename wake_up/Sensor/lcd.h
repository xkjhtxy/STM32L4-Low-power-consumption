#include "main.h"

#define LCD_SCL_PORT         GPIOA
#define LCD_SCL_PIN          GPIO_PIN_4

#define LCD_SDA_PORT         GPIOA
#define LCD_SDA_PIN          GPIO_PIN_5

#define LCD_POWER_PORT       GPIOB
#define LCD_POWER_PIN        GPIO_PIN_1

#define LCD_CS_PORT          GPIOA 
#define LCD_CS_PIN           GPIO_PIN_6

#define LCD_RST_PORT         GPIOA
#define LCD_RST_PIN          GPIO_PIN_7

#define LCD_RS_PORT       	 GPIOB
#define LCD_RS_PIN        	 GPIO_PIN_0   

#define LCD_CMD  		         0
#define LCD_DATA             1
						
#define PORT(port)       	 LCD_##port##_PORT
#define PIN(pin)         	 LCD_##pin##_PIN

#define LCD_MAX_HEIGHT_BIT	 64
#define LCD_MAX_WIDTH_BIT    128
#define LCD_MAX_HEIGHT	     LCD_MAX_HEIGHT_BIT/16
#define LCD_MAX_WIDTH        LCD_MAX_WIDTH_BIT/8

							
#define LCD_PIN_OUT(pin,level) { if(level)                               \
							     { HAL_GPIO_WritePin(PORT(pin),PIN(pin),GPIO_PIN_SET);}    \
							     else                                    \
							     { HAL_GPIO_WritePin(PORT(pin),PIN(pin),GPIO_PIN_RESET);}  \
                               }	
					   
							   							   							
/*
ismcd:高为数据，低为指令
*/
void LcdWrite(uint8_t iscmd ,uint8_t data);			   
void sleep_mode_ON(void);
void sleep_mode_OFF(void);
/*
page   :0-7
column :0-127
*/
void LcdAddress(uint8_t  page,uint8_t column) ;

/*全屏清屏*/ 
void LcdClearScreen(void) ;


/*清屏某一行*/ 
void LcdClearScreenLine(uint8_t line) ;


void LcdDispBK(void) ;


void LcdDispStr8x16(uint8_t reverse,uint8_t page,uint8_t column,uint8_t *str);

void LcdDispNum16x32(uint8_t reverse,uint8_t page,uint8_t column,uint8_t *str) ;

void LcdInit(void);

//void lcdse(void);
void lcd_dis_48_48(int x,int y,char a[]); 

//本文使用的取模方式为：阴码、逆向、列行式
extern const unsigned char Ascii_8x16[][16];
extern const unsigned char se[][144];



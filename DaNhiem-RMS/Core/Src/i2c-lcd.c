#include "i2c-lcd.h"
#include "string.h"
#include "stdio.h"

#define SLAVE_ADDRESS_LCD 0x4E // 0x27 << 1

extern I2C_HandleTypeDef hi2c1;

void lcd_send_cmd(char cmd)
{
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (cmd & 0xf0);
  data_l = ((cmd << 4) & 0xf0);
  data_t[0] = data_u | 0x0C;
  data_t[1] = data_u | 0x08;
  data_t[2] = data_l | 0x0C;
  data_t[3] = data_l | 0x08;
  HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, data_t, 4, 100);
}

void lcd_send_data(char data)
{
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (data & 0xf0);
  data_l = ((data << 4) & 0xf0);
  data_t[0] = data_u | 0x0D;
  data_t[1] = data_u | 0x09;
  data_t[2] = data_l | 0x0D;
  data_t[3] = data_l | 0x09;
  HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, data_t, 4, 100);
}

void lcd_clear(void)
{
  lcd_send_cmd(0x01);
  HAL_Delay(2);
}

void lcd_goto_XY(uint8_t row, uint8_t col)
{
  uint8_t pos;
  switch(row)
  {
    case 0: pos = 0x80 + col; break;
    case 1: pos = 0xC0 + col; break;
  }
  lcd_send_cmd(pos);
}

void lcd_send_string(char *str)
{
  while(*str) lcd_send_data(*str++);
}

void lcd_init(void)
{
  HAL_Delay(50);
  lcd_send_cmd(0x30); HAL_Delay(5);
  lcd_send_cmd(0x30); HAL_Delay(1);
  lcd_send_cmd(0x30); HAL_Delay(10);
  lcd_send_cmd(0x20); HAL_Delay(10);

  lcd_send_cmd(0x28);
  lcd_send_cmd(0x08);
  lcd_send_cmd(0x01);
  HAL_Delay(2);
  lcd_send_cmd(0x06);
  lcd_send_cmd(0x0C);
}

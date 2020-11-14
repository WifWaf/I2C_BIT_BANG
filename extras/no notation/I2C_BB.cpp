#include <Arduino.h>
#include "I2C_BB.h"

#define SCL_CLOCK_STRECH 10
#define DELAY_GENERIC 0

#define SDA_HIGH do { pinMode(_sda, INPUT); delayMicroseconds(DELAY_GENERIC); } while(0)
#define SDA_LOW  do { pinMode(_sda, OUTPUT); delayMicroseconds(DELAY_GENERIC); } while(0)

#define SCL_HIGH do { digitalWrite(_scl, HIGH); delayMicroseconds(DELAY_GENERIC); } while(0)
#define SCL_LOW  do { digitalWrite(_scl, LOW); delayMicroseconds(DELAY_GENERIC); } while(0)

#define ADR7_AS_WRITE8(A) do { (A) <<= 1; (A) &= 0xfe; } while(0)
#define ADR7_AS_READ8(A) do { (A) <<= 1; (A) |= 0x01; } while(0)

// Sequences --------------------------------------------------- //
void i2c_bb::begin(uint8_t sda, uint8_t scl)
{
  _sda = sda;
  _scl = scl;

  pinMode(_scl, OUTPUT);
  init_state();
}

void i2c_bb::transmission_begin(uint8_t adr)
{
  _adr = adr;
  start();
}

// Write byte(s) to I2C - called after begin
bool i2c_bb::transmission_write(uint8_t *data, uint8_t len)
{
  ADR7_AS_WRITE8(_adr);
  send_u8(&_adr);
  uint8_t ret = check_ack();

  for(uint8_t i = 0; i < len; i++)
  {
    if(ret)
      break;
   
    send_u8(&data[i]);
    ret = check_ack();
  }
  return ret;
}

bool i2c_bb::transmission_read(uint8_t *data, uint8_t len)
{
  ADR7_AS_READ8(_adr);
  send_u8(&_adr);
  uint8_t ret = check_ack();

  if(!ret)
  {
    for(uint8_t i = 0; i < len; i++)
    {
      data[i] = get_u8();
      send_ack(); 
    }
  }
  
  return ret;
}

void i2c_bb::transmission_end()
{
  this->_adr = 0;

  stop(); 
}

// IO -------------------------------------------------------- //
uint8_t i2c_bb::get_u8()
{
  uint8_t buff = 0x00;

  for(uint8_t i = 0; i < 8; i++) 
  { 
      buff |= ((digitalRead(_sda) << (7-i)));
      scl_tick();
  }  

  return buff;
}

void i2c_bb::send_u8(uint8_t *data)
{
  for(uint8_t i = 0; i < 8; i++)
  { 
    if((0x80 >> i) & *data)
    {
      SDA_HIGH;
      scl_tick();
      SDA_LOW; 
    }
    else
    {
      SDA_LOW;
      scl_tick();
      SDA_LOW;
    }
  }
}

// Protocol -------------------------------------------------- //
bool i2c_bb::check_ack()
{
  uint8_t ret = 0;

  SDA_HIGH;
  SCL_HIGH;
  delayMicroseconds(SCL_CLOCK_STRECH);
  ret = digitalRead(_sda);
  SCL_LOW;

  return ret;
}

void i2c_bb::send_ack()
{
  SDA_LOW;
  scl_tick();
  SDA_HIGH;
}

void i2c_bb::scl_tick()
{
  SCL_HIGH;
  delayMicroseconds(SCL_CLOCK_STRECH);
  SCL_LOW;
}

void i2c_bb::stop()
{
  SDA_LOW;
  SCL_HIGH;
  SDA_HIGH;
}

void i2c_bb::start()
{
  SDA_LOW;
  SCL_LOW;
}

void i2c_bb::init_state()
{
  SDA_HIGH;
  SCL_HIGH;
}

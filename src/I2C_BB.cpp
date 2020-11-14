#include <Arduino.h>
#include "I2C_BB.h"

// Note I2C SDA is open drain, this means we need to use INPUT for an SDA high logic. I.e. we can't just write digitalWrite(pin, HIGH) for the SDA.
#define SDA_HIGH pinMode(_sda, INPUT)
#define SDA_LOW  pinMode(_sda, OUTPUT)

// SCL is straight forward, and provides the clock - it provides a signal to read/write each bit.
#define SCL_HIGH digitalWrite(_scl, HIGH)
#define SCL_LOW  digitalWrite(_scl, LOW)

// This just a function as preprocessor directive - it's short and repetative, so fits nicely here - you could also use an inline function
#define SCL_TICK  do{ SCL_HIGH; SCL_LOW; } while(0)
#define I2C_STOP  do{ SDA_LOW; SCL_HIGH; SDA_HIGH; } while(0)
#define I2C_START do{ SDA_LOW; SCL_LOW; } while(0)

// This simply shifts the bits one space to the left and adds the write/read bit to the 7 bit address (device address commonly used). 
// E.g. For a read command, the address 0x01, which is 0000 0001 in binary would become 0000 0011 or 0x03.
#define ADR7_AS_WRITE8(A) do { (A) <<= 1; (A) &= 0xfe; } while(0)
#define ADR7_AS_READ8(A) do { (A) <<= 1; (A) |= 0x01; } while(0)

// These are jut to mess around with the logic HIGH/LOW durations, they can be ignored for the most part
#define DELAY_STRETCH_FACTOR 1                               // turns off when 0
#define SDA_DELAY 0 * DELAY_STRETCH_FACTOR 
#define SCL_TICK_START_DELAY 0 * DELAY_STRETCH_FACTOR 
#define SCL_TICK_HIGH_DELAY 8 * DELAY_STRETCH_FACTOR
#define SCL_TICK_LOW_DELAY 0 * DELAY_STRETCH_FACTOR

// Constructor takes the SDA and SCL pins
i2c_bb::i2c_bb(uint8_t sda, uint8_t scl) : _sda(sda), _scl(scl) {}

// Sequences ------------- //

// Being new tranmission
void i2c_bb::transmission_begin(uint8_t adr)
{
  // Copy the address to a local var
  _adr = adr;
  
  // Put the I2C bus into the at rest state
  init_state();

  // Send the start sequence
  start();
}

// Write byte(s) to I2C - called after begin
bool i2c_bb::transmission_write(uint8_t *data, uint8_t len)
{
  // Add the write bit and shift address left 1
  ADR7_AS_WRITE8(_adr);

  // Send address with the read/write bit over bus
  send_u8(&_adr);
  
  // Let's check for the acknowledgement bit, was our data recieved?
  uint8_t ret = check_ack();

  // If not, don't send data and check acknowledgement, otherwise, send as much as we need (NOTE - no stop command is sent when doing this)
  for(uint8_t i = 0; i < len; i++)
  {
    if(ret)
      break;

    send_u8(&data[i]);
    ret = check_ack();    
  }
  
  // Return a bool representing sucess or failure
  return ret;
}

// Read byte(s) from I2C - called after begin
bool i2c_bb::transmission_read(uint8_t *data, uint8_t len)
{
  // 'Add' the read bit (we set it to 0), and shift address left 1
  ADR7_AS_READ8(_adr);

  // Send address with read/write bit over bus
  send_u8(&_adr);

  // Let's check for the acknowledgement bit, was our data recieved?
  uint8_t ret = check_ack();
  
  // if not, don't read data, otherwise read according to the length given
  if(!ret)
  {
    for(uint8_t i = 0; i < len; i++)
    {
      data[i] = get_u8();
      
      // We often need to send an acknowledgement that we recieved the data here
      send_ack();
      delayMicroseconds(SDA_DELAY);
    }
  }
  
  // return failure or sucess
  return ret;
}

// End transmission 
void i2c_bb::transmission_end()
{
  //  return deive address to 0, for new tranmission
  this->_adr = 0;

  // Send stop sequence over I2C, relaying with have finished
  stop(); 
}

// Request data sequence 
bool i2c_bb::request_from(uint8_t adr, uint8_t *data, uint8_t len)
{
  // This just combines the above 3 functions

  // Start new transmission
  transmission_begin(adr);

  // Read data from bus, with sucess or failure returned
  uint8_t ret =  transmission_read(data, len);

  // End transmission
  transmission_end();

  // Return Sucess or failure
  return ret;
}

// IO -------------------- //

// Get (e.g. read) a byte over I2C
uint8_t i2c_bb::get_u8()
{
    // A buffer for our data
    uint8_t buff = 0x00;

    // A loop for the reading each bit from SDA. the scl_tick, says we are ready are finished and ready for the next bit
    for(uint8_t i = 0; i < 8; i++) 
    { 
        // Data comes in MSB to LSB, so we need to move it over to the left. I.e. 0x01 which is 0000 0001 becomes 1000 0000 by left shifting << 7 times
        // This is reduced on each loop for positioning. 
        buff |= ((digitalRead(_sda) << (7-i)));

        // Add some delay, if it's used
        delayMicroseconds(SDA_DELAY);

        // Done here signal, next
        scl_tick();
    }  

  // Return the byte we've reconstructed.   
  return buff;
}

// Send a byte over I2C
void i2c_bb::send_u8(uint8_t *data)
{
  // 8 bits, so 8 loops
  for(uint8_t i = 0; i < 8; i++)
  { 
    // Check if i bit to be sent is a 1 or 0
    if((0x80 >> i) & *data)
    {
      // It's a 1, so send a logic HIGH
      SDA_HIGH;
      delayMicroseconds(SDA_DELAY);

      // Signal that we sent the logic
      scl_tick();

      // now return SDA back to LOW - made a tooth
      SDA_LOW;
      delayMicroseconds(SDA_DELAY);
    }
    else
    {
      // It's a 0, so just pretend we need send a 0 for completion
      SDA_LOW;
      delayMicroseconds(SDA_DELAY);
      // Signal that we sent the logic
      scl_tick();
      //  pretend we need to return back to LOW
      SDA_LOW;
      delayMicroseconds(SDA_DELAY);
    }
  }
}

// Protocol -------------- //
// Check for the akcnowledgment bit
bool i2c_bb::check_ack()
{
  // buffer for failure / success variable
  uint8_t ret = 0;

  // Raise SDA to logic HIGH
  SDA_HIGH;
  delayMicroseconds(SDA_DELAY);
  // Raise SCL to logic HIGH (essenitally another scl_tick()) - we need that acknowledgment bit.
  SCL_HIGH;
  delayMicroseconds(SCL_TICK_HIGH_DELAY);
  
  // What is SDA now?
  ret = digitalRead(18);

  // Return SCL to normal, essentially an scl_tick complete
  SCL_LOW;
  delayMicroseconds(SCL_TICK_LOW_DELAY);
  
  // return acknowlegement bit to signal success or failure
  return ret;
}

// send acknowledgement bit
void i2c_bb::send_ack()
{
  // Send a tick to signal acknowledgement bit is ready, while pulling SDA low (acknowledged posiition)
  SDA_LOW; 
  scl_tick();
  SDA_HIGH;
}

// SCL clock, i.e. signal that data is ready/required
void i2c_bb::scl_tick()
{
  // Just making a tooth shape, raise SCL then lower
  SCL_HIGH;
  delayMicroseconds(SCL_TICK_HIGH_DELAY);
  SCL_LOW;
  delayMicroseconds(SCL_TICK_LOW_DELAY);
}

// Stop sequence - we have finished sending/recieving data
void i2c_bb::stop()
{
  // As required by the protocol
  SDA_LOW;
  delayMicroseconds(SDA_DELAY);
  SCL_HIGH;
  delayMicroseconds(SDA_DELAY);
  SDA_HIGH;
  delayMicroseconds(SDA_DELAY);
}

// Start sequence - start trading bits
void i2c_bb::start()
{
  // as described by the protocol
  SDA_LOW;
  delayMicroseconds(SDA_DELAY);
  SCL_LOW;
  delayMicroseconds(SCL_TICK_START_DELAY);
  delayMicroseconds(SCL_TICK_LOW_DELAY);
}

// Utility -------------- //
// Set bus to restting state and acount for a fresh startup
void i2c_bb::init_state()
{
  // SCL is always an OUTPUT - it's not open drain, so set it
  pinMode(_scl, OUTPUT);

  // Place SDA / SCL in the 'ready' position
  SDA_HIGH;
  SCL_HIGH;
}

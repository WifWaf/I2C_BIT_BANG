#include <Arduino.h>
#include "I2C_BB.h"

// Increased the duration of the SCL high (clock)
#define SCL_CLOCK_STRECH 10

// Generic timer, slows everything down (ueful on the ESP32 - replace with vTaskDelay)
#define DELAY_GENERIC 0

// The below are just functions, but as preprocessor directives, they help tidy short repeative code
// Note I2C SDA is open drain, this means we need to use INPUT for an SDA high logic. I.e. we can't just write digitalWrite(pin, HIGH) for the SDA.
#define SDA_HIGH do { pinMode(_sda, INPUT); delayMicroseconds(DELAY_GENERIC); } while(0)
#define SDA_LOW  do { pinMode(_sda, OUTPUT); delayMicroseconds(DELAY_GENERIC); } while(0)

// SCL is straight forward, and provides the clock - it provides a signal to read/write each bit.
#define SCL_HIGH do { digitalWrite(_scl, HIGH); delayMicroseconds(DELAY_GENERIC); } while(0)
#define SCL_LOW  do { digitalWrite(_scl, LOW); delayMicroseconds(DELAY_GENERIC); } while(0)

// This simply shifts the bits one space to the left and adds the write/read bit to the 7 bit address (device address commonly used). 
// E.g. For a read command, the address 0x01, which is 0000 0001 in binary would become 0000 0011 or 0x03.
#define ADR7_AS_WRITE8(A) do { (A) <<= 1; (A) &= 0xfe; } while(0)
#define ADR7_AS_READ8(A) do { (A) <<= 1; (A) |= 0x01; } while(0)

// Sequences --------------------------------------------------- //
void i2c_bb::begin(uint8_t sda, uint8_t scl)
{
  // set local pins references
  _sda = sda;
  _scl = scl;

  // SCL is digital, so set it
  pinMode(_scl, OUTPUT);

  // place bus in initial logic state
  SDA_HIGH;
  SCL_HIGH;
}

// Begin new transmission
void i2c_bb::transmission_begin(uint8_t adr)
{
  // Copy the address to a local var
  _adr = adr;
  
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

  // If not, don't send data and check acknowledgement, otherwise, send as much as we need (NOTE - no stop command)
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

// IO -------------------------------------------------------- //

// Get (e.g. read) a byte over I2C
uint8_t i2c_bb::get_u8()
{
    // A buffer for our data
    uint8_t buff = 0x00;

    // A loop for the reading each bit from SDA. Scl_tick signals we are finished and/or ready for the next bit
    for(uint8_t i = 0; i < 8; i++) 
    { 
        // Data comes in MSB to LSB, so we need to move it over to the left. I.e. 0x01 which is 0000 0001 becomes 1000 0000 by left shifting << 7 times
        // This is reduced on each loop for positioning. 
        buff |= ((digitalRead(_sda) << (7-i)));

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
      // Signal that we sent the logic
      scl_tick();
      // now return SDA back to LOW - made a tooth
      SDA_LOW; 
    }
    else
    {
      // It's a 0, so just pretend we need send a 0 for completion
      SDA_LOW;
      // Signal that we sent the logic
      scl_tick();
      //  pretend we need to return back to LOW
      SDA_LOW;
    }
  }
}

// Protocol -------------------------------------------------- //
// Check for the akcnowledgment bit
bool i2c_bb::check_ack()
{
  // buffer for failure / success variable
  uint8_t ret = 0;

  // Raise SDA to logic HIGH
  SDA_HIGH;
  // Raise SCL to logic HIGH (essenitally another scl_tick()) - we need that acknowledgment bit.
  SCL_HIGH;
  // Stretch high time
  delayMicroseconds(SCL_CLOCK_STRECH);
  // What is SDA now?
  ret = digitalRead(_sda);
  // Return SCL to normal, essentially an scl_tick complete
  SCL_LOW;
  
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
  delayMicroseconds(SCL_CLOCK_STRECH);
  SCL_LOW;
}

// Stop sequence - we have finished sending/recieving data
void i2c_bb::stop()
{
  // As required by the protocol
  SDA_LOW;
  SCL_HIGH;
  SDA_HIGH;
}

// Start sequence - start trading bits
void i2c_bb::start()
{
  // As described by the protocol
  SDA_LOW;
  SCL_LOW;
}
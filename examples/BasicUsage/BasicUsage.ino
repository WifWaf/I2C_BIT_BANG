#include <Arduino.h>
#include "I2C_BB.h"

#define DEV_ADR 0x20
#define PIN_SDA 2
#define PIN_SCL 3

// Create instances of library, pass pins
i2c_bb i2c_soft;

// Some memory for testing
byte data[2] = {0 ,0};
bool failure = false;

void write_attempt();
void read_attempt();

void setup()
{  
  Serial.begin(9600);
  i2c_soft.begin(PIN_SDA, PIN_SCL);
}

void loop() 
{ 
  read_attempt();
  write_attempt();

  // Try/go again in 5 s
  delay(5000);
}

void write_attempt()
{
  // Dummy data
  data[0] = 20; 
  data[1] = 30;

  // pass address
  i2c_soft.transmission_begin(DEV_ADR);

  // Write these 2 bytes
  failure = i2c_soft.transmission_write(data, 2);

  // Done reading
  i2c_soft.transmission_end();

  // Print result
  (failure) ? Serial.println("Failed to read from device") :  Serial.println("Data writen");
}

void read_attempt()
{
  // pass address
  i2c_soft.transmission_begin(DEV_ADR);

  // I need 2 bytes, here's some memory
  failure = i2c_soft.transmission_read(data, 1);

  // done reading
  i2c_soft.transmission_end();

  // Print result
  (failure) ? Serial.println("Failed to read from device") :  Serial.println("Data read");
}

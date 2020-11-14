#ifndef I2C_BB_H_
#define I2C_BB_H_

#include "Arduino.h"

class i2c_bb
{
    public:
        i2c_bb(uint8_t sda = 21, uint8_t scl = 22);

        void transmission_begin(uint8_t adr);
        bool transmission_write(uint8_t *data, uint8_t len);
        bool transmission_read(uint8_t *data, uint8_t len);
        void transmission_end();

        bool request_from(uint8_t adr, uint8_t *data, uint8_t len);
    protected:
    private:
        uint8_t _sda;
        uint8_t _scl;
        uint8_t _adr;

        uint8_t get_u8();
        void send_u8(uint8_t *data);
        
        void init_state();
        bool check_ack();
        void send_ack();
        void scl_tick();
        void stop();
        void start();
};




#endif // I2C_BB_H_
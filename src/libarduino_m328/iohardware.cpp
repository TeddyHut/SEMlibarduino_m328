/*
 * iohardware.cpp
 *
 * Created: 24/01/2019 4:51:01 AM
 *  Author: teddy
 */

#include "iohardware.h"

#include <avr/io.h>
#include <avr/interrupt.h>

void libmodule::hw::panic()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, true);
    asm("BREAK");
    while(true);
}

libarduino_m328::TWISlave0 *libarduino_m328::TWISlave0::instance = nullptr;
libarduino_m328::TWISlave0 libarduino_m328::twiSlave0;

ISR(TWI_vect)
{
    libarduino_m328::isr_twi_slave0();
}

void libarduino_m328::DigitalOut::set(bool const state)
{
    digitalWrite(pm_pin, state);
}

libarduino_m328::DigitalOut::DigitalOut(uint8_t const pin) : pm_pin(pin)
{
    pinMode(pm_pin, OUTPUT);
}

bool libarduino_m328::DigitalIn::get() const
{
    return digitalRead(pm_pin) ? pm_onlevel : !pm_onlevel;
}

libarduino_m328::DigitalIn::DigitalIn(uint8_t const pin, uint8_t const pinmode /*= INPUT_PULLUP*/, bool const onlevel /*= false*/) : pm_pin(pin), pm_onlevel(onlevel)
{
    pinMode(pm_pin, pinmode);
}

void libarduino_m328::isr_twi_slave0()
{
    if(TWISlave0::instance != nullptr)
        TWISlave0::instance->handle_isr();
}

bool libarduino_m328::TWISlave0::communicating() const
{
    return pm_state == State::Transaction;
}

bool libarduino_m328::TWISlave0::attention() const
{
    return pm_result != Result::Wait;
}

libmodule::twi::TWISlave::Result libarduino_m328::TWISlave0::result() const
{
    return pm_result;
}

void libarduino_m328::TWISlave0::reset()
{
    pm_result = Result::Wait;
}

libmodule::twi::TWISlave::TransactionInfo libarduino_m328::TWISlave0::lastTransaction()
{
    if(pm_result == Result::Received || pm_result == Result::Sent)
        pm_result = Result::Wait;
    return pm_previoustransaction;
}

void libarduino_m328::TWISlave0::set_callbacks(Callbacks *const callbacks)
{
    pm_callbacks = callbacks;
}

void libarduino_m328::TWISlave0::set_address(uint8_t const addr)
{
    if(!communicating()) {
        //Set address
        TWAR = addr << 1;
        //Enable address match, TWI, and TWI interrupt
        TWCR = 1 << TWEA | 1 << TWEN | 1 << TWIE;
    } else libmodule::hw::panic();
}

void libarduino_m328::TWISlave0::set_recvBuffer(uint8_t buf[], uint8_t const len)
{
    //TODO: See if something like pm_recvbuf = {buf, len, pos};
    pm_recvbuf.buf = buf;
    pm_recvbuf.len = len;
    enableCheck();
}

void libarduino_m328::TWISlave0::set_sendBuffer(uint8_t const buf[], uint8_t const len)
{
    pm_sendbuf.buf = buf;
    pm_sendbuf.len = len;
    enableCheck();
}

void libarduino_m328::TWISlave0::handle_isr()
{
    //Mask out prescale bits
    auto twsr = TWSR & 0b11111000;
    switch(twsr) {
    //---Miscellaneous States---
    default:
    case 0xf8: //No relevant state information available;
        libmodule::hw::panic();
        break;
    case 0x00: //Bus error due to an illegal start or stop condition;
        pm_state = State::Idle;
        pm_result = Result::Error;
        pm_currenttransaction.buf = nullptr;
        pm_currenttransaction.len = 0;
        TWCR |= 1 << TWSTO;
        break;
    //---Slave Receiver Mode---
    case 0x60: //SLA+W received; ACK returned;
    case 0x68: //Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned;
    case 0x70: //General call address; ACK has been returned;
    case 0x78: //Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned;
        pm_state = State::Transaction;
        pm_bufpos = 0;
        pm_currenttransaction.buf = pm_recvbuf.buf;
        pm_currenttransaction.dir = TransactionInfo::Type::Receive;
        //Receive data and send ACK
        TWCR |= TWEA;
        break;
    case 0x80: //Previously addressed with own SLA+W; data has been received; ACK has been returned;
    case 0x90: //Previously addressed with general call; data has been received; ACK has been returned;
        //If the (past the) end has been reached, respond with NACK and discard data;
        if(pm_bufpos >= pm_recvbuf.len) {
            pm_result = Result::NACKSent;
            //NACK returned by default (TWEA set to 0)
            break;
        }
        pm_recvbuf.buf[pm_bufpos++] = TWDR;
        //Respond with ACK, and receive next byte
        TWCR |= TWEA;
        break;
    //---Slave Transmitter Mode---
    case 0xa8: //Own SLA_R has been received; ACK has been returned;
    case 0xb0: //Arbitration lost in SLA+R/W as Master; Own SLA+R has been received; ACK has been returned;
        pm_state = State::Transaction;
        pm_bufpos = 0;
        pm_currenttransaction.buf = pm_sendbuf.buf;
        pm_currenttransaction.dir = TransactionInfo::Type::Send;
    //[[fallthrough]];
    case 0xb8: //Data byte in TWDR has been transmitted; ACK has been received;
        //If len has been reached, send 0
        if(pm_bufpos >= pm_sendbuf.len)
            TWDR = 0;
        //Otherwise, send data in buffer
        else
            TWDR = pm_sendbuf.buf[pm_bufpos++];
        //Data byte will be transmitted and ACK should be received
        TWCR |= TWEA;
        break;
    //---Finish Transaction---
    case 0xc8: //Last data byte in TWDR has been transmitted; ACK has been received;
    case 0xc0: //Data byte in TWDR has been transmitted; NOT ACK has been received;
    case 0x88: //Previously addressed with owm SLA+W; data has been received; NOT ACK has been returned;
    case 0x98: //Previously addressed with general call; data has been received; NOT ACK has been returned;
    case 0xa0: //A STOP condition or repeated START condition has been received while still addressed as slave;
        //Fill previous transaction struct
        pm_previoustransaction = pm_currenttransaction;
        pm_previoustransaction.len = pm_bufpos;
        //Clear current transaction struct
        pm_currenttransaction.buf = nullptr;
        pm_currenttransaction.len = 0;
        //Set result as appropriate
        pm_result = (pm_previoustransaction.dir == TransactionInfo::Type::Send ? Result::Sent : Result::Received);
        //Make callbacks
        if(pm_callbacks != nullptr) {
            if(pm_previoustransaction.dir == TransactionInfo::Type::Send)
                pm_callbacks->sent(pm_previoustransaction.buf, pm_previoustransaction.len);
            else
                pm_callbacks->received(pm_previoustransaction.buf, pm_previoustransaction.len);
        }
        //Since at this stage we don't know whether it was a repeated START or a STOP, set state to Idle
        pm_state = State::Idle;
        //Switch to not addressed Slave mode; own SLA will be recognised;
        TWCR |= TWEA;
        break;
    }
    //Clear interrupt flag and run next operation.
    TWCR |= 1 << TWINT;
}

void libarduino_m328::TWISlave0::enableCheck()
{
    if(pm_sendbuf.buf != nullptr && pm_sendbuf.len > 0 && pm_recvbuf.buf != nullptr && pm_recvbuf.buf > 0 && TWAR != 0x00 && !communicating()) {
        //Enable address match, TWI and TWI interrupt
        TWCR = 1 << TWEA | 1 << TWEN | 1 << TWIE;
    } else if(!communicating()) {
        //Disable TWI
        TWCR = 0x00;
    }
    //Perhaps panic if this is called while communicating
}

libarduino_m328::TWISlave0::TWISlave0()
{
    instance = this;
}

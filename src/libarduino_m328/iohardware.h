/*
 * iohardware.h
 *
 * Created: 24/01/2019 4:50:39 AM
 *  Author: teddy
 */

#pragma once

#include <Arduino.h>
#include <libmodule/utility.h>
#include <libmodule/twislave.h>

namespace libmodule
{
    namespace hw
    {
        void panic(const char *);
    }
}

namespace libarduino_m328
{
    class DigitalOut : public libmodule::utility::Output<bool>
    {
    public:
        //Calls Arduino digitalWrite()
        void set(bool const state) override;
        //Calls Arduino pinMode()
        DigitalOut(uint8_t const pin);
    private:
        uint8_t const pm_pin;
    };
    class DigitalIn : public libmodule::utility::Input<bool>
    {
    public:
        //Calls Arduino digitalRead()
        bool get() const override;
        //Calls Arduino pinMode()
        DigitalIn(uint8_t const pin, uint8_t const pinmode = INPUT_PULLUP, bool const onlevel = false);
    private:
        uint8_t const pm_pin;
        bool const pm_onlevel;
    };

    //The decision to re-implement an interface instead of using the Arduino TWI is because the Arduino TWI is blocking
    //It also doesn't provide enough information.

    void isr_twi_slave0();

    class TWISlave0 : public libmodule::twi::TWISlave
    {
        friend void isr_twi_slave0();
    public:
        bool communicating() const override;
        bool attention() const override;
        Result result() const override;
        void reset() override;
        TransactionInfo lastTransaction() override;
        void set_callbacks(Callbacks *const callbacks) override;
        void set_address(uint8_t const addr) override;
        void set_recvBuffer(uint8_t buf[], uint8_t const len) override;
        void set_sendBuffer(uint8_t const buf[], uint8_t const len) override;

        TWISlave0();
    private:
        static TWISlave0 *instance;
        void handle_isr();
        //Enables TWI if buffers have content, and address register != 0x00
        void enableCheck();

        enum class State {
            Idle,
            Transaction,
        } pm_state = State::Idle;

        Callbacks *pm_callbacks = nullptr;
        volatile Result pm_result = Result::Wait;
        volatile TransactionInfo pm_previoustransaction;
        TransactionInfo pm_currenttransaction;

        struct {
            uint8_t *buf = nullptr;
            uint8_t len = 0;
        } volatile pm_recvbuf;
        struct {
            uint8_t const *buf = nullptr;
            uint8_t len = 0;
        } volatile pm_sendbuf;
        uint8_t pm_bufpos = 0;
    };
    extern TWISlave0 twiSlave0;
}

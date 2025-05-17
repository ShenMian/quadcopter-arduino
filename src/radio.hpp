#pragma once

#include <RF24.h>

#include "config.hpp"

class Radio {
  public:
    Radio(
        uint8_t ce,
        uint8_t csn,
        const uint8_t* write_address,
        const uint8_t* read_address
    ) {
        radio_.begin(ce, csn);
        radio_.setPALevel(RF24_PA_MAX, true);
        radio_.startListening();
        radio_.openWritingPipe(write_address);
        radio_.openReadingPipe(1, read_address);
    }

    template<typename T>
    bool write(const T& package) {
        static_assert(
            sizeof(T) <= 32,
            "nRF24L01 maximum payload size is 32 bytes"
        );
        return radio_.write(&package, sizeof(T));
    }

    template<typename T>
    bool read(T& package) {
        static_assert(sizeof(T) <= 32);
        if (radio_.available())
            return false;
        radio_.read(&package, sizeof(package));
        return true;
    }

  private:
    RF24 radio_;
};
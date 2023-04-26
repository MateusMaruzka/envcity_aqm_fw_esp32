#ifndef MUX
#define MUX

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>


#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdexcept>

class mux{

    public:

    mux(gpio_num_t *pinNumbers, size_t len);
    ~mux(){};

    int inc();
    int dec();
    int selectOutput(uint32_t pinNumber);

    uint32_t getValue(){return this->value;};
    uint32_t getLen(){return this->len;};

    gpio_num_t *pins;

    private:    
    
    uint32_t value;
    uint32_t len;

};

#endif /* MUX */

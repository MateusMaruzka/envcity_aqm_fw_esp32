#include "mux.hpp"

#define TAG "mux"

mux::mux(gpio_num_t *pinNumbers, size_t len){

        uint64_t pinSel = 0ULL; 
        
        this->len = len;
        this->value = 0;

        for(auto i = 0; i < len; i++){
        
            if(!GPIO_IS_VALID_OUTPUT_GPIO(pinNumbers[i])){
                //throw std::invalid_argument("Invalid pin number");
                ESP_LOGE(TAG, "Invalid pin number");
                return;
            }
            
            pinSel |= (1ULL << pinNumbers[i]);
        
        }
        /*for(int i = 0; i < 39; i++){
            ESP_LOGE(TAG, "GPIO%d=%d", i, GPIO_IS_VALID_OUTPUT_GPIO((gpio_num_t)i));
        }*/
        //ESP_LOGE(TAG, "pinSel %lld", pinSel);

        gpio_config_t config = {
        .pin_bit_mask = pinSel,
        .mode         = (gpio_mode_t)GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE
        };
        
        ESP_ERROR_CHECK(gpio_config(&config));

        //this->pins = (gpio_num_t *) malloc(len * sizeof(gpio_num_t));
        this->pins = new gpio_num_t[len];

        if(this->pins != nullptr){
            memcpy(this->pins, pinNumbers, len * sizeof(gpio_num_t));
            ESP_LOGE(TAG, "hyehye");
        }else{
            ESP_LOGE(TAG, "Failed to allocate memory for mux pins");
        }
    }


int mux::inc(){
    
    for(uint32_t i = 0; i < this->len; i++){
    
        gpio_set_level(this->pins[i], (++this->value & (1 << i)) >> i); 

    }

    return 0;
}

int mux::dec(){
    
    for(uint32_t i = 0; i < this->len; i++){
    
        gpio_set_level(this->pins[i], (--this->value & (1 << i)) >> i); 

    }

    return 0;
}

int mux::selectOutput(uint32_t pinNumber){

    for(uint32_t i = 0; i < this->len; i++){
    
        gpio_set_level(this->pins[i], (pinNumber & (1 << i)) >> i); 

    }

    this->value = pinNumber;

    return 0;

}



#include <unity.h>

#include "Freertos/FreeRTOS.h"
#include "Freertos/task.h"
#include "esp_err.h"
#include "mux.hpp"

// void setUp(void) {
// // set stuff up here
// }

// void tearDown(void) {
// // clean stuff up here
// }

void test_mux_constructor(void) {
    gpio_num_t pins[] = {GPIO_NUM_20, GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23};
    mux *ptr = new mux(pins, 4);
    TEST_ASSERT_NOT_NULL(ptr);
    delete ptr;

    //pins[0] = GPIO_NUM_34; // GPIO28 is a invalid pin
    //ptr = new (std::nothrow) mux(pins, 4);
    //TEST_ASSERT_NULL(ptr);
}

void test_mux_selectOutput(void) {

    gpio_num_t pins[] = {GPIO_NUM_12, GPIO_NUM_0, GPIO_NUM_25};
    mux *ptr = new mux(pins, 3);
    TEST_ASSERT_NOT_NULL(ptr);
    
    ptr->selectOutput(0b111);

    for(int i = 0; i < ptr->getLen(); i++){
        TEST_ASSERT_EQUAL(1, gpio_get_level(ptr->pins[i]));  
    }
    
}

void test_function_calculator_multiplication(void) {
    TEST_ASSERT_EQUAL(50, 50);
}

void test_function_calculator_division(void) {
    TEST_ASSERT_EQUAL(32, 31);
}


extern "C" void app_main() {
    
    ESP_LOGE("hehe", "hehe");
    UNITY_BEGIN();
    RUN_TEST(test_mux_constructor);
    RUN_TEST(test_mux_selectOutput);
    RUN_TEST(test_function_calculator_multiplication);
    //RUN_TEST(test_function_calculator_division);
    UNITY_END();

    while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    };
}
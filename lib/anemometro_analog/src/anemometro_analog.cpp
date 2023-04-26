#include "anemometro_analog.hpp"


float Anemometro::windSpeed(float voltage){

    /*
    0 - > 0.4
    2 -> 32.4

    y = a*x + b
    0.4 = a*0 + b -> b = 0.4
    32.4 = a * 2 + 0.4
    (32.4 - 0.4)/2 = a

    y = 16 * x + 0.4

    */
    return 16.0 * voltage + 0.4; 
}
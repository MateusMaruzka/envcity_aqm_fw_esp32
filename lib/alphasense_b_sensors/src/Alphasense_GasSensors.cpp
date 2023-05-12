#include <iostream>
#include "Alphasense_GasSensors.hpp"


float ajuste_temp[6][4][9] = {
   {{0.70, 0.70, 0.70, 0.70, 1.00, 3.00, 3.50, 4.00, 4.50}, {0.20, 0.20, 0.20, 0.20, 0.30, 1.00, 1.20, 1.30, 1.50}, {-1.00, -0.50, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 1.00}, {55.00, 55.00, 55.00, 50.00, 31.00, 0.00, -50.00, -150.00, -250.00}},
   {{-0.60, -0.60, 0.10, 0.80, -0.70, -2.50, -2.50, -2.20, -1.80}, {0.20, 0.20, 0.00, -0.30, 0.30, 1.00, 1.00, 0.90, 0.70}, {-14.00, -14.00, 3.00, 3.00, 2.00, 1.00, -1.20, -1.20, -1.20}, {52.00, 51.00, 48.00, 45.00, 26.00, 0.00, -65.00, -125.00, -180.00}},
   {{2.90, 2.90, 2.20, 1.80, 1.70, 1.60, 1.50, 1.40, 1.30}, {1.80, 1.80, 1.40, 1.10, 1.10, 1.00, 0.90, 0.90, 0.80}, {0.80, 0.80, 0.80, 0.80, 0.90, 1.00, 1.10, 1.20, 1.30}, {-25.00, -25.00, -25.00, -25.00, -16.00, 0.00, 56.00, 200.00, 615.00}},
   {{1.30, 1.30, 1.30, 1.30, 1.00, 0.60, 0.40, 0.20, -1.50}, {2.20, 2.20, 2.20, 2.20, 1.70, 1.00, 0.70, 0.30, -2.50}, {1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 0.40, -0.10, -4.00}, {7.00, 7.00, 7.00, 7.00, 4.00, 0.00, 0.50, 5.00, 67.00}},
   {{0.90, 0.90, 1.00, 1.30, 1.50, 1.70, 2.00, 2.50, 3.70}, {0.50, 0.50, 0.60, 0.80, 0.90, 1.00, 1.20, 1.50, 2.20}, {0.50, 0.50, 0.50, 0.60, 0.60, 1.00, 2.80, 5.00, 5.30}, {1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 8.50, 23.00, 103.00}},
   {{1.60, 1.60, 1.60, 1.60, 1.60, 1.60, 1.90, 3.00, 5.80}, {1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.20, 1.90, 3.60}, {1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 2.00, 3.50, 7.00}, {-4.00, -4.00, -4.00, -4.00, -4.00, 0.00, 20.00, 140.00, 450.00}},
};

/*std::map<std::string, std::array<std::array<float, 9>, 4>> ajuste_temp = {

        {"CO-B4", {{{0.70, 0.70, 0.70, 0.70, 1.00, 3.00, 3.50, 4.00, 4.50}, {0.20, 0.20, 0.20, 0.20, 0.30, 1.00, 1.20, 1.30, 1.50}, {-1.00, -0.50, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 1.00}, {55.00, 55.00, 55.00, 50.00, 31.00, 0.00, -50.00, -150.00, -250.00}}}},

        {"CO-A4", {{{1.00, 1.00, 1.00, 1.00, -0.20, -0.90, -1.50, -1.50, -1.50}, {-1.10, -1.10, -1.10, -1.10, 0.20, 1.00, 1.70, 1.70, 1.70}, {1.90, 2.90, 2.70, 3.90, 2.10, 1.00, -0.60, -0.30, -0.50}, {13.00, 12.00, 16.00, 11.00, 4.00, 0.00, -15.00, -18.00, -36.00}}}},

        {"H2S-A4", {{{3.00, 3.00, 3.00, 1.00, -1.00, -2.00, -1.50, -1.00, -0.50}, {-1.50, -1.50, -1.50, -0.50, 0.50, 1.00, 0.80, 0.50, 0.30}, {9.00, 9.00, 9.00, 9.00, 3.00, 1.00, 0.30, 0.30, 0.30}, {50.00, 46.00, 43.00, 37.00, 25.00, 0.00, -8.00, -16.00, -20.00}}}},

        {"H2S-B4", {{{-0.60, -0.60, 0.10, 0.80, -0.70, -2.50, -2.50, -2.20, -1.80}, {0.20, 0.20, 0.00, -0.30, 0.30, 1.00, 1.00, 0.90, 0.70}, {-14.00, -14.00, 3.00, 3.00, 2.00, 1.00, -1.20, -1.20, -1.20}, {52.00, 51.00, 48.00, 45.00, 26.00, 0.00, -65.00, -125.00, -180.00}}}},

        {"NO-A4", {{{1.70, 1.70, 1.60, 1.50, 1.50, 1.50, 1.50, 1.60, 1.70}, {1.10, 1.10, 1.10, 1.00, 1.00, 1.00, 1.00, 1.10, 1.10}, {0.70, 0.70, 0.70, 0.70, 0.80, 1.00, 1.20, 1.40, 1.60}, {-25.00, -25.00, -25.00, -25.00, -16.00, 0.00, 56.00, 200.00, 615.00}}}},

        {"NO-B4", {{{2.90, 2.90, 2.20, 1.80, 1.70, 1.60, 1.50, 1.40, 1.30}, {1.80, 1.80, 1.40, 1.10, 1.10, 1.00, 0.90, 0.90, 0.80}, {0.80, 0.80, 0.80, 0.80, 0.90, 1.00, 1.10, 1.20, 1.30}, {-25.00, -25.00, -25.00, -25.00, -16.00, 0.00, 56.00, 200.00, 615.00}}}},

        {"NO2-A43F", {{{0.80, 0.80, 1.00, 1.20, 1.60, 1.80, 1.90, 2.50, 3.60}, {0.40, 0.40, 0.60, 0.70, 0.90, 1.00, 1.10, 1.40, 2.00}, {0.20, 0.20, 0.20, 0.20, 0.70, 1.00, 1.30, 2.10, 3.50}, {-4.00, -4.00, -4.00, -4.00, -2.00, 0.00, 10.00, 35.00, 132.00}}}},

        {"NO2-B43F", {{{1.30, 1.30, 1.30, 1.30, 1.00, 0.60, 0.40, 0.20, -1.50}, {2.20, 2.20, 2.20, 2.20, 1.70, 1.00, 0.70, 0.30, -2.50}, {1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 0.40, -0.10, -4.00}, {7.00, 7.00, 7.00, 7.00, 4.00, 0.00, 0.50, 5.00, 67.00}}}},

        {"OX-A431", {{{1.00, 1.20, 1.20, 1.60, 1.70, 2.00, 2.10, 3.40, 4.60}, {0.50, 0.60, 0.60, 0.80, 0.90, 1.00, 1.10, 1.70, 2.30}, {0.10, 0.10, 0.20, 0.30, 0.70, 1.00, 1.70, 3.00, 4.00}, {-5.00, -5.00, -4.00, -3.00, 0.50, 0.00, 9.00, 42.00, 134.00}}}},

        {"OX-B431", {{{0.90, 0.90, 1.00, 1.30, 1.50, 1.70, 2.00, 2.50, 3.70}, {0.50, 0.50, 0.60, 0.80, 0.90, 1.00, 1.20, 1.50, 2.20}, {0.50, 0.50, 0.50, 0.60, 0.60, 1.00, 2.80, 5.00, 5.30}, {1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 8.50, 23.00, 103.00}}}},

        {"SO2-A4", {{{1.30, 1.30, 1.30, 1.20, 0.90, 0.40, 0.40, 0.40, 0.40}, {3.30, 3.30, 3.30, 3.00, 2.30, 1.00, 1.00, 1.00, 1.00}, {1.50, 1.50, 1.50, 1.50, 1.00, 1.00, 1.00, 1.00, 1.00}, {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 5.00, 25.00, 45.00}}}},

        {"SO2-B4", {{{1.60, 1.60, 1.60, 1.60, 1.60, 1.60, 1.90, 3.00, 5.80}, {1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.20, 1.90, 3.60}, {1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 2.00, 3.50, 7.00}, {-4.00, -4.00, -4.00, -4.00, -4.00, 0.00, 20.00, 140.00, 450.00}}}}

};*/


AlphasenseGasSensor::AlphasenseGasSensor(AlphasenseSensorParam param){

    //this->_sensorModel = param.model;
    //this->_sensorNum = param.sensorNum;
    //this->_boardType = param.boardType;
    this->model = param._model;
    this->_gain = param.gain;
    this->_sensitivity = param.sensitivity;
    this->_ae_zero = param.ae_zero;
    this->_we_zero = param.we_zero;
    this->_we_sensor = param.we_sensor;
   // this->_ae_sensor = param.ae_sensor;
    this->_electr_we = param.electronic_we;
    this->_electr_ae = param.electronic_ae;

    //kt = ajuste_temp[param.model][0].data();
    kt = &ajuste_temp[param._model][0][0];

}

void AlphasenseGasSensor::sensorConfiguration(){

    std::cout << "Model:" <<  _sensorModel << std::endl;
    std::cout << "Sensor Number:" << _sensorNum << std::endl;
    std::cout << "Board Type:" <<  _boardType << std::endl;
    // std::cout << "Primary Algorithm:",  corrected_we.__name__ << std::endl;
    // std::cout << "Secondary Algorithm:",  func_aux_wec.__name__ << std::endl;
    std::cout << "Gain:" <<  _gain << "[mV/nA]" << std::endl;
    std::cout << "Sensitivity:" <<  _sensitivity << "[mV/ppb]" << std::endl;
    std::cout << "-----------Working Electrode-----------" << std::endl;
    std::cout << "Electronic WE:" <<  _electr_we << "[mV]" << std::endl;
    std::cout << "WE Zero:" << _we_zero << "[mV]" << std::endl;
    std::cout << "WE sensor:" <<  _we_sensor << "[nA/ppm]" << std::endl;
    std::cout << "-----------Aux Electrode-----------" << std::endl;
    std::cout << "Electronic AE:" <<  _electr_ae << "[mV]" << std::endl;
    std::cout << "AE Zero:" << _ae_zero << "[mV]" << std::endl;
        
}

float AlphasenseGasSensor::algorithm1(float raw_we, float raw_ae, float temp){
    //auto kt = ajuste_temp["CO-B4"][temp];
    //std::cout << kt << std::endl;
    auto t = *(kt + (int)(temp / 10) + 3);
    return (raw_we - this->_electr_we ) - t*(raw_ae - this->_electr_ae);
}

float AlphasenseGasSensor::algorithm2(float raw_we, float raw_ae, float temp){
    //auto kt = ajuste_temp["CO-B4"][temp / 10 + 3];
    // auto kt = this->kt[1][temp / 10 + 3];
    auto t = *(kt + (int)(temp / 10) + 3 + 1*9);
    return (raw_we - this->_electr_we) - (this->_we_zero / this->_ae_zero)*t*(raw_ae - this->_electr_ae);
}

float AlphasenseGasSensor::algorithm3(float raw_we, float raw_ae, float temp){
    
    auto t = *(kt + (int)(temp / 10) + 3 + 2*9);
    return (raw_we - this->_electr_we) - (this->_we_zero - this->_ae_zero) - t*(raw_ae - this->_electr_ae);
}

float AlphasenseGasSensor::algorithm4(float raw_we, float temp){
    
    auto t = *(kt + (int)(temp / 10) + 3 + 3*9);
    return (raw_we - this->_electr_we - this->_we_zero - t);
}


void AlphasenseGasSensor::fourAlgorithms(float we, float ae, float *v, float temp){

    v[0] = this->algorithm1(we, ae, temp) / this->_sensitivity;
    v[1] = this->algorithm2(we, ae, temp) / this->_sensitivity;
    v[2] = this->algorithm3(we, ae, temp) / this->_sensitivity;
    v[3] = this->algorithm4(we, temp)     / this->_sensitivity;

}

float AlphasenseGasSensor::simpleRead(float raw_we, float raw_ae){

    return ((raw_we - this->_electr_we) - (raw_ae - this->_electr_ae))/this->_sensitivity;
}

float AlphasenseGasSensor::ppb(float raw_we, float raw_ae, float temp){
    return this->simpleRead(raw_we, raw_ae) / this->_sensitivity;
}

Alphasense_COB4::Alphasense_COB4(AlphasenseSensorParam param) : AlphasenseGasSensor(param){}

float Alphasense_COB4::ppb(float we, float ae, float temp) {
    return (float)algorithm1(we, ae, temp);
}

float Alphasense_H2S::ppb(float we, float ae, float temp){
    return this->algorithm1(we, ae, temp) / this->getSensitivity();
}

float Alphasense_SO2::ppb(float we, float ae, float temp){
    return algorithm1(we, ae, temp);
}

float Alphasense_NH3::ppb(float we, float ae, float temp){
    return simpleRead(we, ae);
}

float Alphasense_OX::ppb(float we_raw, float ae_raw, float no2, float temp){

    return this->algorithm1((we_raw - no2*this->no2_sensitivity*getGain()), ae_raw, temp);
}

void Alphasense_OX::fourAlgorithms(float we_raw, float ae_raw, float *ppb, float no2, float temp){

    ppb[0] = this->algorithm1((we_raw - no2*this->no2_sensitivity*getGain()), ae_raw, temp);
    ppb[1] = this->algorithm2((we_raw - no2*this->no2_sensitivity*getGain()), ae_raw, temp);
    ppb[2] = this->algorithm3((we_raw - no2*this->no2_sensitivity*getGain()), ae_raw, temp);
    ppb[3] = this->algorithm4((we_raw - no2*this->no2_sensitivity*getGain()), temp);

}


std::ostream& operator<< (std::ostream& os, AlphasenseGasSensor& obj) {
    return obj.print(os);
}

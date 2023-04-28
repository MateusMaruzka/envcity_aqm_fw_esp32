#ifndef ALPHASENSE_GASSENSORS
#define ALPHASENSE_GASSENSORS

#include <iostream>
#include <map>
#include <string>
#include <array>

typedef enum {COB4_n, H2SB4_n, NH3, NO2B43F_n, OXB431_n, SO2B4_n} AlphasenseModel;


struct AlphasenseSensorParam {

    std::string model;
    AlphasenseModel _model;
    double gain;
    int we_zero;
    int ae_zero;
    int we_sensor;
   // double ae_sensor;
    double sensitivity;
    int electronic_we;
    int electronic_ae;
    double no2_sensitivity;

};

class AlphasenseGasSensor
{

public:

    AlphasenseGasSensor() {};
    AlphasenseGasSensor(AlphasenseSensorParam);

    virtual float ppb(float, float, float);
    
    void sensorConfiguration();

    virtual float algorithm1(float raw_we, float raw_ae, float temp);
    virtual float algorithm2(float raw_we, float raw_ae, float temp);
    virtual float algorithm3(float raw_we, float raw_ae, float temp);
    virtual float algorithm4(float raw_we, float temp);

    virtual void fourAlgorithms(float we, float ae, float *v, float temp);

    float simpleRead(float raw_we, float raw_ae);

    float getSensitivity(){return this->_sensitivity;}

    double getGain(){
        return this->_gain;
    }

    std::ostream& print(std::ostream& os)  const {
        return os << "Teste Sobrecarga" << std::endl;
    }

private:
    
    std::string _sensorModel;
    AlphasenseModel model;
    int _sensorNum;
    int _boardType;
    double _gain, _sensitivity;
    double _ae_zero, _we_zero;
    double _we_sensor;
    double _electr_we, _electr_ae;

    float *kt;
};

// Alphasense_COB4::Alphasense_COB4(AlphasenseSensorParam param) : AlphasenseGasSensor(param){}
class Alphasense_COB4 : public AlphasenseGasSensor
{
public:
    Alphasense_COB4(AlphasenseSensorParam);
    float ppb(float, float, float);
};

class Alphasense_NH3 : public AlphasenseGasSensor
{
public:
    Alphasense_NH3(AlphasenseSensorParam param) : AlphasenseGasSensor(param) {}
    float ppb(float, float, float);
};

class Alphasense_H2S : public AlphasenseGasSensor
{

public:
    Alphasense_H2S(AlphasenseSensorParam param) : AlphasenseGasSensor(param) {}
    float ppb(float, float, float);
};

class Alphasense_SO2 : public AlphasenseGasSensor
{

public:
    Alphasense_SO2(AlphasenseSensorParam param) : AlphasenseGasSensor(param) {}
    float ppb(float, float, float);
};

class Alphasense_NO2 : public AlphasenseGasSensor
{

public:
    Alphasense_NO2(AlphasenseSensorParam param) : AlphasenseGasSensor(param) {}
};

class Alphasense_OX : public AlphasenseGasSensor
{
private:
    double no2_sensitivity;

public:
///    Alphasense_OX(AlphasenseSensorParam param) : AlphasenseGasSensor(param) {}
    Alphasense_OX(AlphasenseSensorParam param) :  AlphasenseGasSensor(param){
        this->no2_sensitivity = param.no2_sensitivity;
    }

    float ppb(float we_raw, float ae_raw, float no2, float temp);
    void fourAlgorithms(float we, float ae, float *ppb, float no2, float temp);
};

std::ostream& operator<< (std::ostream& os, AlphasenseGasSensor& obj);


#endif /* ALPHASENSE_GASSENSORS */

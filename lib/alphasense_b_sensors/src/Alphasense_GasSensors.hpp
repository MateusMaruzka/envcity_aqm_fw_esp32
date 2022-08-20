#ifndef ALPHASENSE_GASSENSORS
#define ALPHASENSE_GASSENSORS

#include <iostream>
#include <map>
#include <string>
#include <array>


//extern std::map<std::string, std::array<std::array<float, 9>, 4>> ajuste_temp;

typedef enum {COB4_n, H2SB4_n, NOB4_n, NO2B43F_n, OXB431_n, SO2B4_n} AlphasenseModel;


struct AlphasenseSensorParam {

    std::string model;
    AlphasenseModel _model;
    double gain;
    int we_zero;
    int ae_zero;
    int we_sensor;
    double ae_sensor;
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

    float ppb();
    
    void sensorConfiguration();

    virtual double algorithm1(float raw_we, float raw_ae, float temp);
    virtual double algorithm2(float raw_we, float raw_ae, float temp);
    virtual double algorithm3(float raw_we, float raw_ae, float temp);
    virtual double algorithm4(float raw_we, float temp);

    double simpleRead(float raw_we, float raw_ae);

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
    double _we_sensor, _ae_sensor;
    double _electr_we, _electr_ae;

    float *kt;
};

// Alphasense_COB4::Alphasense_COB4(AlphasenseSensorParam param) : AlphasenseGasSensor(param){}
class Alphasense_COB4 : public AlphasenseGasSensor
{
public:
    Alphasense_COB4(AlphasenseSensorParam);
    void teste();
};

class Alphasense_NH3 : public AlphasenseGasSensor
{
public:
    Alphasense_NH3(AlphasenseSensorParam param) : AlphasenseGasSensor(param) {}
    void teste();
};

class Alphasense_H2S : public AlphasenseGasSensor
{

public:
    Alphasense_H2S(AlphasenseSensorParam param) : AlphasenseGasSensor(param) {}
    void teste();
};

class Alphasense_SO2 : public AlphasenseGasSensor
{

public:
    Alphasense_SO2(AlphasenseSensorParam param) : AlphasenseGasSensor(param) {}
    void teste();
};

class Alphasense_NO2 : public AlphasenseGasSensor
{

public:
    Alphasense_NO2(AlphasenseSensorParam param) : AlphasenseGasSensor(param) {}
    void teste();
};

class Alphasense_OX : public AlphasenseGasSensor
{

public:
    Alphasense_OX(AlphasenseSensorParam param) : AlphasenseGasSensor(param) {}
    void teste();
};

std::ostream& operator<< (std::ostream& os, AlphasenseGasSensor& obj);


#endif /* ALPHASENSE_GASSENSORS */

#ifndef ANEMOMETRO_ANALOG
#define ANEMOMETRO_ANALOG

class Anemometro {

    public:
        Anemometro(){};
        ~Anemometro(){};

        float windSpeed(float voltage);

};



#endif /* ANEMOMETRO_ANALOG */

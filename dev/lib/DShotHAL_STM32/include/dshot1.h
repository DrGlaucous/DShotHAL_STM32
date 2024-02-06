#pragma once



//#include "tim.h"    	// header from stm32cubemx code generate

#include <Arduino.h>

#include <stdbool.h>	
#include <math.h>		// lrintf


/* Enumeration */
typedef enum dshot_type_e
{
    DSHOT150,
    DSHOT300,
    DSHOT600,

} dshot_type_t;




class DShotHAL{


    public:
    DShotHAL();
    ~DShotHAL();

    void begin();
    void stop();

    void send_dshot_packet(uint16_t packet);


    private:


    HardwareTimer* outputTimer;




};












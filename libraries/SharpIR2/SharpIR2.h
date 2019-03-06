/*
	SharpIR
	Arduino library for retrieving distance (in cm) from the analog GP2Y0A21Y and GP2Y0A02YK
	From an original version of Dr. Marcal Casas-Cartagena (marcal.casas@gmail.com)     
	
    Version : 1.0 : Guillaume Rico
	https://github.com/guillaume-rico/SharpIR
*/

#ifndef SharpIR2_h
#define SharpIR2_h

#define NB_SAMPLE 1

#ifdef ARDUINO
  #include "Arduino.h"
#elif defined(SPARK)
  #include "Particle.h"
#endif

class SharpIR2
{
  public:

    SharpIR2 (int irPin, long sensorModel);
    double distance();

  private:

    void sort(int a[], int size);
    
    int _irPin;
    long _model;
};

#endif
#ifndef OBSTACLESENSOR_H_
#define OBSTACLESENSOR_H_

#include <Arduino.h>

class ObstacleSensor {
private:
    byte _ir_sensor_pin;
public:
    ObstacleSensor( byte ir_sensor_pin )
        :  _ir_sensor_pin( ir_sensor_pin ) {
        pinMode( ir_sensor_pin, INPUT );
    }

    bool has_obstacle() {
        return digitalRead( _ir_sensor_pin ) == LOW;
    }
}; // class ObstacleSensor

#endif // OBSTACLESENSOR_H_

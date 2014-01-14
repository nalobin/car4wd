#ifndef CARDIRECT_H_
#define CARDIRECT_H_

#include <Arduino.h>
#include <DistanceSensor.h>

const byte MAX_AXES = 3;

const byte LEFT  = 0;
const byte RIGHT = 1;

const byte FORWARD  = LOW;
const byte BACKWARD = HIGH;

const int DISTANCE_SENSOR_CHECK_INTERVAL  = 10; // ms
const int DISTANCE_SENSOR_MIN_DISTANCE    = 5; // cm
const int DISTANCE_SENSOR_PROBE_MAX       = 20; // попытки поиска пути
const int DISTANCE_SENSOR_PROBE_INTERVAL  = 300; // интервал между попытками поиска пути

struct Wheel {
    byte speed_pin;
    byte dir_pin1;
    byte dir_pin2;
    float speed;
    float speed_step;
    byte dir;
};

// Класс n-осевой тачки с попарными неповоротными колёсами
class CarDirect {
private:
    byte _axes;
    Wheel _wheels    // колёса
        [2]          // сторона LEFT | RIGHT
        [MAX_AXES];  // индекс оси колеса; 0 - передняя ось
    float _max_speed;
    unsigned int _stop_fluent_tick;
    DistanceSensor *_front_sensor;
    DistanceSensor *_rear_sensor;

    int delay_checking_sensors( byte dir, int delay_ms ) {
        if (   dir == FORWARD  && !_front_sensor
            || dir == BACKWARD && !_rear_sensor ) {
            delay( delay_ms );
            return -1;
        }

        DistanceSensor *sensor = dir == FORWARD ? _front_sensor : _rear_sensor;

        int checks = delay_ms / DISTANCE_SENSOR_CHECK_INTERVAL;

        for ( int check = 0; check < checks; check++ ) {
            if ( sensor->getDistanceCentimeter() < DISTANCE_SENSOR_MIN_DISTANCE ) {
                this->stop();
                break;
            }
            
            delay( DISTANCE_SENSOR_CHECK_INTERVAL );
        }

        return sensor->getDistanceCentimeter();
    }
public:
    CarDirect( byte axes, byte wheels[][5], float max_speed = 4000,
        DistanceSensor *front_sensor = 0 , DistanceSensor *rear_sensor = 0 )
        : _axes( axes ), _max_speed( max_speed ), _front_sensor( front_sensor ), _rear_sensor( rear_sensor )
     {
        for ( byte i = 0; i < _axes * 2; i++ ) {
            Wheel &wheel = this->_wheels[ wheels[i][0] ][ wheels[i][1] ];
            wheel.speed_pin = wheels[i][2];
            wheel.dir_pin1  = wheels[i][3];
            wheel.dir_pin2  = wheels[i][4];
            wheel.speed     = 0;
            wheel.dir       = FORWARD;
        }
    }

    // Низкоуровневое управление
    CarDirect &brake_wheel( byte side, byte axis, byte brake = 1, int delay_ms = 0 ) {
        Wheel &wheel = this->_wheels[ side ][ axis ];

        if ( brake ) {
            analogWrite( wheel.speed_pin, 0 );
        }

        wheel.speed = 0;

        if ( delay_ms ) {
            delay( delay_ms );
        }

        return *this;
    }

    CarDirect &rotate_wheel( byte side, byte axis, byte dir, float speed, int delay_ms = 0 ) {
        this->brake_wheel( side, axis, 0, 0 );

        Wheel &wheel = this->_wheels[ side ][ axis ];

        // Считаем что правые колеса движутся вперед при подаче на первый dir_pin HIGH,
        // а левые при подаче на 2ой
        byte forward = dir == FORWARD ^ side == LEFT ? HIGH : LOW;

        digitalWrite( wheel.dir_pin1, forward  );
        digitalWrite( wheel.dir_pin2, !forward );

        speed = min( speed, _max_speed );
        speed = max( speed, 0          );

        byte analog_speed = byte( 0xFF * speed / _max_speed );

        analogWrite( wheel.speed_pin, analog_speed );

        wheel.speed = speed;
        wheel.dir = dir;

        if ( delay_ms ) {
            delay( delay_ms );
        }

        return *this;
    }

    CarDirect &rotate_axis( byte axis, byte dir, float speed, int delay_ms = 0 ) {
        this->rotate_wheel( LEFT , axis, dir, speed );
        this->rotate_wheel( RIGHT, axis, dir, speed );

        if ( delay_ms ) {
            delay( delay_ms );
        }


        return *this;
    }

    CarDirect &rotate_side( byte side, byte dir, float speed, int delay_ms = 0 ) {
        for ( byte axis = 0; axis < _axes; axis++ ) {
            this->rotate_wheel( side, axis, dir, speed );
        }

        if ( delay_ms ) {
            delay( delay_ms );
        }

        return *this;
    }

    // left_coef, right_coef  -- коэф. скорости левых и правых колёс от -1 до 1.
    //             1.0 -- колеса крутятся вперёд с указанной максимальной скоростью (по умолчанию)
    //           0.0 -- колеса неподвижны
    //            -1.0 -- колеса крутятся назад с указанной максимальной скоростью
    CarDirect &go( float speed, int delay_ms = 0, float left_coef = 1.0, float right_coef = 1.0 ) {
        this->rotate_side( LEFT , left_coef  >= 0.0 ? FORWARD : BACKWARD, int( abs( speed ) * left_coef  ) );
        this->rotate_side( RIGHT, right_coef >= 0.0 ? FORWARD : BACKWARD, int( abs( speed ) * right_coef ) );

        if ( delay_ms ) {
            if ( left_coef * right_coef > 1 ) {
                // если едем вперед или назад, то используем датчики расстояния
                this->delay_checking_sensors( left_coef  >= 0.0 ? FORWARD : BACKWARD, delay_ms );
            }
            else {
                delay( delay_ms );
            }
        }

        return *this;
    }

    // Высокоуровневое управление
    CarDirect &stop( int delay_ms = 0 ) {
        for ( byte axis = 0; axis < _axes; axis++ ) {
            this->brake_wheel( LEFT , axis, 1 );
            this->brake_wheel( RIGHT, axis, 1 );
        }

        if ( delay_ms ) {
            delay( delay_ms );
        }

        return *this;
    }

    CarDirect &stop_fluently_init( unsigned int stop_fluent_time, unsigned int stop_fluent_tick  ) {
        _stop_fluent_tick = stop_fluent_tick;

        int cycles = stop_fluent_time / stop_fluent_tick || 1;

        // Расчёт шага изменения скорости
        for ( byte axis = 0; axis < _axes; axis++ ) {
            for ( byte side = LEFT; side < RIGHT; side++  ) {
                this->_wheels[ side ][ axis ].speed_step = -1 * this->_wheels[ side ][ axis ].speed / cycles;
            }
        }
    }

    CarDirect &stop_fluently_tick( unsigned int stop_fluent_tick ) {
        for ( byte axis = 0; axis < _axes; axis++ ) {
            for ( byte side = LEFT; side < RIGHT; side++  ) {
                this->rotate_wheel(
                    side,
                    axis,
                    this->_wheels[ side  ][ axis ].dir,
                    this->_wheels[ side  ][ axis ].speed + this->_wheels[ side  ][ axis ].speed_step
                );
            }
        }

        delay( _stop_fluent_tick );

        return *this;
    }

    CarDirect &forward( float speed, int delay_ms = 0 ) {
        this->go( speed, delay_ms, 1.0, 1.0 );

        return *this;
    }

    CarDirect &backward( float speed, int delay_ms = 0 ) {
        this->go( speed, delay_ms, -1.0, -1.0 );

        return *this;
    }

    CarDirect &rotate_left( float speed, int delay_ms = 0 ) {
        this->go( speed, delay_ms, -1.0, 1.0 );

        return *this;
    }

    CarDirect &rotate_right( float speed, int delay_ms = 0 ) {
        this->go( speed, delay_ms, 1.0, -1.0 );

        return *this;
    }

    // Повернуть с буксом во время движения при помощи большей скорости внешних колёс
    CarDirect &drift( byte side, byte dir, float speed, int delay_ms = 0, float coef = 0.5 ) {
        float dir_coef = dir == FORWARD ? 1 : -1;

        this->go(
            speed,
            delay_ms,
            dir_coef * speed * ( side == LEFT  ? coef : 1.0 ), // left_coef
            dir_coef * speed * ( side == RIGHT ? coef : 1.0 )  // right_coef
        );

        return *this;
    }

    // drift sugar
    CarDirect &drift_forward_left( float speed, int delay_ms = 0, float coef = 0.5 ) {
        this->drift( LEFT, FORWARD, speed, coef );
        return *this;
    }
    CarDirect &drift_forward_right( float speed, int delay_ms = 0, float coef = 0.5 ) {
        this->drift( RIGHT, FORWARD, speed, delay_ms, coef );
        return *this;
    }
    CarDirect &drift_backward_left( float speed, int delay_ms = 0, float coef = 0.5 ) {
        this->drift( LEFT, BACKWARD, speed, delay_ms, coef );
        return *this;
    }
    CarDirect &drift_backward_right( float speed, int delay_ms = 0, float coef = 0.5 ) {
        this->drift( RIGHT, BACKWARD, speed, delay_ms, coef );
        return *this;
    }

    // поворачиваем машину передом в пустое пространсвто
    bool search_path() {
        if ( !_front_sensor ) {
            this->stop();
            return false;
        }

        // Будем крутиться в случайную сторону
        byte dir = random( 0, 1 );
        byte try_num = 0;
        while ( _front_sensor->isCloser( DISTANCE_SENSOR_MIN_DISTANCE * 10 ) ) {
            if ( try_num++ > DISTANCE_SENSOR_PROBE_MAX ) {
                // Замуровали демоны!!!
                this->stop();
                return false;                
            }

            if ( dir ) {
                this->rotate_left( _max_speed * 0.5, DISTANCE_SENSOR_PROBE_INTERVAL );
            }
            else {
                this->rotate_right( _max_speed * 0.5, DISTANCE_SENSOR_PROBE_INTERVAL );   
            }
        }

        return true;
    }

}; // class CarDirect

#endif // CARDIRECT_H_

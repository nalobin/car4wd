#ifndef CARDIRECT_H_
#define CARDIRECT_H_

#include <Arduino.h>
#include "ObstacleSensor.h"
#include <DistanceSensor.h>
#include <AFMotor.h>

const byte MAX_AXES = 3;

const byte LEFT  = 0;
const byte RIGHT = 1;

const byte BEFORE = 0;
const byte AFTER  = 1;

const byte _FORWARD = 0;
const byte _BACKWARD = 1;

const int DISTANCE_SENSOR_CHECK_INTERVAL  = 10; // ms
const int DISTANCE_SENSOR_MIN_DISTANCE    = 40; // cm
const int DISTANCE_SENSOR_PROBE_MAX       = 40; // попытки поиска пути
const int DISTANCE_SENSOR_PROBE_INTERVAL  = 250; // интервал между попытками поиска пути

struct Wheel {
    AF_DCMotor *motor;
    float speed;
    float speed_step;
    byte dir;
};

typedef void ( *GoCallback )( float );

// Класс n-осевой тачки с попарными неповоротными колёсами
class CarDirect {
private:
    byte _axes;
    Wheel _wheels    // колёса
        [2]          // сторона LEFT | RIGHT
        [MAX_AXES];  // индекс оси колеса; 0 - передняя ось
    float _max_speed;
    unsigned int _stop_fluent_tick;
    DistanceSensor **_front_distance_sensors;
    byte _front_distance_sensors_cnt;
    DistanceSensor **_rear_distance_sensors;
    byte _rear_distance_sensors_cnt;
    ObstacleSensor **_front_obstacle_sensors;
    byte _front_obstacle_sensors_cnt;
    ObstacleSensor **_rear_obstacle_sensors;
    byte _rear_obstacle_sensors_cnt;

    GoCallback _go_callbacks[2][2];

    void delay_checking_sensors( byte dir, int delay_ms ) {
        if (   dir == _FORWARD  && ( !_front_distance_sensors_cnt || !_front_obstacle_sensors_cnt )
            || dir == _BACKWARD && ( !_rear_distance_sensors_cnt || !_rear_obstacle_sensors_cnt ) ) {
            delay( delay_ms );
            return;
        }

        int checks = delay_ms / DISTANCE_SENSOR_CHECK_INTERVAL;

        DistanceSensor **distance_sensors = dir == _FORWARD ? _front_distance_sensors : _rear_distance_sensors;
        byte distance_sensors_cnt = dir == _FORWARD ? _front_distance_sensors_cnt : _rear_distance_sensors_cnt;

        ObstacleSensor **obstacle_sensors = dir == _FORWARD ? _front_obstacle_sensors : _rear_obstacle_sensors;
        byte obstacle_sensors_cnt = dir == _FORWARD ? _front_obstacle_sensors_cnt : _rear_obstacle_sensors_cnt;

        for ( int check = 0; check < checks; check++ ) {
            // first check obstacle sensors
            for ( byte sensor_num = 0; sensor_num < obstacle_sensors_cnt; sensor_num++ ) { 
                ObstacleSensor *sensor = obstacle_sensors[ sensor_num ];
                bool has = sensor->has_obstacle();
                Serial.println( "Obstacle sensor check" );
                Serial.println( has );
                if ( has ) {
                    this->stop();
                    return;
                }
            }

            // then check distance sensors
            for ( byte sensor_num = 0; sensor_num < distance_sensors_cnt; sensor_num++ ) { 
                DistanceSensor *sensor = distance_sensors[ sensor_num ];
                int cm = sensor->getDistanceCentimeter();
                Serial.println( "Distance sensor check" );
                Serial.println( cm );
                if ( cm > 0 && cm < DISTANCE_SENSOR_MIN_DISTANCE ) {
                    this->stop();
                    return;
                }
            }

            // no obstacles found. do nothing (actually run)
            delay( DISTANCE_SENSOR_CHECK_INTERVAL );
        }


    }
public:
    CarDirect( byte axes, byte wheels[][3], float max_speed = 4000  )
        : _axes( axes ), _max_speed( max_speed ),
        _front_distance_sensors_cnt( 0 ),
        _rear_distance_sensors_cnt( 0 ),
        _front_obstacle_sensors_cnt( 0 ),
        _rear_obstacle_sensors_cnt( 0 )
     {
        for ( byte i = 0; i < _axes * 2; i++ ) {
            Wheel &wheel = this->_wheels[ wheels[i][0] ][ wheels[i][1] ];
            wheel.motor = new AF_DCMotor( wheels[i][2] );
            wheel.speed     = 0;
            wheel.dir       = _FORWARD;
        }

        for ( byte dir = _FORWARD; dir <= _BACKWARD; dir++ ) {
            for ( byte type = BEFORE; type <= AFTER; type++ ) {
                _go_callbacks[ dir ][ type ] = 0;
            }           
        }
    }

    void set_front_distance_sensors( DistanceSensor **sensors, byte cnt ) {
        _front_distance_sensors = sensors;
        _front_distance_sensors_cnt = cnt;
    }
    void set_rear_distance_sensors( DistanceSensor **sensors, byte cnt ) {
        _rear_distance_sensors = sensors;
        _rear_distance_sensors_cnt = cnt;
    }
    void set_front_obstacle_sensors( ObstacleSensor **sensors, byte cnt ) {
        _front_obstacle_sensors = sensors;
        _front_obstacle_sensors_cnt = cnt;
    }
    void set_rear_obstacle_sensors( ObstacleSensor **sensors, byte cnt ) {
        _rear_obstacle_sensors = sensors;
        _rear_obstacle_sensors_cnt = cnt;
    }

    void set_go_callback( byte dir, byte type, GoCallback callback ) {
        _go_callbacks[ dir ][ type ] = callback;
    }

    // Низкоуровневое управление
    CarDirect &brake_wheel( byte side, byte axis, byte brake = 1, int delay_ms = 0 ) {
        Wheel &wheel = this->_wheels[ side ][ axis ];

        if ( brake ) {
            wheel.motor->run( RELEASE );
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

        wheel.motor->run( dir == _FORWARD ^ side == LEFT ? FORWARD : BACKWARD );

        speed = min( speed, _max_speed );
        speed = max( speed, 0          );

        byte analog_speed = byte( 0xFF * speed / _max_speed );
        Serial.print( "speed " );
        Serial.println( analog_speed );

        wheel.motor->setSpeed( analog_speed );
        
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
        if ( _go_callbacks[_FORWARD][BEFORE] && ( left_coef > 0.0 || right_coef > 0.0 ) ) {
            _go_callbacks[_FORWARD][BEFORE]( speed );
        }
        if ( _go_callbacks[_BACKWARD][BEFORE] && ( left_coef < 0.0 || right_coef < 0.0 ) ) {
            _go_callbacks[_BACKWARD][BEFORE]( speed );
        }

        this->rotate_side( LEFT , left_coef  >= 0.0 ? _FORWARD : _BACKWARD, int( abs( speed * left_coef  ) ) );
        this->rotate_side( RIGHT, right_coef >= 0.0 ? _FORWARD : _BACKWARD, int( abs( speed * right_coef ) ) );

        if ( delay_ms ) {
            if ( left_coef * right_coef > 0 ) {
                // если едем вперед или назад, то используем датчики расстояния
                this->delay_checking_sensors( left_coef  >= 0.0 ? _FORWARD : _BACKWARD, delay_ms );
            }
            else {
                delay( delay_ms );
            }
        }

        if ( _go_callbacks[_FORWARD][AFTER] && ( left_coef > 0.0 || right_coef > 0.0 ) ) {
            _go_callbacks[_FORWARD][AFTER]( speed );
        }
        if ( _go_callbacks[_BACKWARD][AFTER] && ( left_coef < 0.0 || right_coef < 0.0 ) ) {
            _go_callbacks[_BACKWARD][AFTER]( speed );
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

        int cycles = stop_fluent_time / stop_fluent_tick;
        if ( !cycles ) {
            cycles = 1;
        }
        
        // Serial.print( "stop_fluently_init " );
        // Serial.print( stop_fluent_time );
        // Serial.print( " " );
        // Serial.print( stop_fluent_tick );
        //Serial.print( " " );
        Serial.println( cycles );

        // Расчёт шага изменения скорости
        for ( byte axis = 0; axis < _axes; axis++ ) {
            for ( byte side = LEFT; side <= RIGHT; side++  ) {
                this->_wheels[ side ][ axis ].speed_step = -1 * this->_wheels[ side ][ axis ].speed / cycles;
            }
        }
    }

    CarDirect &stop_fluently_tick( unsigned int stop_fluent_tick ) {
        for ( byte axis = 0; axis < _axes; axis++ ) {
            for ( byte side = LEFT; side <= RIGHT; side++  ) {
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
    CarDirect &drift( byte side, byte dir, float speed, int delay_ms = 0, float coef = 0.4 ) {
        float dir_coef = dir == _FORWARD ? 1 : -1;

        this->go(
            speed,
            delay_ms,
            dir_coef * ( side == LEFT  ? coef : 1.0 ), // left_coef
            dir_coef * ( side == RIGHT ? coef : 1.0 )  // right_coef
        );

        return *this;
    }

    // drift sugar
    CarDirect &drift_forward_left( float speed, int delay_ms = 0, float coef = 0.4 ) {
        this->drift( LEFT, _FORWARD, speed, delay_ms, coef );
        return *this;
    }
    CarDirect &drift_forward_right( float speed, int delay_ms = 0, float coef = 0.4 ) {
        this->drift( RIGHT, _FORWARD, speed, delay_ms, coef );
        return *this;
    }
    CarDirect &drift_backward_left( float speed, int delay_ms = 0, float coef = 0.4 ) {
        this->drift( LEFT, _BACKWARD, speed, delay_ms, coef );
        return *this;
    }
    CarDirect &drift_backward_right( float speed, int delay_ms = 0, float coef = 0.4 ) {
        this->drift( RIGHT, _BACKWARD, speed, delay_ms, coef );
        return *this;
    }

    // Rotating car in order to go forward. Use distance sensors only.
    bool search_path() {
        if ( !_front_distance_sensors_cnt ) {
            // no forward sensors found
            this->stop();
            return false;
        }

        // Randomly select rotation direction
        byte dir = random( 2 );
        for ( byte try_num = 0; try_num < DISTANCE_SENSOR_PROBE_MAX; try_num++ ) {
            Serial.println( "searching..." );

            bool has_obstacle = false;

            // First check distance sensors (for longer forward runs)
            for ( byte sensor_num = 0; sensor_num < _front_distance_sensors_cnt; sensor_num++ ) { 
                DistanceSensor *sensor = _front_distance_sensors[ sensor_num ];
                int cm = sensor->getDistanceCentimeter();
                // Serial.println( "Distance sensor check" );
                // Serial.println( cm );
                if ( cm < DISTANCE_SENSOR_MIN_DISTANCE * 2 ) {
                    has_obstacle = true;
                }
            }

            if ( !has_obstacle ) {
                // Should ensure no obstacles on obstacle sensors
                for ( byte sensor_num = 0; sensor_num < _front_obstacle_sensors_cnt; sensor_num++ ) { 
                    ObstacleSensor *sensor = _front_obstacle_sensors[ sensor_num ];
                    if ( sensor->has_obstacle() ) {
                        has_obstacle = true;
                    }
                }
            }

            if ( !has_obstacle ) {
                return true;
            }

            // See an obstacle
            if ( dir ) {
                this->rotate_left( _max_speed, DISTANCE_SENSOR_PROBE_INTERVAL );
            }
            else {
                this->rotate_right( _max_speed, DISTANCE_SENSOR_PROBE_INTERVAL );   
            }
        }

        Serial.println( "shit, they blocked me!" );
        this->stop();
        return false;
    }

}; // class CarDirect

#endif // CARDIRECT_H_

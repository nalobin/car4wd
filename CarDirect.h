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

const int DISTANCE_SENSOR_MIN_DISTANCE = 40; // cm

struct Wheel {
    AF_DCMotor *motor;
    float speed;
    float speed_step;
    byte dir;
};

struct CarState {
    bool is_rotating;
    int dir;
    float speed;
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
    DistanceSensor **_front_distance_sensors;
    byte _front_distance_sensors_cnt;
    DistanceSensor **_rear_distance_sensors;
    byte _rear_distance_sensors_cnt;
    ObstacleSensor **_front_obstacle_sensors;
    byte _front_obstacle_sensors_cnt;
    ObstacleSensor **_rear_obstacle_sensors;
    byte _rear_obstacle_sensors_cnt;
    int _tick_ms;  // tick interval in ms
    int _do_ticks; // ticks to process current cmd

    void _reset_speed_steps() {
        for ( byte axis = 0; axis < _axes; axis++ ) {
            for ( byte side = LEFT; side <= RIGHT; side++  ) {
                this->_wheels[ side ][ axis ].speed_step = 0;
            }
        }
    }
public:
    CarDirect( byte axes, byte wheels[][3], int tick_ms, float max_speed = 4000  )
        : _axes( axes ), _max_speed( max_speed ), _do_ticks( 0 ), _tick_ms( tick_ms ),
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

    // low level handling
    CarDirect &brake_wheel( byte side, byte axis, byte brake = 1 ) {
        Wheel &wheel = this->_wheels[ side ][ axis ];

        if ( brake ) {
            wheel.motor->run( RELEASE );
        }

        wheel.speed = 0;

        return *this;
    }

    CarDirect &rotate_wheel( byte side, byte axis, byte dir, float speed ) {
        // this->brake_wheel( side, axis, 0 );

        Wheel &wheel = this->_wheels[ side ][ axis ];

        wheel.motor->run( dir == _FORWARD ^ side == LEFT ? FORWARD : BACKWARD );

        speed = min( speed, _max_speed );
        speed = max( speed, 0          );


        byte analog_speed = byte( 0xFF * speed / _max_speed );

        wheel.motor->setSpeed( analog_speed );
        
        wheel.speed = speed;
        wheel.dir = dir;

        return *this;
    }

    CarDirect &rotate_axis( byte axis, byte dir, float speed ) {
        this->rotate_wheel( LEFT , axis, dir, speed );
        this->rotate_wheel( RIGHT, axis, dir, speed );

        return *this;
    }

    CarDirect &rotate_side( byte side, byte dir, float speed ) {
        for ( byte axis = 0; axis < _axes; axis++ ) {
            this->rotate_wheel( side, axis, dir, speed );
        }

        return *this;
    }

    // left_coef, right_coef  -- коэф. скорости левых и правых колёс от -1 до 1.
    //             1.0 -- колеса крутятся вперёд с указанной максимальной скоростью (по умолчанию)
    //           0.0 -- колеса неподвижны
    //            -1.0 -- колеса крутятся назад с указанной максимальной скоростью
    CarDirect &go( float speed, int delay_ms = 1000, float left_coef = 1.0, float right_coef = 1.0 ) {
        this->rotate_side( LEFT , left_coef  >= 0.0 ? _FORWARD : _BACKWARD, int( abs( speed * left_coef  ) ) );
        this->rotate_side( RIGHT, right_coef >= 0.0 ? _FORWARD : _BACKWARD, int( abs( speed * right_coef ) ) );

        if ( delay_ms <= 0 ) {
            delay_ms = 1000;
        }

        // plan ticks todo
        _do_ticks = delay_ms / _tick_ms;
        if ( !_do_ticks ) {
            _do_ticks = 1;
        }

        this->_reset_speed_steps();

        return *this;
    }

    bool has_obstacle( byte dir, int min_distance = DISTANCE_SENSOR_MIN_DISTANCE ) {
        if ( dir == _FORWARD  && !_front_distance_sensors_cnt && !_front_obstacle_sensors_cnt
            || dir == _BACKWARD && !_rear_distance_sensors_cnt  && !_rear_obstacle_sensors_cnt  ) {
            // no sensors to check
            return false;
        }

        DistanceSensor **distance_sensors = dir == _FORWARD ? _front_distance_sensors : _rear_distance_sensors;
        byte distance_sensors_cnt = dir == _FORWARD ? _front_distance_sensors_cnt : _rear_distance_sensors_cnt;

        ObstacleSensor **obstacle_sensors = dir == _FORWARD ? _front_obstacle_sensors : _rear_obstacle_sensors;
        byte obstacle_sensors_cnt = dir == _FORWARD ? _front_obstacle_sensors_cnt : _rear_obstacle_sensors_cnt;

        // first check obstacle sensors
        for ( byte sensor_num = 0; sensor_num < obstacle_sensors_cnt; sensor_num++ ) { 
            ObstacleSensor *sensor = obstacle_sensors[ sensor_num ];
            bool has = sensor->has_obstacle();
            Serial.println( "Obstacle sensor check" );
            Serial.println( has );
            if ( has ) {
                return true;
            }
        }

        // then check distance sensors
        for ( byte sensor_num = 0; sensor_num < distance_sensors_cnt; sensor_num++ ) { 
            DistanceSensor *sensor = distance_sensors[ sensor_num ];
            int cm = sensor->getDistanceCentimeter();
            Serial.println( "Distance sensor check" );
            Serial.println( cm );
            if ( cm > 0 && cm < min_distance ) {
                return true;
            }
        }

        // no obstacles found
        return false;
    }

    CarState get_state() const {
        CarState state;

        state.speed = 0;
        state.dir = -1;
        byte dir_wheels[2] = { 0, 0 };
        for ( byte axis = 0; axis < _axes; axis++ ) {
            for ( byte side = LEFT; side <= RIGHT; side++  ) {
                dir_wheels[ this->_wheels[ side ][ axis ].dir ]++;
                state.speed = max( state.speed, this->_wheels[ side ][ axis ].speed );
            }
        }

        state.is_rotating = false;
        if ( dir_wheels[_FORWARD] == _axes * 2 ) {
            state.dir = _FORWARD;
        }
        else if ( dir_wheels[_BACKWARD] == _axes * 2 ) {
            state.dir = _BACKWARD;
        }
        else if ( state.speed ) {
            state.is_rotating = true;
        }

        return state;
    }

    // process tick events and returns true if processing should continue
    bool process_tick() {

        if ( _do_ticks <= 0 ) {
            // current action processed
            return false;
        }

        --_do_ticks;

        // Изменение скорости (если оно задано)
        for ( byte axis = 0; axis < _axes; axis++ ) {
            for ( byte side = LEFT; side <= RIGHT; side++  ) {
                if ( this->_wheels[ side  ][ axis ].speed_step ) {
                    this->rotate_wheel(
                        side,
                        axis,
                        this->_wheels[ side  ][ axis ].dir,
                        this->_wheels[ side  ][ axis ].speed + this->_wheels[ side  ][ axis ].speed_step
                    );
                }
            }
        }

        // get current car direction
        CarState state = this->get_state();

        if ( state.is_rotating ) {
            // do not check sensors
            return true;
        }

        if ( this->has_obstacle( state.dir ) ) {
            this->stop(); // force stop
            return false;
        }

        // no obstacles found and has ticks todo
        return true;
    }

    // do current action in blocking mode
    CarDirect &done() {
        while ( this->process_tick() ) {
            delay( _tick_ms ); 
        }

        return *this;
    }

    // high level actions
    CarDirect &stop() {
        for ( byte axis = 0; axis < _axes; axis++ ) {
            this->brake_wheel( LEFT , axis, 1 );
            this->brake_wheel( RIGHT, axis, 1 );
        }

        _do_ticks = 0;

        return *this;
    }

    CarDirect &stop_fluently( unsigned int stop_fluent_time_ms = 1000  ) {
        // plan ticks todo
        _do_ticks = stop_fluent_time_ms / _tick_ms;
        if ( !_do_ticks ) {
            _do_ticks = 1;
        }

        // Расчёт шага изменения скорости
        for ( byte axis = 0; axis < _axes; axis++ ) {
            for ( byte side = LEFT; side <= RIGHT; side++  ) {
                this->_wheels[ side ][ axis ].speed_step = -1 * this->_wheels[ side ][ axis ].speed / _do_ticks;
            }
        }

        return *this;
    }

    CarDirect &forward( float speed, int delay_ms = 1000 ) {
        this->go( speed, delay_ms, 1.0, 1.0 );

        return *this;
    }

    CarDirect &backward( float speed, int delay_ms = 1000 ) {
        this->go( speed, delay_ms, -1.0, -1.0 );

        return *this;
    }

    CarDirect &rotate_left( float speed, int delay_ms = 1000 ) {
        this->go( speed, delay_ms, -1.0, 1.0 );

        return *this;
    }

    CarDirect &rotate_right( float speed, int delay_ms = 1000 ) {
        this->go( speed, delay_ms, 1.0, -1.0 );

        return *this;
    }

    // Повернуть с буксом во время движения при помощи большей скорости внешних колёс
    CarDirect &drift( byte side, byte dir, float speed, int delay_ms = 1000, float coef = 0.4 ) {
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
    CarDirect &drift_forward_left( float speed, int delay_ms = 1000, float coef = 0.4 ) {
        this->drift( LEFT, _FORWARD, speed, delay_ms, coef );
        return *this;
    }
    CarDirect &drift_forward_right( float speed, int delay_ms = 1000, float coef = 0.4 ) {
        this->drift( RIGHT, _FORWARD, speed, delay_ms, coef );
        return *this;
    }
    CarDirect &drift_backward_left( float speed, int delay_ms = 1000, float coef = 0.4 ) {
        this->drift( LEFT, _BACKWARD, speed, delay_ms, coef );
        return *this;
    }
    CarDirect &drift_backward_right( float speed, int delay_ms = 1000, float coef = 0.4 ) {
        this->drift( RIGHT, _BACKWARD, speed, delay_ms, coef );
        return *this;
    }
}; // class CarDirect

#endif // CARDIRECT_H_

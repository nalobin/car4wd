#ifndef CARDIRECT_H_
#define CARDIRECT_H_

#include <Arduino.h>

const byte AXES  = 2;

const byte FRONT = 0;
const byte REAR  = AXES - 1;

const byte LEFT  = 0;
const byte RIGHT = 1;

const byte FORWARD  = LOW;
const byte BACKWARD = HIGH;

const byte SPEED_PIN = 0;
const byte DIR_PIN   = 1;
const byte BRAKE_PIN = 2;

struct Wheel {
    byte speed_pin;
    byte dir_pin;
    byte brake_pin;
    byte speed;
    byte dir;
};

// Класс n-осевой тачки с попарными неповоротными колёсами
class CarDirect {
private:
    Wheel _wheels
        [2]    // сторона LEFT | RIGHT
        [AXES];    // ось колеса начиная с передней части, FRONT | REAR
    byte _can_brake;
    int _max_speed;
public:
    CarDirect ( byte wheels_cnt, byte wheels[][5], int max_speed = 4000 ) {
        this->_max_speed = max_speed;
                this->_can_brake = 1;

        for ( byte i = 0; i < wheels_cnt; i++ ) {
            Wheel &wheel = this->_wheels[ wheels[i][0] ][ wheels[i][1] ];
            wheel.speed_pin = wheels[i][2];
            wheel.dir_pin   = wheels[i][3];
            wheel.brake_pin = wheels[i][4];
            wheel.speed     = 0;
            wheel.dir       = FORWARD;

            this->_can_brake &= wheel.brake_pin;
        }
    }

    // Низкоуровневое управление
    CarDirect &brake_wheel( byte side, byte axis, byte brake = 1, int delay_ms = 0 ) {
        Wheel &wheel = this->_wheels[ side ][ axis ];

        if ( this->_can_brake ) {
            digitalWrite( wheel.brake_pin, brake ? HIGH : LOW );
        }

        if ( brake ) {
            analogWrite( wheel.speed_pin, 0 );
        }

        wheel.speed = 0;

        if ( delay_ms ) {
            delay( delay_ms );
        }

        return *this;
    }

    CarDirect &rotate_wheel( byte side, byte axis, byte dir, int speed, int delay_ms = 0 ) {
        this->brake_wheel( side, axis, 0, 0 );

        Wheel &wheel = this->_wheels[ side ][ axis ];

        if ( this->_wheels[ side ][ axis ].dir_pin ) {
            digitalWrite( wheel.dir_pin, dir );
        }

        speed = min( speed, _max_speed );
        speed = max( speed, 0          );

        byte analog_speed = byte( 0xFF * speed / _max_speed );

        analogWrite( wheel.dir_pin, analog_speed );

        wheel.speed = speed;
        wheel.dir = dir;

        if ( delay_ms ) {
            delay( delay_ms );
        }

        return *this;
    }

    CarDirect &rotate_axis( byte axis, byte dir, int speed, int delay_ms = 0 ) {
        this->rotate_wheel( LEFT , axis, dir, speed );
        this->rotate_wheel( RIGHT, axis, dir, speed );

        if ( delay_ms ) {
            delay( delay_ms );
        }


        return *this;
    }

    CarDirect &rotate_side( byte side, byte dir, int speed, int delay_ms = 0 ) {
        for ( byte axis = 0; axis < AXES; axis++ ) {
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
    CarDirect &go( int speed, int delay_ms = 0, float left_coef = 1.0, float right_coef = 1.0 ) {
        this->rotate_side( LEFT , left_coef  >= 0.0 ? FORWARD : BACKWARD, int( abs( speed ) * left_coef  ) );
        this->rotate_side( RIGHT, right_coef >= 0.0 ? FORWARD : BACKWARD, int( abs( speed ) * right_coef ) );

        if ( delay_ms ) {
            delay( delay_ms );
        }

        return *this;
    }

    // Высокоуровневое управление
    CarDirect &stop( int delay_ms = 0 ) {
        for ( byte axis = 0; axis < AXES; axis++ ) {
            this->brake_wheel( LEFT , axis, 1 );
            this->brake_wheel( RIGHT, axis, 1 );
        }

        if ( delay_ms ) {
            delay( delay_ms );
        }

        return *this;
    }

    CarDirect &stop_fluently( int in_ms ) {
        const int tick = 50; // интервал времени между изменениями скорости
        const int cycles = in_ms / 50 || 1;

        float speed_step[2][AXES];
        float speed     [2][AXES];

        // Расчёт шага изменения скорости
        for ( byte axis = 0; axis < AXES; axis++ ) {
            for ( byte side = LEFT; side < RIGHT; side++  ) {
                speed     [ side ][ axis ] = this->_wheels[ side ][ axis ].speed;
                speed_step[ side ][ axis ] = speed[ side ][ axis ] / cycles;
            }
        }

        for ( int cycle = 0; cycle < cycles; cycle++ ) {
            for ( byte axis = 0; axis < AXES; axis++ ) {
                for ( byte side = LEFT; side < RIGHT; side++  ) {
                    speed[ side  ][ axis ] -= speed_step[ side  ][ axis ];
                    this->rotate_wheel( side , axis, this->_wheels[ side  ][ axis ].dir, int( speed[ side  ][ axis ] ) );
                }
            }

            delay( tick );
        }

        this->stop(); // чтобы уж точно ))

        return *this;
    }

    CarDirect &forward( int speed, int delay_ms = 0 ) {
        this->go( speed, delay_ms, 1.0, 1.0 );

        return *this;
    }

    CarDirect &backward( int speed, int delay_ms = 0 ) {
        this->go( speed, delay_ms, -1.0, -1.0 );

        return *this;
    }

    CarDirect &rotate_left( int speed, int delay_ms = 0 ) {
        this->go( speed, delay_ms, -1.0, 1.0 );

        return *this;
    }

    CarDirect &rotate_right( int speed, int delay_ms = 0 ) {
        this->go( speed, delay_ms, 1.0, -1.0 );

        return *this;
    }

    // Повернуть с буксом во время движения при помощи большей скорости внешних колёс
    CarDirect &drift( byte side, byte dir, int speed, int delay_ms = 0, float coef = 0.5 ) {
        float dir_coef = dir == FORWARD ? 1 : -1;

        this->go(
            speed,
            delay_ms,
            byte( dir_coef * speed * ( side == LEFT  ? coef : 1.0 ) ), // left_coef
            byte( dir_coef * speed * ( side == RIGHT ? coef : 1.0 ) )  // right_coef
        );

        return *this;
    }

    // drift sugar
    CarDirect &drift_forward_left( int speed, int delay_ms = 0, float coef = 0.5 ) {
        this->drift( LEFT, FORWARD, speed, coef );
        return *this;
    }
    CarDirect &drift_forward_right( int speed, int delay_ms = 0, float coef = 0.5 ) {
        this->drift( RIGHT, FORWARD, speed, delay_ms, coef );
        return *this;
    }
    CarDirect &drift_backward_left( int speed, int delay_ms = 0, float coef = 0.5 ) {
        this->drift( LEFT, BACKWARD, speed, delay_ms, coef );
        return *this;
    }
    CarDirect &drift_backward_right( int speed, int delay_ms = 0, float coef = 0.5 ) {
        this->drift( RIGHT, BACKWARD, speed, delay_ms, coef );
        return *this;
    }

    // TODO движение по датчикам -- ехать до препятствия и т.д.
}; // class CarDirect

#endif // CARDIRECT_H_

#include "CarDirect.h"
#include <IRremote.h>
#include <DistanceSRF04.h>

const float MAX_SPEED = 3900; // максимальная скорость машинки в сантиметрах в минуту

const float VERY_SLOW = MAX_SPEED * 0.2; 
const float SLOW      = MAX_SPEED * 0.4; 
const float AVERAGE   = MAX_SPEED * 0.6; 
const float FAST      = MAX_SPEED * 0.8; 
const float VERY_FAST = MAX_SPEED;
float current_speed = AVERAGE;
bool is_stopping_fluently = false;

// переводит расстояние в см и скорость в скорость и время в мс
#define l2t( speed, length ) ( speed ), ( ( length ) / ( speed ) * 60000 )

const byte AXIS = 0;

byte wheels[][5] = {
    { RIGHT, AXIS, 3, 2, 4 },
    { LEFT , AXIS, 5, 6, 7 },
};

DistanceSRF04 front_sensor;
CarDirect car( 1, wheels, MAX_SPEED, &front_sensor );

const int IR_RECV_PIN = 0;
IRrecv irrecv( IR_RECV_PIN );
decode_results results;

// Коды кнопок IR пульта
const unsigned long IR_FORWARD              = B00000;
const unsigned long IR_BACKWARD             = B00001;
const unsigned long IR_ROTATE_LEFT          = B00010;
const unsigned long IR_ROTATE_RIGHT         = B00011;
const unsigned long IR_DRIFT_FORWARD_LEFT   = B00100;
const unsigned long IR_DRIFT_FORWARD_RIGHT  = B00101;
const unsigned long IR_DRIFT_BACKWARD_LEFT  = B00110;
const unsigned long IR_DRIFT_BACKWARD_RIGHT = B00111;
const unsigned long IR_STOP                 = B01000;
const unsigned long IR_SPEED_UP             = B01001;
const unsigned long IR_SLOW_DOWN            = B01010;
const unsigned int IR_TIME = 300;

const unsigned int STOP_FLUENT_TIME = 300;
const unsigned int STOP_FLUENT_TICK = 50;

const unsigned int SERIAL_TIME = 300;

void setup() {
    Serial.begin( 9600 );
    Serial.setTimeout( 50 );
    irrecv.enableIRIn(); // Start the IR receiver
    
    front_sensor.begin( 8, 9 ); // echo & trigger pins

    test_predefined_drive();
    // autopilot();
}

void loop() {
    bool cmd_processed = false;

    // процессим IR сигналы
    if ( irrecv.decode( &results ) ) {
        Serial.println( results.value, HEX );
        cmd_processed = process_ir_press_button( results.value );
        irrecv.resume(); // Receive the next value
    }

    if ( !cmd_processed ) {
        // процессим команды с serial port
        if ( Serial.available() > 0 && Serial.find( "DO " ) ) {
            char cmd[128];
            if ( Serial.readBytesUntil( ':', cmd, sizeof( cmd ) ) ) {
                int speed_percent = Serial.parseInt();
                if ( speed_percent ) {
                    cmd_processed = process_serial_cmd( String( cmd ), speed_percent );
                }
            }
        }
    }

    if ( cmd_processed ) {
        is_stopping_fluently = false;
        return;
    }

    // нет команды на выполнение
    if ( is_stopping_fluently ) {
        car.stop_fluently_tick( STOP_FLUENT_TICK );
    }
    else {
        is_stopping_fluently = true;
        car.stop_fluently_init( STOP_FLUENT_TIME, STOP_FLUENT_TICK );
    }
}

void test_predefined_drive() {
    // вперед-назад
    car .forward ( SLOW     , 200 )
        .forward ( AVERAGE  , 300 )
        .forward ( VERY_FAST, 200 )
        .stop    ( 200 )
        .backward( VERY_FAST, 200 )
        .backward( AVERAGE  , 200 )
        .backward( SLOW     , 200 )
        .stop    ( 200 );
      
      // вперёд с поворотом направо и разворотом с помощью дрифта
    car .forward     ( VERY_FAST, 200  )
        .drift_forward_right( VERY_FAST, 300 )
        .backward    ( VERY_FAST, 200 )
        .drift_backward_left( VERY_FAST, 300 )
        .forward     ( AVERAGE,   250 ) 
        .rotate_right( SLOW, 1000 ) // пробуем развернуться
        .stop    ();
      
    car .forward( l2t( VERY_FAST, 200 ) )
        .backward( l2t( VERY_FAST, 200 ) )
        .stop_fluently_init( 300, 50 )
        .stop_fluently_tick( 50 )
        .stop_fluently_tick( 50 )
        .stop_fluently_tick( 50 )
        .stop_fluently_tick( 50 )
        .stop_fluently_tick( 50 )
        .stop_fluently_tick( 50 );
}

void autopilot() {
    while ( 1 ) {
        // едем до препятствия
        car.forward( l2t( AVERAGE, 1000 ) );

        // поворачиваемся в какую нибудь сторону и ищем проезд
        if ( !car.search_path() ) {
            break;
        }
    }
}

bool process_ir_press_button( unsigned long button ) {
    switch ( button ) {
        case IR_FORWARD:
            car.forward( current_speed, IR_TIME );
            break;
        case IR_BACKWARD:
            car.backward( current_speed, IR_TIME );
            break;
        case IR_ROTATE_LEFT:
            car.rotate_left( current_speed, IR_TIME );
            break;
        case IR_ROTATE_RIGHT:
            car.rotate_right( current_speed, IR_TIME );
            break;
        case IR_DRIFT_FORWARD_LEFT:
            car.drift_forward_left( current_speed, IR_TIME );
            break;
        case IR_DRIFT_FORWARD_RIGHT:
            car.drift_forward_right( current_speed, IR_TIME );
            break;
        case IR_DRIFT_BACKWARD_LEFT:
            car.drift_backward_left( current_speed, IR_TIME );
            break;
        case IR_DRIFT_BACKWARD_RIGHT:
            car.drift_backward_right( current_speed, IR_TIME );
            break;
        case IR_STOP:
            car.stop();
            break;
        case IR_SPEED_UP:
            current_speed = current_speed + VERY_SLOW;
            if ( current_speed > MAX_SPEED ) {
                current_speed = MAX_SPEED;
            }
            break;
        case IR_SLOW_DOWN:
            current_speed = current_speed  - VERY_SLOW;
            if ( current_speed < VERY_SLOW || current_speed > MAX_SPEED ) {
                current_speed = VERY_SLOW;
            }
            break;
        default:
            return false;
    }

    // TODO выводим текущую скорость на экран

    return true;
}

bool process_serial_cmd( const String &cmd, int speed_percent ) {
    float speed = speed_percent * MAX_SPEED / 100;

    if ( cmd == "forward" ) {
        car.forward( speed, SERIAL_TIME );
    }
    else if ( cmd == "backward" ) {
        car.backward( speed, SERIAL_TIME );
    }
    else if ( cmd == "rotate_left" ) {
        car.rotate_left( speed, SERIAL_TIME );
    }
    else if ( cmd == "rotate_right" ) {
        car.rotate_right( speed, SERIAL_TIME );
    }
    else if ( cmd == "drift_forward_left" ) {
        car.drift_forward_left( speed, SERIAL_TIME );
    }
    else if ( cmd == "drift_forward_right" ) {
        car.drift_forward_right( speed, SERIAL_TIME );
    }
    else if ( cmd == "drift_backward_left" ) {
        car.drift_backward_left( speed, SERIAL_TIME );
    }
    else if ( cmd == "drift_backward_right" ) {
        car.drift_backward_right( speed, SERIAL_TIME );
    }
    else if ( cmd == "stop" ) {
        car.stop();
    }
    else {
        return false;
    }

    // TODO выводим текущую скорость на экран

    return true;   
}

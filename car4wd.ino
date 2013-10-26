#include "CarDirect.h"
#include <IRremote.h>

const int MAX_SPEED = 3900; // максимальная скорость машинки в сантиметрах в минуту

const int VERY_SLOW = MAX_SPEED * 0.2; 
const int SLOW      = MAX_SPEED * 0.4; 
const int AVERAGE   = MAX_SPEED * 0.6; 
const int FAST      = MAX_SPEED * 0.8; 
const int VERY_FAST = MAX_SPEED;
int current_speed = AVERAGE;

// переводит расстояние в см и скорость в скорость и время в мс
#define l2t( speed, length ) ( speed ), ( ( length ) / ( speed ) * 60000 )

byte wheels[][5] = {
    { LEFT , FRONT, 10, 11, 0 },
    { LEFT , REAR , 12, 13, 0 },
    { RIGHT, FRONT, 14, 15, 0 },
    { RIGHT, REAR , 16, 17, 0 }
};

CarDirect car( 4, wheels, MAX_SPEED );

const int IR_RECV_PIN = 9;
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

const int IR_TIME = 300;

void setup() {
    Serial.begin( 9600 );
    irrecv.enableIRIn(); // Start the IR receiver

    test_predefined_drive();
}

void loop() {
    if ( irrecv.decode( &results ) ) {
        Serial.println( results.value, HEX );
        process_ir_press_button( results.value );
        irrecv.resume(); // Receive the next value
    }
    else {
        // нет команды на выполнение
        car.stop_fluently( 300 );
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
        .stop_fluently( 300 );
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

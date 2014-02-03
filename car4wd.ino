#include "CarDirect.h"
#include <IRremote.h>
#include <DistanceSRF04.h>
#include <AFMotor.h>

const float MAX_SPEED = 3900; // максимальная скорость машинки в сантиметрах в минуту

const float VERY_SLOW = MAX_SPEED * 0.4; 
const float SLOW      = MAX_SPEED * 0.6; 
const float AVERAGE   = MAX_SPEED * 0.8; 
const float FAST      = MAX_SPEED * 0.8; 
const float VERY_FAST = MAX_SPEED * 1;
float current_speed = MAX_SPEED;
bool is_stopping_fluently = false;
bool autopilot_mode = false;

// переводит расстояние в см и скорость в скорость и время в мс
#define l2t( speed, length ) ( speed ), ( ( length ) / ( speed ) * 60000 )

const byte FRONT_AXIS = 0;
const byte REAR_AXIS  = 1;

// digital pins

// analog pins
const byte IR_RECV_PIN         = A5;
const byte US_SENSOR_ECHO_PIN  = A0;
const byte US_SENSOR_TRIG_PIN  = A1;
const byte BEEPER_PIN          = 10; // ШИМ
const byte FORWARD_LIGHTS_PIN  = 11;
const byte BACKWARD_LIGHTS_PIN = 12;

byte wheels[][3] = {
    { RIGHT, FRONT_AXIS, 1 },
    { LEFT , FRONT_AXIS, 2 },
/*  { RIGHT, REAR_AXIS , 3 },
    { LEFT , REAR_AXIS , 4 }*/
};

DistanceSRF04 front_sensor;
CarDirect car( 1, wheels, MAX_SPEED, &front_sensor );

IRrecv irrecv( IR_RECV_PIN );
decode_results results;

unsigned long prev_ir_cmd = 0;

// Коды кнопок IR пульта
const unsigned long IR_FORWARD              = 0xCD171AFF;
const unsigned long IR_BACKWARD             = 0x85883E95;
const unsigned long IR_ROTATE_LEFT          = 0xE8DFBFBB;
const unsigned long IR_ROTATE_RIGHT         = 0x2401508B;
const unsigned long IR_DRIFT_FORWARD_LEFT   = 0x5621E826;
const unsigned long IR_DRIFT_FORWARD_RIGHT  = 0xDF7539BA;
const unsigned long IR_DRIFT_BACKWARD_LEFT  = 0x888B042E;
const unsigned long IR_DRIFT_BACKWARD_RIGHT = 0x100A4C9E;
const unsigned long IR_STOP                 = 0xFB442204;
const unsigned long IR_SPEED_UP             = 0xF91B3490;
const unsigned long IR_SLOW_DOWN            = 0xCF39C4E5;
const unsigned long IR_AUTOPILOT            = 0x4B64311A;
const unsigned int IR_TIME = 50;

const unsigned int STOP_FLUENT_TIME = 1000;
const unsigned int STOP_FLUENT_TICK = 20;

const unsigned int SERIAL_TIME = 300;

void setup() {
    Serial.begin( 9600 );
    Serial.setTimeout( 50 );
    irrecv.enableIRIn(); // Start the IR receiver
    
    front_sensor.begin( US_SENSOR_ECHO_PIN, US_SENSOR_TRIG_PIN );
    int null_distances[] = { 8 };
    front_sensor.setNullDistances( null_distances, 1 );

/*  pinMode( BEEPER_PIN         , OUTPUT );
    pinMode( FORWARD_LIGHTS_PIN , OUTPUT );
    pinMode( BACKWARD_LIGHTS_PIN, OUTPUT );
*/

/*  car.set_go_callback( FORWARD , BEFORE, before_forward  );
    car.set_go_callback( BACKWARD, BEFORE, before_backward );
    car.set_go_callback( FORWARD , AFTER , after_forward );
    car.set_go_callback( BACKWARD, AFTER , after_backward );
*/

    // test_predefined_drive();
    // autopilot();
}

void loop() {
    bool cmd_processed = false;

    // процессим IR сигналы
    if ( irrecv.decode( &results ) ) {
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
    
    if ( autopilot_mode ) {
        car.forward( FAST, 1000 );
        car.search_path();
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
    car .forward ( AVERAGE, 3000 )
        .forward ( FAST     , 1000 )
        .forward ( VERY_FAST  , 1000 )
        .stop    ( 200 )
        .backward( VERY_FAST  , 1000 )
        .backward( FAST     , 1000 )
        .backward( AVERAGE, 3000 )
        .stop    ( 200 );
        
   car.rotate_right( FAST, 10000 ).rotate_left( FAST, 10000 ).stop();
      
      // вперёд с поворотом направо и разворотом с помощью дрифта
      car .forward     ( FAST, 5000  )
        .drift_forward_left( FAST, 6000 )
        .backward    ( AVERAGE, 4000 )
        .drift_backward_right( FAST, 6000 )
        .forward     ( AVERAGE,   5000 ) 
        .rotate_right( FAST, 5000 ) // пробуем развернуться
        .stop    ();      
}

void autopilot() {
    while ( 1 ) {
        // едем до препятствия
        car.forward( VERY_FAST, 5000 );
        
        Serial.println( "stopped" );

        // поворачиваемся в какую нибудь сторону и ищем проезд
        if ( !car.search_path() ) {
            break;
        }
    }
}

bool process_ir_press_button( unsigned long button ) {
    if ( button == REPEAT ) {
        if ( prev_ir_cmd  ) {
            button = prev_ir_cmd;
        }
        else {
            return false;
        }
    }
    else {
        prev_ir_cmd = button;
    }
    
    // Serial.println( "ir cmd" );
    // Serial.println( button, HEX );

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
         case IR_AUTOPILOT:
            autopilot_mode = !autopilot_mode;
            return false;
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
    else if ( cmd == "autopilot" ) {
        autopilot_mode = !autopilot_mode;
        return false;
    }
    else {
        return false;
    }

    // TODO выводим текущую скорость на экран

    return true;   
}

// movement callbacks
void before_forward( float speed ) {
    // tone( BEEPER_PIN, speed / 10  );

    digitalWrite( FORWARD_LIGHTS_PIN, HIGH );
}

void before_backward( float speed ) {
    // tone( BEEPER_PIN, speed / 5 );

    digitalWrite( BACKWARD_LIGHTS_PIN, HIGH );
}

void after_forward( float speed ) {
    // noTone( BEEPER_PIN );

    digitalWrite( FORWARD_LIGHTS_PIN, LOW );
}

void after_backward( float speed ) {
    // noTone( BEEPER_PIN );

    digitalWrite( BACKWARD_LIGHTS_PIN, LOW );
}

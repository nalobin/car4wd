#include "CarDirect.h"
#include "ObstacleSensor.h"
#include <IRremote.h> // REQUIRES to switch to #define IR_USE_TIMER1   // tx = pin 9 in IRemoteInt.h
#include <DistanceSRF04.h>
#include <AFMotor.h>

const float MAX_SPEED = 3900; // максимальная скорость машинки в сантиметрах в минуту

const float VERY_SLOW = MAX_SPEED * 0.4; 
const float SLOW      = MAX_SPEED * 0.6; 
const float AVERAGE   = MAX_SPEED * 0.8; 
const float FAST      = MAX_SPEED * 0.8; 
const float VERY_FAST = MAX_SPEED * 1;
const int TICK_INTERVAL = 20; // ms

float current_speed = SLOW;

bool autopilot_searching_path = false;
bool autopilot_mode = false;
byte autopilot_dir = _FORWARD;
unsigned int ticks = 0;

bool front_lights_on = false;
bool back_lights_on  = false;

// переводит расстояние в см и скорость в скорость и время в мс
#define l2t( speed, length ) ( speed ), ( ( length ) / ( speed ) * 60000 )

const byte FRONT_AXIS = 0;
const byte REAR_AXIS  = 1;

const byte BEEPER_PIN          = A11;
const byte FORWARD_LIGHTS_PIN  = 26;
const byte BACKWARD_LIGHTS_PIN = 27;

// free digital pins of Motor Shield 2, 13 (+9,10 if no servos)

byte wheels[][3] = {
    { RIGHT, FRONT_AXIS, 2 },
    { LEFT , FRONT_AXIS, 1 },
    { RIGHT, REAR_AXIS , 4 },
    { LEFT , REAR_AXIS , 3 }
};

CarDirect car( 2, wheels, TICK_INTERVAL, MAX_SPEED );

IRrecv irrecv( A5 );
decode_results results;

unsigned long prev_ir_cmd = 0;

// Коды кнопок IR пульта
const unsigned long IR_FORWARD              = 0x88877;
const unsigned long IR_BACKWARD             = 0x8A857;
const unsigned long IR_ROTATE_LEFT          = 0x848B7;
const unsigned long IR_ROTATE_RIGHT         = 0x828D7;
const unsigned long IR_DRIFT_FORWARD_LEFT   = 0x8C03F;
const unsigned long IR_DRIFT_FORWARD_RIGHT  = 0x8A05F;
const unsigned long IR_DRIFT_BACKWARD_LEFT  = 0x8906F;
const unsigned long IR_DRIFT_BACKWARD_RIGHT = 0x8D02F;
const unsigned long IR_STOP                 = 0x8C837;
const unsigned long IR_SPEED_UP             = 0x8F807;
const unsigned long IR_SLOW_DOWN            = 0x802FD;
const unsigned long IR_AUTOPILOT            = 0x838C7;

// Коды кнопок IR пульта Samsung
/*const unsigned long IR_FORWARD              = 0xE0E006F9;
const unsigned long IR_BACKWARD             = 0xE0E08679;
const unsigned long IR_ROTATE_LEFT          = 0xE0E0A659;
const unsigned long IR_ROTATE_RIGHT         = 0xE0E046B9;
const unsigned long IR_DRIFT_FORWARD_LEFT   = 0xE0E020DF;
const unsigned long IR_DRIFT_FORWARD_RIGHT  = 0x6604CFF0;
const unsigned long IR_DRIFT_BACKWARD_LEFT  = 0xE0E030CF;
const unsigned long IR_DRIFT_BACKWARD_RIGHT = 0xE0E0708F;
const unsigned long IR_STOP                 = 0xE0E0906F;
const unsigned long IR_SPEED_UP             = 0xE0E0E01F;
const unsigned long IR_SLOW_DOWN            = 0xE0E0D02F;
const unsigned long IR_AUTOPILOT            = 0xE0E0F00F;
*/
const unsigned int IR_TIME = 2000;

const unsigned int SERIAL_TIME = 300;

// Sensors should be defined here (as global variables)
// включить только если реально подключен. иначе будет крутить вечно
DistanceSRF04 front_distance_sensor;
DistanceSensor *front_distance_sensors[] = { &front_distance_sensor };
DistanceSRF04 rear_distance_sensor;
DistanceSensor *rear_distance_sensors[] = { &rear_distance_sensor };

ObstacleSensor left_front_obstacle_sensor  = ObstacleSensor( 22 );
ObstacleSensor right_front_obstacle_sensor = ObstacleSensor( 23 );
ObstacleSensor *front_obstacle_sensors[] = { &left_front_obstacle_sensor, &right_front_obstacle_sensor };

void setup() {
    Serial.begin( 9600 );
    Serial.setTimeout( 50 );
    Serial1.begin( 9600 );
    Serial1.setTimeout( 50 );
    irrecv.enableIRIn(); // Конфликтует с M1, M2 на Motor Shield
 
    front_distance_sensor.begin( A3, A4 ); // echo, trigger
    // int null_distances[] = { 8 };
    // front_distance_sensor.setNullDistances( null_distances, 1 );
    car.set_front_distance_sensors( front_distance_sensors, 1 );
    car.set_front_obstacle_sensors( front_obstacle_sensors, 2 );

    rear_distance_sensor.begin( A2, A1 ); // echo, trigger
    car.set_rear_distance_sensors( rear_distance_sensors, 1 );
    // car.set_rear_obstacle_sensors( rear_obstacle_sensors, 2 );

    pinMode( BEEPER_PIN         , OUTPUT );
    pinMode( FORWARD_LIGHTS_PIN , OUTPUT );
    pinMode( BACKWARD_LIGHTS_PIN, OUTPUT );

    // test_predefined_drive();
}

void loop() {
    bool cmd_processed = false;

    // процессим IR сигналы
    if ( irrecv.decode( &results ) ) {
        cmd_processed = process_ir_press_button( results.value );
        irrecv.resume(); // Receive the next value
    }
    
    if ( !cmd_processed ) {
        // Процессим команды с serial port.
        if ( Serial1.available() > 0 ) {
            if ( String str = Serial1.readString() ) {
                String prev_cmd;
                for ( int i = 0; i < str.length(); i++ ) {
                    // Символ — отдельная команда.
                    String cmd( str.charAt( i ) );

                    // не дублируем последнюю выполненную команду для большей интерактивности
                    if ( cmd == prev_cmd ) {
                        continue;
                    }

                    Serial.println( cmd );
                    cmd_processed = process_serial_cmd( cmd );
                    prev_cmd = cmd;
                }
            }
        }
    }

    CarState state = car.get_state();

    if ( state.speed ) {
        if( state.is_rotating ) {
            before_rotation( state.speed );
        }
        else if ( state.dir == _FORWARD ) {
            before_forward( state.speed );
        }
        else if ( state.dir == _BACKWARD  ) {
            before_backward( state.speed );
        }
    }
    else {
        on_stop();
    }

    delay( TICK_INTERVAL );

    if ( state.speed ) {
        if( state.is_rotating ) {
            after_rotation( state.speed );
        }
        else if ( state.dir == _FORWARD ) {
            after_forward( state.speed );
        }
        else if ( state.dir == _BACKWARD ) {
            after_backward( state.speed );
        }
    }

    bool done_prev_action = !car.process_tick();
    ++ticks;

    if ( autopilot_mode ) {
        do_autopilot_action( done_prev_action );
        return;
    }

    if ( done_prev_action ) {
        if ( state.speed && !state.is_rotating ) {
            car.stop_fluently( 2000 );
        }
        else {
            car.stop();
        }
    }
}

void test_predefined_drive() {
    // вперед-назад
    car.forward ( AVERAGE  , 3000 ).done()
       .forward ( FAST     , 1000 ).done()
       .forward ( VERY_FAST, 1000 ).done()
       .stop    ()
       .backward( VERY_FAST, 1000 ).done()
       .backward( FAST     , 1000 ).done()
       .backward( AVERAGE  , 3000 ).done()
       .stop    ();
       
    car.rotate_right( FAST, 10000 ).done()
       .rotate_left ( FAST, 10000 ).done()
       .stop();  
}

// Starts autopilot movement, returns whether there is no way out.
void do_autopilot_action( bool done_prev_action ) {
    if ( autopilot_searching_path ) {
        if ( car.has_obstacle( autopilot_dir, DISTANCE_SENSOR_MIN_DISTANCE * 2 ) ) {
            if ( done_prev_action ) {
                // continue to search path  in opposite direction
                // autopilot_dir = autopilot_dir == _FORWARD ? _BACKWARD: _FORWARD;
            }

            return; // continue to search path
        }

        // path found
        autopilot_searching_path = false;
        car.stop();
        return; 
    }

    // not searching a path
    if ( car.has_obstacle( autopilot_dir ) ) {
        // start new path search
        autopilot_searching_path = true;

        if ( random( 2 ) ) {
            car.rotate_left( current_speed, 5000 );            
        }
        else {
            car.rotate_right( current_speed, 5000 );
        }
        return; // new search just started
    }

    if ( !done_prev_action ) {
        // has action to complete
        return;
    }

    // start new action

    // change direction randomly
/*    bool should_change_dir = random( 20 ) == 0;

    if ( should_change_dir ) {
        autopilot_dir = autopilot_dir == _FORWARD ? _BACKWARD: _FORWARD;
        if ( random( 2 ) ) {
            car.rotate_left( current_speed, 4000 );
        }
        else {
            car.rotate_right( current_speed, 4000 );   
        }
        return;
    }*/

    if ( autopilot_dir == _FORWARD ) {
        car.forward( current_speed, 5000 );
    }
    else {
        car.backward( current_speed, 5000 ); // should have rear sensors
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
    
    Serial.println( "IR cmd" );
    Serial.println( button, HEX );

    switch ( button ) {
        case IR_FORWARD:
            car.forward( current_speed, IR_TIME );
            break;
        case IR_BACKWARD:
            car.backward( current_speed, IR_TIME );
            break;
        case IR_ROTATE_LEFT:
            car.rotate_left( current_speed, IR_TIME / 4 );
            break;
        case IR_ROTATE_RIGHT:
            car.rotate_right( current_speed, IR_TIME / 4 );
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
            if ( autopilot_mode ) {
                car.stop();
            }
            break;
        case IR_SLOW_DOWN:
            current_speed = current_speed  - VERY_SLOW;
            if ( current_speed < VERY_SLOW || current_speed > MAX_SPEED ) {
                current_speed = VERY_SLOW;
            }
            if ( autopilot_mode ) {
                car.stop();
            }
            break;
         case IR_AUTOPILOT:
            if ( autopilot_mode ) {
                car.stop();
            }
            autopilot_mode = !autopilot_mode;
            return false;
        default:
            return false;
    }

    // TODO выводим текущую скорость на экран

    return true;
}

bool process_serial_cmd( const String &cmd ) {

    if ( cmd == "F" ) {
        car.forward( current_speed, SERIAL_TIME );
    }
    else if ( cmd == "B" ) {
        car.backward( current_speed, SERIAL_TIME );
    }
    else if ( cmd == "L" ) {
        car.rotate_left( current_speed, SERIAL_TIME );
    }
    else if ( cmd == "R" ) {
        car.rotate_right( current_speed, SERIAL_TIME );
    }
    else if ( cmd == "G" ) {
        car.drift_forward_left( current_speed, SERIAL_TIME );
    }
    else if ( cmd == "I" ) {
        car.drift_forward_right( current_speed, SERIAL_TIME );
    }
    else if ( cmd == "H" ) {
        car.drift_backward_left( current_speed, SERIAL_TIME );
    }
    else if ( cmd == "J" ) {
        car.drift_backward_right( current_speed, SERIAL_TIME );
    }
    else if ( cmd == "S" ) {
        if ( !autopilot_mode ) {
            car.stop();
        }
    }
    else if ( cmd == "W" ) {
        digitalWrite( FORWARD_LIGHTS_PIN, HIGH );
        front_lights_on = true;
    }
    else if ( cmd == "w" ) {
        digitalWrite( FORWARD_LIGHTS_PIN, LOW );
        front_lights_on = false;
    }
    else if ( cmd == "U" ) {
        digitalWrite( BACKWARD_LIGHTS_PIN, HIGH );
        back_lights_on = true;
    }
    else if ( cmd == "u" ) {
        digitalWrite( BACKWARD_LIGHTS_PIN, LOW );
        back_lights_on = false;
    }
    else if ( cmd == "0" || cmd == "1" || cmd == "2" ) {
        current_speed = VERY_SLOW;
    }
    else if ( cmd == "3" || cmd == "4" ) {
        current_speed = SLOW;
    }
    else if ( cmd == "5" || cmd == "6" ) {
        current_speed = AVERAGE;
    }
    else if ( cmd == "7" || cmd == "8" ) {
        current_speed = FAST;
    }
    else if ( cmd == "9" || cmd == "10"  || cmd == "q" ) {
        current_speed = VERY_FAST;
    }
    else if ( cmd == "X" || cmd == "x" ) {
        if ( autopilot_mode ) {
            car.stop();
        }

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
    //tone( BEEPER_PIN, speed / 10  );

    if ( ticks % 2 == 0 && !front_lights_on ) {
        digitalWrite( FORWARD_LIGHTS_PIN, digitalRead( FORWARD_LIGHTS_PIN ) == HIGH ? LOW : HIGH );
    }

    if ( !back_lights_on ) {
        digitalWrite( BACKWARD_LIGHTS_PIN, LOW );
    }
}

void before_backward( float speed ) {
    //tone( BEEPER_PIN, speed / 5 );

    if ( !front_lights_on ) {
        digitalWrite( FORWARD_LIGHTS_PIN , LOW );
    }

    if ( ticks % 2 == 0 && !back_lights_on ) {
        digitalWrite( BACKWARD_LIGHTS_PIN, digitalRead( BACKWARD_LIGHTS_PIN ) == HIGH ? LOW : HIGH );
    }
}

void before_rotation( float speed ) {
    //tone( BEEPER_PIN, speed / 5 );

    if ( ticks % 2 != 0 ) {
        return;
    }

    if ( !front_lights_on ) {
        digitalWrite( FORWARD_LIGHTS_PIN , digitalRead( FORWARD_LIGHTS_PIN  ) == HIGH ? LOW : HIGH );
    }

    if ( !back_lights_on ) {
        digitalWrite( BACKWARD_LIGHTS_PIN, digitalRead( BACKWARD_LIGHTS_PIN ) == HIGH ? LOW : HIGH );
    }
 }

void after_forward( float speed ) {
    //noTone( BEEPER_PIN );
}

void after_backward( float speed ) {
    //noTone( BEEPER_PIN );
}

void after_rotation( float speed ) {
    //noTone( BEEPER_PIN );
}

void on_stop() {
    if ( !front_lights_on ) {
        digitalWrite( FORWARD_LIGHTS_PIN , LOW );
    }

    if ( !back_lights_on ) {
        digitalWrite( BACKWARD_LIGHTS_PIN, LOW );
    }
}

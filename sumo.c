#include <stdlib.h>
#include <math.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* OSEK declarations */
DeclareCounter(SysTimerCnt);
DeclareAlarm(display_cyclic_alarm);
DeclareAlarm(color_sensor_cyclic_alarm);
DeclareAlarm(motor_control_cyclic_alarm);
DeclareAlarm(sonar_sensor_cyclic_alarm);
DeclareTask(DisplayTask);
DeclareTask(MotorcontrolTask);
DeclareTask(ColorSensorTask);
DeclareTask(SonarSensorTask);
DeclareResource(DrivingControlResource);

/* Ports */
#define PORT_COLOR NXT_PORT_S1
#define PORT_SONAR NXT_PORT_S2
#define MOTOR_LEFT NXT_PORT_B
#define MOTOR_RIGHT NXT_PORT_A

/* Priorites */
#define PRIO_IDLE 5
#define PRIO_COLOR 50
#define PRIO_SEEK 20
#define PRIO_FOLLOW 30

/* Task lengths */
#define MOTORCONTROLTASK_PERIOD_LENGTH 50
#define SONARSENSORTASK_PERIOD_LENGTH 100

/* When the sonars mesured distance is larger than this seek behavior will
   be activated */
#define SEEK_DISTANCE_THRESHOLD 70

/* Used to detect surface. When below robot an edge has been detected*/
#define COLOR_THRESHOLD 300

/* Global driving command. The motor control task will use the information
   in this object to decide what to do with the motors. */
struct dc_t {
    U32 duration;
    S32 speed_left;
    S32 speed_right;
    int priority;
} dc = {0, 0, 0, PRIO_IDLE};

/* When seeking has been started, this will be set to system tick time.
   Otherwise it will be 0.
   The variable will be reset inside change_driving_command() if dc.prio isn't
   a seeking priority. */
U32 SeekStarted;

/* Because the sonar sensor returns max distance if something is too close 
   this value will hold the previous sensed value so this case can be
   detected. */
S32 LastSonarValue;

/* Semaphore for signaling when in setup mode. Locked when positive. */
int InSetupPhase;

/* When setup is started this will be set to current system tick time. Used
   for timing in setup. */
U32 SetupStarted;

/*
 * OSEK system callbacks 
 */
void ecrobot_device_initialize() {
    // init sensors
    ecrobot_init_nxtcolorsensor(PORT_COLOR, NXT_LIGHTSENSOR_RED);
    ecrobot_init_sonar_sensor(PORT_SONAR);

    // init variables
    SeekStarted = 0;
    LastSonarValue = 255; // Max value
    InSetupPhase = 1;
    SetupStarted = 0;
    srand(systick_get_ms());
}

void ecrobot_device_terminate() {
    // kill motors
    nxt_motor_set_speed(MOTOR_LEFT, 0, 0);
    nxt_motor_set_speed(MOTOR_RIGHT, 0, 0);

    // terminate sensors
    ecrobot_term_nxtcolorsensor(PORT_COLOR);
    ecrobot_term_sonar_sensor(PORT_SONAR);
}

void user_1ms_isr_type2(void) { 
    StatusType ercd;

    // increment system timer
    ercd = SignalCounter(SysTimerCnt);
    if ( ercd != E_OK) {
        ShutdownOS(ercd);
    }
}

/* This function updates the global driving command data structure if the
   supplied priority is higher or equal to the current priority.
   If the priority isn't PRIO_SEEK the function will reset SeekStarted to 0.
*/
void change_driving_command(int priority, int speed_left, int speed_right,
                            int duration) {
    GetResource(DrivingControlResource);

    // If we're not in PRIO_SEEK, reset SeekStarted
    if (dc.priority != PRIO_SEEK) {
        SeekStarted = 0;
    }

    // only change command if priority is higher or equal to current priority
    if (priority >= dc.priority) {
        dc.priority = priority;
        dc.speed_left = speed_left;
        dc.speed_right = speed_right;
        dc.duration = duration;
    }

    ReleaseResource(DrivingControlResource);
}

/*
 * Tasks 
 */
/* This task is responsible for all control of the motors */
TASK(MotorcontrolTask) {
    if (InSetupPhase)
        TerminateTask();

    GetResource(DrivingControlResource);

    // calculate speed and stuff
    if (dc.duration > 0) {
        dc.duration -= MOTORCONTROLTASK_PERIOD_LENGTH;
    } else {
        dc.priority = PRIO_IDLE;
        dc.duration = 0;
        dc.speed_left = 0;
        dc.speed_right = 0;
    }

    // update motor speed
    nxt_motor_set_speed(MOTOR_LEFT, -dc.speed_left, 1);
    nxt_motor_set_speed(MOTOR_RIGHT, -dc.speed_right, 1);

    ReleaseResource(DrivingControlResource);    

    TerminateTask();
}

/* This task updates the color sensor. */
/* Measured color values:
    On white: 500-600
    On black: 250-300
    On "nothing": 150
*/
TASK(ColorSensorTask) {
    if (InSetupPhase)
        TerminateTask();

    ecrobot_process_bg_nxtcolorsensor();

    U16 color = ecrobot_get_nxtcolorsensor_light(PORT_COLOR);

    // back up if edge is detected
    if (color < COLOR_THRESHOLD) {
        GetResource(DrivingControlResource);

        change_driving_command(PRIO_COLOR, -100, -100, 1250);
        
        ReleaseResource(DrivingControlResource);
    }

    TerminateTask();
}

/* This task reads the sonar sensor and acts from input.
   NOTE: Should not be run more often that 100ms.
*/
TASK(SonarSensorTask) {
    if (InSetupPhase)
        TerminateTask();

    int duration = 3000; // default duration   
    S32 distance = ecrobot_get_sonar_sensor(PORT_SONAR);

    // I have no idea why this is 0 the first time the task is activated :(
    if (LastSonarValue == 0)
        LastSonarValue = 255;

    // detect close object
    if (distance == 255 && LastSonarValue < 20) {
        // same behavior as below
        change_driving_command(PRIO_FOLLOW, 100, 100, 150);
    } else {
        LastSonarValue = distance;
    }

    /* Seek behavior:
                  0 - seek_delay_1: seek in one direction
       seek_delay_2 - seek_delay_3: seek in other direction
       seek_delay_3 - seek_delay_4: seek in other direction
       seek_delay_4 - seek_delay_4 + N: go forward
                    > seek_delay_4 + N: restart */
    int seek_delay_1 = 1000;
    int seek_delay_2 = seek_delay_1 * 2;
    int seek_delay_3 = seek_delay_2 * 2;
    int seek_delay_4 = seek_delay_3 + 2000;
    if (distance > SEEK_DISTANCE_THRESHOLD) {
        int speed_left, speed_right; // these will contain new speed
        
        if (SeekStarted == 0) {
            // start seeking
            SeekStarted = systick_get_ms();

            // randomize initial direction
            int modifier = copysign(1, rand() - RAND_MAX/2);
            speed_left = -70 * modifier;
            speed_right = 70 * modifier;
            change_driving_command(PRIO_SEEK, speed_left, speed_right,
                                   duration);
        } else {
            U32 current_sys_tick = systick_get_ms();
            U32 delta = current_sys_tick - SeekStarted;

            if ((delta > seek_delay_1 && 
                    delta <= seek_delay_1 + SONARSENSORTASK_PERIOD_LENGTH) ||
                (delta > seek_delay_2 && 
                    delta <= seek_delay_2 + SONARSENSORTASK_PERIOD_LENGTH)) {
                // invert direction
                GetResource(DrivingControlResource);
               
                speed_left = dc.speed_left * -1;
                speed_right = dc.speed_right * -1;
               
                ReleaseResource(DrivingControlResource);
                change_driving_command(PRIO_SEEK, speed_left, speed_right, 
                                       duration);
            } else if (delta > seek_delay_3 && delta <= seek_delay_4) {
                // go forward for a while
                speed_left = 100;
                speed_right = 100;
                change_driving_command(PRIO_SEEK, speed_left, speed_right,
                                       duration);

            } else if (delta > seek_delay_4) {
                // schedule seeking restart by setting SeekStarted to 0
                SeekStarted = 0;
            }
        }
    } else {
        // something found, go straight forward
        change_driving_command(PRIO_FOLLOW, 100, 100, 150);
    }

    TerminateTask();
}

/* This task is responsible for any display work.
*/
TASK(DisplayTask) {
    if (InSetupPhase)
        TerminateTask();

    display_clear(0);

    // display dc struct
    GetResource(DrivingControlResource);
    display_goto_xy(0, 0);
    display_string("PRIO: ");
    switch (dc.priority) {
        case PRIO_IDLE:
            display_string("IDLE");
            break;
        case PRIO_SEEK:
            display_string("SEEK");
            break;
        case PRIO_FOLLOW:
            display_string("FOLLOW");
            break;
        case PRIO_COLOR:
            display_string("COLOR");
            break;
        default:
            display_string("WTF: ");
            display_int(dc.priority, 3);
            break;
    }
    display_goto_xy(0, 1);
    display_string("SPEED L/R: ");
    display_goto_xy(0, 2);
    display_int(dc.speed_left, 4);
    display_string("/");
    display_int(dc.speed_right, 4);
    display_goto_xy(0, 3);
    display_string("DURATION: ");
    display_int(dc.duration, 0);
    ReleaseResource(DrivingControlResource);

    display_goto_xy(0, 4);
    display_string("SYSTEM: ");
    display_int(systick_get_ms(), 0);

    // display color sensor value
    U16 color = ecrobot_get_nxtcolorsensor_light(PORT_COLOR);
    display_goto_xy(0, 5);
    display_string("COLOR: ");
    display_int(color, 0);

    // display sonar sensor value
    S32 sonar = ecrobot_get_sonar_sensor(PORT_SONAR);
    display_goto_xy(0, 6);
    display_string("SONAR: ");
    display_int(sonar, 0);
    display_string(" / ");
    display_int(LastSonarValue, 0);

    display_update();

    TerminateTask();
}

/* This task will auto run at startup. It will do some waiting and then unlock
   the semaphore that blocks all the other tasks.
 */
TASK(SetupTask) {
    if (SetupStarted == 0) {
        // Init setup
        SetupStarted = systick_get_ms();
        InSetupPhase = 5; // We'll be in setup for 5 seconds
    }  

    while(InSetupPhase > 0) {
        SetupStarted = systick_get_ms();
        display_clear(0);
        display_goto_xy(5, 2);
        display_string("SETUP");
        display_goto_xy(7, 4);
        display_int(InSetupPhase, 0);
        display_update();

        // blinkenlights!
        switch(InSetupPhase % 3) {
            case 2:
                ecrobot_set_nxtcolorsensor(PORT_COLOR, NXT_LIGHTSENSOR_GREEN);
                ecrobot_process_bg_nxtcolorsensor();
                break;
            case 1:
                ecrobot_set_nxtcolorsensor(PORT_COLOR, NXT_LIGHTSENSOR_BLUE);
                ecrobot_process_bg_nxtcolorsensor();
                break;
            case 0:
                ecrobot_set_nxtcolorsensor(PORT_COLOR, NXT_LIGHTSENSOR_RED);
                ecrobot_process_bg_nxtcolorsensor();
                break;            
        }

        ecrobot_sound_tone(1800, 50, 100);

        // Wait one second
        while(systick_get_ms() - SetupStarted < 1000) {};
        InSetupPhase--;
    }
    ecrobot_set_nxtcolorsensor(PORT_COLOR, NXT_LIGHTSENSOR_RED);
    ecrobot_sound_tone(2800, 500, 100);

    TerminateTask();
}

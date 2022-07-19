// VRC 2022 - Chubotics (Chu Van An High School, Hanoi)

#include <PS2X_lib.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/* CONFIGURATION */
// PS2 gamepad pin definitions
#define PIN_PS2_DAT     12 // DATA (MISO)
#define PIN_PS2_CMD     13 // CMD (MOSI)
#define PIN_PS2_ATT     15 // ATT (SS)
#define PIN_PS2_CLK     14 // CLK (SCK)
// PS2 gamepad configuration
#define PS2_READ_DELAY  50 // minimum time in ms between reads
// DC motor PWM channel configuration
#define PCH_DC1_B       9 // 1st DC motor, backward
#define PCH_DC1_F       8 // 1st DC motor, forward
#define PCH_DC2_B       11
#define PCH_DC2_F       10
#define PCH_DC3_B       13
#define PCH_DC3_F       12
#define PCH_DC4_B       15
#define PCH_DC4_F       14
// DC motor channel configuration
#define DCH_LEFT        1 // left motor (note that side is relative to front)
#define DCH_RIGHT       4 // right motor
#define DCH_TURBINE     2 // turbine spinner motor
#define DCH_LIFT        3 // lift motor
// servo motor PWM channel configuration
#define PCH_GRIP_L      4 // left grip (180 deg)
#define PCH_GRIP_R       5 // right grip (180 deg)
#define PCH_ARM      6 // arm (360 deg)
// driving speeds
#define DSP_FAST        3071 // "fast" speed
#define DSP_SLOW        1850 // "slow" speed
#define DSP_AUTO_DRV    511 // autonomous driving speed
#define DSP_AUTO_TURN   767 // autonomous turning speed
#define DSP_AUTO_CORR   450 // autonomous line correction speed
// arm/grip configuration
#define GRIP_CLOSE_ANGLE 35 // the grip's angle at closing position relative to opening position
#define GRIP_ANGLE_PAD    90 // the grip's padding angle (for semi-360 deg servos which have a tendency to correct erratically). GRIP_ANGLE_PAD + GRIP_CLOSE_ANGLE <= 180.
#define ARM_SPEED       0.5f // arm rotation speed
// 180 deg servo calibration - performed on 23:19 06-06-2022 with right arm motor
#define S180_PW_MIN    440 // minimum pulse width, corresponding to 0 deg position
#define S180_PW_MAX    2270 // maximum pulse width, corresponding to 180 deg position
// 360 deg servo calibration - cloned from 180 deg calibration above, may change later
#define S360_PW_MIN    400 // minimum pulse width, corresponding to 100% rotation speed
#define S360_PW_MID    1400 // midpoint pulse width, corresponding to no rotation
// turbine configuration
#define DSP_TURBINE     4095 // turbine speed
// autonomous mode sensors pin configuration
#define PIN_LINE_S1     32 // line sensor S1/S5 (far left - left/right turn detection)
#define PIN_LINE_S2     2 // line sensor S2 (near left - left line border)
#define PIN_LINE_S3     36 // line sensor S3 (center - line existence detection)
#define PIN_LINE_S4     39 // line sensor S4 (near right - right line border)
#define PIN_SW_BLOCK    25 // block detection switch, pressed when block is sliding through
#define PIN_SW_LIFT     0 // lift limiting switch
// autonomous mode configuration
#define LINE_LOW          // uncomment this line if line sensor is active low (e.g. during testing)
// lift configuration
#define LSP_UP          -950 // lift up speed (when out of limit switch)
#define LSP_HOLD        -150 // lift hold speed in up position (to prevent lift from falling, not recommended)
#define LSP_DOWN_INIT   500 // lift initial down speed (to get out of limit switch)

PS2X ps2x;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// control DC motor speed - dch=1/2/3/4 corresponding to DC channel, dcspeed=-4096..4096 with <0 meaning backward
void dc_control(int dch, int dcspeed) {
  switch(dch) {
    case 1:
      pwm.setPWM(PCH_DC1_B, 0, (dcspeed < 0) ? (-dcspeed) : 0);
      pwm.setPWM(PCH_DC1_F, 0, (dcspeed > 0) ?   dcspeed  : 0);
      break;
    case 2:
      pwm.setPWM(PCH_DC2_B, 0, (dcspeed < 0) ? (-dcspeed) : 0);
      pwm.setPWM(PCH_DC2_F, 0, (dcspeed > 0) ?   dcspeed  : 0);
      break;
    case 3:
      pwm.setPWM(PCH_DC3_B, 0, (dcspeed < 0) ? (-dcspeed) : 0);
      pwm.setPWM(PCH_DC3_F, 0, (dcspeed > 0) ?   dcspeed  : 0);
      break;
    case 4:
      pwm.setPWM(PCH_DC4_B, 0, (dcspeed < 0) ? (-dcspeed) : 0);
      pwm.setPWM(PCH_DC4_F, 0, (dcspeed > 0) ?   dcspeed  : 0);
      break;
  }
}

// control servo angle - sch=2..7 corresponding to servo PWM channel, sangle=0..180
void servo_control_angle(int sch, float sangle) {
  //Serial.print(sch, DEC); Serial.print(' '); Serial.println(1000 + sangle * 1000 / 180, DEC);
  pwm.writeMicroseconds(sch, S180_PW_MIN + sangle * (S180_PW_MAX - S180_PW_MIN) / 180);
}

// control free rotation servo rotation speed - sch = 2..7 corresponding to servo PWM channel, sspeed=-1..1, -1=full speed counterclockwise, 1=full speed clockwise
void servo_control_speed(int sch, float sspeed) {
  pwm.writeMicroseconds(sch, S360_PW_MID + sspeed * (S360_PW_MID - S360_PW_MIN));
}

// grip control functions
void grip_open() {
  servo_control_angle(PCH_GRIP_L, GRIP_ANGLE_PAD);
  servo_control_angle(PCH_GRIP_R, 180 - GRIP_ANGLE_PAD);
}

void grip_close() {
  servo_control_angle(PCH_GRIP_L, GRIP_ANGLE_PAD + GRIP_CLOSE_ANGLE);
  servo_control_angle(PCH_GRIP_R, 180 - GRIP_ANGLE_PAD - GRIP_CLOSE_ANGLE);
}

// line sensor read functions
bool line_read_pin(int sensor) {
  int val;
  switch(sensor) {
    case 1: val = digitalRead(PIN_LINE_S1); break;
    case 2: val = digitalRead(PIN_LINE_S2); break;
    case 3: val = digitalRead(PIN_LINE_S3); break;
    case 4: val = digitalRead(PIN_LINE_S4); break;
  }
#ifdef LINE_LOW
  return (val == LOW);
#else
  return (val == HIGH);
#endif
}

// wheel control functions
void whl_forward(int dspeed) {
  dc_control(DCH_LEFT, dspeed);
  dc_control(DCH_RIGHT, dspeed);
}

void whl_backward(int dspeed) {
  dc_control(DCH_LEFT, -dspeed);
  dc_control(DCH_RIGHT, -dspeed);
}

void whl_left(int dspeed) { // turn left while moving forward
  dc_control(DCH_LEFT, 0);
  dc_control(DCH_RIGHT, dspeed);
}

void whl_left_back(int dspeed) { // turn left while moving backward
  dc_control(DCH_LEFT, -dspeed);
  dc_control(DCH_RIGHT, 0);
}

void whl_right(int dspeed) {
  dc_control(DCH_LEFT, dspeed);
  dc_control(DCH_RIGHT, 0);
}

void whl_right_back(int dspeed) {
  dc_control(DCH_LEFT, 0);
  dc_control(DCH_RIGHT, -dspeed);
}

void whl_clockwise(int dspeed) {
  dc_control(DCH_LEFT, dspeed);
  dc_control(DCH_RIGHT, -dspeed);
}

void whl_counterclockwise(int dspeed) {
  dc_control(DCH_LEFT, -dspeed);
  dc_control(DCH_RIGHT, dspeed);
}

void whl_stop() {
  dc_control(DCH_LEFT, 0);
  dc_control(DCH_RIGHT, 0);
}

unsigned long t_ps2_read; // timestamp of last PS2 read (workaround for unstable readouts)

void setup() {
  /* initialize serial for debugging */
  Serial.begin(115200);
  Serial.println(F("itsmevjnk's VRC 2022 firmware"));

  /* initialize PWM controller */
  Serial.print(F("Initializing PCA9685..."));
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); // for both servo and DC
  Wire.setClock(400000); // set I2C speed to 400KHz
  Serial.println(F("done."));

  grip_open(); // set arms to open position, which is also useful for installation

  /* initialize PS2 gamepad */
  Serial.print(F("Initializing PS2 gamepad"));
  while(1) {
    Serial.print('.');
    if(!ps2x.config_gamepad(PIN_PS2_CLK, PIN_PS2_CMD, PIN_PS2_ATT, PIN_PS2_DAT, false, false)) {
      Serial.println(F("done."));
      t_ps2_read = millis();
      break;
    } else delay(1000);
  }

  /* initialize sensor inputs */
  Serial.print(F("Initializing sensor input pins..."));
  pinMode(PIN_LINE_S1, INPUT); pinMode(PIN_LINE_S2, INPUT); pinMode(PIN_LINE_S3, INPUT); pinMode(PIN_LINE_S4, INPUT); // S1 through S4 are actively driven (by the 74HC14D) so there's no need for pullup
  pinMode(PIN_SW_BLOCK, INPUT_PULLUP); pinMode(PIN_SW_LIFT, INPUT_PULLUP); // momentary switches require pullup as they're passively driven
}

// lift control functions
bool lift_limit_changed = false; // set when lift limit switch is released
int lift_up() { // return 0 on completion
  if(digitalRead(PIN_SW_LIFT) == LOW) {
    dc_control(DCH_LIFT, LSP_HOLD);
    Serial.println(F("Raising lift complete, holding"));
    lift_limit_changed = false;
    return 0;
  }
  
  dc_control(DCH_LIFT, LSP_UP);
  Serial.println(F("Raising lift at constant speed"));
  return -1; // not completed
}

int lift_down() {
  if(digitalRead(PIN_SW_LIFT) == LOW) {
    dc_control(DCH_LIFT, LSP_DOWN_INIT);
    Serial.println(F("Lowering lift at initial speed"));
    lift_limit_changed = false;
    return -1;
  }
  
  dc_control(DCH_LIFT, 0);
  Serial.println(F("Lowering lift complete"));
  return 0; // not completed
}

int mot_speed = DSP_FAST; // driving speed, defaults to DSP_FAST
bool speed_toggled = false; // set when SELECT is pressed, used to prevent speed from being toggled multiple times in one press
int autonomous = 0; // 0 - manual mode, 1 - autonomous mode selection, 2 - autonomous active
bool auto_toggled = false; // set when START is pressed, same reason as above
bool grip_closed = false; // set when arm is closed
bool grip_toggled = false; // set when TRIANGLE is pressed, same reason as above
bool auto_l2 = false; // set to lift energy blocks to 2nd level
bool auto_right = false; // set if bot is operating on right half (take right turn at junction instead of left)
bool lift_raised = false; // set when lift is in UP position
bool lift_toggled = false;
bool arm_held = false;
bool arm_hold_toggled = false;

void loop() {
  //if(digitalRead(PIN_SW_BLOCK) == HIGH) Serial.println("high"); else Serial.println("low");
  bool acty_dc = false; // set if there's activity on DC motor, and if there's none motors will be turned off (only affects manual mode)

  while(millis() - t_ps2_read < PS2_READ_DELAY);
  ps2x.read_gamepad(false, false); // read new input
  delay(10);
  ps2x.read_gamepad(false, false); // read new input
  t_ps2_read = millis();

  if(ps2x.Button(PSB_PAD_UP)) {
    // forward
    acty_dc = true;
    Serial.print(F("PSB_PAD_UP forward "));
    whl_forward(mot_speed);
  } else if(ps2x.Button(PSB_PAD_DOWN)) {
    // backward/backward turn left/right
    acty_dc = true;
    Serial.print(F("PSB_PAD_DOWN"));
    if(ps2x.Button(PSB_PAD_LEFT)) {
      // backward turn left
      Serial.print(F("+PSB_PAD_LEFT backward turn left "));
      whl_left_back(mot_speed);
    } else if(ps2x.Button(PSB_PAD_RIGHT)) {
      // backward turn right
      Serial.print(F("+PSB_PAD_RIGHT backward turn right "));
      whl_right_back(mot_speed);
    } else {
      // backward
      acty_dc = true;
      Serial.print(F(" backward "));
      whl_backward(mot_speed);
    }
  } else if(ps2x.Button(PSB_PAD_LEFT)) {
    // turn left
    acty_dc = true;
    Serial.print(F("PSB_PAD_LEFT turn left "));
    whl_left(mot_speed);
  } else if(ps2x.Button(PSB_PAD_RIGHT)) {
    // turn right
    acty_dc = true;
    Serial.print(F("PSB_PAD_RIGHT turn right ")); 
    whl_right(mot_speed);
  } else if(ps2x.Button(PSB_L2)) {
    // turn counterclockwise
    acty_dc = true;
    Serial.print(F("PSB_L2 turn counterclockwise "));
    whl_counterclockwise(mot_speed);
  } else if(ps2x.Button(PSB_R2)) {
    // turn clockwise
    acty_dc = true;
    Serial.print(F("PSB_R2 turn clockwise "));
    whl_clockwise(mot_speed);
  }

  if(ps2x.Button(PSB_L3)) {
    // toggle arm hold
    if(!arm_hold_toggled) {
      arm_hold_toggled = true;
      arm_held = !arm_held;
      Serial.print(F("PSB_L3 "));
      Serial.print((arm_held) ? F("enable ") : F("disable "));
      Serial.println(F("arm hold"));
    }
  } else arm_hold_toggled = false;
  
  if(ps2x.Button(PSB_L1)) {
    // open arm
    Serial.println(F("PSB_L1 open arm"));
    servo_control_speed(PCH_ARM, ARM_SPEED);
    arm_held = false;
  } else if(ps2x.Button(PSB_R1)) {
    // close arm
    Serial.println(F("PSB_R1 close arm"));
    servo_control_speed(PCH_ARM, -ARM_SPEED);
    arm_held = false;
  } else if(arm_held) servo_control_speed(PCH_ARM, -ARM_SPEED);
  else servo_control_speed(PCH_ARM, 0);

  if(acty_dc) Serial.println(mot_speed, DEC); // show speed in debug log
  else whl_stop(); // put motors to rest

  if(ps2x.Button(PSB_SELECT)) {
    // modifier key
    if(ps2x.Button(PSB_TRIANGLE)) {
      // toggle speed
      if(!speed_toggled) {
        speed_toggled = true;
        Serial.print(F("PSB_SELECT + PSB_TRIANGLE toggle speed "));
        Serial.print(mot_speed, DEC);
        Serial.print(F(" -> "));
        mot_speed = (mot_speed == DSP_FAST) ? DSP_SLOW : DSP_FAST;
        Serial.println(mot_speed, DEC);
      }
    } else speed_toggled = false;
    
    if(ps2x.Button(PSB_CIRCLE)) {
      Serial.println(F("PSB_SELECT + PSB_CIRCLE lift block"));
      lift_raised = true; while(lift_up() != 0);
      grip_closed = false; grip_open();
    }
  } else {
    if(ps2x.Button(PSB_TRIANGLE)) {
      if(!grip_toggled) {
        grip_toggled = true;
        grip_closed = !grip_closed;
        Serial.print(F("PSB_TRIANGLE "));
        if(grip_closed) {
          grip_close();
          Serial.print(F("close "));
        } else {
          grip_open();
          Serial.print(F("open "));
        }
        Serial.println(F("grip"));
      }
    } else grip_toggled = false;

    if(ps2x.Button(PSB_SQUARE)) {
      Serial.println(F("PSB_SQUARE rotate turbine"));
      dc_control(DCH_TURBINE, DSP_TURBINE);
    } else dc_control(DCH_TURBINE, 0);

    if(ps2x.Button(PSB_CIRCLE)) {
      if(!lift_toggled) {
        lift_toggled = true;
        lift_raised = !lift_raised;
        Serial.print(F("PSB_CIRCLE "));
        if(lift_raised) {
          Serial.println(F("raise lift"));
          while(lift_up() != 0);
        } else {
          Serial.println(F("lower lift"));
          while(lift_down() != 0);
        }
      }
    } else lift_toggled = false;
  }
}

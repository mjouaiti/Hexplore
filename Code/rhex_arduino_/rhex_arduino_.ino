#include <Dynamixel2Arduino.h>
#include <actuator.h>

#include "leg_info.hh"
#include "control_parameters.hh"
#include "desired_values.hh"
#include "conversions.hh"
#include "pd_control.hh"
#include "gait_parameters.hh"

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
const uint8_t DXL_ID_CNT = 6;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2, 3, 4, 5, 6};

const uint16_t SW_START_ADDR = 104; //Goal velocity
const uint16_t SW_ADDR_LEN = 4;

typedef struct sw_data{
  int32_t goal_velocity;
} __attribute__((packed)) sw_data_t;

DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];
sw_data_t sw_data[DXL_ID_CNT];



// Control Table //
#define MOVING_SPEED 32
#define PRESENT_POS 37
#define PRESENT_SPEED 39
#define PRESENT_VOLTAGE 45
#define LED 25

//Rewritable globals
float desired_vel;
float desired_theta;
float actual_vel;
float actual_theta;
float control_signal;
float actual_p;

//Deadzone
int dead_buffer = 40;

// Legs setup //
const int legs_active = 6;

// Packet Setup //
const int packet_length =  2 * legs_active;
word packet[packet_length]; 

// Button Setup //
int button_state;
int last_button_state = 0;

// Battery Check //
int low_battery = 1; // 1 = red, 3 = yellow, 2 = green
int prev_low_battery = 0;
int voltage;
int voltage_check;

using namespace ControlTableItem;  

void jump_ready(){
  int t_start = millis(); 
  for (int i = 1; i <= legs_active; i++){
    legs[i].desired_theta = 90;
    update_gait(i, STAND, t_start);
  }
  SERIAL_USB.println("JUMP READY");
}

void jump(){

  int t_start = millis();
  while (millis() - t_start < 900){
    SERIAL_USB.println(t_start - millis());
    if (millis() - t_start > 0){
      dxl.writeControlTableItem(MOVING_SPEED, 1, 1023);
      dxl.writeControlTableItem(MOVING_SPEED, 4, 2047);
    }
    if (millis() - t_start > 100){
      dxl.writeControlTableItem(MOVING_SPEED, 2, 1023);
      dxl.writeControlTableItem(MOVING_SPEED, 5, 2047);
    }
    if (millis() - t_start > 190){
      dxl.writeControlTableItem(MOVING_SPEED, 3, 1023);
      dxl.writeControlTableItem(MOVING_SPEED, 6, 2047);
    }
  }
  for (int i = 1; i <= legs_active; i++){
    legs[i].desired_theta = 0;
    update_gait(i, STAND, t_start);
  }

}

void setup() {
  DEBUG_SERIAL.begin(115200);
  dxl.setPortProtocolVersion(2.0);
  dxl.begin(115200); //baudrate set to 1 Mbps (max)
  dxl.scan();
  // Serial2.begin(57600); //set up serial usb input
  // pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN); //setup user button
  // pinMode(BOARD_LED_PIN, OUTPUT); //setup LED
  int t_start = millis();
  for (int i = 1; i <= legs_active; i++){ //legs stored at their index
    dxl.torqueOff(legs[i].id);
    dxl.setOperatingMode(legs[i].id, OP_VELOCITY); //change servo to wheel mode
    dxl.torqueOn(legs[i].id);
    update_gait(i, initial_gait, t_start); //set initial parameters, initial_gait in gait_parameters
  }

  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

}

int count = 0;
void loop() {
  // put your main code here, to run repeatedly:
//time count
  count++;
  
  prev_low_battery = low_battery;
  //Every 100 loop iterations, find max voltage supplied to each leg and compare with nominal
  if (count%10 == 0){
    voltage = 0;
    for (int i = 1; i <= legs_active; i++){
      voltage_check = dxl.readControlTableItem(PRESENT_VOLTAGE, legs[i].id);
      if (voltage_check > voltage) voltage = voltage_check;
    }
    SERIAL_USB.println(voltage);

    if (voltage > 73){ //green
      low_battery = 2;
    }
    else if (voltage < 71){ //red
      low_battery = 1;
    }
    else{
      low_battery = 3; //yellow
    }
  }

  if (prev_low_battery != low_battery){
    SERIAL_USB.println("Should switch led color here");
    for (int i = 1; i <= legs_active; i++){
      dxl.writeControlTableItem(LED, legs[i].id, low_battery);
      // dxl.ledOn(DXL_ID) : dxl.ledOff(DXL_ID);
    }
  }
  
  //bluetooth control
  if (SERIAL_BT.available()){ 
    char a = (char)(SERIAL_BT.read());
    int gait = -1;
    switch (a){
    case 'q': 
      gait = STAND; 
      break; //stand
    case 'w': 
      gait = WALK; 
      break; //forwards
    case 'a': 
      gait = LEFT; 
      break; //left
    case 's': 
      gait = REVERSE; 
      break; //reverse
    case 'd': 
      gait = RIGHT; 
      break; //right 
    case 'e':
      gait = PRONK;
      break;
    case 'x':
      jump_ready();
      break;
    case 'j':
      jump();
      break;
    }

    if (gait != -1){
      int t_start = millis(); 
      for (int i = 1; i <= legs_active; i++){
        update_gait(i, gait, t_start);
      }
    }
  }

  //button control
  // button_state = digitalRead(BOARD_BUTTON_PIN);
  // if (button_state > last_button_state) user_button_pressed();
  // else if (button_state < last_button_state) user_button_released();
  // last_button_state = button_state;


  //primary for-loop

  for(int i = 1; i <= legs_active; i++){
    info_xels_sw[i].id = legs[i].id;
    actual_p = dxl.readControlTableItem(PRESENT_POSITION, legs[i].id);
    actual_theta = P_to_Theta(actual_p); // converted to degrees, relative to leg
    actual_vel = dynV_to_V(dxl.readControlTableItem(PRESENT_VELOCITY, legs[i].id)); // converted to degrees/ms, relative to leg
    if (!legs[i].deadzone){
      
      if (actual_p == 0 || actual_p == 1023){ //entering deadzone
        legs[i].deadzone = true;
        if (actual_p == 0) legs[i].dead_from_neg = true;
        else legs[i].dead_from_neg = false;
        continue;
      }

      if (legs[i].gait == STAND){ //standing or sitting
        if (legs[i].right_side){
          desired_theta = Theta_to_ThetaR(legs[i].desired_theta);
        }
        else{
          desired_theta = legs[i].desired_theta;
        }
        actual_theta = actual_theta - legs[i].zero; //zero out leg thetas, accounts for small servo irregularities
        control_signal = pd_controller(actual_theta, desired_theta, actual_vel, 0, kp_hold, kd_hold); 
      }
      else { //walking, turning
        //compute absolute desired values (theta and velocity) from clock time
        vals v = get_desired_vals(millis(), legs[i]);
        //translate theta and v to relative (left and right)
        if (legs[i].right_side){
          desired_vel = -v.global_velocity; //relative
          desired_theta = Theta_to_ThetaR(v.global_theta); // relative
        }
        else{ //left side, relative is same as global
          desired_vel = v.global_velocity;
          desired_theta = v.global_theta;
        }
        actual_theta = actual_theta - legs[i].zero;
        
        control_signal = pd_controller(actual_theta, desired_theta, actual_vel, desired_vel, legs[i].kp, legs[i].kd);  
      }


      int new_vel = V_to_dynV(actual_vel + control_signal);
      info_xels_sw[i].p_data = (uint8_t*)&new_vel;
    }

    else{ //deadzone
      if ((actual_p > 0) & (actual_p < dead_buffer) || (actual_p < 1023) & (actual_p > 1023 -dead_buffer)){ //exiting deadzone
        legs[i].deadzone = false;
      }
      float signed_recovery_speed = legs[i].dead_from_neg == true ? -legs[i].recovery_speed : legs[i].recovery_speed;
      info_xels_sw[i].p_data = (uint8_t*)V_to_dynV(signed_recovery_speed);
    }
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;


  dxl.syncWrite(&sw_infos);
  // dxl.syncWrite(MOVING_SPEED, 1, packet, packet_length); //simultaneously write to each of 6 servoes with updated commands
}

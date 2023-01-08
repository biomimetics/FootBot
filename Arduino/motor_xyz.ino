#include "HX711.h"

// Serial Read Variables
const unsigned int MAX_MESSAGE_LENGTH = 18;
const char tail = '>';
char command;

// Define Motor Parameters
const int dirPin_x = 2;     //Low for up, High for down
const int stepPin_x = 3;
const int dirPin_y = 4;     //Low for up, High for down
const int stepPin_y = 5;
const int dirPin_z = 8;     //Low for up, High for down
const int stepPin_z = 9;
const int enPin_x = 11;
const int enPin_y = 12;
const int enPin_z = 13;

double step_status[3] = {0, 0, 0}; //Count number of total steps taken by motor

int hold_time = 1000; //hold for 1000 milli - sec after each motor move

// HX711 circuit wiring
uint8_t LOADCELL_DOUT_PIN = A5;
uint8_t LOADCELL_SCK_PIN = A4;

// Load Cell Mass Varaibles
double mass_measurement;   // Initial mass


HX711 scale;

void setup(){
  pinMode(stepPin_x, OUTPUT);
  pinMode(dirPin_x, OUTPUT);
  pinMode(stepPin_y, OUTPUT);
  pinMode(dirPin_y, OUTPUT);
  pinMode(stepPin_z, OUTPUT);
  pinMode(dirPin_z, OUTPUT);
  pinMode(enPin_x, OUTPUT);
  pinMode(enPin_y, OUTPUT);
  pinMode(enPin_z, OUTPUT);
  Serial.begin(9600); //Serial Baudrate
  digitalWrite(enPin_x, HIGH);
  digitalWrite(enPin_y, HIGH);
  digitalWrite(enPin_z, HIGH);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(505);
  scale.tare();
}

// START LOOPING _________________________________________________________________________________________________
void loop(){

 //Check to see if anything is available in the serial receive buffer
  while (Serial.available() > 0) {
    //Create a place to hold the incoming message
    static char message[MAX_MESSAGE_LENGTH];
    static unsigned int message_pos = 0;

    //Read the next available byte in the serial receive buffer
    char inByte = Serial.read();

    //Message coming in (check not terminating character) and guard for over message size
    if (((inByte != tail) && (message_pos < MAX_MESSAGE_LENGTH-1))||(message_pos == 0)) {
      //Add the incoming byte to our message
      message[message_pos] = inByte;
      if (message[0] == 'M'){
        message_pos++;
      }
    }
    //Full message received... Proceed to the actual Command
    else {
      //Add null character to string
      message[message_pos] = '\0';
      //Reset for the next message
      message_pos = 0;

      //Serial.println(message);
      
      double motor_steps = read_motor_steps(message);
      int motor_num = message[2]-'0';
      int motor_dir = message[11]-'0';
      int mass_read = message[15]-'0';

      //Serial.println(step_status_1); 
      //Serial.println(motor_steps);
      //Serial.println(motor_dir);
      motor_move(motor_steps, motor_dir, motor_num);

      if (mass_read != 0) {
        delay(hold_time);
        mass_measurement = get_weight();
        printresults(mass_measurement);
      }
      else {
        Serial.println("Readings not taken ");
      }
    }
  }
}


//OTHER FUNCTIONS _________________________________________________________________________________________________

void motor_move(int mt_steps, int mt_dir, int mt_index){
  int P_dir;
  int P_step;
  int P_en;
  if (mt_index == 1){
    P_dir = dirPin_x;
    P_step = stepPin_x;
    P_en = enPin_x;
  }
  else if (mt_index == 2){
    P_dir = dirPin_y;
    P_step = stepPin_y;
    P_en = enPin_y;
  }
  else {
    P_dir = dirPin_z;
    P_step = stepPin_z;
    P_en = enPin_z;
  }

  if (mt_dir == 1){
    motorup(mt_steps, P_dir, P_step, P_en);
  }
  else{
    motordown(mt_steps, P_dir, P_step, P_en);
  }
}

double get_weight(){         //Read the mass value from the scale
  double m = scale.get_units(1); //Only get 3 reading and average them
  return m;
}

void motordown(double number,int dirPin, int stepPin, int enPin){ //make motor go down for number of steps
  digitalWrite(enPin, LOW);
  delay(10);
  digitalWrite(dirPin, HIGH);
  for (int i = 0; i < (number); i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
  digitalWrite(enPin,HIGH);
}

void motorup(double number,int dirPin, int stepPin, int enPin){ //make motor go up for number of steps
  digitalWrite(enPin, LOW);
  delay(10);
  digitalWrite(dirPin, LOW);
  for (int i = 0; i < (number); i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
  digitalWrite(enPin,HIGH);
}

void printresults(double f1){
    Serial.print(f1);
    Serial.println(" N ");
}

double read_motor_steps(char command[]){
  int hundred_thousands = command[4] - '0';
  int ten_thousands = command[5] - '0';
  int thousands = command[6] - '0';
  int hundreds = command[7] - '0';
  int tens = command[8] -'0';
  int singles = command[9] -'0';
  double amount = 100000*hundred_thousands + 10000*ten_thousands + 1000*thousands + 100*hundreds + 10*tens + singles;
  return amount;
}
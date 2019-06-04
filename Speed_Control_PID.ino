/* Rotating Tank Speed Controller
 *  
 * Written by Sebastian Viasus, Summer 2015
 * Modified by Jason Goodman, Summer 2018
 */

#include <math.h>;
#include <Wire.h>
#include <Adafruit_LiquidCrystal.h>
#include <Adafruit_MCP4725.h>
#include <PID_v1.h>

// LCD setup: Connect via i2c, default address #0 (A0-A2 not jumpered)
Adafruit_LiquidCrystal lcd(0);

//RPM sensor interrupt pin
const int encoderPin = 2;

// RPM geometry variables
const float calibration_factor = 1.0;  // Adjust this to fine-tune RPM
const int pulses_per_revolution = 32; // How many counts for a full circle of rotation?
const float radius_of_sensorwheel = 1.9;
const float distance_from_table_center = 13.8;
const float pulses_per_tablerotation = pulses_per_revolution * distance_from_table_center / radius_of_sensorwheel;

// Timer variables
volatile float average_rpm = 0.0;  // Current RPM value
const float weight = 0.2;  // Weight of new data in autoregressive moving average
unsigned long delta_t;
unsigned long time = 0;
float set_rpm = 0.00; // The target value that you want the RPM to be

// Pushbutton setup
const int increase_buttonPin= 11;  // Increase Pin
const int  decrease_buttonPin= 12;  // Decrease Pin

// DAC setup
#define DAC_RESOLUTION    12
Adafruit_MCP4725 dac;

#define MAXRPM 20

uint32_t voltage = 0;  // Initial voltage 
const int v_offset = 1400;  // Zero point offset for voltage control

#define BASE_STEP .02
float step = BASE_STEP;  // Buttonpress step size, RPM

// PID controller setup
double Setpoint_RPM;
double Input_RPM;
double Output_voltage;
PID myPID(&Input_RPM, &Output_voltage, &Setpoint_RPM,200,100,0,DIRECT);


/*-----------------------SETUP---------------------------*/


void setup() {
  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x62);
  dac.setVoltage(voltage+v_offset,false);
    
  Serial.begin(9600);
 
 //Setting up lCD
  lcd.begin(16,2);
  lcd.print("RPM:    Set RPM:");
  
  lcd.setCursor(0,1);
  lcd.print(average_rpm); 
  
  // Setting up the pushbutton pins to be an input:

  pinMode(increase_buttonPin, INPUT);
  pinMode(decrease_buttonPin, INPUT);
  pinMode(2,INPUT);     
  Serial.println("Arduino Started.");

  //creating interrupt 
  attachInterrupt(0, calculate_time,RISING); 

  //PID Setup
  // Limiting the output values 
  myPID.SetOutputLimits(0,1600);
  //Turning PID On 
  myPID.SetMode(AUTOMATIC);

  //Initializing PID variables we are linked to 
  Setpoint_RPM = 0.;

  }

/*-----------------MAIN LOOP------------------*/

void loop() {
  //--------------------Button Configurations -----------
  
  if (digitalRead(increase_buttonPin) == LOW) { // Button is pushed
      Setpoint_RPM=min(Setpoint_RPM+step,MAXRPM);
      step = step * 1.02;
      Serial.println(step);
  } else if (digitalRead(decrease_buttonPin) == LOW) { // Button is pushed
    Setpoint_RPM=max(Setpoint_RPM-step,0);
    step = step * 1.02;
  } else {
    step = BASE_STEP;
  }

  Input_RPM = average_rpm;
  myPID.Compute();
  voltage = (int)Output_voltage;
  
  
  dac.setVoltage(voltage+v_offset,false);
  
//-----------------------------Monitoriing RPM-------------------------------------
  if( millis()-time > 2*delta_t)// If we wait a long time for an RPM update... 
  {
    average_rpm = (1-weight) * average_rpm + (weight) * (60.0*1000)/((millis()-time) * pulses_per_tablerotation);
  }
  //Prints rpm
  print_rpm();
  delay(50);   
    Serial.print("voltage = ");
    Serial.print(Output_voltage);
    Serial.print(", Set RPM = ");
    Serial.print(Setpoint_RPM);
    Serial.print(", True RPM = ");
    Serial.println(average_rpm);
}


/*------------FUNCTIONS--------------------------------*/

// Function that is called every time that there is an interrupt 



void calculate_time()
{
    
   // Calculating and storing each RPM value when an interrupt occurs 
   // Making Sure you aren't recieving interrupts while calculating
  detachInterrupt(0); 
  delay(1);  // debounce   
  //Calculating the change in time since the sensor was last triggered 
   delta_t = millis()-time;
//  Serial.print("Interrupt! ");
//  Serial.println(delta_t);
   time= millis();
  average_rpm = (1-weight) * average_rpm + (weight) * (60.0*1000)/(delta_t * pulses_per_tablerotation);
  
  // Continue monitoring interrupts
  
  attachInterrupt(0,calculate_time,RISING);
  
}


// Function to print the average RPM 
void print_rpm()
{ 
        
      //Erasing previous info from bottom row
         lcd.setCursor(0,1);
         lcd.print("               ");
         
      //Update RPM displayed
         lcd.setCursor(0,1);
         lcd.print(average_rpm);
         lcd.setCursor(12,1);
         lcd.print(Setpoint_RPM); 
}

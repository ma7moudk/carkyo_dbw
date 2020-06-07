#include <Wire.h>
#include <SoftwareSerial.h>

//Arduino pins: 3, 4, 5, 6, 7, 9, 10, Analog: 0, 1, 2

int commonGND_relay_pin =3 ;
int sensorA_relay_pin = 4;
int sensorB_relay_pin = 5;
int sensorGND_relay_pin = 6;

int brake_pin = 7;


char last_mode = 'm';

int sw_serial_rx = 10;
int sw_serial_tx = 9;

int ss_rx = 11;
int ss_tx = 12;

int analog_pin_pot = A0;
int analog_pin_sensor_a = A1; // Original sensor_a pin
int analog_pin_sensor_b = A2; // Original sensor_b pin

float angleFb ;

SoftwareSerial ss (sw_serial_rx, sw_serial_tx);
//SoftwareSerial ss2 (ss_rx, ss_tx);


int count =+ 0;

char x = 's', y = 's';
char rf_state = 's' ;
unsigned long sum = 0;
unsigned long last_sent_time ;
unsigned long last_manual_time ;
unsigned long last_forced_manual_time ;
unsigned long period = 30;
float pot_val = 0.0;

float voltageNeeded, voltageNeededComp;

// DAC 
#define MCP4725_ADDR1 0x60
#define MCP4725_ADDR2 0x61

int out, out1, out2;


// PID Variables:
float Kp=1/7.0 , Ki=0.002 , Kd=0.01;
//kp was initially set with (1/8.0 = 2.5/20 = max_change / max_steering) 
float voltageP, voltageI, voltageD, voltagePID;
float angle_diff;
float abs_angle_diff;
float boundary_max = 2.0;
float boundary_min = 0.01;
int count_ = 0;
int stopFreq = 2;

void setup()
{
  pinMode(brake_pin, OUTPUT);
  pinMode(sensorA_relay_pin, OUTPUT);
  pinMode(sensorB_relay_pin, OUTPUT);
  pinMode(sensorGND_relay_pin, OUTPUT);
  pinMode(commonGND_relay_pin, OUTPUT);
  
  digitalWrite(brake_pin, HIGH); // Default Active low relay
  
  digitalWrite(sensorA_relay_pin , LOW ); // Default Active HIGH relay
  digitalWrite(sensorB_relay_pin , LOW); // Default Active HIGH relay
  digitalWrite(sensorGND_relay_pin , LOW); // Default Active HIGH relay
  digitalWrite(commonGND_relay_pin , LOW); // Default Active HIGH relay
 
  ss.begin(9600);
 // ss2.begin(9600);
  Serial.begin(9600);
  Wire.begin();
  delay(100);
  
  // init DACs
  voltageNeeded = 2.5;
  applyVoltage(voltageNeeded);
  last_sent_time = millis();
}

void loop()
{
/*
  if (ss2.available())
  {
    String temp = ss2.readStringUntil('\n');
    String bMaxC = ss2.readStringUntil(',');
    String bMinC = ss2.readStringUntil(',');
    String kpC = ss2.readStringUntil(',');
    String kiC = ss2.readStringUntil(',');
    String kdC = ss2.readStringUntil('\n');
    if (bMaxC.toFloat() < 0.5 || bMaxC.toFloat() > 2.5 ||
        bMinC.toFloat() > 1.0 ||
        kpC.toFloat() == 0.0 || kpC.toFloat() > 0.5 || 
        kiC.toFloat() >= 0.3 ||
        kdC.toFloat() == 0.9)
    {
    }
    else
    { 
      boundary_max = bMaxC.toFloat();
      boundary_min = bMinC.toFloat();
      Kp = kpC.toFloat();
      Ki = kiC.toFloat();
      Kd = kdC.toFloat();
    }
    
//    Serial.print("KP: ");
//    Serial.print(Kp);
//    Serial.print(", KI: ");
//    Serial.print(Ki);
//    Serial.print(", KD: ");
//    Serial.println(Kd);
  }
*/
  if (ss.available ())
  {
    x = ss.read();
    Serial.println(x);
    Serial.println("Message read");

  }
  else if (Serial.available())
  {
    //y = Serial.read();
    char brakeCmd = Serial.readStringUntil(',')[0];
    char mode = Serial.readStringUntil(',')[0];
    String strAngleCmd = Serial.readStringUntil('\n');
//    ss2.print("brakeCmd: ");
//    ss2.print(brakeCmd);
//    ss2.print("mode: ");
//    ss2.print(mode);
//    ss2.print("AngleCmd: ");
//    ss2.println(strAngleCmd);
    float angleCmd = strAngleCmd.toFloat();
    //float voltageNeeded = calculateVoltage(angleCmd, angleFb);
    
    y = brakeCmd;
    

    if (mode == 'm')
    {
      enable_manual_mode();
    }

    // Apply 2.5 voltage for 2 seconds after switching to auromatic mode  
    else if(mode == 'a' && last_mode == 'm' && (millis() - last_manual_time) < 2000 ) 
    {
      last_mode = 'a';
      voltageNeeded = 2.5;
      applyVoltage(voltageNeeded);
    }
          
    else if(mode == 'a')
    {
      last_mode = 'a';
      enable_automatic_mode();
      voltageNeeded = calculateVoltage(angleCmd, angleFb);
      applyVoltage(voltageNeeded);
    }

  }

  if (x == 'e' || x == 'E')
  {
    digitalWrite(brake_pin , LOW);
    rf_state = 'e' ;
  }
  else if ( x == 's' || x == 'S')
  {
    rf_state = 's' ;
    if (y == 'e')

      digitalWrite(brake_pin , LOW);
    else if (y == 's')
      digitalWrite(brake_pin , HIGH);
  }
  else
  {
    rf_state = 'a';
  }
  
 sum += analogRead(analog_pin_pot);
 count +=1 ;
 
 if (millis() - last_sent_time > period)
 {
   pot_val = float(sum) / count;
   pot_val = pot_val - 70 ;
   if (pot_val < 0.0)
    pot_val = 0.0;
   angleFb = calculate_angleFb(pot_val);
   
   sum = 0.0;
   count = 0;
   last_sent_time = millis();
   send_serial(pot_val , rf_state);
 }
}

void applyVoltage(float v)
{
  out = (int) (v * 4096.0 / 5.0);
  out1 = out >> 4;
  out2 = (out & 15) << 4 ;
  Wire.beginTransmission(MCP4725_ADDR1);
  Wire.write(64);
  Wire.write(out1);
  Wire.write(out1);
  Wire.endTransmission();

  float voltageNeededComp = 5 - v;
  out = (int) (voltageNeededComp * 4096.0 / 5.0);
  out1 = out >> 4;
  out2 = (out & 15) << 4 ;
  Wire.beginTransmission(MCP4725_ADDR2);
  Wire.write(64);
  Wire.write(out1);
  Wire.write(out2);
  Wire.endTransmission();
}

void check_mode()
{
  float sensor_a = analogRead(analog_pin_sensor_a);
  float sensor_b = analogRead(analog_pin_sensor_b);
  float max_sensors_voltage_diff = 0.75;
  if (abs(sensor_a - sensor_b) > max_sensors_voltage_diff)
  {
    last_forced_manual_time = millis();
    enable_manual_mode();
  }

}

void enable_manual_mode()
{
  last_mode = 'm';
  last_manual_time = millis();

  digitalWrite(sensorA_relay_pin , LOW); // Default Active high relay
  digitalWrite(sensorB_relay_pin , LOW); // Default Active high relay
  digitalWrite(sensorGND_relay_pin , LOW); // Default Active high relay
  digitalWrite(commonGND_relay_pin , LOW);

}

bool starting = true;
int high_counter = 0; 
void enable_automatic_mode()
{
  
  /* if(millis() - last_forced_manual_time > 10000)
   * Don't enable automatic mode unless 10 seconds period passed after last try to force steering manually
   * TODO Raspbeery pi should know that
  {*/
  digitalWrite(sensorA_relay_pin , HIGH);
  digitalWrite(sensorB_relay_pin , HIGH);
  digitalWrite(sensorGND_relay_pin , HIGH); // Default Active high relay
  digitalWrite(commonGND_relay_pin , HIGH);

 // }

}

void send_serial(float pot , char state) {
  byte start = 255;
  int x = int(pot);
  byte low  = x;
  byte high = x >> 8;
  Serial.write(start);
  Serial.write(start);
  Serial.write(high);
  Serial.write(low);
  Serial.write(state);
}

float calculate_angleFb(int pot_val )
{
  float angle_fb = 101.0;
  float low_pot_coeff[] = {-0.04303426 , 22.70881886};
  float high_pot_coeff[] = {0.05340562 , -26.73804045};
  if ((pot_val > 50) && (pot_val < 974))
  {
      //new equation (2 lines)
      if (pot_val > 518)
          angle_fb = - ( high_pot_coeff[0] * pot_val + high_pot_coeff[1] );
      else if (pot_val < 509)
          angle_fb = low_pot_coeff[0] * pot_val + low_pot_coeff[1];
      else
          angle_fb = 0.0;
  }
  return angle_fb;
 }

float calculateVoltageAbs(float steering_angle_deg, float angle_fb)
{
  float voltage;
  float last_diff = abs_angle_diff;

  angle_diff = steering_angle_deg - angle_fb; // +ve >> moves to left
  abs_angle_diff = abs(angle_diff);
  
  if (abs_angle_diff > 20)
    abs_angle_diff = 20.0;
    
  if (abs_angle_diff > 0.3)
  {      
      voltageP = Kp * abs_angle_diff;
      voltageI = voltageI + (abs_angle_diff * Ki);
      if (voltageI > 2.0 )
        voltageI = 2.0; 

      voltageD = (abs_angle_diff-last_diff) * Kd;

      voltagePID = voltageP + voltageI + voltageD;
  }
  else
      voltagePID = 0.0;


  // boundries check
  if (voltagePID >= 2.0)
    voltagePID = 2.0;
  else if(voltagePID <= 0.01)
    voltagePID = 0.01;

  if (angle_diff > 0.3)
    voltage = 2.5 - voltagePID;
  else if (angle_diff < -0.3)
    voltage = 2.5 + voltagePID;
  else
    voltage = 2.5;
  

  return voltage;
}

float calculateVoltage(float steering_angle_deg, float angle_fb)
{
  float voltage;
  float last_diff = angle_diff;

  angle_diff = angle_fb - steering_angle_deg; //fb - set_point
  abs_angle_diff = abs(angle_diff);
    
  if (abs_angle_diff > 0.3)
  {      
      voltageP = Kp * angle_diff;
      voltageI = voltageI + (angle_diff * Ki);
      if (voltageI > 2.0 )
        voltageI = 2.0; 
      if (voltageI < -2.0 )
        voltageI = -2.0; 
        
      voltageD = (angle_diff-last_diff) * Kd;

      voltagePID = voltageP + voltageI + voltageD;


    // PID accumulation determines to move in opposite direction 
    // reset I component and stop moving for this iteration
    //  +ve diff >> moves to left
    //  +ve voltage >> move to left
    if (voltagePID > 0.0 && angle_diff < 0 ) 
    {
      voltagePID = 0.0;
      voltageI = 0.0;
    }
    if (voltagePID < 0.0 && angle_diff > 0 )
    {
      voltagePID = 0.0;
      voltageI = 0.0;
    }

  // Apply more voltage in left to right steering
  if (voltagePID < 0.0)
    voltagePID *= 1.2;
    
    // boundries check
    if (voltagePID >= boundary_max)
      voltagePID = boundary_max;
    else if (voltagePID <= -boundary_max)
      voltagePID = -boundary_max;
      
    // boundries check
    if (voltagePID <= boundary_min && voltagePID > 0)
      voltagePID = boundary_min;
    else if (voltagePID >= -boundary_min && voltagePID < 0)
      voltagePID = -boundary_min;    
  
  }
  else
  {
      voltagePID = 0.0;
      voltageI = 0.0; // Reset I TODO
  }

  
  voltage = 2.5 + voltagePID;
  //if (angleFb < -25.2 || angleFb > 20)
  //  voltage = 2.5;

/*
  // Prevent steering from exceeding Max limits
  if (angleFb <= -24) // MAX CMD: -21.77
    voltage = 2.0;
  else if (angleFb >= 20) // Max CMD: 19.48
    voltage = 3.0;
  */  
  return voltage;
}

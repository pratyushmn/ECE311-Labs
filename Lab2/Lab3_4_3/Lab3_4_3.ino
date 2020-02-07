#define encoder0PinA 7
#define encoder0PinB 4

#include "Timer.h"
#include "math.h"


//PARAMETERS
const double Freq = 0.5; //Hz
const double Amplitude = 1.5;
const double K = 20;
const double VelocityAmplitude = 0.3;

// DO NOT EDIT BELOW THIS LINE ------------------------------------------------------

// INITIALIZATIONS
//timer
Timer t;

//Convert continuous time model to discrete time model
const double a = 2.46;
const double b = 17.13;
const double T_sample = 5; //ms
const double T = T_sample/1000; //seconds

//Discrete A matrix
const double a11 = 1;
const double a12 = (1-exp(-b*T))/b;
const double a21 = 0;
const double a22 = exp(-b*T);

// Discrete B matrix
const double b1 = a*(T - (1-exp(-b*T))/b)/b;
const double b2 = a*(1-exp(-b*T))/b;

//Observer poles
const double lp1 = 0.1;
const double lp2 = 0.1;

// Controller gain
const double Kp = -b/a*((exp(-(a*K+b)*T)-exp(-b*T))/(1-exp(-b*T)));

//Observer gains
const double L_1 = (lp1+lp2) + a22 + 1;
const double L_2 = (lp1*lp2 - a22*(1 - L_1))/a12;

//Hardware constants
//encoder clicks to position constant
double K_encoder = 2.2749*0.00001;
//Motor
const int PWM_A   = 11;
const int DIR_A   = 8;
//Encoder
enum PinAssignments {
  encoderPinA = 2,
  encoderPinB = 3,
};
//Data trasnfer rate
const int Tplot = 10;
// Experiment Time
const int T_exp = 15000 + 5*Tplot;


//Variables
double ref;

volatile int encoderPos = 0;
int lastReportedPos = 1;
boolean A_set = false;
boolean B_set = false;

//Total elapsed time
double T_tot = 0;

//Motor Command
//Motor computed duty cycle
int Duty_cycle; 
//Initial controller's value
double u =0;

//Observer initial states
double x_old_1 = 0.0;
double x_new_1 = 0.0;
double x_old_2 = 0;
double x_new_2 = 0;

//Timer event IDs, for stopping later
int take_reading_event_ID = -1;
int plot_event_ID = -1;


void setup() {
  //Encoder
  pinMode(encoderPinA, INPUT); 
  pinMode(encoderPinB, INPUT); 
  digitalWrite(encoderPinA, HIGH);  // turn on pullup resistor
  digitalWrite(encoderPinB, HIGH);  // turn on pullup resistor
  attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(1, doEncoderB, CHANGE);
  
  
  Serial.begin (19200);
  
 // Motor pins
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT); 
  
  // clock setup
  TCCR1A = _BV(COM1A1) | _BV(WGM21) | _BV(WGM20);
  TCCR1B = _BV(CS10);
  OCR1A = 0;// up to 1024 PIN 9
  
 
  
  // Perform takeReading every T_sample ms
  take_reading_event_ID = t.every(T_sample, takeReading);
  // Perform Plot every Tplot ms
  plot_event_ID = t.every(Tplot, Plot);
  // Perform doAfter after T_exp ms
  t.after(T_exp, doAfter);
}


void loop(){ 
  // Update the direction of motor based on the sign of Duty_cycle 
  if (Duty_cycle>0){
    digitalWrite(DIR_A,HIGH);
    OCR1A = Duty_cycle;
  }
  
  
  if (Duty_cycle<=0){
    digitalWrite(DIR_A,LOW);
    OCR1A = -Duty_cycle ;
  }
  
  // Update timer
  t.update();
}


// control input computation
void takeReading(){
   
  // Total Time update   
  T_tot = T_tot+T_sample/1000; 
 
  //Observer  
  x_new_1 =  (a11*x_old_1 + a12*x_old_2 + b1*u - L_1*(x_old_1 - encoderPos*K_encoder));
  x_new_2 =  (a21*x_old_1 + a22*x_old_2 + b2*u - L_2*(x_old_1 - encoderPos*K_encoder));
 
  // Reference square wave signal
  if(sin(2*M_PI*Freq*T_tot)>=0){
    ref = -VelocityAmplitude;
  }
  if (sin(2*M_PI*Freq*T_tot)<0){
    ref = +VelocityAmplitude;
  }
  
  // Controller
  u = Kp*(ref-x_new_2);

  // Saturation
  if (u>11.75){
    u=11.75;
  }else if (u<-11.75){
    u=-11.75;  
  }

  Duty_cycle = round(u/11.75* 1024); 

  if (Duty_cycle>512){
    Duty_cycle=512;  
  }else if (Duty_cycle<-512){
    Duty_cycle=-512;
  }
      
  // Update old states of the observer
  x_old_1 = x_new_1;
  x_old_2 = x_new_2;
}  


void doAfter(){
  // Turn off motor
  OCR1A = 0;
  Duty_cycle = 0;
  
  // Stop the other events
  t.stop(take_reading_event_ID);
  t.stop(plot_event_ID);
}


void Plot(){
  Serial.println(encoderPos);
  Serial.println(ref*1000);
}


// Interrupt on A changing state
void doEncoderA(){
  // Test transition
  A_set = digitalRead(encoderPinA) == HIGH;
  // and adjust counter + if A leads B
  encoderPos += (A_set != B_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoderB(){
  // Test transition
  B_set = digitalRead(encoderPinB) == HIGH;
  // and adjust counter + if B follows A
  encoderPos += (A_set == B_set) ? +1 : -1;
}

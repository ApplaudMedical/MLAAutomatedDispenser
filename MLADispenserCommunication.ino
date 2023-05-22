
#include <Stepper.h>  // Includes the stepper library, Arduino provided


unsigned long startTime;    // variable to store the start time
unsigned long endTime;      // variable to store the end time
unsigned long elapsedTime;  // variable to store the elapsed time

unsigned long startInso;  // variable to store the start time
unsigned long endInso;    // variable to store the end time
unsigned long InsoTime;   // variable to store the elapsed time


#define START "start"
#define STOP "stop"
#define RESET "reset"
#define tpoTrigger 12
#define pi 3.141592653589


// Pump Parameters to Change
float volumerate = 1.91581;    //// mL/min
float Tbolus = .25;      /// min
int num_iterations = 2;  // Number of cycles to do
float diameter = 26.0;   //// Syringe plunger diameter (mm)
int Insocycles = 500;    /// Insonation iterations to acheive 5 seconds of Insonation

// Syringe Area Calculation
float radius = diameter / 2.0;
float Area = pi * radius * radius;  //(mm^2)

// Conver from mL/min to mm^3/min
float Newrate = 1000 * volumerate;  // Flowrate now in mm^3/min

// Calculate the velocity input for the stepper.
float V = Newrate / Area;  // velocity in mm/min
int Vnew = round(V);

// change this to fit the number of steps per revolution, property of the stepper motor
const int stepsPerRevolution = 200;

// Calculate the steps travels to acheive the correct flow rate
float travelstep = V * Tbolus * stepsPerRevolution;
const int travelsteps = round(travelstep);  // round to nearest whole integer, stepper command takes integers

// Calculate the volume that will be dispensed
float Tvolume = Area * V * Tbolus * num_iterations / 1000;  //(mL)

// initialize the stepper library on pins 4 through 7:
Stepper myStepper(travelsteps, 4, 5, 6, 7);

// String to store incoming serial data
String inputString = "";

// Define the iteration value to acheive the desired num_iteration, but serial monitor output will start count from 1
int num_iterations1 = num_iterations + 1;

// Pin number of the LED
const int ledPin = 13;

//setting up the forward and backward pin
int forwardbutton = 3;
int backwardbutton = 2;
int forwardstate;
int backwardstate;


void setup() {
  pinMode(ledPin, OUTPUT);  // Set the LED pin to output mode
  pinMode(tpoTrigger, OUTPUT);
 
  pinMode(forwardbutton, INPUT);
  pinMode(backwardbutton, INPUT);
  digitalWrite(forwardbutton, LOW);
  digitalWrite(backwardbutton, LOW);

  // initialize the serial port:
  Serial.begin(9600);

  // The portion of code that sets up the serial monitor to begin program.
  Serial.print("You will be dispensing ");
  Serial.print(Tvolume);
  Serial.println(" mL");

  Serial.println(" The program will begin with a -start- command to the serial monitor");
}


void loop() {
  while (Serial.available() > 0) {
    // Read incoming serial data and append to inputString
    char incomingChar = Serial.read();
    inputString += incomingChar;
  }
  /////// This part is to move the pump F or B with the buttons ////////
  // Setting a variable to read the value of the button through a digital pin
  forwardstate = digitalRead(forwardbutton);
  backwardstate = digitalRead(backwardbutton);

  // Checking if the button is pressed
  if (forwardstate == HIGH) {
    Serial.println("MovingForward");
    myStepper.setSpeed(60);  // Set the speed of the stepper , RPM
    myStepper.step(10);      // Stepper motor on
  } else if (backwardstate == HIGH) {
    Serial.println("MovingBackwards");
    myStepper.setSpeed(60);  // Set the speed of the stepper , RPM
    myStepper.step(-10);     // Stepper motor on
  }

  ////////// Section for the Dispensing Routines /////////
  
  if (inputString.indexOf("start") != -1) {
    // Set the speed of the stepper , RPM
    myStepper.setSpeed(Vnew);
    for (int j = 1; j < num_iterations1; ++j) {

      // Infusing Scheme
      Serial.println("Infusing");
      digitalWrite(ledPin, HIGH);

      startTime = millis();        // store the start time
      myStepper.step(travelstep);  // Stepper motor on
      endTime = millis();          // store the end time

      elapsedTime = endTime - startTime;  // calculate the elapsed time


      // Prints bolus placement time to Serial Monitor
      Serial.print("Elapsed time: ");
      Serial.print(elapsedTime);
      Serial.println(" ms");

      Serial.println("Insonating");
      startInso = millis();
      // Sets 5 Seconds of insonation at 10 microsecond intervals
      for (int a = 0; a < Insocycles; ++a) {
        digitalWrite(tpoTrigger, HIGH);
        delay(10);
        digitalWrite(tpoTrigger, LOW);
      }

      endInso = millis();              // store the end time
      InsoTime = endInso - startInso;  // calculate the elapsed time

      // Prints insonation time to Serial Monitor
      Serial.print("Elapsed time: ");
      Serial.print(InsoTime);
      Serial.println(" ms");

      digitalWrite(ledPin, LOW);
      delay(50);  // Turn on the LED

      Serial.print("Iteration:");
      Serial.println(j);
    }
    inputString = "";  // Clear the input string

  } else if (inputString.indexOf("stop") != -1) {
    // step one revolution in the other direction:
    Serial.println("Program Ended, Reset Arduino but pushing button on board.");
    while (1) {
    }
    inputString = "";  // Clear the input string
  }

  else if (inputString.indexOf("stop") != -1) {
  }
}

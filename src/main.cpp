#include <Arduino.h>
#include <Servo.h>

Servo X_axis;   //declare the servos
Servo Y_axis_1;
Servo Y_axis_2;
int currentY = 0;  // used for startup
int currentX = 90;  
int Min_y_cam_fov = 0;  // this is the same fov camera llimits set in python code
int Max_y_cam_fov = 180; 

const int trigPin = 9; 
const int echoPin = 10; 

unsigned long LastSignalTime = 0 ;  //delay to prevcent lag within the arduino
const unsigned DebounceDelay = 300;




void attachAllServos() {
  X_axis.attach(4);
  Y_axis_1.attach(2);
  Y_axis_2.attach(3);    // def to set up pins on connection
 
}

void set_distnace_sensor(){
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);


}

void intialStartPosition(){ // def to write servo to start config
  X_axis.write(currentX);
  Y_axis_1.write(currentY);
  Y_axis_2.write(180 - currentY);
  
}


void X_axis_movement(int anglex){
 int step = (anglex > currentX) ? 1 : -1;
  for (int pos = currentX; pos != anglex; pos += step) {
    X_axis.write(pos);
    delay(5);                                     // rotate my x anlge to the set position smoothly 
  }
  currentX = anglex;

}

void Y_axis_movement(int angley){
 angley = constrain(angley, Min_y_cam_fov, Max_y_cam_fov);
 int step = (angley > currentY) ? 1 : -1;                  
  for (int pos = currentY; pos != angley; pos += step) {
    Y_axis_1.write(pos);
    Y_axis_2.write(Max_y_cam_fov + Min_y_cam_fov - pos);    // to understnad the math, max angle + min angle = 180 since the max rotation is 180deg, then by removing the currenr anlge to moes to the set limits within the range/FOV
    delay(5);                                                  // above happens on line 49 aswell
  }
  Y_axis_1.write(angley);                                  // write both y servos in opposition to each other smoothly 
  Y_axis_2.write(Max_y_cam_fov + Min_y_cam_fov - angley);
  currentY = angley;
}

int getDistacne(){
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

long duration = pulseIn(echoPin, HIGH);

int dist = duration/58;
return dist;


}



void setup() {
  Serial.begin(9600);
  attachAllServos();
  intialStartPosition();
  set_distnace_sensor();
}

void loop() {


 

  if (Serial.available() > 0) return;\
    int dist = getDistacne(); //find distncae from the snesor to the target
    Serial.print(dist );
    Serial.println("cm");
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    Serial.println("Received: " + data);

    if (dist > 15 && commaIndex > 0) {
        int servo_x_angle = data.substring(0, commaIndex).toInt();
        int servo_y_angle = data.substring(commaIndex + 1).toInt();

        if (servo_x_angle >= 0 && servo_x_angle <= 180 &&
            servo_y_angle >= 0 && servo_y_angle <= 180) {

        

          X_axis_movement(servo_x_angle);
          Y_axis_movement(servo_y_angle);
        }
    }
    else {
      Serial.println("Object error");
      intialStartPosition();

    }
  }


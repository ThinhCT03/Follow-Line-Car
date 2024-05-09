#include <Servo.h>          //Thư viện cho servo
#include <NewPing.h>        //Thư viện cho SRF

//our L298N control pins
const int LeftMotorForward = 7;
const int LeftMotorBackward = 6;
const int RightMotorForward = 5;
const int RightMotorBackward = 4;
const int motorLeftspeed = 3; //Kết nối chân ENA với chân 3 arduino
const int motorRightspeed = 8;//Kết nối chân ENB với chân 8 arduino

//line sensors connection
const int L_S = 9; //cb dò line trái
const int S_S = 2; //cb dò line giữa
const int R_S = 10; //cb dò line phải

int left_sensor_state; //biến lưu cảm biến hồng ngoại line trái
int s_sensor_state; //biến lưu cảm biến hồng ngoại line giữa
int right_sensor_state; //biến lưu cảm biến hồng ngoại line phải

//SRF pins
#define trig_pin A1 //analog input 1
#define echo_pin A2 //analog input 2

#define maximum_distance 200
boolean goesForward = false;
int distance = 100;

NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
Servo servo_motor; //đặt tên cho servo


void setup(){

  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(motorLeftspeed, OUTPUT);
  pinMode(motorRightspeed, OUTPUT);
 
  pinMode(L_S, INPUT);
  pinMode(R_S, INPUT);
  pinMode(S_S, INPUT);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  
  servo_motor.attach(10); //kết nối servo với chân D10

  servo_motor.write(115); //set up góc servo ban đầu hướng về trước, thông số này có thể thay đổi
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);

  Serial.begin(9600);
  analogWrite(motorLeftspeed, 110);
  analogWrite(motorRightspeed, 110);
  delay(2000);
}

void loop(){

  int distanceRight = 0;
  int distanceLeft = 0;
  delay(50);

  left_sensor_state = digitalRead(L_S);
  s_sensor_state = digitalRead(S_S);
  right_sensor_state = digitalRead(R_S);

  if ((digitalRead(L_S) == 0) && (digitalRead(S_S) == 1) && (digitalRead(R_S) == 0)) {
    moveForward(); // đi tiến
  }

  if ((digitalRead(L_S) == 1) && (digitalRead(S_S) == 1) && (digitalRead(R_S) == 0)) {
    turnLeft(); // rẻ trái
  }
  if ((digitalRead(L_S) == 1) && (digitalRead(S_S) == 0) && (digitalRead(R_S) == 0)) {
    turnLeft(); // rẻ trái
  }

  if ((digitalRead(L_S) == 0) && (digitalRead(S_S) == 1) && (digitalRead(R_S) == 1)) {
    turnRight(); // rẻ phải
  }
  if ((digitalRead(L_S) == 0) && (digitalRead(S_S) == 0) && (digitalRead(R_S) == 1)) {
    turnRight(); // rẻ phải
  }

  if ((digitalRead(L_S) == 1) && (digitalRead(S_S) == 1) && (digitalRead(R_S) == 1)) {
    moveStop(); // stop
  }

    //XỬ LÝ KHI XE GẶP VẬT CẢN
  if (distance <= 45){
    moveStop();
    delay(300);
    moveBackward();
    delay(400);
    moveStop();
    delay(300);
    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);
    //TRƯỜNG HỢP HAI BÊN KHÔNG CÓ VẬT CẢN
    if (distance < distanceRight && distance < distanceLeft) {
    digitalWrite(LeftMotorBackward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(LeftMotorForward, LOW);
    digitalWrite(RightMotorBackward, LOW);//cho xe xoay trái

    delay(400);
    moveStop();
    delay(200);
    moveForward();
    delay(600);
    moveStop();
    delay(200);

    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorBackward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorForward, LOW); //cho xe xoay phải

    delay(550);
    moveStop();
    delay(200);
    moveForward();
    delay(600);
    moveStop();
    delay(200);

    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorBackward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorForward, LOW); //cho xe xoay phải
    delay(500);
    moveStop();
    delay(200);
    moveForward();

    while (left_sensor_state == LOW) {
      left_sensor_state = digitalRead(L_S);
      s_sensor_state = digitalRead(S_S);
      right_sensor_state = digitalRead(R_S);
    }
    }

    //TRƯỜNG HỢP CÓ VẬT CẢN BÊN TRÁI
    if (distance >= distanceLeft){
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorBackward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorForward, LOW); //cho xe xoay sang phải

    delay(400);
    moveStop();
    delay(200);
    moveForward();
    delay(600);
    moveStop();
    delay(200);

    digitalWrite(LeftMotorBackward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(LeftMotorForward, LOW);
    digitalWrite(RightMotorBackward, LOW);//cho xe xoay trái

    delay(550);
    moveStop();
    delay(200);
    moveForward();
    delay(600);
    moveStop();
    delay(200);

    digitalWrite(LeftMotorBackward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(LeftMotorForward, LOW);
    digitalWrite(RightMotorBackward, LOW);//cho xe xoay trái một đoạn
    delay(500);
    moveStop();
    delay(200);
    moveForward();

    while (right_sensor_state == LOW) {
      left_sensor_state = digitalRead(L_S);
      s_sensor_state = digitalRead(S_S);
      right_sensor_state = digitalRead(R_S);
    }
    }



    //TRƯỜNG HỢP VẬT CẢN BÊN PHẢI
    if (distance >= distanceRight){
    digitalWrite(LeftMotorBackward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(LeftMotorForward, LOW);
    digitalWrite(RightMotorBackward, LOW);//cho xe xoay trái

    delay(400);
    moveStop();
    delay(200);
    moveForward();
    delay(600);
    moveStop();
    delay(200);

    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorBackward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorForward, LOW); //cho xe xoay phải

    delay(550);
    moveStop();
    delay(200);
    moveForward();
    delay(600);
    moveStop();
    delay(200);

    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorBackward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorForward, LOW); //cho xe xoay phải
    delay(500);
    moveStop();
    delay(200);
    moveForward();

    while (left_sensor_state == LOW) {
      left_sensor_state = digitalRead(L_S);
      s_sensor_state = digitalRead(S_S);
      right_sensor_state = digitalRead(R_S);
    }
    }

  }
    distance = readPing();
}

int lookRight(){  
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft(){ 
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
  delay(100);
}

int readPing(){
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0){
    cm=250;
  }
  return cm;
}

void moveStop(){
  
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
 
}

void moveForward(){

  if(!goesForward){

    goesForward=true;
    
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
  
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW); 

    
  }
}

void moveBackward(){

  goesForward=false;

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);

 
}


void turnRight(){
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  
  delay(250);
  
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
 
  
  
}

void turnLeft(){
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  delay(250);
  
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

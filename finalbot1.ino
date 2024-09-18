#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include <math.h>
#include <Servo.h>
#define PI 3.1415926535897932384626433832795

//locomotion
#define DIR1 30
#define DIR2 31
#define DIR3 32
#define DIR4 33

#define CONDIR1 34
#define CONSTEP1 35
#define CONDIR2 38
#define CONSTEP2 39

#define ppServopwm1 A2
#define ppServopwm2 A3

#define Relayopen  22
#define Relayclose 23

Servo ppServo1;
Servo ppServo2;
Servo ballpushservo;

#define PWM1 2
#define PWM2 3
#define PWM3 4
#define PWM4 5

// #define ballpushservopwm 8

//rollers bldc
//#define bldcpwm1 12
//#define bldcpwm2 11
#define ballpushservopwm 11

#define bldcpwm1 9
#define bldcpwm2 10


//#define flapper1pwm 6
#define flapper2pwm 7
#define flapper1pwm 12
#define led 13

Servo esc1; 
Servo esc2;

Servo flapper1;
Servo flapper2;

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

struct GamePadEventData {
    uint8_t X, Y, Z1, Z2, Rz;
};
#define NUM_BUTTONS 12
class JoystickEvents {
public:
    int x1, y1, x2, y2, rz;
    int hat;
    int buttonPressed; // Stores the button press event
    int buttonState[NUM_BUTTONS]; // Array to store button states

    JoystickEvents() {
        // Initialize button states to 0 (released)
        for(int i = 0; i < NUM_BUTTONS; i++) {
            buttonState[i] = 0;
        }
    }
    virtual void OnGamePadChanged(const GamePadEventData *evt);
    virtual void OnHatSwitch(uint8_t hat);
    virtual void OnButtonUp(uint8_t but_id);
    virtual void OnButtonDn(uint8_t but_id);
};

#define RPT_GEMEPAD_LEN  5

class JoystickReportParser : public HIDReportParser {
    JoystickEvents *joyEvents;

    uint8_t oldPad[RPT_GEMEPAD_LEN];
    uint8_t oldHat;
    uint16_t oldButtons;

public:
    JoystickReportParser(JoystickEvents *evt);

    virtual void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);
};

JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);


double headingRadians = 0;
double headingPower = 0;
int stepsToRotate = 126;
int concount = 0 ;

void setup() {
    Serial.begin(115200);
    
    pinMode(DIR1,OUTPUT);
    pinMode(DIR2,OUTPUT);
    pinMode(DIR3,OUTPUT);
    pinMode(DIR4,OUTPUT);
    pinMode(PWM1,OUTPUT);
    pinMode(PWM2,OUTPUT);
    pinMode(PWM3,OUTPUT);
    pinMode(PWM4,OUTPUT);

    pinMode(led,OUTPUT);

    pinMode(CONDIR1,OUTPUT);
    pinMode(CONSTEP1,OUTPUT);

    pinMode(CONDIR2,OUTPUT);
    pinMode(CONSTEP2,OUTPUT);


    pinMode(Relayclose,OUTPUT);
    pinMode(Relayopen,OUTPUT);

    analogWrite(PWM1,0);
    analogWrite(PWM2,0);
    analogWrite(PWM3,0);
    analogWrite(PWM4,0);

    ppServo1.attach(ppServopwm1);
    ppServo2.attach(ppServopwm2);
    ppServo1.write(180);
    ppServo2.write(0);

    flapper1.attach(flapper1pwm,1000,2000);
    flapper2.attach(flapper2pwm,1000,2000);
    flapper1.write(0);
    flapper2.write(180);

    ballpushservo.attach(ballpushservopwm);
    ballpushservo.write(180);

    esc1.attach(bldcpwm1); 
    esc1.writeMicroseconds(1500); 
    esc2.attach(bldcpwm2); 
    esc2.writeMicroseconds(1500);

    #if !defined(MIPSEL)
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
    #endif
    Serial.println("Start");

    if (Usb.Init() == -1)
        Serial.println("OSC did not start.");

    delay(200);

    if (!Hid.SetReportParser(0, &Joy))
        ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);
}
    String direction ;
    float m[4];
    float g;
    int servovalue = 0 , servomaxvalue = 90 , throttleval1=1500 , throttleval2=1500;
    int ballpushservovalue = 180;
void loop() {
    Usb.Task();
    if (JoyEvents.x1 == 0 && JoyEvents.y1 == 0 && JoyEvents.x2 == 0 && JoyEvents.y2 == 0) {
        digitalWrite(led,LOW);
        return;
    }
    if (Usb.getUsbTaskState() == USB_STATE_RUNNING) {
    digitalWrite(led,HIGH);
    Serial.println("USB device connected.");
    Serial.print("X1: ");
    Serial.print(128-JoyEvents.x1);
    Serial.print("\tY1: ");
    Serial.print(128-JoyEvents.y1);
    Serial.print("\tX2: ");
    Serial.print(JoyEvents.x2);
    Serial.print("\tY2: ");
    Serial.print(JoyEvents.y2);
    Serial.print("\tHat: ");
    Serial.print(JoyEvents.hat);
    Serial.print("\tButton Pressed: ");
    Serial.print(JoyEvents.buttonPressed);
    Serial.print("\tDirection : ");
    Serial.print("\t");
    Serial.print(direction);
    Serial.print("\tServo Value: ");
    Serial.print("\t");
    Serial.print(servovalue);
    Serial.print("\t 180 - Servo Value: ");
    Serial.print("\t");
    Serial.print(180-servovalue);
    Serial.print("\tThrottle val 1 : ");
    Serial.print("\t");
    Serial.print(throttleval1);
    Serial.print("\tThrottle val 2 : ");
    Serial.print("\t");
    Serial.print(throttleval2);
    Serial.print("\tBall Push Servo value : ");
    Serial.print("\t");
    Serial.print(ballpushservovalue);
    Serial.print("\tButton States: ");
    for(int i = 0; i < NUM_BUTTONS; i++) {
        Serial.print(JoyEvents.buttonState[i]);
        Serial.print(" ");
    }
    Serial.print("  ");

   if((abs(128-JoyEvents.x2)<40) && (abs(128-JoyEvents.y2)<40)){
        headingRadians = 0;
        headingPower = 0;
        m[0]=0;m[1]=0;m[2]=0;m[3]=0;
    }
    else{
      headingRadians = atan2((128-JoyEvents.x2),(128-JoyEvents.y2));
      if(headingRadians<0){
      headingRadians = -(headingRadians);
      }
      else{
      headingRadians = (2*PI-headingRadians);
      } 
      m[0]=(headingPower*cos(headingRadians-PI/4));
      m[1]=(headingPower*sin(headingRadians-PI/4));
      m[2]=-(headingPower*cos(headingRadians-PI/4));
      m[3]=-(headingPower*sin(headingRadians-PI/4));

    }
    headingPower = sqrt(((128-JoyEvents.x2)*(128-JoyEvents.x2))+((128-JoyEvents.y2)*(128-JoyEvents.y2)));
    Serial.print("\ttheta:");
    Serial.print(headingRadians);
    Serial.print("\tpower:");
    Serial.print(headingPower);
    Serial.print("\tBefore\tM0: ");
    Serial.print(m[0]);
    Serial.print("\tM1: ");
    Serial.print(m[1]);
    Serial.print("\tM2: ");
    Serial.print(m[2]);
    Serial.print("\tM3: ");
    Serial.print(m[3]);
    for(int i =0;i<=3;i++){
        if(m[i]>90){ m[i]=90;}
        if(m[i]<-90){ m[i]=-90;}
    }
    g=abs(m[0]);
    for(int i=1;i<4;i++){
      if(g<abs(m[i])){
        g=abs(m[i]);
      }
    }
    if(g>90){ g=90;}
    if(g<-90){ g=-90;}
    g = map(abs(g),0,90,0,255);
    Serial.print("\tg : ");
    Serial.print(g);
    Serial.print(" ");
    if(JoyEvents.buttonState[4]==1){
      rotate(-g);
      direction = "Rright ";
    }
    else if(JoyEvents.buttonState[5]==1){
        rotate(g);
        direction = "Rleft  ";
    }
    else if(JoyEvents.buttonState[11]==1){
        motormovement(0,0,0,0);
        direction = "stop  ";
    }
    else{
        motormovement(m[0],m[1],m[2],m[3]);
        direction = "FBRL  "; 
    }
    if(JoyEvents.hat==2){
      if(concount==0){
        digitalWrite(CONDIR1,LOW);
        digitalWrite(CONDIR2,HIGH);
        concount = 120;
      }
    }
    if(JoyEvents.hat==6){
      if(concount==0){
        digitalWrite(CONDIR1,HIGH);
        digitalWrite(CONDIR2,LOW);
        concount = 120;
      }
    }
    if(concount!=0){
      for(int i=0;i<=stepsToRotate;i++){
          digitalWrite(CONSTEP1, HIGH);
          digitalWrite(CONSTEP2, HIGH);
          delayMicroseconds(100); 
          digitalWrite(CONSTEP1, LOW);
          digitalWrite(CONSTEP2, LOW);
          delayMicroseconds(100); 
      }
      concount--;
    }
    /*if(JoyEvents.hat==0){
      for(int i=0;i<=90;i++){
        flapper1.write(i);
        flapper2.write(180-i);
  }
    }
    if(JoyEvents.hat==4){
      for(int i=90 ;i>=0;i--){
        flapper1.write(i);
        flapper2.write(180-i);
      }
    }*/
    if(JoyEvents.hat==0){
    if(servovalue<servomaxvalue){
        servovalue+=2;
      }
    }
    if(JoyEvents.hat==4){
    if(servovalue>0){
      servovalue-=2;
      }
    }
    flapper1.write(servovalue);
    flapper2.write(180-servovalue);
    delay(15);
    digitalWrite(Relayopen,HIGH);
    digitalWrite(Relayclose,HIGH);
    if(JoyEvents.buttonState[0]==1){
      digitalWrite(Relayopen,LOW);
      digitalWrite(Relayclose,HIGH);
    }
    if(JoyEvents.buttonState[2]==1){
      digitalWrite(Relayopen,HIGH);
      digitalWrite(Relayclose,LOW);
    } 
    if(JoyEvents.buttonState[1]==1){
      for(int i =0;i<=180;i++){
        ppServo1.write(i);
        ppServo2.write(180-i);
      }
    }
    if(JoyEvents.buttonState[3]==1){
      for(int i = 180;i>=0;i--){
        ppServo1.write(i);
        ppServo2.write(180-i);
      }
    }
    if(JoyEvents.buttonState[7]==1){
      if(throttleval1!=1400){
        throttleval1 -= 10 ;
        esc1.writeMicroseconds(throttleval1); 
        
      }
      if(throttleval2!=1600){
        throttleval2 += 10 ;
        esc2.writeMicroseconds(throttleval2);
        
      }
      if(throttleval1==1400 && throttleval2==1600){
      if(ballpushservovalue!=120){
      ballpushservovalue -=10;
      }
      }
    }
    else if(JoyEvents.buttonState[6]==1){
      if(throttleval1!=1600){
        throttleval1 += 10 ;
        esc1.writeMicroseconds(throttleval1); 
        Serial.println("bldc");
       
      }
      if(throttleval2!=1400){
        throttleval2 -= 10 ;
        esc2.writeMicroseconds(throttleval2); 
      }
     
    }
    else{
      if(throttleval1!=1500){
        if(throttleval1<1500){
          throttleval1+=10;
        }
        if(throttleval1>1500){
          throttleval1-=10;
          
        }
      }
      if(throttleval2!=1500){
        if(throttleval2<1500){
          throttleval2+=10;
        }
        if(throttleval2>1500){
          throttleval2-=10;
        }
      }
      esc1.writeMicroseconds(throttleval1); 
      esc2.writeMicroseconds(throttleval2); 
      if(ballpushservovalue != 180){
      ballpushservovalue += 10;
      }
    }
    ballpushservo.write(ballpushservovalue);
    Serial.println();
  }
  else{
    digitalWrite(led,LOW);
    Serial.println("No USB device connected.");
  }
}
JoystickReportParser::JoystickReportParser(JoystickEvents *evt) :
joyEvents(evt),
oldHat(0xDE),
oldButtons(0) {
    for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++)
        oldPad[i] = 0xD;
}

void JoystickReportParser::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
    bool match = true;

    // Checking if there are changes in report since the method was last called
    for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++)
        if (buf[i] != oldPad[i]) {
            match = false;
            break;
        }

    // Calling Game Pad event handler
    if (!match && joyEvents) {
        joyEvents->OnGamePadChanged((const GamePadEventData*)buf);

        for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++) oldPad[i] = buf[i];
    }

    uint8_t hat = (buf[5] & 0xF);

    // Calling Hat Switch event handler
    if (hat != oldHat && joyEvents) {
        joyEvents->OnHatSwitch(hat);
        oldHat = hat;
    }

    uint16_t buttons = (0x0000 | buf[6]);
    buttons <<= 4;
    buttons |= (buf[5] >> 4);
    uint16_t changes = (buttons ^ oldButtons);

    // Calling Button Event Handler for every button changed
    if (changes) {
        for (uint8_t i = 0; i < 0x0C; i++) {
            uint16_t mask = (0x0001 << i);

            if (((mask & changes) > 0) && joyEvents) {
                if ((buttons & mask) > 0) {
                    joyEvents->OnButtonDn(i + 1);
                    joyEvents->buttonPressed = i + 1; // Set button pressed
                } else {
                    joyEvents->OnButtonUp(i + 1);
                }
            }
        }
        oldButtons = buttons;
    }
}

void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt) {
    // Store the values in variables
    x1 = evt->Y;
    y1 = evt->Z1;
    x2 = evt->Z2;
    y2 = evt->Rz;
}

void JoystickEvents::OnHatSwitch(uint8_t hat) {
    this->hat = hat; // Store hat value
}

void JoystickEvents::OnButtonUp(uint8_t but_id) {
    buttonState[but_id - 1] = 0;
    JoyEvents.buttonPressed = 0;
}

void JoystickEvents::OnButtonDn(uint8_t but_id) {
     buttonState[but_id - 1] = 1;
     JoyEvents.buttonPressed = but_id;
}

void motormovement(float m0,float m1,float m2,float m3){
  int p0 = map(abs(m[0]),0,90,0,255);
  int p1 = map(abs(m[1]),0,90,0,255);
  int p2 = map(abs(m[2]),0,90,0,255);
  int p3 = map(abs(m[3]),0,90,0,255);
  (m[0]<0)? digitalWrite(DIR1,LOW):digitalWrite(DIR1,HIGH);
  (m[1]<0)? digitalWrite(DIR2,LOW):digitalWrite(DIR2,HIGH);
  (m[2]<0)? digitalWrite(DIR3,LOW):digitalWrite(DIR3,HIGH);
  (m[3]<0)? digitalWrite(DIR4,LOW):digitalWrite(DIR4,HIGH);
  analogWrite(PWM1,p0);
  analogWrite(PWM2,p1);
  analogWrite(PWM3,p2);
  analogWrite(PWM4,p3);
  Serial.print("\tp0:");
  Serial.print(p0);
  Serial.print("\tp1:");
  Serial.print(p1);
  Serial.print("\tp2:");
  Serial.print(p2);
  Serial.print("\tp3:");
  Serial.print(p3);
  Serial.print("    ");
}
void rotate(float g1){
  if(g1<0){
     digitalWrite(DIR1,LOW);
     digitalWrite(DIR2,LOW);
     digitalWrite(DIR3,LOW);
     digitalWrite(DIR4,LOW);
  }
  else{
    digitalWrite(DIR1,HIGH);
    digitalWrite(DIR2,HIGH);
    digitalWrite(DIR3,HIGH);
    digitalWrite(DIR4,HIGH);
  }
  analogWrite(PWM1,abs(g1));
  analogWrite(PWM2,abs(g1));
  analogWrite(PWM3,abs(g1));
  analogWrite(PWM4,abs(g1));
}
//this is the main code for arduino for this system
//finite state machine with 3 states, use serial monitor to swtich between states


#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <Servo.h>

#include <LCDWIKI_GUI.h> //Core graphics library
#include <LCDWIKI_SPI.h> //Hardware-specific library
#include <ft6336g.h> //touch library

/////state machine
enum class state {HOME,BURNOUT,CASTING};
state liveState = state::HOME;


///////////////////THERMOCOUPLE
#define MAXDO   40
#define MAXCS   43
#define MAXCLK  42
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);
float temp;
/////////////////////THERMOCOUPLE


///////////////////RELAY
int START = 22; //1 relay
int STOP = 23; //2
int PL = 25; //3
int B2 = 27; //4
int B3 = 37; //5
//////////////////RELAY


/////////////////SERVO
Servo s1;
////////////////SERVO


/////////////////LCD PANEL
//display
#define MODEL ST7796S
#define CS   10    
#define CD   9
#define RST  8
#define LED  5

LCDWIKI_SPI mylcd(MODEL,CS,CD,MISO,MOSI,RST,SCK,LED);

#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define NAVY    0x000F  /*   0,   0, 128 */
#define PINK    0xF81F

//touch screen
#define OTT_MAX_TOUCH  2 
#define INT  7
#define CRST 6 
#define SCL  A5
#define SDA  A4

FT6336 mytp(INT,CRST,SCL,SDA);

/////////////////LCD PANEL


class LCD {//class to control LCD panel (display and touch)
  public:
    unsigned long idk;  // Public, can be accessed and modified directly

    void test(){
      mylcd.Set_Text_Mode(0);
      //display 1 times string
      mylcd.Fill_Screen(0x0000);
      mylcd.Set_Text_colour(RED);
      mylcd.Set_Text_Back_colour(BLACK);
      mylcd.Set_Text_Size(1);
      mylcd.Print_String("Hello World!", 0, 0);
      mylcd.Print_Number_Float(01234.56789, 2, 0, 8, '.', 0, ' ');  
      mylcd.Print_Number_Int(0xDEADBEF, 0, 16, 0, ' ',16);
      //mylcd.Print_String("DEADBEF", 0, 16);

      //display 6 times string
      mylcd.Set_Text_colour(YELLOW);
      mylcd.Set_Text_Size(6);
      mylcd.Print_String("Hello!", 0, 266);

      

    }

    void home(){
      mylcd.Fill_Screen(BLACK);

      mylcd.Set_Text_Mode(0);
      //display 1 times string
      mylcd.Set_Rotation(90);
      mylcd.Set_Text_colour(CYAN);
      mylcd.Set_Text_Back_colour(BLACK);
      mylcd.Set_Text_Size(6);
      mylcd.Print_String("KASA v2.5", 0, 0);

      mylcd.Set_Text_colour(GREEN);
      mylcd.Set_Text_Back_colour(BLACK);
      mylcd.Set_Text_Size(5);
      mylcd.Print_String("WELCOME", 0, 100);
      mylcd.Print_String("TO", 0, 150);
      mylcd.Print_String("OUR", 0, 200);
      mylcd.Print_String("BOOTH!", 0, 250);

    }

    void display_temp(float temp_) {
      mylcd.Set_Text_colour(YELLOW);
      mylcd.Set_Text_Size(5);
      mylcd.Set_Text_Back_colour(BLACK);
      mylcd.Print_Number_Float(temp_, 2, 90, 400, '.', 0, ' ');

      mylcd.Set_Text_colour(WHITE);
      mylcd.Set_Text_Size(3);
      mylcd.Print_String("Temperature:",60,360);
      //delay(100);
    }


    void graphs() {
      mylcd.Set_Draw_color(WHITE);
      mylcd.Draw_Rectangle(40,150,mylcd.Get_Display_Width()-40,250);
      //mylcd.Draw_Line();

    }

    void burnout(){
      mylcd.Fill_Screen(0x0000);
      //KASA Logo
      mylcd.Set_Text_colour(CYAN);
      mylcd.Set_Text_Back_colour(BLACK);
      mylcd.Set_Text_Size(6);
      mylcd.Print_String("BURNOUT", 0, 0);

      mylcd.Set_Draw_color(WHITE);
      //mylcd.Draw_Rectangle(0,0,80,200);
      mylcd.Fill_Rectangle(20,80,mylcd.Get_Display_Width()-20,180);
      mylcd.Fill_Rectangle(20,220,mylcd.Get_Display_Width()-20,320);

      mylcd.Set_Text_colour(WHITE);
      mylcd.Set_Text_Size(2);
      mylcd.Print_String("Power Level vs Time",50,60);
      mylcd.Print_String("Temperature vs Time",50,200);


      mylcd.Set_Draw_color(BLACK);
      mylcd.Fill_Rectangle(23,80,mylcd.Get_Display_Width()-20,177);
      mylcd.Fill_Rectangle(23,220,mylcd.Get_Display_Width()-20,317);

      mylcd.Set_Draw_color(PINK);
      mylcd.Fill_Rectangle(23,170,75,177);
      mylcd.Fill_Rectangle(75,150,125,177);
      mylcd.Fill_Rectangle(125,120,175,177);
      mylcd.Fill_Rectangle(175,150,225,177);
      mylcd.Fill_Rectangle(225,160,300,177);


      mylcd.Set_Draw_color(YELLOW);
      int x_width = 50;
      
      mylcd.Draw_Line(25,315,75,305);
      mylcd.Draw_Line(75,305,125,285);
      mylcd.Draw_Line(125,285,175,255);
      mylcd.Draw_Line(175,255,225,235);
      mylcd.Draw_Line(225,235,275,230);
      mylcd.Draw_Line(275,230,300,230);

      mylcd.Draw_Line(25,314,75,304);
      mylcd.Draw_Line(75,304,125,284);
      mylcd.Draw_Line(125,284,175,254);
      mylcd.Draw_Line(175,254,225,234);
      mylcd.Draw_Line(225,234,275,229);
      mylcd.Draw_Line(275,229,300,229);
    }

    void casting(){
      mylcd.Fill_Screen(0x0000);
      //KASA Logo
      mylcd.Set_Text_colour(CYAN);
      mylcd.Set_Text_Back_colour(BLACK);
      mylcd.Set_Text_Size(6);
      mylcd.Print_String("CASTING", 0, 0);
      mylcd.Set_Draw_color(WHITE);
      //mylcd.Draw_Rectangle(0,0,80,200);
      mylcd.Fill_Rectangle(20,80,mylcd.Get_Display_Width()-20,180);
      mylcd.Fill_Rectangle(20,220,mylcd.Get_Display_Width()-20,320);

      mylcd.Set_Text_colour(WHITE);
      mylcd.Set_Text_Size(2);
      mylcd.Print_String("Power Level vs Time",50,60);
      mylcd.Print_String("Temperature vs Time",50,200);


      mylcd.Set_Draw_color(BLACK);
      mylcd.Fill_Rectangle(23,80,mylcd.Get_Display_Width()-20,177);
      mylcd.Fill_Rectangle(23,220,mylcd.Get_Display_Width()-20,317);

      mylcd.Set_Draw_color(PINK);
      mylcd.Fill_Rectangle(23,120,300,177);


      mylcd.Set_Draw_color(YELLOW);
      int x_width = 50;
      mylcd.Draw_Line(25,315,75,275);
      mylcd.Draw_Line(75,275,125,255);
      mylcd.Draw_Line(125,255,175,245);
      mylcd.Draw_Line(175,245,225,235);
      mylcd.Draw_Line(225,235,275,225);
      mylcd.Draw_Line(275,225,300,223);

      mylcd.Draw_Line(25,314,75,274);
      mylcd.Draw_Line(75,274,125,254);
      mylcd.Draw_Line(125,254,175,244);
      mylcd.Draw_Line(175,244,225,234);
      mylcd.Draw_Line(225,234,275,224);
      mylcd.Draw_Line(275,224,300,222);

      

    }
};


class Microwave {// class to control microwave (using control input too)
  public:

    void off() {
      button(STOP);
      Serial.println("System turned off");
      //millis function
    }

    void on() {
      Serial.println("System turned on");
    }

    void onroutine(int power, int time) {
      button(STOP);

      powerlevel(power);

      set_time(time);
    }


    void powerlevel(int power){
      for(int i = 10; i >= power; i--)
      {
        digitalWrite(PL,HIGH);
        delay(50);
        digitalWrite(PL,LOW);
        delay(50);
      }
      digitalWrite(PL,HIGH);
      return;

    }

    void set_time(int seconds) {
      //some algo to get time in seconds
      int turn = seconds/4;
      //Serial.println(turn);
      if (seconds < 100){
        s1.write(135);
        delay(1500);
        //delay(turn*1000*2); //servo calibration line
        s1.write(90);
      }
      else{
        s1.write(175);
        delay(3000);
        //delay(turn*1000*2); //servo calibration line
        s1.write(90);
      }
      

    }

    void button(int but){//pressed button
      digitalWrite(but,LOW);
      delay(100);
      digitalWrite(but,HIGH);
    }
};

LCD lcd;

Microwave microwave;



void setup() {
  /////////////////////////////////////////////////////THERMOCOUPLE
  Serial.begin(9600);
  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc
  Serial.println("MAX31855 test");
  delay(500);
  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }
  //////////////////////////////////////////////////////THERMOCOUPLE
  

  /////////////////////////////////SERVO
  s1.attach(44);
  s1.write(90);//90+ is clockwise
  /////////////////////////////////SERVO

  /////////////////////////////////////////////////////RELAY
  pinMode(START,OUTPUT);
  pinMode(STOP,OUTPUT);
  pinMode(PL,OUTPUT);
  pinMode(B2,OUTPUT);


  digitalWrite(START,HIGH);
  digitalWrite(STOP,HIGH);
  digitalWrite(PL,HIGH);
  digitalWrite(B2,HIGH);
  delay(100);
  digitalWrite(STOP,LOW);
  delay(100);
  digitalWrite(STOP,HIGH);
  ////////////////////////////////////////////////////RELAY
  
  ///////////////////////////////////LCD PANEL
  mylcd.Init_LCD();
  mylcd.Fill_Screen(BLACK);
  mylcd.Set_Rotation(90);

  /*
  mylcd.Set_Text_Mode(0);
  //display 1 times string
  
  mylcd.Set_Text_colour(CYAN);
  mylcd.Set_Text_Back_colour(BLACK);
  mylcd.Set_Text_Size(6);
  mylcd.Print_String("KASA v2.5", 0, 0);
  */
  ///////////////////////////////////LCD PANEL

  //lcd.casting();



  Serial.println("SYSTEM READY FOR OPERATION");

}

bool newState = true;
const unsigned long control_seconds = 10000;// x/1000 for seconds
float previousMillis = 0.0;
int livePower = 0;

////////BURNOUT AND CAST TEMPERATURES
const int mold_t_burnout = 350;
const int cast_t_burnout = 800;

//need sigmoid math in here too....is this worth it....i dont think so, can show graphs and python script

//loop acts as a state machine
void loop() {

  //uint32_t = t;
  temp = readTemp();
  lcd.display_temp(temp);
  
  switch(liveState) {
      case state::HOME:

        if (newState){
          microwave.off();
          lcd.home();
          newState = false;
        }

        
        if (Serial.available() > 0){
          char a = Serial.read();
          if (a == 'b'){
            liveState = state::BURNOUT;
            newState = true;
            Serial.println("Switching to BURNOUT");
          }
          if (a== 'k'){
            liveState = state::CASTING;
            newState = true;
            Serial.println("Switching to CASTING");
          }
        }

      break;

      case state::CASTING:
        //lcd.display_temp(temp);
        //Serial.println("entered cast state");
        if (newState){
          lcd.casting();
          microwave.onroutine(10,600);
          newState = false;
        }
      


        if (Serial.available() > 0){
          char a = Serial.read();
          if (a=='h'){
            liveState = state::HOME;
            newState = true;
            Serial.println("Switching to HOME");
          }
          if (a=='b'){
            liveState = state::BURNOUT;
            newState = true;
            Serial.println("Switching to BURNOUT");
          }
        }
      break;

      case state::BURNOUT: 
        //lcd.display_temp(temp);
        if (newState){
          lcd.burnout();
          microwave.onroutine(1,30);
          newState = false;
          previousMillis = millis();
        }
        
        float mold_temp = air_to_mold(temp);
        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis >= control_seconds + 500) {
          previousMillis = currentMillis;
          livePower = PID(mold_temp);//this will run the PID controller for burnout (assuming the same constants for burnout and casting)
          livePower = 5;//example
          microwave.onroutine(livePower,control_seconds/10000);
        }


        if (Serial.available() > 0){
          char a = Serial.read();
          if (a=='h'){
            liveState = state::HOME;
            newState = true;
            Serial.println("Switching to HOME");
          }
          if (a=='k'){
            liveState = state::CASTING;
            newState = true;
            Serial.println("Switching to CASTING");
          }
        }
        
      break;

      
      
      
  }

}

float readTemp(){
   //Serial.println(thermocouple.readInternal());
   double c = thermocouple.readCelsius();
   if (isnan(c)) {
     Serial.println("Thermocouple fault(s) detected!");
     uint8_t e = thermocouple.readError();
     if (e & MAX31855_FAULT_OPEN) Serial.println("FAULT: Thermocouple is open - no connections.");
     if (e & MAX31855_FAULT_SHORT_GND) Serial.println("FAULT: Thermocouple is short-circuited to GND.");
     if (e & MAX31855_FAULT_SHORT_VCC) Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
   } else {
     return c;
   }
}

float air_to_mold(float temp_){
  //GOVERNING EQUATIONS FOR CALCULATING MOLD TEMP FROM MEASURED AIR TEMP
  return temp_*3;
}


///////////////////PID Controller

// PID constants
float Kp = 1.0;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 0.01; // Derivative gain

// PID variables
float setpoint = mold_t_burnout;  // Desired value
float input = 0.0;     // Current value
float output = 0.0;    // Control output

float prevError = 0.0;
float integral = 0.0;

// Time tracking
unsigned long lastTime = 0;

float PID(float input) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0; // Convert to seconds
    lastTime = now;

    float error = setpoint - input;
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    return output;
}


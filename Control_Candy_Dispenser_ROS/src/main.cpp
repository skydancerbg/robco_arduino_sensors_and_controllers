#include <ESP8266WiFi.h>
#include <Ticker.h>  //Ticker Library
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

// #     #################################
// #     # 
// #     # Created by Stefan 
// #     # SRG - Service Robotics Group Bulgaria
// #     # Version 1.0 from Apr. 7th, 2019.
// #     #
// #     #################################

// ############################################################################################
//
// Based on the work of Agustin Nunez on EspRos
// https://github.com/agnunez/espros
//and https://circuits4you.com/2018/01/02/esp8266-timer-ticker-example/
// ############################################################################################

// Init constants and global variables
#define DEBUG 0
#define LED_BUILTIN 2 //Specific for nodeMCU v3 LoLin board
    // !! The LED_BUILTIN predefined pin is beeing redefined here (beacuse it is on a different pin on the nodemcu), 
    // !! You are going to get a warning while compailing. Discard that warning...
    //
    // There is only one LED on nodemcu LoLin
    // https://arduino.stackexchange.com/questions/38477/does-the-node-mcu-v3-lolin-not-have-a-builtin-led
    // the ESP8266 has a builtin led that is attached to D4 as labeled on LoLin boards which maps to GPIO2.
    // It's confusing with esp8266 modules because, pin numbers on board don't map to GPIO - Pin D4 is actually GPIO2. 
    // !! One thing to note is that the led is active low. In other words ... setting PIN 2 to '0' will turn 
    // !! the LED ON and setting PIN 2 to '1'- HIGH will turn the  LED OFF
    // Lolin Builtin_Led Picture: https://i.pinimg.com/originals/65/d4/f9/65d4f900af42813b46f10d1636b0492d.jpg
    // This is the only LED on the LoLin boards and differs from other devkits that have an LED on GPIO16.

#define PIN_D1_PWM_A 5  // gpio5 = D1  PWM_A
// #define PIN_D2_PWM_B 4  // gpio4 = D2  PWM_B
#define PIN_D3_DA 0  // gpio0 = D3  DA (A- A+) Output to Motor A
// #define PIN_D4_DB 2  // gpio2 = D4  DB (B- B+) //Conflict - same as the the BUILTIN LED pin !!!!!!!!!!!!!    

// WiFi Definitions //
//////////////////////
// WiFi configuration. Replace '***' with your data
const char* ssid = "code";
const char* password = "codeass123";
IPAddress server(192,168,1,2);      // Set the rosserial socket server IP address
const uint16_t serverPort = 11411;    // Set the rosserial socket server port

ros::NodeHandle nh;

char motor_on_str[9] = "motor_on";
char motor_off_str[10] = "motor_off";
char new_duriation_set_str[35] = "New motor run duriation is set";
// char initial_message_str[14] = "initial_state";

// bool publishFeedbackNow = false;
// bool motorTurnedON = false;
// bool attachTikerOnlyOnce = true;
static uint32 motorRunDuriation = 1000;  //in milliseconds - Here we set the default duriation to 1 second      
void motorStopByTheTimer();
void publishFeedback();
void changeDuriationCallback(const std_msgs::Int16& msg);
void startMotorCallback(const std_msgs::Bool& msg);
void ledON();
void ledOFF();

Ticker runMotorTimer(motorStopByTheTimer, 1000, MILLIS);
// unsigned long previousMillis = 0;
// unsigned long previousSpinOnceMillis = 0; 

// To be used with the publisher
std_msgs::String str_msg;
std_msgs::Bool bool_msg;




ros::Publisher pub_feedback("/robco/candy_dispenser/feedback", &str_msg);


// For ease of use on the nodemcu side, we will be sending an Bool to /robko/candy_dispenser/start_motor : true - turn motor On
// The motor will be dtopped by the timer, after motorRunDuriation is passed - no need to send stop command (flase) in order to stop it!
ros::Subscriber<std_msgs::Bool> sub_start_motor("/robco/candy_dispenser/start_motor", &startMotorCallback);
ros::Subscriber<std_msgs::Int16> sub_change_duriation("/robco/candy_dispenser/motor_run_duriation", &changeDuriationCallback);

// void commandCallback(const std_msgs::Int16& msg) {
  
//     led=abs(msg.data);
//     di

void setupWiFi(){

    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void setup()
{
  // LED_BUILTIN defined to be 2, look at the comment about LoLin LED pin in the define section above
  pinMode(LED_BUILTIN, OUTPUT);
  // Motor A Driver pins setup
    pinMode(PIN_D1_PWM_A, OUTPUT); // инициализируем Pin как выход
    pinMode(PIN_D3_DA, OUTPUT); // инициализируем Pin как выход


  ledOFF();  // i.e. digitalWrite(LED_BUILTIN, HIGH);   // LED OFF - HIGH (LED ON - LOW)

  // pinMode(2, OUTPUT);
  // pinMode(12, OUTPUT);
  // pinMode(13, OUTPUT);
  // pinMode(15, OUTPUT);


  setupWiFi();
  delay(2000);
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.advertise(pub_feedback);
  nh.subscribe(sub_start_motor);
  nh.subscribe(sub_change_duriation);

  while (!nh.connected())
    {
        nh.spinOnce();
    }

  // IN YOUR CODE YOU CAN USE nh.loginfo("YOUR_TEXT");
  // FOR PRINTING DIAGNOSTICS AND OTHER MESSAGES TO THE TERMINAL
  // PRINT "IS CONNECTED" MESSAGE TO THE TERMINAL ON THE ROS SIDE
  nh.loginfo("CANDY DISPENSER CONTROLLER CONNECTED");

  // IMPORTANT: HERE, IN THE SETUP, IS THE ONLY PLACE IN YOUR CODE YOU ARE ALLOWED TO USE DELAY!!!
  // WE HAVE TO CALL nh.spinOnce() IN OUR LOOP, AS OFTEN, AS POSSIBLE FOR ROSSERIAL TO PROCESS THE INCOMMING MESSAGES!
  // DELAY IS A BLOCKING FUNCTION AND WILL PREVENT nh.spinOnce() FROM BEEING CALLED IN THE DESIRED RATE!
  // USE THE "BLINK WITHOUT DELAY" EXAMPLE CONCEPT OR TIMERS INSTEAD !!!!
  // WITH THIS DELAY WE GIVE SOME TIME FOR MAKING THE ROSSERIAL CONNECTION
  delay(1);
    ///////////////////////////////////////////////////
}

//======================================
//        LOOP
//======================================

void loop()
{
  runMotorTimer.update();
  nh.spinOnce();
}  
//==============END LOOP================

//======================================
void startMotorCallback(const std_msgs::Bool& msg)
{  
    nh.loginfo("Entered startMotorCallback");

    if(msg.data==true){
      runMotorTimer.start();
      digitalWrite(PIN_D1_PWM_A, HIGH);   // PWM_B HIGH - ON 100%
      digitalWrite(PIN_D3_DA, LOW);   // DB HIGH - Direction of rotation  (B- B+)     
      ledON();
      str_msg.data = motor_on_str; //set the feedback message content 
      publishFeedback();
      nh.loginfo("Motor ON");
    } 
}

//======================================
void motorStopByTheTimer()
{
      nh.loginfo("Entered motorStopByTheTimer");
      digitalWrite(PIN_D1_PWM_A, LOW);   // PWM_B HIGH, изменяется направление вращения двигателя на контактах (B- B+)
      str_msg.data = motor_off_str;
      // motorTurnedON = false;
      digitalWrite(LED_BUILTIN, HIGH);   // LED ON - LOW
      nh.loginfo("Motor turned OFF");
      publishFeedback();
      runMotorTimer.stop();

}

//======================================
void changeDuriationCallback(const std_msgs::Int16& msg) 
{
    // Convert from float seconds with fraction to int miliseconds
    // motorRunDuriation = (int)msg.data *1000 + (msg.data - (int)msg.data)*100;
    motorRunDuriation = (int)msg.data;
    runMotorTimer.interval(motorRunDuriation);
    str_msg.data = new_duriation_set_str;
    nh.loginfo("Motor run duriation changed");
}

//======================================
void publishFeedback(){
      pub_feedback.publish( &str_msg );
}

//======================================
void ledON()
{
  digitalWrite(LED_BUILTIN, LOW);   // LED OFF - HIGH / LED ON - LOW
}

//======================================
void ledOFF()
{      
  digitalWrite(LED_BUILTIN, HIGH);   // LED OFF - HIGH / LED ON - LOW
}
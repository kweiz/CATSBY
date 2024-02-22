//Connections
// Arduino:
//                                Serial MP3 Player Module (OPEN-SMART)
// D8 --------------------------- TX
// D7 --------------------------- RX
// D12 = Touch Sensor
// D9 = Servo Motor
// D4 = Vibration Motor
// D5 = Microphone
// D2 = LED

// Include required libraries:
#include <SoftwareSerial.h>  // import library for the MP3 speaker
#include <Adafruit_TiCoServo.h> // include the TiCoServo library
#include <Adafruit_NeoPixel.h> //include the Adafruit Neopixel library

//constants won't change. They're used here to set pin numbers:
const int SENSOR_PIN = 12;  // the Arduino's input pin that connects to the sensor's SIGNAL pin
const int SOUND_PIN = 5;    // Sound detection with microphone
const int VIBR_PIN = 4;     // the Arduino's output pin that connects to the vibration motor's SIGNAL pin
const int LEDPIN = 2;       // the arduino's output pin that connects to the LED's SIGNAL Pin

#define N_LEDS 15     //number of LEDs in the strip
#define SERVO_PIN 9 // the servo pin number
#define SERVO_MIN 1000 // 1 ms pulse
#define SERVO_MAX 1500 // 2 ms pulse

// Variables will change
int angle = 45;         // the default angle of servo motor (head straight)
int currentTouchState;  // the current state of touch sensor
boolean val = 0;

//when setting up Neopixel library
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, LEDPIN, NEO_GRB + NEO_KHZ800);
Adafruit_TiCoServo myservo;

// Define the RX and TX pins to establish UART communication with the MP3 Player Module.
#define MP3_RX 8  // to TX
#define MP3_TX 7  // to RX

// Define Cases in the Act function (see below row 80)
#define STOP 0
#define PURR 1
#define MEOW 2

// Select storage device to TF card
static int8_t select_SD_card[] = { 0x7e, 0x03, 0X35, 0x01, 0xef };  // 7E 03 35 01 EF
// Play with index: /00/001xxx.mp3
static int8_t play_first_song[] = { 0x7e, 0x04, 0x41, 0x00, 0x01, 0xef };  // 7E 04 41 00 01 EF
// Play with index: /00/002xxx.mp3
static int8_t play_second_song[] = { 0x7e, 0x04, 0x41, 0x00, 0x02, 0xef };  // 7E 04 41 00 02 EF
// Play the song.
static int8_t play[] = { 0x7e, 0x02, 0x01, 0xef };  // 7E 02 01 EF
// Pause the song.
static int8_t pause[] = { 0x7e, 0x02, 0x02, 0xef };  // 7E 02 02 EF
//volume down
static int8_t volume_down[] = { 0x7e, 0x02, 0x06, 0xef };  //7E 02 06 EF

// Define the Serial MP3 Player Module.
SoftwareSerial MP3(MP3_RX, MP3_TX);

void setup() {
 // Initiate the serial monitor
  Serial.begin(9600);
  // set arduino pin to input mode
  pinMode(SENSOR_PIN, INPUT);
  // set arduino pin to input mode
  pinMode(SOUND_PIN, INPUT);
  // initialize digital pin MP3 as an output
  pinMode(MP3, OUTPUT);
  // Initiate the Serial MP3 Player Module.
  MP3.begin(9600);
  //initiate the vibration motor as output
  pinMode(VIBR_PIN, OUTPUT);
  //initiate the LED as output
  pinMode(LEDPIN, OUTPUT);
  // attaches the servo on pin 9 to the servo object.
  myservo.attach(SERVO_PIN);
  // Select the SD Card.
  send_command_to_MP3_player(select_SD_card, 5);
  //initialises the neopixel library
  strip.begin ();
}

//functuon to read SD card
void send_command_to_MP3_player(int8_t command[], int len) {
  Serial.print("\nMP3 Command => ");
  for (int i = 0; i < len; i++) {
    MP3.write(command[i]);
    Serial.print(command[i], HEX);
  }
  Serial.println();
}

//colour for default
void WarmYellow() {
  strip.fill(strip.Color(255, 150, 0)); //RGB code for the warmyellow colour
  strip.setBrightness (10); //try to keep low brightness to not draw too much current 
  strip.show();
}
//colour for meowing
void colourVibrant() {
  strip.fill(strip.Color(252, 3, 244)); //RGB code for the vibrant (purple) colour
  strip.setBrightness (10);
  strip.show(); // This sends the updated pixel color to the hardware.
}

//colour for purring
void colourBlue() {
  strip.fill(strip.Color(3, 127, 252)); //RGB code for the blue colour
  strip.setBrightness (10);
  strip.show();
}

void act(int activity1) {
  switch (activity1) {
    case 0:  // do nothing the code to stop everything
      WarmYellow(); //Turn on the warmyellow/default colour on LED
      break;
    case 1:  //vibrate to the purrsound (Vibration Motor + MP3 Player)
      colourBlue(); //Turn on the blue colour on LED
      send_command_to_MP3_player(play_second_song, 6);  //play the second sound file on SD card
      digitalWrite(VIBR_PIN, HIGH); //Vibrate to the sequence of the purring audio file
      delay(970);
      digitalWrite(VIBR_PIN, LOW);
      delay(1079);
      digitalWrite(VIBR_PIN, HIGH);
      delay(1011);
      digitalWrite(VIBR_PIN, LOW);
      break;
    case 2:   //play meow sound (Servo Motor + MP3 player)
      colourVibrant();                                 
      send_command_to_MP3_player(play_first_song, 6);  //play the first sound file on the SD card "meow"
      for (angle = 45; angle >= 0; angle -= 1) {       //goes from 40 degrees (start point) to 0 degrees in steps of 1 degree
        Serial.println(angle);
        myservo.write(angle);  // tell servo to go to position in variable 'myservo'
        delay(15);             // waits 15 ms for the servo to reach its position
      }
      for (angle = 0; angle <= 45; angle += 1) {  // goes back from 0 degrees to 40 degrees (start point)
        myservo.write(angle);
        delay(15);
      }
      break;
  }
}

void loop() {

if (currentTouchState = digitalRead(SENSOR_PIN)) {    //if the sensor is touched, play "PURR" 
    Serial.println("The sensor is touched");
    act(PURR);
  }
  val = digitalRead(SOUND_PIN); // if the microphone detects sounds turn its head 
  if (val == HIGH) {
    Serial.println("Turn head");
    act(MEOW);
  }
  val = digitalRead(SOUND_PIN); //if no sound detected the head should be still
  if (val == LOW) {
    Serial.println("Still head");
    act(STOP);
  }
}

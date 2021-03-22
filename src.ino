
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define I2C_ADDR    0x27

#define BACKLIGHT_PIN  3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

#define  LED_OFF  1
#define  LED_ON  0

#define ECHOPIN 2        // Pin de receptor

#define TRIGPIN 3        // Pin emitator

const int sensor = A2; // Atribuirea unui variabile valoarea pinului analogic A2

float tempc; //temperatura in grade Celsius

float vout; //varibila auxiliara

int adcl,adch;//variabile pentru manipularea a 16 biti rezultati din conversia sensorului de temperatura

LiquidCrystal_I2C  lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);


  void init_ADC(){
    ADMUX = 0b01000010; // SETARE AVCC SI CITIREA de pe canalul Analogic2 ( A2 = ADC2)  
    ADCSRA= 0b10000111;//Enable ADC; Prescaler = 128;
  }
  int readADC (){
    ADCSRA |= (1<<6);//Începe conversia
    while(ADCSRA & (1<<6));//Așteaptă finalizarea conversiei
    adcl = ADCL;
    adch = ADCH;
    return (adch << 8 | adcl);
  }

  void setup()  { /*----( SETUP: RUNS ONCE )----*/
    lcd.backlight(); //pornire lumina spate lcd
    //pinMode(sensor,INPUT); PF2
    DDRF &= ~(1 << 2);

    //Serial.begin(115200); // senz ultrasonic
    //pinMode(32,OUTPUT); // albastru       pinMode(33,OUTPUT); // rosu
    DDRC = 0b00110000; // setare Pc5(digital pin 32) si pc4(digital pin 33) ca si iesiri
    PORTC = 0b00000000;
    /*pinMode(ECHOPIN, INPUT); PE4
      pinMode(TRIGPIN, OUTPUT); PE5*/

    DDRE = 0b00100000;
    PORTE = 0b00000000;

    //Initializare Conversia ADC pentru senzorul LM35
    init_ADC();

    lcd.begin (16, 2); // initialize the lcd
    lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
    lcd.setBacklight(LED_ON);
  }

  void loop()
  {
    //Comenzi senz temp
    vout = readADC(); //Citire din ADC

    vout = ((float)vout * 5) / 1024;

    tempc = vout*100-3; // Temperatura in grade celsius


    /*  Serial.print("in DegreeC=");

      Serial.print("\t");

      Serial.print(tempc);

      Serial.println();


      digitalWrite(TRIGPIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIGPIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGPIN, LOW);*/
    //Comenzi ultrasonic
    PORTE &= ~(1 << 5);
    delayMicroseconds(2);
    PORTE |= (1 << 5);
    delayMicroseconds(10);
    PORTE &= ~(1 << 5);
    // Compute distance
    float distance = pulseIn(ECHOPIN, HIGH), d2;
    distance = distance / 58;
    if (distance < 10) {
      lcd.backlight();
      //Bec rosu aprins, bec albastru stins, se afiseaza datele obtinute prin senzori   digitalWrite(33,HIGH);    digitalWrite(32,LOW);
      PORTC |= (1 << 4);
      PORTC &= ~(1 << 5);
      d2 = distance;
      lcd.clear();
      lcd.home();
      lcd.setCursor(0, 0);
      lcd.print("Miscare");
      lcd.setCursor(2, 1);
      lcd.print("detectata !");
      delay(2000);
      lcd.clear();
      lcd.home();
      lcd.setCursor(0, 0);
      lcd.print("Temperatura C:");
      lcd.setCursor(2, 1);
      lcd.print(tempc);
      delay(2000);
      lcd.clear();
      lcd.home();
      lcd.setCursor(0, 0);
      lcd.print("La distanta:");
      lcd.setCursor(2, 1);
      lcd.print(d2);
      lcd.setCursor(7, 1);
      lcd.print("cm");
      delay(2000);
    }
    else {
      //Bec albastru aprins,bec rosu stins, se cauta miscare    digitalWrite(33,LOW); digitalWrite(32,HIGH);
      PORTC |= (1 << 5);
      PORTC &= ~(1 << 4);
      lcd.backlight();
      lcd.clear();
      lcd.home();
      lcd.setCursor(0, 0);
      lcd.print("Se cauta");
      lcd.setCursor(2, 1);
      lcd.print("Miscare !");
    }
    //Serial.println(distance);
    delay(1500);

  }

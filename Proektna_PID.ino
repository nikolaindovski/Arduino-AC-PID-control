#include <AutoPID.h>            //Vklucuvanje na PID bibliotekite
#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <math.h>

#include <avr/io.h>             //Vklucuvanje na Interrupt bibliotekite
#include <avr/interrupt.h>

#include <SPI.h>                //Vklucuvanje na LCD bibliotekite
#include <LiquidCrystal.h>

#define DETECT 2  //zero cross detect               //Definiranje na sopstveni tipovi
#define GATE 7    //TRIAC gate                      //za pinovite za ZCD i gejtot na triakot
#define PULSE 4   //trigger pulse width (counts)
int i=0,I;

int q=1,w=0, f=1;
bool visible=false, doma=true;

int y=0;
int u=0;

//NASI DEKLARACII NADOLE
LiquidCrystal lcd (9, 8, 5, 4, 3, 6);      //Definiranje na podadocni pinovi na LCD
int backLight = 13;

const int buttonPin1 = 10;       //Deklariranje pomosni promenlivi za pinovite na tasterite
int buttonState1 = 1;           
const int buttonPin2 = 11;
int buttonState2 = 1;
const int buttonPin3 = 13;
int buttonState3 = 1;
const int buttonPin4 = 12;
int buttonState4 = 1;
                                 
int pos=20;                     //Deklariranje promenliva za posakuvana temperatura
int A=0;                        //A - pomosna promenliva
// int raz;

int ThermistorPin = A0;          //Deklariranje pomosna promenliva za pinot na sondata
int Vo;
float R1 = 10000;               //Deklariranje parametri za linearizacija na statickata karakteristika na sondata
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

double Kp=5, Ki=0.086, Kd=1;    //Deklariranje parametri za PID regulacija
double Output,Input,Setpoint=20;
bool tuning=false;

double Agol, k ,k1;                 //Deklariranje promenlivi za linearizacija na sinusoidata

unsigned long currentMillis;
unsigned long previousMillisM = 0;
unsigned long previousMillisP = 0;
unsigned long previousMillisD = 0;
unsigned long previousMillisB1 = 0;
unsigned long previousMillisB2 = 0;
unsigned long previousMillisS = 0;

//double //T,pos;

PID myPID (&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);   //Kreiranje PID kontroler povrzan so navedeniot
                                                              //vlez, izlez, i posakuvana cel
PID_ATune aTune(&Input, &Output);


//NASI DEKLARACII NAGORE
                                                                //////////////////////////////////////   setup   //////////////////////////////////////////
void setup(){                                                   //Funkcija za podesuvanje pocetna sostojba na sistemot     
  //NASE NADOLE                                                 
  Serial.begin(9600); 
  pinMode (backLight, OUTPUT);
  pinMode (buttonPin1, INPUT_PULLUP);
  pinMode (buttonPin2, INPUT_PULLUP);
  pinMode (buttonPin3, OUTPUT);         //pin13 ima povrzano dioda i drugi otpornici na samoto arduino
  digitalWrite(buttonPin3,HIGH);        //pa pri deklariranje kako INPUT_PULLUP stoi na 1,7V a ne 5V
//  pinMode (buttonPin3, INPUT_PULLUP); //odnosno ne e na logicka 1. So ova se sovladuva toj problem
  pinMode (buttonPin4, INPUT_PULLUP);
  digitalWrite (backLight, HIGH);
//    lcd.begin (16, 2);
//    lcd.setCursor (0, 0); 
//    lcd.print(" Mihajlo Pupin");
//    lcd.setCursor (0, 1);
//    lcd.print("     Grejach");
//    delay(1500);
    lcd.begin (20, 4);
    lcd.setCursor (0, 0); 
    lcd.print("  Reg. na temp. so");
    lcd.setCursor (0, 1);
    lcd.print(" dig. PID algoritam");
    lcd.setCursor (0, 2);
    lcd.print(" prof:      ucenik:");
    lcd.setCursor (0, 3);
    lcd.print("Petar V.   Nikola I.");
    delay(500);
    lcd.clear();

myPID.SetMode(AUTOMATIC);
  //NASE NAGORE

  // set up pins
  pinMode(DETECT, INPUT);     //zero cross detect
  digitalWrite(DETECT, HIGH); //enable pull-up resistor
  pinMode(GATE, OUTPUT);      //TRIAC gate control

  // set up Timer1 
  //(see ATMEGA 328 data sheet pg 134 for more details)
  OCR1A = 100;      //initialize the comparator
  TIMSK1 = 0x03;    //enable comparator A and overflow interrupts
  TCCR1A = 0x00;    //timer control registers set for
  TCCR1B = 0x00;    //normal operation, timer disabled

  // set up zero crossing interrupt
  attachInterrupt(0,zeroCrossingInterrupt, RISING);    
    //IRQ0 is pin 2. Call zeroCrossingInterrupt 
    //on rising signal
}  

//Interrupt Service Routines
                                                                          //////////////////////////////////////   zeroCrossingInterrupt   //////////////////////////////////////////
void zeroCrossingInterrupt(){ //zero cross detect                         //Funkcija za interrupt pri Zero Cross Detection i vklucuvanje tajmer
  TCCR1B=0x04; //start timer with divide by 256 input
  TCNT1 = 0;   //reset timer - count from zero
}
                                                                          //////////////////////////////////////                         //////////////////////////////////////////
ISR(TIMER1_COMPA_vect){ //comparator match                                //Funkcija za vklucuvanje na triakot
  digitalWrite(GATE,HIGH);  //set TRIAC gate to high
  TCNT1 = 65536-PULSE;      //trigger pulse width
}
                                                                          //////////////////////////////////////                         //////////////////////////////////////////
ISR(TIMER1_OVF_vect){ //timer1 overflow                                   //Funkcija za isklucuvanje na triakot pri istekuvanje na tajmerot
  digitalWrite(GATE,LOW); //turn off TRIAC gate
  TCCR1B = 0x00;          //disable timer stopd unintended triggers
}

//NASI FUNKCII NADOLE                                                     //////////////////////////////////////   MeriTemperatura   //////////////////////////////////////////
float MeriTemperatura () {                                                //Funkcija za merenje na temperaturata preku sondata
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;
// T=map (T, -100,100, 1000,-1000);
  Input=T;
  return T;
}
                                                                          //////////////////////////////////////   PecatiMomentalna   //////////////////////////////////////////
void PecatiMomentalna () {                                                //Procedura za pecatenje na momentalnata temperatura na LCD
    lcd.setCursor(0, 0);
    lcd.print("Temp = ");
    lcd.print(T) ;
    lcd.print(" C");
}

void PodesiTemperatura() {                                            //////////////////////////////////////   PodesiPosakuvana   //////////////////////////////////////////  
unsigned long currentMillis = millis();
if (currentMillis - previousMillisM >= 500){  
  previousMillisM = currentMillis;              
  PecatiMomentalna();   
}
CitajKopcinja();                                                          //Funkcija za podesuvanje na posakuvanata temperatura preku tasterite                                                 
        if (buttonState1 == 0){
          pos--;}
        if (buttonState2 == 0){
          pos++;}
      Setpoint=pos;                                                                         //////////////////////////////////////   PecatiPosakuvana   //////////////////////////////////////////                                        //Procedura za pecatenje na posakuvanata temperatura na LCD
    lcd.setCursor(0, 1);
    BlinkRow("Posakuvana=",0,1,11);
    lcd.print(pos);
    lcd.print(".0 C");
}

void PodesiKpKiKd(){
  lcd.setCursor(0, 0);
  lcd.print("Kp = ");
  lcd.print(Kp);
  lcd.setCursor(0, 1);
  lcd.print("Ki = ");
  lcd.print(Ki);
  lcd.setCursor(0, 2);
  lcd.print("Kd = ");
  lcd.print(Kd);
switch (w){
  case 0: w=1; break;
  case 1: BlinkRow("Kp = ",0,0,5);
          CitajKopcinja();                                                                                                         
          if (buttonState1 == 0) Kp=Kp-0.01;
          if (buttonState2 == 0) Kp=Kp+0.01;
          CitajKopcinja(); 
          if (buttonState3 == LOW) w++;
          if (w>3) w=3;     if (w<1) w=1;
          Serial.print(w);
          break;
  case 2: BlinkRow("Ki = ",0,1,5);
          CitajKopcinja();                                                                                                         
          if (buttonState1 == 0) Ki=Ki-0.01;
          if (buttonState2 == 0) Ki=Ki+0.01;
          if (buttonState3 == LOW) w++;
          if (w>3) w=3;     if (w<1) w=1;
          Serial.print(w);
          break;
  case 3: BlinkRow("Kd = ",0,2,5);
          CitajKopcinja();                                                                                                         
          if (buttonState1 == 0) Kd=Kd-0.01;
          if (buttonState2 == 0) Kd=Kd+0.01;
          if (buttonState3 == LOW) w=0;
          if (w>3) w=3;     if (w<1) w=1;
          Serial.print(w);
          break;
}
}

void AutoTune(){
  while(!tuning){
    tuning=aTune.Runtime();
    lcd.setCursor(0,0);
    lcd.print("Ve molime pocekajte");
      lcd.setCursor(0,2);
      lcd.print("Kp=");
      lcd.print(Kp);
      lcd.setCursor(10, 2);
      lcd.print("Ki=");
      lcd.print(Ki);
      lcd.setCursor(0, 3);
      lcd.print("Kd=");
      lcd.print(Kd);
      lcd.setCursor (7, 1);
      if (f==1) lcd.print(".    ");
      if (f==200) lcd.print(". .");
      if (f==300) lcd.print(". . .");
      if (f==400)  f=0;
      f++;
      Serial.println(tuning);
  }
  if (tuning){
    Kp = aTune.GetKp();
    Ki = aTune.GetKi();
    Kd = aTune.GetKd();
    myPID.SetTunings(Kp,Ki,Kd);
    lcd.clear();
    lcd.setCursor (0, 4);
    lcd.print("Zavrseno");
    delay(5000);
    doma=true;
  }
}

void Informacii(){
  lcd.print(" Proektna za matura");
  lcd.setCursor (0, 1);
  lcd.print("    2018 - 2019");
  lcd.setCursor (0, 2);
  lcd.print(" prof:      ucenik:");
  lcd.setCursor (0, 3);
  lcd.print("Petar V.   Nikola I.");
}
                                                                          //////////////////////////////////////   CitajKopcinja   //////////////////////////////////////////
void CitajKopcinja () {                                                   //Procedura za citanje na sostojbata na tasterite
    buttonState1 = digitalRead(buttonPin1);
    buttonState2 = digitalRead(buttonPin2);
    buttonState3 = digitalRead(buttonPin3);
    buttonState4 = digitalRead(buttonPin4);
    
   // Serial.print(buttonState1); Serial.print(buttonState2); Serial.print(buttonState3); Serial.println(buttonState4);
}

int BlinkRow(char *msg, int q, int r, int c){
  if (visible == false){ 
  lcd.setCursor(q, r);
  lcd.print(msg);
  visible = !visible;
  }
  else if (visible){ 
  lcd.setCursor(q, r);
  for (int brojac=0; brojac<c; brojac++){
    lcd.print(" ");
  }
  visible = !visible;
  }
}

void DisplayMenu() {
if (doma){
  lcd.setCursor(0, 0);
  lcd.print("Podesi temperatura");
  lcd.setCursor(0, 1);
  lcd.print("Podesi Kp, Ki i Kd");
  lcd.setCursor(0, 2);
  lcd.print("Auto Tune PID");
  lcd.setCursor(0, 3);
  lcd.print("Informacii:");
 CitajKopcinja ();
    if (buttonState1==LOW) q++;
    if (q>4) q=4;
    if (buttonState2==LOW) q--;
    if (q<1) q=1;
  //Serial.println(q);
switch (q){
  case 1: BlinkRow("Podesi temperatura",0,0,20); break;
  case 2: BlinkRow("Podesi Kp, Ki i Kd",0,1,20); break;
  case 3: BlinkRow("Auto Tune PID",0,2,20); break;
  case 4: BlinkRow("Informacii:",0,3,20); break;
}
}
if (buttonState3==LOW){
  lcd.clear();
  doma = false;
}
if (doma==false) {
  switch (q){
    case 1: PodesiTemperatura(); break;
    case 2: PodesiKpKiKd(); break;
    case 3: AutoTune(); break;
    case 4: Informacii(); break;
  }
  CitajKopcinja ();
  if (buttonState4==LOW){
    lcd.clear();
    doma=true;
  }
}
}

                                                                           //////////////////////////////////////   KontrolaPID   //////////////////////////////////////////
void KontrolaPID (){                                                       //Funkcija za PID kontrola i presmetka na izleznata velicina 'i'
T = MeriTemperatura();
myPID.Compute();
//i = map (Output, 0,255, 610,10);
//Output=215;
Agol = map (Output, 0,255, 0,180);                                         //Lineariziranje na sinusoidata od naizmenicniot napon
//Serial.print(Agol); Serial.print("  ");

if(Agol > 90){ 
  Agol= (PI*Agol)/180;
  k = 2-sin(Agol);
}
else{
  Agol= (PI*Agol)/180;   
  k = sin(Agol);   
}
k*=100000;
//i = map (k, 0,200000, 610, 10);
i = map (k, 0,200000, 10, 624);
//Serial.print(k); Serial.print("  "); Serial.print(i); Serial.print("  "); Serial.print(Output); Serial.print("  "); Serial.print(sin(Agol));Serial.println("  ");
//myPID.SetTunings(Kp,Ki,Kd);
//Serial.print(Kp); Serial.print("  "); Serial.print(Ki); Serial.print("  "); Serial.println(Kd); 

//y=digitalRead(7);
//
//if (y==1) u=1;
//Serial.print(Input); Serial.print("  "); Serial.print(Setpoint); Serial.print("     "); Serial.print(Output); Serial.print("  "); Serial.print(u); Serial.print("  "); Serial.println(i);
OCR1A = i;
}

void SerialOutputData() {
  if (T<0) T= T * (-1);
  Serial.print(T);
  Serial.print("|  ");
  Serial.print(pos);
  Serial.print("| ");
  Serial.print(Kp);
  Serial.print("| ");
  Serial.print(Ki);
  Serial.print("| ");
  Serial.print(Kd);
  Serial.print("| ");
  Serial.println(0);
}

//NASI FUNKCII NAGORE

void loop(){                                                              //////////////////////////////////////   LOOP   //////////////////////////////////////////

unsigned long currentMillis = millis();

if (currentMillis - previousMillisD >= 100){ 
  previousMillisD = currentMillis;              
  DisplayMenu();   
}

  if (currentMillis - previousMillisS >= 500){  //Povikuvanje na sekoi
  previousMillisS = currentMillis;              //300 milisekundi
  SerialOutputData();   //prakjanje podatoci na smartphone
//  SerialInputData();    //primanje podatoci od smartphone
}
KontrolaPID();          //presmetki i generiranje upravuvacka velicina

}

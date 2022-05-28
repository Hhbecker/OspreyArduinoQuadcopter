
#define CH1 13
#define CH2 12
#define CH3 11
#define CH4 10
#define CH5 9

int ch1Value, ch2Value, ch3Value, ch4Value, ch5Value;
bool abortSwitch = false;
unsigned long duration;

void setup(){
  Serial.begin(115200);
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
}



void loop() {

  ch1Value = pulseIn(CH1, HIGH);
  ch2Value = pulseIn(CH2, HIGH);
  ch3Value = pulseIn(CH3, HIGH);
  ch4Value = pulseIn(CH4, HIGH);
  ch5Value = pulseIn(CH5, HIGH);

  if(pulseIn(CH5, HIGH) > 1500){
    abortSwitch = true;
  }
  
  
  Serial.print("Ch1: ");
  Serial.print(ch1Value);
  Serial.print(" Ch2: ");
  Serial.print(ch2Value);
  Serial.print(" Ch3: ");
  Serial.print(ch3Value);
  Serial.print(" Ch4: ");
  Serial.print(ch4Value);
  Serial.print(" Abort Switch: ");
  Serial.print(abortSwitch);
  
  Serial.println();

  delay(500);
}

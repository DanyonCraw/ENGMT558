//#include <ESP32Servo.h>

//Servo myservo;

#define LEDPIN_R 14
#define LEDPIN_G 27
#define LEDPIN_B 26
#define button 25

unsigned long startTime;
unsigned long reactionTime;

void setup() {
  
  pinMode(button, INPUT_PULLUP);

  pinMode(LEDPIN_R, OUTPUT);
  pinMode(LEDPIN_G, OUTPUT);
  pinMode(LEDPIN_B, OUTPUT);

  digitalWrite(LEDPIN_R, HIGH);
  digitalWrite(LEDPIN_G, HIGH);
  digitalWrite(LEDPIN_B, HIGH);

  Serial.begin(115200);
}

void loop() {
 
  
  if (digitalRead(button) == LOW){ 
    
    Serial.println("button pressed");
    
    digitalWrite(LEDPIN_R, LOW);
    digitalWrite(LEDPIN_G, LOW); 
    digitalWrite(LEDPIN_B, LOW); 
    
    delay(random(2000, 10000));
    startTime = millis();

    Serial.println("Release");

    digitalWrite(LEDPIN_R, HIGH);
    digitalWrite(LEDPIN_G, HIGH);
    digitalWrite(LEDPIN_B, HIGH);
    
    

    while (digitalRead(button) == LOW) {}
    Serial.println("button released");

    reactionTime = millis() - startTime;
    Serial.print("Reaction time: ");
    Serial.print(reactionTime);
    Serial.println("ms");

    //int angle = map(reactionTime, 200, 1000, 0, 180);
    //angle = constrain(angle, 0, 180);
    //myservo.write(angle);

    delay(2000);
  }
}


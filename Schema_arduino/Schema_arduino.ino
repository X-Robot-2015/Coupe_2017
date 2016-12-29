// tableau qui stocke les arguments des fonctions envoyées par raspberry
unsigned char cardArg[16]; 

void setup() {
     // initialisation de la communication raspberry pi
     Serial.begin(9600);
}


void doSerial(){ // fonction qui lit les infos arrivant du raspberry
  
     // On vérifie que les deux informations sont bien arrivées
    if (Serial.available() >= 2)
    {
      cardCommand = Serial.read();
      cardArgCount = Serial.read();
      
      while (Serial.available() < cardArgCount) {};
      for (i = 0; i < cardArgCount; i++)
      {
          cardArg[i] = Serial.read();
      }
      
      switch (cardCommand)
      {
        case 0: // Cas 0
          {
          }
        case 1: // Cas 1
          {  
          }
     }
  }
}
void loop() {
    doSerial();
}

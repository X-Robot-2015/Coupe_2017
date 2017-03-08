// tableau qui stocke les arguments des fonctions envoyées par raspberry
unsigned char cardArg[16];
unsigned char cardCommand, cardArgCount;

void setup() {
     // initialisation de la communication raspberry pi
     Serial.begin(9600);
}

void descendre(){ // Descente de la pince ouverte

    PwmB.pulsewidth_us(1150);
}

void attraper(){ // Attrape Cylindre
    PwmG.pulsewidth_us(1500);
    PwmD.pulsewidth_us(1100);
}

void monter(){ //Monter Cylindre
    PwmB.pulsewidth_us(1700);
}

void relacher(){
    PwmG.pulsewidth_us(1100);
    PwmD.pulsewidth_us(1800);
}

void pince() 
{   
    relacher();
    wait(1);
    descendre();
    wait(1);
    attraper();
    wait(1);
    monter();
    wait(1);
    relacher();
    wait(1);
    
} 

void doSerial(){ // fonction qui lit les infos arrivant du raspberry
  
     // On vérifie que les deux informations sont bien arrivées
    if (Serial.available() >= 2)
    {
      cardCommand = Serial.read();
      cardCommand = Serial.read();
      
      while (Serial.available() < cardArgCount) {};
      for (int i = 0; i < cardArgCount; i++)
      {
          cardArg[i] = Serial.read();
      }
      
      switch (cardCommand)
      {
        case 0: // Cas 0
          {
            pince();
          }
        case 1: // Cas 1
          {  
            fonction1();
          }
     }
  }
}
void loop() {
    doSerial();
}

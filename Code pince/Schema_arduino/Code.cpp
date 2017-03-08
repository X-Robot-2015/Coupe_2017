#include "mbed.h"

//------------------------------------
// Hyperterminal configuration
// 9600 bauds, 8-bit data, no parity
//------------------------------------

Serial pc(SERIAL_TX, SERIAL_RX);
DigitalOut myled(LED1);

PwmOut      PwmG(D10); // Pin Servo droite
PwmOut      PwmB(D2); // Pin Servo de base
PwmOut      PwmD(D9);

unsigned int cardArg[16];
unsigned int cardCommand, cardArgCount;

void setup()
{
    PwmG.period_ms(20);
    PwmD.period_ms(20);
    PwmB.period_ms(20);
    
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

void prend() 
{   
    relacher();
    wait(1);
    descendre();
    wait(1);
    attraper();
    wait(1);
    monter();
    wait(1);
    
    
} 

void pose() 
{   
    
    descendre();
    wait(1);
    relacher();
    wait(1);
    monter();
    wait(1);
    
} 

void loop()
{
    if(pc.readable()){    
        cardCommand = pc.getc();
        pc.putc(cardCommand);
        cardArgCount= pc.getc();
        pc.putc(cardArgCount);
        
        for (int i = 0; i < cardArgCount; i++){
            pc.printf("test3");
            cardArg[i] = pc.getc();
        }
        
        switch(cardCommand)
        {
            case 0 :
                myled = 1;
                prend();
                break;
            case 1 :
                myled = 0;
                pose();
            break;
        }
    }
}

int main()
{
    setup();
    while(1) {
        loop();
    }
}

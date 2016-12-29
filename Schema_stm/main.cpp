#include "mbed.h"

//------------------------------------
// Hyperterminal configuration
// 9600 bauds, 8-bit data, no parity
//------------------------------------

Serial pc(SERIAL_TX, SERIAL_RX);
DigitalOut myled(LED1);
unsigned int cardArg[16];
unsigned int cardCommand, cardArgCount;

void setup()
{
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
                pc.printf("test4");
                break;
            case 1 :
                myled = 0;
                pc.printf("test5");
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

#include "debug.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "queue.h"

void dbgOutputVal(unsigned char outVal){
    //assumes pins 30-37 are already set as outputs
    
    //clear all 8 pins
    PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_7,0);
    PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_6,0);
    PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_5,0);
    PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_4,0);
    PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_3,0);
    PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_2,0);
    PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_1,0);
    PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_0,0);
    
    
    if(outVal == 'T'){
         PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_6,1);
         PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_4,1); 
         PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_2,1); 
    }else if(outVal == 'E'){
        PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_6,1); 
        PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_2,1); 
        PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_0,1); 
    }else if(outVal == 'A'){
        PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_6,1);
        PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_0,1);
    }else if(outVal == 'M'){
        PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_6,1);
        PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_3,1);
        PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_2,1);
        PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_0,1);
    }else if(outVal == '7'){
        PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_5,1);
        PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_4,1);
        PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_2,1);
        PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_1,1);
        PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_0,1);
    }       
}
/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "app_public.h"
#include "queue.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/     

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

//generate global Queue handle
QueueHandle_t qHandle;

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    //create queue
    //qHandle = xQueueCreate(1, sizeof(unsigned int*));
    //if(qHandle == 0){
        //queue was not created
        //PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_7,1);
    //}else
        //PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_5,1);
    //start timer
    DRV_TMR0_Start();
}

int appSendCount(unsigned int count){
    portBASE_TYPE taskWoken;
    if(xQueueSendToFrontFromISR(qHandle, (void*)count, &taskWoken) == pdPASS)
        return 1;
    else
        return 0;
}



void APP_Tasks ( void )
{
    unsigned int buf;
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
            
        
            if (appInitialized)
            {
            PLIB_PORTS_PinDirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_E,PORTS_BIT_POS_7);
            PLIB_PORTS_PinDirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_E,PORTS_BIT_POS_6);
            PLIB_PORTS_PinDirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_E,PORTS_BIT_POS_5);
            PLIB_PORTS_PinDirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_E,PORTS_BIT_POS_4);
            PLIB_PORTS_PinDirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_E,PORTS_BIT_POS_3);
            PLIB_PORTS_PinDirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_E,PORTS_BIT_POS_2);
            PLIB_PORTS_PinDirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_E,PORTS_BIT_POS_1);
            PLIB_PORTS_PinDirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_E,PORTS_BIT_POS_0);
            PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_7,1);
            PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_6,1);
            PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_E,PORTS_BIT_POS_5,1);
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            
//            while(1){
//                xQueueReceive(qHandle,(void *)buf,portMAX_DELAY);
//                unsigned int val = *((unsigned int *)buf);
//                switch(val){
//                    case 0:{
//                        dbgOutputVal('T');
//                        break;
//                    }
//                    case 1:{
//                        dbgOutputVal('E');
//                        break;
//                    }
//                    case 2:{
//                        dbgOutputVal('A');
//                        break;
//                    }
//                    case 3:{
//                        dbgOutputVal('M');
//                        break;
//                    }
//                    case 4:{
//                        dbgOutputVal('7');
//                        break;
//                    }
//                    default:{
//                        
//                        break;
//                    }
//                    
//                    
//                }
//            }
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */

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
#include "Mc32DriverAdc.h"
#include "Mc32DriverAdc.h"
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
#define Courant_Max 0.8
#define Time_Overcurent_Wait 0x35B60

//Variable globale 

//mesures pour adc 
float Tension = 0;
float Courant = 0;
float Tension_Mem [3];

//atente pour adc
uint32_t Temps_ADC = 22;
S_ADCResults Value_ADC;

//-valeur consigne--//
float Consigne = 5;

//-Valeurs pour le correcteur
float Kp = 0.02;
float Ki = 0.00005;

//-- variable liée aux différents correcteur PID --// 
float Up_k = 0;        // facteur proportionnelle; 
float Ui_k = 0;        // facteur intégrateur => actuel 
float Ui_k_1 = 0;      // facteur intégrateur => passé 
float Uk;              // facteur globale

//-- variables pour représenter l'erreur entre la consigne et la position  
float erreurk;          // représentant l'erreur actuel entre la position et la consigne 

//variable d'attente si le courant est trop élevé 
uint8_t OverCurent = 0;
uint32_t Wait_AfterOverCurent = 0;
//--variable modification du pwm--//
 uint32_t Value_PWM = 0;
 
 //tableau avec valeurs pour LED RGB
// uint8_t Data_LEDs [Numer_of_LEDs][Couleurs_LEDs][Datas_LEDs];
// uint32_t LED_Num;
// uint32_t Color;
// uint32_t Data;
// uint32_t i;
// uint32_t a;

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            BSP_InitADC10();
            DRV_TMR0_Start();
            DRV_OC0_Start();
            UpdateAppState(APP_STATE_WAIT);
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            //activation du pin Enable du driver de gate
            DRIVER_ENOn();
            //temps pour relecture adc 
            if (Temps_ADC <20)
            {
                Temps_ADC++;
            }
            else 
            {
                //lecture et calcul en tension des valeurs du courant et de la tension 
                Value_ADC = BSP_ReadAllADC();
                Courant = (((uint16_t)Value_ADC.Chan0 *3.3)/1024);
                Tension = (((uint16_t)(Value_ADC.Chan1) *3.3)/1024)*2;
                Temps_ADC = 0;
                
            }
            //vérification overcurent
            if (Courant > Courant_Max)
            {
                OverCurent = 1;
            }
            
            if(OverCurent == 0)
            {
                //calcul erreur 
                erreurk = Consigne- Tension;

                 //-- proportionnel --//
                Up_k = Kp * erreurk; 

                //-- intégrateur --// 
                Ui_k = Ki * erreurk + Ui_k_1;

                //
                if(Ui_k > 100)
                Ui_k = 100;

                if(Ui_k<0)
                Ui_k = 0;

                //Calcul de Uk
                Uk = Up_k + Ui_k ; 

                //calcul puis changement du pwm 
                Value_PWM = Uk * 2180;
                //envoi de la valeur calculée sur l'OC
                DRV_OC0_PulseWidthSet(Value_PWM);
                //mémoir de Ui_k
                Ui_k_1 = Ui_k;

                //envoi de LED non fonctionnel
                //SendDataLed(Data_LEDs);
            }
            else
            {  
                DRV_OC0_PulseWidthSet(0);
                //géstion du temps d'attente overcurent
                if(Wait_AfterOverCurent < Time_Overcurent_Wait)
                {
                    Wait_AfterOverCurent++;
                }
                else
                {
                    Wait_AfterOverCurent = 0;
                    OverCurent = 0;
                }
            }
            
            //va dans l'état attente
            UpdateAppState(APP_STATE_WAIT);
            break;
        }
        case APP_STATE_WAIT :
        {
            //état attente rien ne se passe --> attend intéruption OC pour continuer
        }  
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
void UpdateAppState(APP_STATES newState)
{
    appData.state = newState;
}

//void SendDataLed(uint8_t Data_LED[Numer_of_LEDs] [Couleurs_LEDs] [Datas_LEDs])
//{
//   
//    i2c_start();
//    for(LED_Num = 0; LED_Num< 8;LED_Num ++)
//    {
//        for ( Color =0;Color<3;Color++)
//        {
//          for( Data = 0;Data<8;Data++)
//          {
//              if (Data_LED[LED_Num][Color][Data] != 0 )
//              {
//                  i2c_write (LED_ON);
//              }
//              else
//              {
//                  i2c_write(LED_OFF);
//              }
//            
//          }
//        }
//    }
//    i2c_stop();
//
//    
//}
/*******************************************************************************
 End of File
 */

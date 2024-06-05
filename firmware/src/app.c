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
#define COURANT_MAX 0.8
#define TIME_OVERCURRENT_WAIT 0x35B60 //
#define VINGT 20
#define TENSION_ENTREE 3.3
#define VALEUR_MAX_ADC 1024
#define DEUX 2
#define MAX_OVERSHOOT 100
#define MIN_OVERSHOOT 0
#define VAL_PWM_MAX 2180

//Variable globale 

//mesures pour adc 
float tension = 0;
float courant = 0;
float tension_Mem [3];

//atente pour adc
uint32_t temps_ADC = 22;
S_ADCResults Value_ADC;

//-valeur consigne--//
float consigne = 5;

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
uint8_t overCurent = 0;
uint32_t wait_AfteroverCurent = 0;
//--variable modification du pwm--//
 uint32_t value_PWM = 0;
 
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
            BSP_InitADC10();    // Initialisation de l'ADC
            DRV_TMR0_Start();   // Initialisation Timer1
            DRV_OC0_Start();    // Initialisation OC1
            UpdateAppState(APP_STATE_WAIT);
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            // Activation du pin Enable du driver de gate
            DRIVER_ENOn();
            // Temps pour relecture adc 
            if (temps_ADC < VINGT)
            {
                temps_ADC++;  
            }
            else 
            {
                // Lecture et calcul en tension des valeurs du courant et de la tension 
                Value_ADC = BSP_ReadAllADC();  
                courant = (((uint16_t)Value_ADC.Chan0 *TENSION_ENTREE)/VALEUR_MAX_ADC);  // Lis le courant de sortie (Iout) et calcul en tension 
                tension = (((uint16_t)(Value_ADC.Chan1) *TENSION_ENTREE)/VALEUR_MAX_ADC)*DEUX; // Lis le courant de sortie (Vout), on fait fois 2 car il y a le pont diviseur avant 
                temps_ADC = 0;
            }
            // Vérification overcurrent
            if (courant > COURANT_MAX)
            {
                overCurent = 1;
            }
            
            if(overCurent == 0)
            {
                // Calcul erreur 
                erreurk = consigne- tension;

                 //-- Proportionnel --//
                Up_k = Kp * erreurk; 

                //-- Intégrateur --// 
                Ui_k = Ki * erreurk + Ui_k_1;

                // Pour limiter l'overshoot
                if(Ui_k > MAX_OVERSHOOT)
                Ui_k = MAX_OVERSHOOT;

                if(Ui_k < MIN_OVERSHOOT)
                Ui_k = MIN_OVERSHOOT;

                //Calcul de Uk
                Uk = Up_k + Ui_k ; 

                // Calcul puis changement du pwm 
                value_PWM = Uk * VAL_PWM_MAX;
                // Envoi de la valeur calculée sur l'OC
                DRV_OC0_PulseWidthSet(value_PWM);
                // Mémoire de Ui_k
                Ui_k_1 = Ui_k;

                // Envoi de LED non fonctionnel
                // SendDataLed(Data_LEDs);
            }
            else
            {  
                DRV_OC0_PulseWidthSet(0);
                // Gestion du temps d'attente overcurrent
                if(wait_AfteroverCurent < TIME_OVERCURRENT_WAIT)
                {
                    wait_AfteroverCurent++;
                }
                else
                {
                    wait_AfteroverCurent = 0;
                    overCurent = 0;
                }
            }
            
            // Va dans l'état attente
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
//    for(LED_Num = 0; LED_Num < Numer_of_LEDs;LED_Num ++)
//    {
//        for ( Color =0;Color < Couleurs_LEDs; Color++)
//        {
//          for( Data = 0;Data < Datas_LEDs;Data++)
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

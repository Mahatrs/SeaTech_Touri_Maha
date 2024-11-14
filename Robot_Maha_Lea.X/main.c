#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "ChipConfig.h"
#include "IO.h"
#include "timer.h"
#include "PWM.h"
#include "Robot.h"
#include "main.h"

int main(void) {
    /***********************************************************************************************/
    //Initialisation oscillateur
    /***********************************************************************************************/
    InitOscillator();

    /***********************************************************************************************/
    // Configuration des input et output (IO)
    /***********************************************************************************************/
    InitIO();
    InitPWM();
    InitADC1();
    //PWMSetSpeed(-20, MOTEUR_DROIT);
    //PWMSetSpeed(20, MOTEUR_GAUCHE);

    InitTimer1();
    InitTimer23();
    InitTimer4();

    LED_BLANCHE_1 = 0;
    LED_BLEUE_1 = 0;
    LED_ORANGE_1 = 0;
    LED_ROUGE_1 = 0;
    LED_VERTE_1 = 0;

    LED_BLANCHE_2 = 1;
    LED_BLEUE_2 = 1;
    LED_ORANGE_2 = 1;
    LED_ROUGE_2 = 1;
    LED_VERTE_2 = 1;
    /***********************************************************************************************/
    // Boucle Principale
    /***********************************************************************************************/


    while (1) {
        if (ADCIsConversionFinished()) {
            ADCClearConversionFinishedFlag();
            unsigned int * result = ADCGetResult();
            float volts = ((float) result [0])* 3.3 / 4096;
            robotState.distanceTelemetreDroit2 = 34 / volts - 5;
            volts = ((float) result [1])* 3.3 / 4096;
            robotState.distanceTelemetreDroit = 34 / volts - 5;
            volts = ((float) result [2])* 3.3 / 4096;
            robotState.distanceTelemetreCentre = 34 / volts - 5;
            volts = ((float) result [3])* 3.3 / 4096;
            robotState.distanceTelemetreGauche = 34 / volts - 5;
            volts = ((float) result [4])* 3.3 / 4096;
            robotState.distanceTelemetreGauche2 = 34 / volts - 5;

            //            int ADCValue0 = result[0];
            //            int ADCValue1 = result[1];
            //            int ADCValue2 = result[2];


            if (robotState.distanceTelemetreCentre >= 40) {
                LED_ORANGE_1 = 0;
            } else {
                LED_ORANGE_1 = 1;
            }
            if (robotState.distanceTelemetreDroit >= 40) {
                LED_BLEUE_1 = 0;
            } else {
                LED_BLEUE_1 = 1;
            }
            if (robotState.distanceTelemetreGauche >= 40) {
                LED_ROUGE_1 = 0;
            } else {
                LED_ROUGE_1 = 1;
            }
            if (robotState.distanceTelemetreGauche2 >= 40) {
                LED_VERTE_1 = 0;
            } else {
                LED_VERTE_1 = 1;
            }
            if (robotState.distanceTelemetreDroit2 >= 40) {
                LED_BLANCHE_1 = 0;
            } else {
                LED_BLANCHE_1 = 1;
            }
        }



    }
    // fin main

}

unsigned char stateRobot;

void OperatingSystemLoop(void) {
    if (timestamp > 60000) {
        PWMSpeedConsigne(0, MOTEUR_DROIT);
        PWMSpeedConsigne(0, MOTEUR_GAUCHE);
        stateRobot = STATE_ATTENTE;
    } else {
        switch (stateRobot) {
            case STATE_ATTENTE:
                //timestamp = 0;
                PWMSpeedConsigne(0, MOTEUR_DROIT);
                PWMSpeedConsigne(0, MOTEUR_GAUCHE);
                stateRobot = STATE_ATTENTE_EN_COURS;

            case STATE_ATTENTE_EN_COURS:
                if (timestamp > 1000)
                    stateRobot = STATE_AVANCE;
                break;

            case STATE_AVANCE:
                PWMSpeedConsigne(30, MOTEUR_DROIT);
                PWMSpeedConsigne(30, MOTEUR_GAUCHE);
                stateRobot = STATE_AVANCE_EN_COURS;
                break;
            case STATE_AVANCE_EN_COURS:
                SetNextRobotStateInAutomaticMode();
                break;

            case STATE_TOURNE_GAUCHE:
                PWMSpeedConsigne(0, MOTEUR_DROIT);
                PWMSpeedConsigne(20, MOTEUR_GAUCHE);
                stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
                break;
            case STATE_TOURNE_GAUCHE_EN_COURS:
                SetNextRobotStateInAutomaticMode();
                break;

            case STATE_TOURNE_DROITE:
                PWMSpeedConsigne(20, MOTEUR_DROIT);
                PWMSpeedConsigne(0, MOTEUR_GAUCHE);
                stateRobot = STATE_TOURNE_DROITE_EN_COURS;
                break;
            case STATE_TOURNE_DROITE_EN_COURS:
                SetNextRobotStateInAutomaticMode();
                break;

            case STATE_TOURNE_SUR_PLACE_GAUCHE:
                PWMSpeedConsigne(-20, MOTEUR_DROIT);
                PWMSpeedConsigne(20, MOTEUR_GAUCHE);
                stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS;
                break;
            case STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS:
                SetNextRobotStateInAutomaticMode();
                break;

            case STATE_RECULE:
                PWMSpeedConsigne(-8, MOTEUR_DROIT);
                PWMSpeedConsigne(-8, MOTEUR_GAUCHE);
                stateRobot = STATE_RECULE_EN_COURS;
                break;
            case STATE_RECULE_EN_COURS:
                SetNextRobotStateInAutomaticMode();
                break;


            case STATE_TOURNE_SUR_PLACE_DROITE:
                PWMSpeedConsigne(20, MOTEUR_DROIT);
                PWMSpeedConsigne(-20, MOTEUR_GAUCHE);
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS;
                break;
            case STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS:
                SetNextRobotStateInAutomaticMode();
                break;

            default:
                stateRobot = STATE_ATTENTE;
                break;
        }
    }
}

unsigned char nextStateRobot = 0;

void SetNextRobotStateInAutomaticMode() {
    unsigned char positionObstacle = PAS_D_OBSTACLE;

    unsigned char sensorState = 0b00000;
    if (robotState.distanceTelemetreGauche2 < 45)
        sensorState |= 1 << 4;
    if (robotState.distanceTelemetreGauche < 43)
        sensorState |= 1 << 3;
    if (robotState.distanceTelemetreCentre < 40)
        sensorState |= 1 << 2;
    if (robotState.distanceTelemetreDroit < 43)
        sensorState |= 1 << 1;
    if (robotState.distanceTelemetreDroit2 < 45)
        sensorState |= 1 << 0;

    switch (sensorState) {

        case 0b00000:

            nextStateRobot = STATE_AVANCE;

            break;

        case 0b00001:

            nextStateRobot = STATE_TOURNE_GAUCHE;

            break;

        case 0b00010:

            nextStateRobot = STATE_TOURNE_GAUCHE;

            break;

        case 0b00011:

            nextStateRobot = STATE_TOURNE_GAUCHE;

            break;

        case 0b00100:

            nextStateRobot =STATE_TOURNE_SUR_PLACE_GAUCHE;

            break;

        case 0b00101:

            nextStateRobot =STATE_TOURNE_GAUCHE;

            break;

        case 0b00110:

            nextStateRobot =STATE_TOURNE_GAUCHE;

            break;

        case 0b00111:

            nextStateRobot =STATE_TOURNE_GAUCHE;

            break;

        case 0b01000:

            nextStateRobot = STATE_TOURNE_DROITE;

            break;

        case 0b01001:

            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;//STATE_TOURNE_SUR_PLACE_DROITE;

            break;

        case 0b01010:

            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;

            break;

        case 0b01011:

            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;//STATE_TOURNE_SUR_PLACE_GAUCHE;

            break;

        case 0b01100:

            nextStateRobot = STATE_TOURNE_DROITE;

            break;

        case 0b01101:

            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;

            break;

        case 0b01110:

            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;

            break;

        case 0b01111:

            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;

            break;

        case 0b10000:

            nextStateRobot = STATE_TOURNE_DROITE;

            break;

        case 0b10001:

            nextStateRobot = STATE_AVANCE;

            break;

        case 0b10010:

            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;

            break;

        case 0b10011:

            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;

            break;

        case 0b10100:

            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;

            break;

        case 0b10101:

            nextStateRobot = STATE_RECULE;//STATE_TOURNE_SUR_PLACE_GAUCHE;

            break;

        case 0b10110:

            nextStateRobot =STATE_TOURNE_SUR_PLACE_DROITE;

            break;

        case 0b10111:

            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;

            break;

        case 0b11000:

            nextStateRobot = STATE_TOURNE_DROITE;//STATE_TOURNE_SUR_PLACE_DROITE;

            break;

        case 0b11001:

            nextStateRobot =STATE_TOURNE_SUR_PLACE_DROITE;

            break;

        case 0b11010:

            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;

            break;

        case 0b11011:

            nextStateRobot = STATE_RECULE;//STATE_TOURNE_SUR_PLACE_DROITE;

            break;

        case 0b11100:

            nextStateRobot = STATE_TOURNE_DROITE;

            break;

        case 0b11101:

            nextStateRobot =STATE_TOURNE_SUR_PLACE_DROITE;

            break;

        case 0b11110:

            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;

            break;

        case 0b11111:

            nextStateRobot =STATE_RECULE; //STATE_TOURNE_SUR_PLACE_DROITE;

            break;


    }




    //        //Détermination de la position des obstacles en fonction des télémètres
    //        if (robotState.distanceTelemetreDroit < 30 &&
    //                robotState.distanceTelemetreCentre > 20 &&
    //                robotState.distanceTelemetreGauche > 30 &&
    //                robotState.distanceTelemetreGauche2 > 30 &&
    //                robotState.distanceTelemetreDroit2 > 20) //Obstacle à droite
    //            positionObstacle = OBSTACLE_A_DROITE;
    //        else if (robotState.distanceTelemetreDroit > 30 &&
    //                robotState.distanceTelemetreCentre > 20 &&
    //                robotState.distanceTelemetreGauche < 30 &&
    //                robotState.distanceTelemetreGauche2 > 20 &&
    //                robotState.distanceTelemetreDroit2 > 30) //Obstacle à gauche
    //            positionObstacle = OBSTACLE_A_GAUCHE;
    //        else if (robotState.distanceTelemetreCentre < 20) //Obstacle en face
    //            positionObstacle = OBSTACLE_EN_FACE;
    //        else if (robotState.distanceTelemetreDroit > 30 &&
    //                robotState.distanceTelemetreCentre > 20 &&
    //                robotState.distanceTelemetreGauche > 30 &&
    //                robotState.distanceTelemetreGauche2 > 40 &&
    //                robotState.distanceTelemetreDroit2 > 40) //pas d?obstacle
    //            positionObstacle = PAS_D_OBSTACLE;
    //        else if (robotState.distanceTelemetreDroit > 20 &&
    //                robotState.distanceTelemetreCentre > 30 &&
    //                robotState.distanceTelemetreGauche > 30 &&
    //                robotState.distanceTelemetreGauche2 > 30 &&
    //                robotState.distanceTelemetreDroit2 < 30) //Obstacle à droite
    //            positionObstacle = OBSTACLE_A_DROITE_2;
    //        else if (robotState.distanceTelemetreDroit > 30 &&
    //                robotState.distanceTelemetreCentre > 30 &&
    //                robotState.distanceTelemetreGauche > 20 &&
    //                robotState.distanceTelemetreGauche2 < 30 &&
    //                robotState.distanceTelemetreDroit2 > 30) //Obstacle à gauche
    //            positionObstacle = OBSTACLE_A_GAUCHE_2;
    //    
    //        //Détermination de l?état à venir du robot
    //        if (positionObstacle == PAS_D_OBSTACLE)
    //            nextStateRobot = STATE_AVANCE;
    //        else if (positionObstacle == OBSTACLE_A_DROITE)
    //            nextStateRobot = STATE_TOURNE_GAUCHE;
    //        else if (positionObstacle == OBSTACLE_A_GAUCHE)
    //            nextStateRobot = STATE_TOURNE_DROITE;
    //        else if (positionObstacle == OBSTACLE_A_DROITE_2)
    //            nextStateRobot = STATE_TOURNE_GAUCHE;
    //        else if (positionObstacle == OBSTACLE_A_GAUCHE_2)
    //            nextStateRobot = STATE_TOURNE_DROITE;
    //        else if (positionObstacle == OBSTACLE_EN_FACE)
    //            if (robotState.distanceTelemetreDroit < 30 &&
    //                    robotState.distanceTelemetreGauche < 30)
    //                nextStateRobot = STATE_RECULE;
    //            else
    //                nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
    //
    //    //Si l?on n?est pas dans la transition de l?étape en cours
    //    


    if (nextStateRobot != stateRobot - 1)
        stateRobot = nextStateRobot;
}

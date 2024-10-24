/* 
 * File:   Robot.h
 * Author: TP-EO-5
 *
 * Created on 2 octobre 2024, 11:59
 */

#ifndef ROBOT_H
#define	ROBOT_H
typedef struct robotStateBITS {
    union {
        struct {
               unsigned char taskEnCours;
            float vitesseGaucheConsigne;
            float vitesseGaucheCommandeCourante;
            float vitesseDroiteConsigne;
            float vitesseDroiteCommandeCourante;
            float distanceTelemetreDroit;
            float distanceTelemetreCentre;
            float distanceTelemetreGauche;
            float distanceTelemetreDroit2;
            float distanceTelemetreGauche2;
        };
    };
} ROBOT_STATE_BITS;
extern volatile ROBOT_STATE_BITS robotState;
#endif /* ROBOT_H */


/* 
 * File:   PWM.h
 * Author: TP-EO-5
 *
 * Created on 2 octobre 2024, 12:06
 */

#ifndef PWM_H
#define	PWM_H
#define MOTEUR_DROIT 0
#define MOTEUR_GAUCHE 1



void InitPWM(void);
//void PWMSetSpeed(float vitesseEnPourcents, unsigned char nbMotor);
void PWMUpdateSpeed();
void PWMSpeedConsigne(float vitesseEnPourcents, char moteur);
#endif /* PWM_H */
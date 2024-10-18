/* 
 * File:   ADC.h
 * Author: TP-EO-5
 *
 * Created on 7 octobre 2024, 11:51
 */

#ifndef ADC_H
#define	ADC_H
void InitADC1(void);
void ADC1StartConversionSequence();
void ADCClearConversionFinishedFlag(void);
unsigned int * ADCGetResult(void);
unsigned char ADCIsConversionFinished(void);
#endif	/* ADC_H */

/*
 * distanceDriver.c
 *
 *  Created on: Apr 24, 2023
 *      Author: jsb19
 */

#include "distanceDriver.h"

uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t TxChannel;
uint32_t channel;
int i;

bool pollDistanceSensor(void)
{
  if(OPT3101_CheckDistanceSensor())
  {
    TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
    return true;
  }
  return false;
}

int32_t Left(int32_t left){
  return (1247*left)/2048 + 22;
}

int32_t Right(int32_t right){
  return  (right*(59*right + 7305) + 2348974)/32768;
}

void initDistanceDriver(void)
{
    I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
    channel = 0;
    Clock_Init48MHz();

    OPT3101_Init();
    OPT3101_Setup();
    OPT3101_CalibrateInternalCrosstalk();
    OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
    TxChannel = 3;
    OPT3101_StartMeasurementChannel(channel);

    LPF_Init(100,8);
    LPF_Init2(100,8);
    LPF_Init3(100,8);
    EnableInterrupts();
    i = 0;
}

void getDistances(uint32_t *distanceBuf)
{
    if(TxChannel <= 2){ // 0,1,2 means new data
      if(TxChannel==0){
        if(Amplitudes[0] > 1000){
            distanceBuf[0] = FilteredDistances[0] = Left(LPF_Calc(Distances[0]));
        }else{
            distanceBuf[0] = FilteredDistances[0] = 500;
        }
      }else if(TxChannel==1){
        if(Amplitudes[1] > 1000){
            distanceBuf[1] = FilteredDistances[1] = LPF_Calc2(Distances[1]);
        }else{
            distanceBuf[1] = FilteredDistances[1] = 500;
        }
      }else {
        if(Amplitudes[2] > 1000){
            distanceBuf[2] = FilteredDistances[2] = Right(LPF_Calc3(Distances[2]));
        }else{
            distanceBuf[2] = FilteredDistances[2] = 500;
        }
      }
      TxChannel = 3; // 3 means no data
      channel = (channel+1)%3;
      OPT3101_StartMeasurementChannel(channel);
      i = i + 1;
    }
}
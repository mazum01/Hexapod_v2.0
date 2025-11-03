#ifndef LOOPTIMER_HPP
#define LOOPTIMER_HPP

#include <Arduino.h>

class LoopTimerClass
{

  public:
  LoopTimerClass( long frequency );
  long DeltaT();
  long LoopDeltaT();
  long TargetDeltaT();
  float DeltaTseconds();
  bool Update();
  void SetFrequency(long frequency);
  
  private:
  
  // Loop timing variables
  long _loopHz;
  unsigned long _dTTarget; // target loop cycle time in microseconds (1,000,000 us / Hz)
  unsigned long _loopdT; // actual loop cycle time in microseconds
  unsigned long _dT; // last triggered loop cycle time (should ~= _dTTarget)
  float _dTs; // the actual loop cycle time in seconds
  unsigned long _loopStartms; // starting micros() timestamp for computing dT
};

LoopTimerClass::LoopTimerClass( long frequency )
{
  _loopHz = frequency;
  if (_loopHz <= 0) _loopHz = 1;
  _dTTarget = 1000000UL / (unsigned long)_loopHz; // loop cycle time in microseconds
  _dTs = 0.0;
  _loopdT = 0;
  _loopStartms = micros();
}

long LoopTimerClass::DeltaT()
{
  return _dT;
}

long LoopTimerClass::LoopDeltaT()
{
  return _loopdT;
}

long LoopTimerClass::TargetDeltaT()
{
  return _dTTarget;
}

float LoopTimerClass::DeltaTseconds()
{
  return _dTs;
}

// Allow updating the target frequency at runtime
inline void LoopTimerClass::SetFrequency(long frequency)
{
  _loopHz = frequency;
  if (_loopHz <= 0) _loopHz = 1;
  _dTTarget = 1000000UL / (unsigned long)_loopHz;
  _loopStartms = micros();
}

bool LoopTimerClass::Update()
{

  // Get the loop timing
  unsigned long currentLoopMicroS = micros();
  bool timerFired = false;  // assume that we are not at the loop trigger timing
  _loopdT = currentLoopMicroS - _loopStartms;
  if ( _loopdT >= _dTTarget )
  {

    // get the loop dT in seconds
    _loopStartms = currentLoopMicroS;
    _dT = _loopdT;
    _dTs = ((float) _dT) / 1000000.0;
    timerFired = true;
  }    

  return timerFired;
}

#endif

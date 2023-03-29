// This code was taken from team DAVE's github page:
// https://github.com/FRC3683/OpenAutoBalance

#include "autoBalance.h"
#include <frc/DataLogManager.h>
#include <iostream>

autoBalance::autoBalance(){
    state = 0;
    debounceCount = 0;

    /**********
     * CONFIG *
     **********/
    autoDirection = 1;

    //Speed the robot drived while scoring/approaching station, default = 0.4
    robotSpeedFast = autoDirection*0.5;
    
    //Speed the robot drives while balancing itself on the charge station.
    //Should be roughly half the fast speed, to make the robot more accurate, default = 0.2
    robotSpeedSlow = autoDirection*0.305;

    //Angle where the robot knows it is on the charge station, default = 13.0
    onChargeStationDegree = autoDirection*10.5;

    //Angle where the robot can assume it is level on the charging station
    //Used for exiting the drive forward sequence as well as for auto balancing, default = 6.0
    levelDegree = autoDirection*3.0;

    //Amount of time a sensor condition needs to be met before changing states in seconds
    //Reduces the impact of sensor noice, but too high can make the auto run slower, default = 0.2
    debounceTime = 0.2;
		
	//Amount of time to drive towards to scoring target when trying to bump the game piece off
	//Time it takes to go from starting position to hit the scoring target
	singleTapTime = 0.4;
		
	//Amount of time to drive away from knocked over gamepiece before the second tap
	scoringBackUpTime = 0.2;
		
	//Amount of time to drive forward to secure the scoring of the gamepiece
	doubleTapTime = 0.3;
}

int autoBalance::getState(){
    return state;
}

int autoBalance::secondsToTicks(double time){
    return (int)(time*50);
}

//Let's first define all of our gyro functionality. This should most certainly be in a separate file (class, even) but I'm not dealing with that right now.
double autoBalance::getAngleDelta(AHRS *g, double roll)
{
    ahrsRollReadouts[0] = ahrsRollReadouts[1];
    ahrsRollReadouts[1] = roll;
    return ahrsRollReadouts[1] - ahrsRollReadouts[0];
}

void autoBalance::trackAngleDelta(double delta){
  // We need too cycle all of the values
  for (uint i = GYRO_TICK_N-1; i > 0; i--)
  {
    previousTickDeltas[i] = previousTickDeltas[i-1];
  }
  previousTickDeltas[0] = delta;
  //gyroTicks++;
}

void autoBalance::tmpTrackAngleDelta(double delta){ // circular queue
// 
    tmpPreviousTickDeltas[tmpPreviousTickDeltas_end++] = delta;
    tmpPreviousTickDeltas_end %= (GYRO_TICK_N+1);
    if(tmpPreviousTickDeltas_counter < GYRO_TICK_N){
        tmpPreviousTickDeltas_counter++;
    }
    else{
        tmpPreviousTickDeltas_start++;
        tmpPreviousTickDeltas_start %= GYRO_TICK_N;
    }
}


int autoBalance::getTicksSinceLastEval(){
  return gyroTicks;
}

void autoBalance::evaluatedData(){
  gyroTicks = 0;
}

bool autoBalance::trackedTicksNegative(){
  for (uint i = 0; i < GYRO_TICK_N; i++)
  {
    if (previousTickDeltas[i] > 0)
    {
      return false;
    }
  }

  return true;
}

bool autoBalance::trackedTicksGreaterThan(double magnitude){
  for (uint i = 0; i < GYRO_TICK_N; i++)
  {
    if (abs(previousTickDeltas[i]) < magnitude)
    {
      return false;
    }
  }

  return true;
}

double autoBalance::avgTrackedTicks(){
  float tot = 0.f;
  for (uint i = 0; i < GYRO_TICK_N; i++)
  {
    tot += previousTickDeltas[i];
  }

  return tot / GYRO_TICK_N;
}


// testing the new algorithm for autobalancing by Andy Yun
// this is not yet complete, so DO NOT USE During Competition
// double autoBalance::tmpAutoBalanceRoutine(frc::AnalogGyro *g){
//     double roll = -g->GetAngle();
//     if (!doAnyAuto) {
//         return 0.0;
//     }
//     if (taxiBalance) {
//         state = 4;
//     }
//     if (!doingBalance) {
//         state = 5;
//     }
//     switch(state){ // 0: initial state, 1: climb mode, 2: blancing, 3: reverse climb mode, 4: charge taxi
//         case 0:
//     }
// }

//routine for automatically driving onto and engaging the charge station.
//returns a value from -1.0 to 1.0, which left and right motors should be set to.
double autoBalance::autoBalanceRoutine(AHRS *g){
    double roll = g->GetRoll();
    double rollDelta = getAngleDelta(g, roll);

    std::cout<<currentSpeed<<" , "<<roll<<" , "<<state<<std::endl;

    switch (state){
        //drive forwards to approach station, exit when tilt is detected
        case 0:
            if(roll > onChargeStationDegree){
                state = 1;
            }
            return robotSpeedFast;
        //driving up charge station, drive slower, stopping when level
        case 1:
            if(roll < onChargeStationDegree){
                state = 2;
            }
            return robotSpeedSlow;
        //on charge station, stop motors and wait for end of auto
        case 2:
            if(roll < -levelDegree){//||angleDeltaCheck(-1)
                state = 3;
            }else if(roll > levelDegree){//||angleDeltaCheck(1)
                state = 1;
            }else{
                // if(brakeTicks<1){
                //     brakeTicks++;
                //     return -currentSpeed;
                //  }else if(brakeTicks<BRAKE_TICK_NUM){
                //     brakeTicks++;
                //     return currentSpeed;
                // }

                return 0;
            }break;
        case 3:
            if(roll > -onChargeStationDegree){
                state = 2;
            }
            return -robotSpeedSlow;
        case 4:
            if (autoBalancing&&taxiTicks < CHARGE_TAXI_TICKS) {
                taxiTicks++;
                return -0.6;
            }else if (taxiTicks < AUTO_TAXI_TICKS) {
                taxiTicks++;
                return -0.6;
            }else if (autoBalancing){
                state = 0;
            }else {
                state = 6;
            }break;
        case 5:
            if (taxiTicks < AUTO_SCORE_TICKS) {
                taxiTicks++;
                return 0.6; 
            }else if (autoTaxi){
                taxiTicks = 0;
                state = 4;
            }else if (autoBalancing){
                state = 0;
            }else{
                state = 6;
            }break;
        case 6: return 0.0;
        default: return 0;
    }
    return 0;
}

double autoBalance::climbMode(int direction, double delta){
    //trackAngleDelta(getAngleDelta(g));
    //if (getTicksSinceLastEval()>=GYRO_TICK_N){
    if(angleDeltaCheck(direction, delta)){
        state = 2;
        brakeTicks = 0;
    }
        //evaluatedData();
    //}
    return direction*robotSpeedSlow;
}

bool autoBalance::angleDeltaCheck(int direction, double delta){
    if (delta > direction * -0.5){
        return true;
    }
    return false;
}

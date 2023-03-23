// This code was taken from team DAVE's github page:
// https://github.com/FRC3683/OpenAutoBalance

#include "autoBalance.h"
#include <frc/DataLogManager.h>

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
    robotSpeedSlow = autoDirection*0.4;

    //Angle where the robot knows it is on the charge station, default = 13.0
    onChargeStationDegree = autoDirection*10.0;

    //Angle where the robot can assume it is level on the charging station
    //Used for exiting the drive forward sequence as well as for auto balancing, default = 6.0
    levelDegree = autoDirection*5.0;

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
double autoBalance::getAngleDelta(frc::AnalogGyro *g)
{
  return round(-g->GetRate()*100)/100;
}

void autoBalance::trackAngleDelta(double delta)
{
  // We need too cycle all of the values
  for (uint i = GYRO_TICK_N-1; i > 0; i--)
  {
    previousTickDeltas[i] = previousTickDeltas[i-1];
  }
  previousTickDeltas[0] = delta;
  gyroTicks++;
}

int autoBalance::getTicksSinceLastEval()
{
  return gyroTicks;
}

void autoBalance::evaluatedData()
{
  gyroTicks = 0;
}

bool autoBalance::trackedTicksNegative()
{
  for (uint i = 0; i < GYRO_TICK_N; i++)
  {
    if (previousTickDeltas[i] > 0)
    {
      return false;
    }
  }

  return true;
}

bool autoBalance::trackedTicksGreaterThan(double magnitude)
{
  for (uint i = 0; i < GYRO_TICK_N; i++)
  {
    if (abs(previousTickDeltas[i]) < magnitude)
    {
      return false;
    }
  }

  return true;
}

double autoBalance::avgTrackedTicks()
{
  float tot = 0.f;
  for (uint i = 0; i < GYRO_TICK_N; i++)
  {
    tot += previousTickDeltas[i];
  }

  return tot / GYRO_TICK_N;
}

void autoBalance::crossChargeStation(){

}

//routine for automatically driving onto and engaging the charge station.
//returns a value from -1.0 to 1.0, which left and right motors should be set to.
double autoBalance::autoBalanceRoutine(frc::AnalogGyro *g){
    double roll = -g->GetAngle();

    switch (state){
        //drive forwards to approach station, exit when tilt is detected
        case 0:
            if(roll > onChargeStationDegree){
                state = 1;
            }
            return robotSpeedFast;
        //driving up charge station, drive slower, stopping when level
        case 1:
            return climbMode(1, roll, g);
        //on charge station, stop motors and wait for end of auto
        case 2:
            if(roll < -onChargeStationDegree){
                state = 3;
            } else if(roll > onChargeStationDegree){
                state = 1;
            }else{
                return 0;
            }break;
        case 3:
            return climbMode(-1, roll, g);
        default: return 0;
    }
    return 0;
}

double autoBalance::climbMode(int direction, double roll, frc::AnalogGyro *g){
    trackAngleDelta(getAngleDelta(g));
    if (getTicksSinceLastEval()>=GYRO_TICK_N){
        if (avgTrackedTicks() > direction * -1){
            state = 2;
        }
        evaluatedData();
    }
    return direction*std::max(robotSpeedSlow, 0.25);
}

// Same as auto balance above, but starts auto period by scoring
// a game piece on the back bumper of the robot
/*double autoBalance::scoreAndBalance(){
    switch (state){
        //drive back, then forwards, then back again to knock off and score game piece
        case 0:
            debounceCount++;
            if(debounceCount < secondsToTicks(singleTapTime)){
                return -robotSpeedFast;
            } else if(debounceCount < secondsToTicks(singleTapTime+scoringBackUpTime)){
                return robotSpeedFast;
            } else if(debounceCount < secondsToTicks(singleTapTime+scoringBackUpTime+doubleTapTime)){
                return -robotSpeedFast;
            } else {
                debounceCount = 0;
                state = 1;
                return 0;
            }
        //drive forwards until on charge station
        case 1:
            if(getTilt() > onChargeStationDegree){
                debounceCount++;
            }
            if(debounceCount > secondsToTicks(debounceTime)){
                state = 2;
                debounceCount = 0;
                return robotSpeedSlow;
            }
            return robotSpeedFast;
        //driving up charge station, drive slower, stopping when level
        case 2:
            if (getTilt() < levelDegree){
                debounceCount++; 
            }
            if(debounceCount > secondsToTicks(debounceTime)){
                state = 3;
                debounceCount = 0;
                return 0;
            }
            return robotSpeedSlow;
        //on charge station, ensure robot is flat, then end auto
        case 3:
            if(fabs(getTilt()) <= levelDegree/2){
                debounceCount++;
            }
            if(debounceCount>secondsToTicks(debounceTime)){
                state = 4;
                debounceCount = 0;
                return 0;
            }
            if(getTilt() >= levelDegree) {
                return robotSpeedSlow/2;
            } else if(getTilt() <= -levelDegree) {
                return -robotSpeedSlow/2;
            }
            break;
        case 4:
            return 0;
    }
    return 0;
}*/




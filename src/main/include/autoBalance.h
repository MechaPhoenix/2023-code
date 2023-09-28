// This code was taken from team DAVE's github page:
// https://github.com/FRC3683/OpenAutoBalance

#pragma once
#include <frc/BuiltInAccelerometer.h>
#include <cmath>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include "AHRS.h"
#include "RobotArm.h"

#define GYRO_TICK_N 3
#define CHARGE_TAXI_TICKS 180
#define AUTO_SCORE_TICKS 30
#define AUTO_TAXI_TICKS 118
#define BRAKE_TICK_NUM 3
#define arraySize(a) (sizeof(a)/sizeof(a[0]))
#define DEFAULT_SLOW_SPEED 0.4;
class rollingAverage
{
    float m_grfValues[8] { 0.f };
    int   m_nIdx = 0;
public:
    inline float newValue( float f ) {
        m_grfValues[m_nIdx] = f;
        m_nIdx = (m_nIdx + 1)%arraySize(m_grfValues);
        return getAverage();
    }
    inline float getAverage() const {
        float fSum = 0.f;
        for( uint i = 0; i < arraySize(m_grfValues); i++) {
            fSum += m_grfValues[i];
        }
        return fSum / arraySize(m_grfValues);
    }
};


class autoBalance{
    public:
        rollingAverage avgTilt;
        autoBalance(); 
        double autoBalanceRoutine(AHRS *g, RobotArm *m_arm, frc::DoubleSolenoid *gripperSolenoid);
        int getState();
        //double climbMode(int direction, double delta);
        //bool angleDeltaCheck(int direction, double delta);
        bool autoBalancing = true;
        bool autoTaxi = true;
        bool lowAuto = false;
        bool midAuto = true;
        bool midScored = false;
        int taxiTicks = 0;
        double currentSpeed = 0;
        int state;
        double* ahrsRollReadouts = new double[2];

        double getAngleDelta(AHRS *g, double roll);
        void trackAngleDelta(double delta);

    private:

        // Gyro functions
        // void tmpTrackAngleDelta(double delta);
        // int getTicksSinceLastEval();
        // void evaluatedData();
        // bool trackedTicksNegative();
        // bool trackedTicksGreaterThan(double magnitude);
        // double avgTrackedTicks();

        // Tracked gyro values
        // int gyroTicks = 0;
        // float previousTickDeltas[GYRO_TICK_N] = { 0.f };
        // double tmpPreviousTickDeltas[GYRO_TICK_N + 1] = {}; //use proper queue
        // int tmpPreviousTickDeltas_start = 0, tmpPreviousTickDeltas_end = 1, tmpPreviousTickDeltas_counter = 0;

        frc::BuiltInAccelerometer mAccel{};
        int debounceCount;
        double robotSpeedSlow;
        double robotSpeedFast;
        double robotCorrectionSpeed;
        double onChargeStationDegree;
        double levelDegree;
        double debounceTime;
        double singleTapTime;
        double scoringBackUpTime;
        double doubleTapTime;
        int autoDirection;
};
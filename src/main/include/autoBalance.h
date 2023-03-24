// This code was taken from team DAVE's github page:
// https://github.com/FRC3683/OpenAutoBalance

#pragma once
#include <frc/BuiltInAccelerometer.h>
#include <cmath>
#include <frc/AnalogGyro.h>

#define GYRO_TICK_N 2
#define AUTO_TICK_NUM 70
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
        double autoBalanceRoutine(frc::AnalogGyro *g);
        double scoreAndBalance();
        int secondsToTicks(double time);
        int getState();
        double climbMode(int direction, double tilt, frc::AnalogGyro *g);
        bool doingBalance = true;
        bool doAnyAuto = true;
        int taxiTicks = 0;

    private:

        // Gyro functions
        double getAngleDelta(frc::AnalogGyro *g);
        void trackAngleDelta(double delta);
        int getTicksSinceLastEval();
        void evaluatedData();
        bool trackedTicksNegative();
        bool trackedTicksGreaterThan(double magnitude);
        double avgTrackedTicks();

        // Tracked gyro values
        int gyroTicks = 0;
        float previousTickDeltas[GYRO_TICK_N] = { 0.f };

        frc::BuiltInAccelerometer mAccel{};
        int state;
        int debounceCount;
        double robotSpeedSlow;
        double robotSpeedFast;
        double onChargeStationDegree;
        double levelDegree;
        double debounceTime;
        double singleTapTime;
        double scoringBackUpTime;
        double doubleTapTime;
        int autoDirection;
};
// This code was taken from team DAVE's github page:
// https://github.com/FRC3683/OpenAutoBalance

#pragma once
#include <frc/BuiltInAccelerometer.h>
#include <cmath>

#define arraySize(a) (sizeof(a)/sizeof(a[0]))
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
        double getPitch();
        double getRoll();
        double getTilt();
        double autoBalanceRoutine();
        double scoreAndBalance();
        int secondsToTicks(double time);
        std::string getState();
        
    private:
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
};
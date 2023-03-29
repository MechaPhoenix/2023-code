#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include "mapping.h"

class driveEncoders{
    public:
        
        void updateEncoderDelta();
        double getEncoderDiffRatio();
        int getEncoderDiffDirection();
        double calculateExpectedEncoderDelta();
        bool inRange(double low, double high, double x);

        double prevTickEncoderVal;
        double currentEncoderDelta;
        double expectedEncoderDelta;

    private:

        ctre::phoenix::motorcontrol::can::TalonSRX m_driveMotorController{ARM_CAN_LOW_NUM};

};
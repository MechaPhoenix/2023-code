#include "DriveEncoders.h"

driveEncoders::driveEncoders(){

    prevTickEncoderVal = m_driveMotorController.GetSensorCollection().GetQuadraturePosition();
    
}

void driveEncoders::updateEncoderDelta(){

    currentEncoderDelta = m_driveMotorController.GetSensorCollection().GetQuadraturePosition() - prevTickEncoderVal;
    prevTickEncoderVal = m_driveMotorController.GetSensorCollection().GetQuadraturePosition();

}



double driveEncoders::getEncoderDiffRatio(){

    if (currentEncoderDelta>0){
        return expectedEncoderDelta/currentEncoderDelta;
    }else {
        return 1.1;
    } 

}

int driveEncoders::getEncoderDiffDirection(){

    if((expectedEncoderDelta*currentEncoderDelta)>0){
        return 1;
    }else if((expectedEncoderDelta*currentEncoderDelta)<0){
        return -1;
    }

}

bool driveEncoders::inRange(double low, double high, double x){

	return (low<=x && x<=high);

}
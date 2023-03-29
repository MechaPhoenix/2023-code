// int autoBalance::secondsToTicks(double time){
//     return (int)(time*50);
// }

// //Let's first define all of our gyro functionality. This should most certainly be in a separate file (class, even) but I'm not dealing with that right now.
// double autoBalance::getAngleDelta(AHRS *g, double roll)
// {
//     ahrsRollReadouts[0] = ahrsRollReadouts[1];
//     ahrsRollReadouts[1] = roll;
//     return ahrsRollReadouts[1] - ahrsRollReadouts[0];
// }

// void autoBalance::trackAngleDelta(double delta){
//   // We need too cycle all of the values
//   for (uint i = GYRO_TICK_N-1; i > 0; i--)
//   {
//     previousTickDeltas[i] = previousTickDeltas[i-1];
//   }
//   previousTickDeltas[0] = delta;
//   //gyroTicks++;
// }

// void autoBalance::tmpTrackAngleDelta(double delta){ // circular queue
// // 
//     tmpPreviousTickDeltas[tmpPreviousTickDeltas_end++] = delta;
//     tmpPreviousTickDeltas_end %= (GYRO_TICK_N+1);
//     if(tmpPreviousTickDeltas_counter < GYRO_TICK_N){
//         tmpPreviousTickDeltas_counter++;
//     }
//     else{
//         tmpPreviousTickDeltas_start++;
//         tmpPreviousTickDeltas_start %= GYRO_TICK_N;
//     }
// }


// int autoBalance::getTicksSinceLastEval(){
//   return gyroTicks;
// }

// void autoBalance::evaluatedData(){
//   gyroTicks = 0;
// }

// bool autoBalance::trackedTicksNegative(){
//   for (uint i = 0; i < GYRO_TICK_N; i++)
//   {
//     if (previousTickDeltas[i] > 0)
//     {
//       return false;
//     }
//   }

//   return true;
// }

// bool autoBalance::trackedTicksGreaterThan(double magnitude){
//   for (uint i = 0; i < GYRO_TICK_N; i++)
//   {
//     if (abs(previousTickDeltas[i]) < magnitude)
//     {
//       return false;
//     }
//   }

//   return true;
// }

// double autoBalance::avgTrackedTicks(){
//   float tot = 0.f;
//   for (uint i = 0; i < GYRO_TICK_N; i++)
//   {
//     tot += previousTickDeltas[i];
//   }

//   return tot / GYRO_TICK_N;
// }
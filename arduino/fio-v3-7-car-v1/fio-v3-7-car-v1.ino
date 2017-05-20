/*
 All components of this library are licensed under the BSD 3-Clause 
 License.
 
 Copyright (c) 2015-, Algorithmic Robotics and Control Group @Rutgers 
 (http://arc.cs.rutgers.edu). All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are
 met:
   
 Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.  Redistributions
 in binary form must reproduce the above copyright notice, this list of
 conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution. Neither the name of
 Rutgers University nor the names of the contributors may be used to 
 endorse or promote products derived from this software without specific
 prior written permission.
   
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __AVR_ATmega32U4__
#define __AVR_ATmega32U4__
#endif

#define MODE_PIN 7

#define M1PWM 3
#define M1PH 4

#define M2PWM 5
#define M2PH 6

void setup()
{
  Serial1.begin(57600);
  pinMode(MODE_PIN, OUTPUT);
  digitalWrite(MODE_PIN, HIGH);

  // Update these if the directions of the wheels are wrong
  digitalWrite(M1PH, HIGH);
  digitalWrite(M2PH, HIGH);
}

// Delay time
int delayMS = 10;

// Read buffer, head is 'C', tail is 'M'
int bufferSize =16;
byte readBuffer[16];

// Global loop counter
int loopCount = 0;

// Vehicle ID
int carID = 1;

// Calibration parameters. For all vehicles, at leftMax + rightMax they
// should go roughly along a straight line forward at roughly the same speed
int leftMax = 200;
int rightMax = 200;

// Previous commands
byte hasPreviousCommand = 0;
byte previousLeft = 0;
byte previousRight = 0;

// Run the two motors at different thrusts, negative thrusts makes the wheel
// go backward
void runMotors(int rightThrust, int leftThrust){
  // Left motor
  if(rightThrust > 0){
    digitalWrite(M1PH, HIGH);
  }
  else{
    digitalWrite(M1PH, LOW);
  }

  // Right motor
  if(leftThrust > 0){
    digitalWrite(M2PH, HIGH);
  }
  else{
    digitalWrite(M2PH, LOW);
  }
  
  // Write the motor speeds
  analogWrite(M1PWM, abs(rightThrust)*leftMax/128);
  analogWrite(M2PWM, abs(leftThrust)*rightMax/128);
}


void processCommand(byte readBuffer[]){
   // Parse the command
   byte lc = readBuffer[carID*2-1];
   byte rc = readBuffer[carID*2];
      
   // Parse and send command to motor
   if(hasPreviousCommand == 0 || 
      (hasPreviousCommand == 1 && 
      (previousLeft != lc || previousRight != rc)
      ))
   {
     int rightThrust = (rc & 0x7F)*((rc & 0x80) == 0x80 ? -1 : 1);
     int leftThrust = (lc & 0x7F)*((lc & 0x80) == 0x80 ? -1 : 1);
     runMotors(rightThrust, leftThrust);

     // Save command
     previousRight = rc;
     previousLeft = lc;
     hasPreviousCommand = 1;
   }
}

void resetVehicle(){
  if(loopCount++ < 5) return;
  loopCount = 0;
  // Stop the vehicle
  analogWrite(M1PWM, 0);
  analogWrite(M2PWM, 0);
  hasPreviousCommand = 0;
  while(Serial1.read() != -1);
}

void loop()
{
    // Check whether we have enough buffer and read if we do
    int bytesAvailable = Serial1.available();
    if(bytesAvailable >= bufferSize){
      // We have enough to try, read some buffer
      int bytesRead = Serial1.readBytes(readBuffer, bufferSize);
      
      // Porcee command if it seems to be a good one
      if(readBuffer[0] == 'C' && readBuffer[bufferSize - 1] == 'M'){
        loopCount = 0;
        processCommand(readBuffer);
      }
      // Otherwise, reset and flush buffer
      else {
        resetVehicle();
      }
    }
    else if(bytesAvailable > 0){
      // We do not have enough but have some odd bytes, flush buffer
      resetVehicle();
    }
    delay(delayMS);
}


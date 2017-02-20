/*
  NonLinearPid.h - Library implementing a model free approach based on 
  pid control. 
  Created by Giovanni Balestrieri - UserK, August 28, 2015.


#include "Arduino.h"
#include "NonLinearPid.h"
#include "Propulsion.h"
#include "PID_v2.h"


NonLinearPid::NonLinearPid(float setPointRoll, float setPointPitch, float setPointYaw, float setPointAlt)
{  
  
        SetpointRoll = setPointRoll;
        SetpointPitch = setPointPitch;
        SetpointYaw = setPointYaw;
        SetpointAltitude = setPointAlt;
        
  	//Specify the links and initial tuning parameters
        PID rollPid(&InputRoll, &OutputRoll, &SetpointRoll, consKpRoll, consKiRoll, consKdRoll, DIRECT);
        PID pitchPid(&InputPitch, &OutputPitch, &SetpointPitch, consKpPitch, consKiPitch, consKdPitch, DIRECT);
        PID yawPid(&InputYaw, &OutputYaw, &SetpointYaw, consKpYaw, consKiYaw, consKdYaw, DIRECT);
        PID altitudePid(&InputAltitude, &OutputAltitude, &SetpointAltitude, consKpAltitude, consKiAltitude, consKdAltitude, DIRECT);
               
        //Specify the links and initial tuning parameters
        PID wrollPid(&InputWRoll, &OutputWRoll, &SetpointWRoll, consKpWRoll, consKiWRoll, consKdWRoll, DIRECT);
        PID wpitchPid(&InputWPitch, &OutputWPitch, &SetpointWPitch, consKpWPitch, consKiWPitch, consKdWPitch, DIRECT);
        PID wyawPid(&InputWYaw, &OutputWYaw, &SetpointWYaw, consKpWYaw, consKiWYaw, consKdWYaw, DIRECT);
        
}

void NonLinearPid::changePidState(boolean cond)
{
  
  if (cond)
  {
    // Enable Pid Actions
    rollPid.SetMode(AUTOMATIC);
    //tell the PID to range between 0 and the full throttle
    //SetpointRoll = 0;
    rollPid.SetOutputLimits(-500, 500);

    // Pitch
    pitchPid.SetMode(AUTOMATIC);
    //SetpointPitch = 0;
    //tell the PID to range between 0 and the full throttle
    pitchPid.SetOutputLimits(-500, 500);

    // Yaw
    yawPid.SetMode(AUTOMATIC);
    //SetpointYaw=0;
    //tell the PID to range between 0 and the full throttle
    yawPid.SetOutputLimits(-500, 500);

    // Enable Pid Actions
    wrollPid.SetMode(AUTOMATIC);
    //tell the PID to range between 0 and the full throttle
    //SetpointRoll = 0;
    wrollPid.SetOutputLimits(-500, 500);

    // Pitch
    wpitchPid.SetMode(AUTOMATIC);
    //SetpointPitch = 0;
    //tell the PID to range between 0 and the full throttle
    wpitchPid.SetOutputLimits(-500, 500);

    // Yaw
    wyawPid.SetMode(AUTOMATIC);
    //SetpointYaw=0;
    //tell the PID to range between 0 and the full throttle
    wyawPid.SetOutputLimits(-500, 500);

    enablePid = true;
  }
  else
  { 
    rollPid.SetMode(MANUAL);
    pitchPid.SetMode(MANUAL);
    yawPid.SetMode(MANUAL);
    wrollPid.SetMode(MANUAL);
    wpitchPid.SetMode(MANUAL);
    wyawPid.SetMode(MANUAL);
    
    
    enablePid = false;
  }
  
}


void NonLinearPid::init()
{
 
        extern PID rollPID(&InputRoll, &OutputRoll, &SetpointRoll, consKpRoll, consKiRoll, consKdRoll, 0);
        extern PID pitchPID(&InputPitch, &OutputPitch, &SetpointPitch, consKpPitch, consKiPitch, consKdPitch, 0);
        extern PID yawPID(&InputYaw, &OutputYaw, &SetpointYaw, consKpYaw, consKiYaw, consKdYaw, 0);
        extern PID altitudePID(&InputAltitude, &OutputAltitude, &SetpointAltitude, consKpAltitude, consKiAltitude, consKdAltitude, 0); 
        
        //Specify the links and initial tuning parameters
        extern PID wRollPID(&InputWRoll, &OutputWRoll, &SetpointWRoll, consKpWRoll, consKiWRoll, consKdWRoll, 0);
        extern PID wPitchPID(&InputWPitch, &OutputWPitch, &SetpointWPitch, consKpWPitch, consKiWPitch, consKdWPitch, 0);
        extern PID wYawPID(&InputWYaw, &OutputWYaw, &SetpointWYaw, consKpWYaw, consKiWYaw, consKdWYaw, 0);

}
void control()
{
  if (enablePid)
  {
    // Roll PID
    if (enableRollPid)
    {
      //Serial.println("    ZAK ");
      InputRoll = estXAngle;
      //Serial.println("    ZAK ");
      errorRoll = abs(SetpointRoll - estXAngle); //distance away from setpoint
      
      rollPID.SetTunings(consKpRoll, consKiRoll, consKdRoll);
        
      rollPID.Compute(); // Computes outputRoll

      if (sakura.getPrintPIDVals())
      {
        Serial.println();
        Serial.print("INPUT ");
        Serial.print(InputRoll);
        Serial.print("ErrorRoll:");
        Serial.print(errorRoll);
        Serial.print("Roll PID: ");
        Serial.print(OutputRoll);
        Serial.println();
      }
    }
    else
    {
      Serial.println();
      Serial.println("Roll SS");
      Serial.println();
      OutputRoll = 0;
    }

    // Pitch PID1
    if (enablePitchPid)
    {
      InputPitch = estYAngle;
      errorPitch = abs(SetpointPitch - estYAngle); //distance away from setpoint
      pitchPID.SetTunings(consKpPitch, consKiPitch, consKdPitch);
      pitchPID.Compute(); // Computes outputPitch

      if (sakura.getPrintPIDVals())
      { 
        Serial.print("E:  ");
        Serial.print(errorPitch);
        Serial.print(" A:");
        Serial.print(OutputPitch);
        Serial.println();
      }
    }  
    else
    {
      Serial.println();
      Serial.println("SS Pitch");
      Serial.println();
      OutputPitch = 0;
    }

    if (sakura.getPrintPIDVals())
    {
      Serial.println();      
    }
  }
  else
  {
    OutputRoll = 0;
    OutputPitch = 0;    
    OutputYaw = 0;
  }
}


void controlW()
{
  //Serial.println("K1");
  if (enablePid)
  {
    //Serial.println("K2");
    // Roll W PID
    if (enableWRollPid)
    {
      InputWRoll = y;
      errorWRoll = abs(SetpointWRoll - y); 

      wRollPID.SetTunings(Kw*consKpWRoll, Kw*consKiWRoll, Kw*consKdWRoll);

      wRollPID.Compute();
    }
    else
    {
      OutputWRoll = 0;
    }

    // Pitch PID1
    if (enablePitchPid)
    {
      InputWPitch = x;
      errorWPitch = abs(SetpointWPitch - x);

      wPitchPID.SetTunings(Kw*consKpWPitch, Kw*consKiWPitch, Kw*consKdWPitch);

      wPitchPID.Compute(); // Computes outputPitch
    }  
    else
    {
      OutputWPitch = 0;
    }

    /*
    // Yaw PID
     if (enableYawPid)
     {
     if (filterAng == 1)
     {
     InputWYaw = z;
     errorWYaw = abs(SetpointWYaw - z); //distance away from setpoint
     }
     
     if(errorYaw<thresholdYaw)
     {
     //we're close to setpoint, use conservative tuning parameters
     wYawPID.SetTunings(Kw*consKpWYaw, Kw*consKiWYaw, Kw*consKdWYaw);
     }   
     wYawPID.Compute(); 
     
     }
     else
     {
     OutputYaw=0;
     }
     */
          
    //if (printPIDVals)
    //{
      //Serial.println();      
    //}
     /*
     
  }
  else
  {
    OutputWRoll = 0;
    OutputWPitch = 0;    
    OutputWYaw = 0;
  }
  
}
*/

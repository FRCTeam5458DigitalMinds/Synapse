/*
  2019 - Synapse 2.0
*/

#include <string>
#include <sstream>
#include <Robot.h>
#include <WPILib.h>
#include <stdlib.h>
#include <iostream>
#include <frc/Timer.h>
#include <TimedRobot.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/smartdashboard/SmartDashboard.h>


// Declarations


//Digital Inputs / Electrical components 

// PDP
frc::PowerDistributionPanel pdp{0};
// Gyro
frc::ADXRS450_Gyro Gyro{}; 
// Straightens out the bot
float signed_square(float x){
  return x * fabsf(x);
}
float LastSumAngle;
float turnFact = 0.9;
// Joystick & Racewheel
frc::Joystick JoyAccel1{0}, Xbox{1}, RaceWheel{2};


// Motors

// Right Side Drive Motors
// Right Side
WPI_TalonSRX RightFront{1};
WPI_TalonSRX RightMid{0};
WPI_TalonSRX RightBack{2};
// Left Side
WPI_TalonSRX LeftFront{15};
WPI_TalonSRX LeftMid{13};
WPI_TalonSRX LeftBack{14};

bool beRunning = false;


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Right side of the drive motors
  RightFront.SetInverted(true);
  RightMid.SetInverted(true);
  RightBack.SetInverted(true);


}

void Robot::TeleopPeriodic() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}
void Robot::AutonomousInit() {}


/*Called on every robot packet, no matter what mode*/
void Robot::RobotPeriodic() {

  //Gets axis for each controller (Driving/Operating)
  double JoyY = -JoyAccel1.GetY();
  double WheelX = RaceWheel.GetX();

  //Power get's cut from one side of the bot to straighten out when driving straight
  float sumAngle = Gyro.GetAngle();
  float derivAngle = sumAngle - LastSumAngle;
  float correctionAngle = (sumAngle * 0.00) + (derivAngle *0.00);

  //Drive Code for CNS and modified for Axon
  //Button 5 on the wheel activates point turning
  if (RaceWheel.GetRawButton(5)) {
    RightFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX);
    RightMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX);
    RightBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX);
    LeftFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -WheelX);
    LeftMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -WheelX);
    LeftBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -WheelX);
    Gyro.Reset();
  } 
  else {
    //Code for regular turning
    if ((WheelX < -0.01 || WheelX > 0.01) && (JoyY > 0.06 || JoyY < -0.06)) {
      RightFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + turnFact*(WheelX));
      RightMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + turnFact*(WheelX));
      RightBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + turnFact*(WheelX));
      LeftFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY - turnFact*(WheelX));
      LeftMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  -JoyY - turnFact*(WheelX));
      LeftBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  -JoyY - turnFact*(WheelX));
      Gyro.Reset();
    }
    //Code for driving straight
    else if ((JoyY > 0.1|| JoyY < -0.1)) {
      RightFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY - correctionAngle);
      RightMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY - correctionAngle);
      RightBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY - correctionAngle);
      LeftFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + correctionAngle);
      LeftMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + correctionAngle);
      LeftBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + correctionAngle);
    } 
    else {
      if(!beRunning) {
        //Dont spin any drive train motors if the driver is not doing anything
        RightFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        RightMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        RightBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        LeftFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        LeftFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        LeftBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      }
    }
  }
  
  //Straightens out bot here when driving straight
  LastSumAngle = sumAngle;

}

/*Called every robot packet in testing mode*/
void Robot::TestPeriodic() {}

/*Starts the bot*/
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

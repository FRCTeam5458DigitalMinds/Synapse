#include <Robot.h>
#include <WPILib.h>
#include <iostream>
#include <TimedRobot.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>


// Left Side
WPI_TalonSRX LeftFront{15};
WPI_TalonSRX LeftMid{13};
WPI_TalonSRX LeftBack{14};

// Right Side
WPI_TalonSRX RightFront{1};
WPI_TalonSRX RightMid{0};
WPI_TalonSRX RightBack{2};

//Speed Controller Groups 
frc::SpeedControllerGroup RightMotors{RightFront,RightMid, RightBack};
frc::SpeedControllerGroup LeftMotors{LeftFront,LeftMid,LeftBack};

//Drive Train
frc::DifferentialDrive DriveTrian{RightMotors,LeftMotors};

//Joystick's
frc::Joystick JoyAccel1{0}, RaceWheel{2};

//Gyro
frc::ADXRS450_Gyro Gyro{};

//Helps with driving
float signed_square(float x){
  return x * fabsf(x);
}

//straightens out the bot
float LastSumAngle;

void Robot::RobotInit(){
  m_chooser.SetDefaultOption(kAutoNameDefault,kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom,kAutoNameCustom);
  frc::SmartDashboard::PutData("AutoModes",&m_chooser);

  RightMotors.SetInverted(true);
  LeftMotors.SetInverted(false);
  
  Gyro.Reset();
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}


void Robot::TeleopPeriodic (){
  double JoyY = JoyAccel1.GetY();
  double WheelX = RaceWheel.GetX();

 double SquaredWheelInput = signed_square(WheelX);

  //Power gets cut from one side of the bot to make it straight
  float SumAngle = Gyro.GetAngle();
  float derivAngle = SumAngle - LastSumAngle;
  float correctionAngle = (SumAngle*.1)+(derivAngle*.2);

  //Straightens out the bot here when driving straight
  LastSumAngle = SumAngle;


}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

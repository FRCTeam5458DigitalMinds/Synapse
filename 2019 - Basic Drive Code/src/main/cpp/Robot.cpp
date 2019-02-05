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
  double yInput = JoyAccel1.GetY();
  double xInput = RaceWheel.GetX();
  
  DriveTrian.ArcadeDrive(-xInput,yInput);
  
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

#include <Robot.h>
#include <iostream>
#include <WPILib.h>
#include <TimedRobot.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
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


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // Individual Motor Controller Tests
  
  //Right Motors
  RightFront.Set(ControlMode::PercentOutput, .1);
  RightMid.Set(ControlMode::PercentOutput, -.1);
  RightBack.Set(ControlMode::PercentOutput, .1);

  //Left Motors
  LeftFront.Set(ControlMode::PercentOutput, -.1);
  LeftMid.Set(ControlMode::PercentOutput, .1);
  LeftBack.Set(ControlMode::PercentOutput, -.1);


  }

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
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

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

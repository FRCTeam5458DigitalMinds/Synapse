#include <WPILib.h>
#include <RobotDrive.h>
#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>
#include <iostream>
#include <DigitalInput.h>
#include <math.h>
#include "ctre/Phoenix.h"
#include <ADXRS450_Gyro.h>
#include <cmath>
#include <time.h>
#include "Spark.h"
#include "networktables/NetworkTable.h"

class Robot : public frc::IterativeRobot
{
//Declarations
	RobotDrive *DriveTrain;
	Joystick *JoyAccel, *RaceWheel;
	PowerDistributionPanel *pdp;
	ADXRS450_Gyro *gyro;

	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");

	float yInput, xInput, gyroFact, turnFact, spawn, lastSumAngle, step, side;
	float power = 0;
	float anglePower = 1;

	float straightDistance;

	std::string gameData;


private:
	TalonSRX *leftFront, *rightFront;
	TalonSRX *leftMid, *rightMid;
	TalonSRX *leftBack, *rightBack;


public:
	void RobotInit() override
{
		pdp = new PowerDistributionPanel(0);

		gyro = new ADXRS450_Gyro();
		gyro->Reset();
		JoyAccel = new Joystick(0);
		RaceWheel = new Joystick(1);

		rightFront = new TalonSRX(1);
		leftFront = new TalonSRX(15);
		rightBack = new TalonSRX(2);
		leftBack = new TalonSRX(14);
		leftMid = new TalonSRX(13);
		rightMid = new TalonSRX(0);

		gyroFact = 0.1;
		turnFact = 0.9;
}

	void TeleopInit() override
{
		gyro->Reset();
}

	void TeleopPeriodic() override
{
		frc::Scheduler::GetInstance()->Run();

		yInput = JoyAccel->GetY();
		xInput = RaceWheel->GetX();

		float sumAngle = gyro->GetAngle();
		float derivAngle = sumAngle - lastSumAngle;
		float correctionAngle = (sumAngle * 0.1) + (derivAngle * .2);

		std::cout << "X: " << table->GetNumber("tx",0.0) << std::endl;

		if (RaceWheel->GetRawButton(5))
		{
			rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, xInput);
			rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, xInput);
			leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, xInput);
			leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, xInput);
			gyro->Reset();
		}

		else if (RaceWheel->GetRawButton(11))
	{
			float threshold = 1;
			float slowDownAngle = 3;
			float slowDown = 0.3;

			if (table->GetNumber("tx",0.0) < slowDownAngle && table->GetNumber("tx",0.0) > -slowDownAngle)
		{
				slowDown = 0.2;
				std::cout << "Slow Down Angle Reached" << std::endl;
		}
		if (table->GetNumber("tx",0.0) > threshold)
		{
			std::cout << "Crosshair Left" << std::endl;
			rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, slowDown);
			rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, slowDown);
			leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, slowDown);
			leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, slowDown);
			gyro->Reset();
		}

		else if (table->GetNumber("tx",0.0) < -threshold)
		{
			std::cout << "Crosshair Right" << std::endl;
			rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -slowDown);
			rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -slowDown);
			leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -slowDown);
			leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -slowDown);
			gyro->Reset();
		}
		else
		{
			std::cout << "Centered" << std::endl;
			rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.3);
			rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.3);
			leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
			leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
			gyro->Reset();

		}

	}
		continue.....


}




	Robot() {
		m_robotDrive.SetExpiration(0.1);
		m_timer.Start();
	}

	void AutonomousInit() override {
		m_timer.Reset();
		m_timer.Start();
	}

	void AutonomousPeriodic() override {
		// Drive for 2 seconds
		if (m_timer.Get() < 2.0) {
			// Drive forwards half speed
			m_robotDrive.ArcadeDrive(-0.5, 0.0);
		} else {
			// Stop robot
			m_robotDrive.ArcadeDrive(0.0, 0.0);
		}
	}

	void TeleopInit() override {}

	void TeleopPeriodic() override {
		// Drive with arcade style (use right stick)
		m_robotDrive.ArcadeDrive(m_stick.GetY(), m_stick.GetX());
	}

	void TestPeriodic() override {}

private:
	// Robot drive system
	frc::Spark m_left{0};
	frc::Spark m_right{1};
	frc::DifferentialDrive m_robotDrive{m_left, m_right};

	frc::Joystick m_stick{0};
	frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
	frc::Timer m_timer;
};

START_ROBOT_CLASS(Robot)

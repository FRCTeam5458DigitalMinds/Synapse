#include <WPILib.h>
#include <ADXRS450_Gyro.h>
#include <iostream>
#include <cmath>

#define PI 3.14159265

class AutonomousCalls{
	float overallHeading = 0;
	float kpd = 0.45;
	float kpa = 0.009;
	float degError = 0;
	float output = 0;
	float lastPosition = 0;

	bool isBroken = false;

	int currentPos = 1;

	RobotDrive *bot;
	ADXRS450_Gyro *gyro;
	Encoder *er, *el, *gearEnc;
	CANTalon *l1, *l2, *r1, *r2, *gearTal;

public:

	AutonomousCalls(RobotDrive *b1, ADXRS450_Gyro *g1, Encoder *en1, Encoder *en2, Encoder *rotenc, CANTalon* left1, CANTalon* left2, CANTalon* right1, CANTalon* right2, CANTalon *rot){ //constructor
		bot = b1;
		gyro = g1;
		er = en1;
		el = en2;
		r1 = right1;
		r2 = right2;
		l1 = left1;
		l2 = left2;
		gearTal = rot;
		gearEnc = rotenc;
	}

	~AutonomousCalls(){ //deconstructor

	}

	void Reset(){ //reset everything
		el->Reset();
		er->Reset();
		gearEnc->Reset();
		el->SetSamplesToAverage(5);
		er->SetSamplesToAverage(5);
		gearEnc->SetSamplesToAverage(5);
		el->SetDistancePerPulse(1.0 / 360.0 * 2.0 * PI * 1.75);
		er->SetDistancePerPulse(1.0 / 360.0 * 2.0 * PI * 1.75);
		gearEnc->SetDistancePerPulse(1.0 / 360.0 * 2.0 * PI * 1.75);
		gyro->Reset();
	}

	void setHeading(float angle){
		overallHeading = angle;
	}

	void breakAll(){
		isBroken = true;
	}

	void DriveDistance(double feet, float power){ //drives some distance given the distance and power
		el->Reset();
		overallHeading = gyro->GetAngle();
		while(abs(el->GetDistance()) < abs(feet) && !isBroken){
			bot->Drive(power, (-gyro->GetAngle() - overallHeading) * (1/180)); //keeps the bot driving straight
			Wait(0.005);
		}
		bot->Drive(0.0, 0.0);
	}

	void DriveDistanceEnc(double feet, float power){
		while(abs(el->GetDistance()) < abs(feet)){
			bot->Drive(power, kpa * (el->GetDistance() - er->GetDistance())); //keeps the bot driving straight
			Wait(0.005);
		}
		bot->Drive(0.0, 0.0);
	}

	void TurnAngle(float degrees, float power){ //turns robot a certain amount of degrees
		float goal = overallHeading + degrees;
		overallHeading = goal;
		if(degrees > 0){
			while(goal > gyro->GetAngle() && !isBroken){ //keeps turning right until gyro heading matches what we want
				bot->TankDrive(power, -power);
				Wait(0.05);
			}
			bot->Drive(0.0,0.0);
		}
		else if(degrees < 0){
			while(goal < gyro->GetAngle()){
				bot->TankDrive(-power, power);
				Wait(0.05);
			}
			bot->Drive(0.0, 0.0);
		}
	}

	void TurnAnglePID(float degrees){
		gyro->Reset();
		degError = degrees - gyro->GetAngle();
		while (degError < 0.5 || degError > 0.5) {
			degError = degrees - gyro->GetAngle();
			output = degError * kpa;
			if(output > 0.95){
				output = 0.95;
			}
			else if (output < -0.95){
				output = -0.95;
			}
			bot->TankDrive(output, -output, false);
			Wait(0.005);
		}
		bot->Drive(0.0, 0.0);
	}

	void ToggleGearMech(){
		Reset();
		if(currentPos == 1){
			while (abs(gearEnc->GetDistance()) < 2 && !isBroken){ //turns slightly less than 90 degrees
				gearTal->Set(-.55);
				Wait(0.005);
			}
			currentPos = 2;
			std::cout << "Gear Mech open\n";
		}
		else if(currentPos == 2 && !isBroken){
			while (abs(gearEnc->GetDistance()) > .1 && !isBroken){ //turns slightly less than 90 degrees
				gearTal->Set(.15);
				Wait(0.005);
			}
			currentPos = 1;
			std::cout<< "Gear Mech closed\n";
		}
			gearTal->Set(0);
	}

};

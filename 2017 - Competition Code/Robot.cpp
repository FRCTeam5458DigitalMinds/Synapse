#include <WPILib.h>
#include <RobotDrive.h>
#include <CANTalon.h>
#include <ADXRS450_Gyro.h>
#include "AutonomousCalls.h"
#include "VisionProcessor.h"
#include "CANTalonSRX.h"

#define PIXY_I2C_DEFAULT_ADDR 0x54

class Robot: public frc::IterativeRobot {

	//Declarations
	CANTalon *leftFront, *leftBack, *rightFront, *rightBack, *climber, *gearSweeper, *gearArm;
	CanTalonSRX *rightMiddle, *leftMiddle;
	RobotDrive *drivetrain;
	ADXRS450_Gyro *gyro;
	Encoder *coderR, *coderL, *rotator;
	Joystick *JoyAccel, *raceWheel;
	AutonomousCalls *autobot;
	I2C *i2c;
	VisionProcessor *vision;
	DigitalInput *switch1, *switch2;
	PowerDistributionPanel *pdp;

	//CHANGE THESE FOR AUTONOMOUS MODES!!!!!(FOR THE SIDE THE ROBOT IS PLACED ON) (The distance is in Inches, approximately...) There is no need to use negatives!!!
	const float initialDriveDistanceForLeftCurve = 72; //FIRST distance to travel for left curve auto
	const float degreesToTurnForLeftCurve = 60;
	const float secondDriveDistanceForLeftCurve = 56; //Distance to travel after the angle turn

	const float initialDriveDistanceForRightCurve = 72;
	const float degreesToTurnForRightCurve = 60; //Number of degrees to turn on the right autos
	const float secondDrivedistanceForRightCurve = 56;

	const float initialDriveDistanceForStraight = 70;
	const float driveDistanceBackwardsForStraight = 12;

	const float driveDistanceBackwardsForCurves = 30;

	const float autoArmStallVoltage = .5;
	const float autoSweeperStallVoltage = 1.5;

	const float kpa = (1/180);
	const float kpe = (1/5);

	float lInput, alignment, turnInput, overallHeading, previousEnc;
	bool isPressed = false;
	bool wasPressed = false;
	bool trigger = true, autoShoot = false;
	bool wasPressedGear = false;
	bool isPressedGear = false;
	bool isPressedToggle = false;
	bool wasPressedToggle = false;
	bool isBroken = false;
	bool isUp = false, breakVision2 = false;
	bool breakVision = false, breakScore = false;
	int currentPos = 1, commitAction = 0;
	bool turnedLeft = false, turnedRight = false;
	float autoSpeed = .3, gearSweeperPower = 0, gearArmPower = 0;
	bool step[12] = {true, false, false, false, false, false, false, false, false, false, false, false};
	double creationism = 0;
	float oldValue, newValue, turnSpeed = .15;
	int autoMode, countDown = 0, countUp, teleCount = 0;
	bool takeover = false, hasGear = false;
	bool lastUp = isUp;
	double integral = 0, derivative = 0, lastAngle = 0;
	double integralD = 0, derivativeD = 0, lastDistance = 0, eD = 0;


public:

	Robot()
	{
		//assignments
		gyro = new ADXRS450_Gyro();

		coderR = new Encoder(3, 4);
		coderL = new Encoder(1, 2);
		rotator = new Encoder(5, 6);

		coderR->Reset();
		coderL->Reset();
		rotator->Reset();

		coderR->SetDistancePerPulse(1.0 / 360.0 * 2.0 * PI * 1.75);
		coderL->SetDistancePerPulse(1.0 / 360.0 * 2.0 * PI * 1.75);
		rotator->SetDistancePerPulse(1.0 / 360.0 * 2.0 * PI * 1.75);

		rightBack = new CANTalon(2);
		rightFront = new CANTalon(3);
		leftBack = new CANTalon(0);
		leftFront = new CANTalon(1);
		climber = new CANTalon(5);
		gearArm = new CANTalon(8);
		rightMiddle = new CanTalonSRX(7);
		leftMiddle = new CanTalonSRX(6);
		gearSweeper = new CANTalon(4);

		gearSweeper->SetInverted(true);
		gearSweeper->SetControlMode(CANSpeedController::ControlMode::kVoltage);
		gearSweeper->SetVoltageCompensationRampRate(48.0);
		gearArm->SetControlMode(CANSpeedController::ControlMode::kVoltage);
		gearArm->SetVoltageCompensationRampRate(48.0);

		rightMiddle->SetModeSelect(CanTalonSRX::kMode_SlaveFollower);
		leftMiddle->SetModeSelect(CanTalonSRX::kMode_SlaveFollower);

		rightMiddle->SetDemand(2);
		leftMiddle->SetDemand(0);
		rightMiddle->SetRevMotDuringCloseLoopEn(1);
		leftMiddle->SetRevMotDuringCloseLoopEn(1);

		//this makes our RobotDrive timeout reset
		leftFront->Set(0);
		leftBack->Set(0);
		rightFront->Set(0);
		rightBack->Set(0);

		drivetrain = new RobotDrive(leftFront, leftBack, rightFront, rightBack);
		drivetrain->SetExpiration(0.5); //robot quits after it doesnt get orders for this much time
		drivetrain->SetSafetyEnabled(false); //required to make the CANTalons work
		drivetrain->SetInvertedMotor(RobotDrive::MotorType::kFrontLeftMotor, true);
		drivetrain->SetInvertedMotor(RobotDrive::MotorType::kFrontRightMotor, true);
		drivetrain->SetInvertedMotor(RobotDrive::MotorType::kRearLeftMotor, true);
		drivetrain->SetInvertedMotor(RobotDrive::MotorType::kRearRightMotor, true);

		JoyAccel = new Joystick(0);
		raceWheel = new Joystick(1);

		autobot = new AutonomousCalls(drivetrain, gyro, coderR, coderL, rotator, leftFront, leftBack, rightFront, rightBack, gearSweeper); //our autonomous "interface"

		i2c = new I2C(I2C::Port::kOnboard, PIXY_I2C_DEFAULT_ADDR);
		vision = new VisionProcessor(i2c);

		switch1 = new DigitalInput(7);
		switch2 = new DigitalInput(8);

		pdp = new PowerDistributionPanel(10);
	}

	void RobotInit() {
	}

	void AutonomousInit() override {
		rotator->Reset();
		coderL->Reset();
		coderR->Reset();
		gyro->Reset();
		breakVision = false;
		overallHeading = 0;
		turnSpeed = .15;
		countDown  = 25;
		step[0] = true;
		for(int i = 1; i <= 12; i++){
			step[i] = false;
		}
		if(!switch1->Get() && !switch2->Get()){ //both off
			autoMode = 0;
		}
		else if(switch1->Get() && !switch2->Get()){ //Left switch on
			autoMode = 1;
		}
		else if(!switch1->Get() && switch2->Get()){ //Right switch on
			autoMode = 2;
		}
		else if(switch1->Get() && switch2->Get()){ //Both swtiches on
			autoMode = 3;
		}
	}

	void AutonomousPeriodic() {
		if(autoMode == 0 && IsAutonomous()){
			if(fabs(coderL->GetDistance()) < 59 && IsAutonomous() && step[0]){  //BEGIN STEP 0
				if(commitAction){
				}
				drivetrain->Drive(.3, (-gyro->GetAngle() - overallHeading) * kpa);
				commitAction = false;
				Wait(0.005);
			}
			else if(IsAutonomous() && step[0]){
				coderL->Reset();
				commitAction = true;
				step[0] = false;
				step[1] = true;
				drivetrain->Drive(0,0);
			}
		}
		else if(autoMode == 1 && IsAutonomous()){
			AutoCurveLeft();
		}
		else if(autoMode == 2 && IsAutonomous()){
			AutoCurveRight();
		}
		else if(autoMode == 3 && IsAutonomous()){
			AutoStraight();
		}
	}

	void TeleopInit() {
		isUp = false;
		rotator->Reset();
		drivetrain->Drive(0.0, 0.0);
		trigger = true;
	}

	void TeleopPeriodic() {
			gearArmPower = 0; //Positive is UP
			gearSweeperPower = -2; //Positive is SPIT
			lInput = JoyAccel->GetY();
			turnInput = raceWheel->GetX();
			if(raceWheel->GetX() < .05 && raceWheel->GetX() > -.05 && JoyAccel->GetY() > .01){
				turnInput = (gyro->GetAngle() /*+ (coderL->GetDistance() + coderR->GetDistance())*/) * kpa;
			}
			else if(raceWheel->GetX() < .05 && raceWheel->GetX() > -.05 && JoyAccel->GetY() < -.01){
				turnInput = (-gyro->GetAngle() /*- (coderL->GetDistance() + coderR->GetDistance())*/) * kpa;
			}
			else{
				gyro->Reset();
				turnInput = raceWheel->GetX();
			}

			if(raceWheel->GetRawButton(5)){
				lInput = raceWheel->GetX();
				if(raceWheel->GetX() <= 1){
					turnInput = -1;
				}
				else{
					turnInput = 1;
				}
				gyro->Reset();
			}

			isPressed = raceWheel->GetRawButton(11);
			if(isPressed && !wasPressed){ //saves the last state so we dont do 200 actions a second
				trigger = !trigger;
			}
			wasPressed = isPressed;

			//isPressedGear = JoyAccel->GetRawButton(2);
			if(isPressedGear && !wasPressedGear){

			}
			wasPressedGear = isPressedGear;

			isPressedToggle = JoyAccel->GetTrigger();
			if(isPressedToggle && !wasPressedToggle){
				isUp = !isUp;
				countDown = 50;
			}
			wasPressedToggle = isPressedToggle;

			if(countDown > 0 && isUp){
				gearArmPower = 7.2; //Power to go UP
				gearSweeperPower = -2;
				countDown--;
			}
			else if(countDown > 0 && !isUp){
				gearArmPower = -8; //Power to go DOWN
				countDown--;
			}
			else{
				if(isUp){
					gearArmPower = 2; //Power to STALL while UP
				} else {
					gearArmPower = -1.5; //Power to STALL while DOWN
					gearSweeperPower = 0;
				}
			}
			if(teleCount > 0){
				gearSweeperPower = -2;
				teleCount--;
			}
			if(JoyAccel->GetRawButton(3)){
				gearSweeperPower = 0;
				gearArmPower = 0;
			}
			if(JoyAccel->GetRawButton(5)){
				gearSweeperPower = 12;
			}
			if(JoyAccel->GetRawButton(4)){
				gearSweeperPower = -12;
				teleCount = 75;

			}

			if(gearSweeper->GetOutputCurrent() > 40 && gearSweeperPower < 0){
				gearSweeperPower = -2.4;
				isUp = true;
				countDown = 50;
			}

			if(!trigger){
				drivetrain->Drive(-lInput * 0.5, turnInput); //when toggled max out at 0.5
			}

			else if(trigger){
				drivetrain->Drive(-lInput, turnInput); //when toggled max out at 1
			}
		climber->Set(JoyAccel->GetRawButton(3));
		gearSweeper->Set(gearSweeperPower);
		gearArm->Set(gearArmPower);

		Wait(0.005);
	}

	void TestInit(){
		coderL->Reset();
		gyro->Reset();
		if(!switch1->Get() && !switch2->Get()){ //both off
			autoMode = 0;
			std::cout << "Both off\n";
		}
		else if(switch1->Get() && !switch2->Get()){ //Left switch on
			autoMode = 1;
			std::cout << "LEft on\n";
		}
		else if(!switch1->Get() && switch2->Get()){ //Right switch on
			autoMode = 2;
			std::cout << "Right on\n";
		}
		else if(switch1->Get() && switch2->Get()){ //Both swtiches on
			autoMode = 3;
			std::cout << "Both on\n";
		}
	}

	void TestPeriodic() {
		//std::cout << "POV: " << JoyAccel->GetPOV() << "\nPOV Count: " << JoyAccel->GetPOVCount() << "\n";
		//std::cout << "Propotion: " << -coderR->GetDistance() / coderL->GetDistance() << "\n";
		//std::cout << "Left: " << coderL->Get() << "\nRight: " << coderR->Get() << "\n";
		//std::cout << "Volatage Reading: " << gearSweeper->GetOutputCurrent() << "\n";
		//DriveDistance(60);
		//std::cout << "Turn: " << PointTurn(90) << "\n";
		//std::cout << "Distance: " <<DriveDistance(60) << "\n";
	}

	void AutoCurveLeft(){
		if(countDown > 0 && step[0] && IsAutonomous()){
			gearArm->Set(-8);
			gearSweeper->Set(-2.8);
			countDown--;
			Wait(.005);
		}
		else if(step[0] && IsAutonomous()){
			gearArm->Set(0);
			countDown = 60;
			step[0] = false;
			step[1] = true;
		}
		if(countDown > 0 && step[1] && IsAutonomous()){
			gearArm->Set(7.2);
			gearSweeper->Set(-2.8);
			countDown--;
			Wait(.005);
		}
		else if(step[1] && IsAutonomous()){
			gearArm->Set(2.4);
			countDown = 75;
			step[1] = false;
			step[2] = true;
			gyro->Reset();
			coderR->Reset();
			coderL->Reset();
			isBroken = false;
		}
		if(!isBroken && IsAutonomous() && step[2]){  //BEGIN STEP 2 *initial move forward
			isBroken = DriveDistance(initialDriveDistanceForLeftCurve);
			gearArm->Set(autoArmStallVoltage);
			gearSweeper->Set(-autoSweeperStallVoltage);
			commitAction = false;
		}
		else if(IsAutonomous() && step[2]){
			coderL->Reset();
			coderR->Reset();
			commitAction = true;
			step[2] = false;
			step[3] = true;
			drivetrain->Drive(0,0);
			isBroken = false;
		}
		if(!isBroken && IsAutonomous() && step[3]){ // BEGIN STEP 1 *first turn
			isBroken = PointTurn(degreesToTurnForLeftCurve);
			gearArm->Set(autoArmStallVoltage);
			gearSweeper->Set(-autoSweeperStallVoltage);
		}
		else if(IsAutonomous() && step[3]){
			coderL->Reset();
			coderR->Reset();
			gyro->Reset();
			isBroken = false;
			step[3] = false;
			step[4] = true;
			drivetrain->Drive(0,0);
		}
		if(!isBroken && IsAutonomous() && step[4]){ //Second Move Forward
			isBroken = DriveDistance(secondDriveDistanceForLeftCurve);
			gearArm->Set(autoArmStallVoltage);
			gearSweeper->Set(-autoSweeperStallVoltage);
		}
		else if(IsAutonomous() && step[4]){
			step[4] = false;
			step[5] = true;
			drivetrain->Drive(0, 0);
			isBroken = false;
		}

		if(countDown > 0 && IsAutonomous() && step[5]){
			gearArm->Set(-8);
			gearSweeper->Set(6);
			countDown--;
			std::cout << "SPITTING\n";
			Wait(.005);
		}
		else if(IsAutonomous() && step[5]){
			gearSweeper->Set(0);
			gearArm->Set(-1.2);
			step[5] = false;
			step[6] = true;
			coderL->Reset();
			coderR->Reset();
			gyro->Reset();
			std::cout << "MOVED ON\n";
		}
		if(!isBroken && IsAutonomous() && step[6]){
			isBroken = DriveDistance(-driveDistanceBackwardsForCurves);
		}
		else if(IsAutonomous() && step[6]){
			drivetrain->Drive(0,0);
			gearSweeper->Set(0);
			gearArm->Set(0);
			step[6] = false;
			step[7] = true;
		}
	}

	void AutoStraight(){
		if(countDown > 0 && step[0] && IsAutonomous()){
			gearArm->Set(-8);
			gearSweeper->Set(-2.8);
			countDown--;
			Wait(.005);
		}
		else if(step[0] && IsAutonomous()){
			gearArm->Set(0);
			countDown = 40;
			step[0] = false;
			step[1] = true;
		}
		if(countDown > 0 && step[1] && IsAutonomous()){
			gearArm->Set(7.2);
			gearSweeper->Set(-2.8);
			countDown--;
			Wait(.005);
		}
		else if(step[1] && IsAutonomous()){
			gearArm->Set(2.4);
			countDown = 75;
			step[1] = false;
			step[2] = true;
			gyro->Reset();
		}
		if(!isBroken && IsAutonomous() && step[2]){  //BEGIN STEP 2
			std::cout << "STEP 2\n";
			isBroken = DriveDistance(initialDriveDistanceForStraight);
			commitAction = false;
			gearArm->Set(2.4);
			gearSweeper->Set(-2.8);
		}
		else if(IsAutonomous() && step[2]){
			coderL->Reset();
			coderR->Reset();
			gyro->Reset();
			commitAction = true;
			step[2] = false;
			step[3] = true;
			isBroken = false;
			drivetrain->Drive(0, 0);
		}
		if(countDown > 0 && IsAutonomous() && step[3]){ //BEGIN STEP 3
			gearArm->Set(-8);
			gearSweeper->Set(6);
			countDown--;
			std::cout << "SPITTING\n";
			Wait(.005);
		}
		else if(IsAutonomous() && step[3]){
			gearSweeper->Set(0);
			gearArm->Set(-1.2);
			step[3] = false;
			step[4] = true;
			coderL->Reset();
			coderR->Reset();
			gyro->Reset();
			std::cout << "MOVED ON\n";
		}
		if(!isBroken && IsAutonomous() && step[4]){  //BEGIN STEP 4
			isBroken = DriveDistance(-driveDistanceBackwardsForStraight);
		}
		else if(IsAutonomous() && step[4]){
			coderL->Reset();
			commitAction = true;
			isBroken = false;
			step[4] = false;
			step[5] = true;
			drivetrain->Drive(0, 0);
		}
	}

	void AutoCurveRight(){
		if(countDown > 0 && step[0] && IsAutonomous()){
			gearArm->Set(-8);
			gearSweeper->Set(-2.8);
			countDown--;
			Wait(.005);
		}
		else if(step[0] && IsAutonomous()){
			gearArm->Set(0);
			countDown = 60;
			step[0] = false;
			step[1] = true;
		}
		if(countDown > 0 && step[1] && IsAutonomous()){
			gearArm->Set(7.2);
			gearSweeper->Set(-2.8);
			countDown--;
			Wait(.005);
		}
		else if(step[1] && IsAutonomous()){
			gearArm->Set(2.4);
			countDown = 75;
			step[1] = false;
			step[2] = true;
			gyro->Reset();
			coderR->Reset();
			coderL->Reset();
			isBroken = false;
		}
		if(!isBroken && IsAutonomous() && step[2]){  //BEGIN STEP 2 *initial move forward
			isBroken = DriveDistance(initialDriveDistanceForRightCurve);
			gearArm->Set(autoArmStallVoltage);
			gearSweeper->Set(-autoSweeperStallVoltage);
			commitAction = false;
		}
		else if(IsAutonomous() && step[2]){
			coderL->Reset();
			coderR->Reset();
			commitAction = true;
			step[2] = false;
			step[3] = true;
			drivetrain->Drive(0,0);
			isBroken = false;
		}
		if(!isBroken && IsAutonomous() && step[3]){ // BEGIN STEP 1 *first turn
			isBroken = PointTurn(-degreesToTurnForRightCurve);
			gearArm->Set(autoArmStallVoltage);
			gearSweeper->Set(-autoSweeperStallVoltage);
		}
		else if(IsAutonomous() && step[3]){
			coderL->Reset();
			coderR->Reset();
			gyro->Reset();
			isBroken = false;
			step[3] = false;
			step[4] = true;
			drivetrain->Drive(0,0);
		}
		if(!isBroken && IsAutonomous() && step[4]){ //Second Move Forward
			isBroken = DriveDistance(secondDrivedistanceForRightCurve);
			gearArm->Set(autoArmStallVoltage);
			gearSweeper->Set(-autoSweeperStallVoltage);
		}
		else if(IsAutonomous() && step[4]){
			step[4] = false;
			step[5] = true;
			drivetrain->Drive(0, 0);
			isBroken = false;
		}

		if(countDown > 0 && IsAutonomous() && step[5]){
			gearArm->Set(-8);
			gearSweeper->Set(6);
			countDown--;
			std::cout << "SPITTING\n";
			Wait(.005);
		}
		else if(IsAutonomous() && step[5]){
			gearSweeper->Set(0);
			gearArm->Set(-1.2);
			step[5] = false;
			step[6] = true;
			coderL->Reset();
			coderR->Reset();
			gyro->Reset();
			std::cout << "MOVED ON\n";
		}
		if(!isBroken && IsAutonomous() && step[6]){
			isBroken = DriveDistance(-driveDistanceBackwardsForCurves);
		}
		else if(IsAutonomous() && step[6]){
			drivetrain->Drive(0,0);
			gearSweeper->Set(0);
			gearArm->Set(0);
			step[6] = false;
			step[7] = true;
		}
	}

	float lastSumAngle = 0, lastSumDistance = 0;
	bool DriveDistance(float distance){
		float sumDistance = ((coderR->Get() + (-coderL->Get())) / 2) * ((3.141592 * 4) / 360);
		float derivDistance = sumDistance - lastSumDistance;
		float correctionDistance = ((distance - sumDistance) * .06) - (derivDistance * .4);

		float sumAngle = gyro->GetAngle();
		float derivAngle = sumAngle - lastSumAngle;
		float correctionAngle = (sumAngle * 0.1) + (derivAngle * .3); //kpe ,008

		drivetrain->TankDrive(correctionDistance - correctionAngle, correctionDistance + correctionAngle); //Reverse for Encoders
		//drivetrain->TankDrive(.3, -gyro->GetAngle() * kpa);
		//std::cout << "Left: " << coderL->Get() << "\nRight: " << coderR->Get() << "\nSum: " << sum << "\nDeriv: " << deriv << "\n";
		//std::cout << "Sum distance: " << sumDistance << "\n";
		lastSumAngle = sumAngle;
		lastSumDistance = sumDistance;
		return(fabs(derivDistance) < .1 && fabs(distance - sumDistance) < 6);
	}

	bool PointTurn(float angle){
		double e = angle - gyro->GetAngle();
		integral += e - integral;
		derivative = gyro->GetAngle() - lastAngle;
		turnSpeed = (.016*e) + (.00*integral) - (.11*derivative); //e = .15, i = 0, d = .1
		drivetrain->TankDrive(turnSpeed, -turnSpeed, false);
		lastAngle = gyro->GetAngle();
		std::cout << "Current Angle:" << gyro->GetAngle() << "\n";
		return (fabs(derivative) < .1 && fabs(e) < 2);
	}

};

START_ROBOT_CLASS(Robot)

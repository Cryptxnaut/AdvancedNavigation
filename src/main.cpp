#include "main.h"
#include "Globals.h"
#include "Functions.h"
#include "PurePursuit.h"
#include "MotorMovement.h"
#include "Odometry.h"

#include "PID.h"
#include <iostream>
#include "api.h"
#include <thread>
//#include "mingw.thread.h"
#include <vector>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

 

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello");

	pros::lcd::register_btn1_cb(on_center_button);

	// pros::Controller master(pros::E_CONTROLLER_MASTER);
	// pros::Motor FrontLeft(15, pros::motor_gearset_e::E_MOTOR_GEARSET_06, true, pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);
	// pros::Motor FrontRight(16, pros::motor_gearset_e::E_MOTOR_GEARSET_06, false, pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);
	// pros::Motor MiddleLeft(13, pros::motor_gearset_e::E_MOTOR_GEARSET_06, false, pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);
	// pros::Motor MiddleRight(8, pros::motor_gearset_e::E_MOTOR_GEARSET_06, true, pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);
	// pros::Motor BackLeft(20, pros::motor_gearset_e::E_MOTOR_GEARSET_06, false, pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);
	// pros::Motor BackRight(14, pros::motor_gearset_e::E_MOTOR_GEARSET_06, true, pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);

	pros::Motor FrontLeft(15, true);
	pros::Motor FrontRight(16);
	pros::Motor MiddleLeft(13);
	pros::Motor MiddleRight(8, true);
	pros::Motor BackLeft(20);
	pros::Motor BackRight(14, true);

	


	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor FrontLeft(15, true);   
	pros::Motor FrontRight(16);
	pros::Motor MiddleLeft(13);
	pros::Motor MiddleRight(8, true);
	pros::Motor BackLeft(20);
	pros::Motor BackRight(14, true);

	pros::Imu Inertial(12);
	pros::Vision VisionSensor(9);

	pros::Motor_Group leftMotorGroup ({FrontLeft, MiddleLeft, BackLeft});
	pros::Motor_Group rightMotorGroup ({FrontRight, MiddleRight, BackRight});

	leftMotorGroup.move_relative(1000, 100);
	rightMotorGroup.move_relative(1000, 100);
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

//  pros::Motor FrontLeft(15, pros::motor_gearset_e::E_MOTOR_GEARSET_06, true, pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES );   
// 	pros::Motor FrontRight(16, pros::motor_gearset_e::E_MOTOR_GEARSET_06, false, pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES	);
// 	pros::Motor MiddleLeft(13, pros::motor_gearset_e::E_MOTOR_GEARSET_06, false, pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);
static void odometryTaskWrapper(void* param){
	OdometryClass* odometryInstance = static_cast<OdometryClass*>(param);
	while (true){
		odometryInstance->Odometry();
		pros::delay(10);
	}
}

static void PurePursuitTaskWrapper(void* param){
	OdometryClass* odometryInstance = static_cast<OdometryClass*>(param);
	PurePursuitClass PurePursuit;
	while (true){
		robotState robot = {
			odometryInstance->X,
			odometryInstance->Y,
			odometryInstance->Theta,
			odometryInstance->DeltaTheta,
			odometryInstance->DeltaTheta
		};

		std::vector<wayPoints> path = {
			{0.0, 0.0},
			{10.0, 10.0},
			{54.0, 54.0},
			{90.0, 30.0},
			{100.0, 90.0}
		};

		PurePursuit.PurePursuit(path, robot);
		pros::delay(10);
	}
}


void autonomous() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor FrontLeft(15, true);
	pros::Motor FrontRight(16);
	pros::Motor MiddleLeft(13);
	pros::Motor MiddleRight(8, true);
	pros::Motor BackLeft(20);
	pros::Motor BackRight(14, true);

	FrontLeft.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	FrontRight.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	MiddleLeft.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	MiddleRight.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	BackLeft.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	BackRight.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);

	pros::Imu Inertial(12);
	pros::Vision VisionSensor(9);

	pros::Motor_Group leftMotorGroup ({FrontLeft, MiddleLeft, BackLeft});
	pros::Motor_Group rightMotorGroup ({FrontRight, MiddleRight, BackRight});

	// PIDcontroller PIDInstance;

	// PIDInstance.setGains(0.1, 0.01, 0.1);
	// PIDInstance.setDesiredValue(200);

	while(true){
		OdometryClass odometryInstance;

		pros::Task odometryTask(odometryTaskWrapper, &odometryInstance);
		pros::Task PurePursuitTask(PurePursuitTaskWrapper, &odometryInstance);

		pros::delay(20);

		odometryTask.remove();
		PurePursuitTask.remove();

		// double motorPower = PIDInstance.getMotorPower();
		// leftMotorGroup.move_relative(200, motorPower);
        // rightMotorGroup.move_relative(200, motorPower);
    

	}

	

	PurePursuitClass PurePursuit;
	OdometryClass odometryInstance;
	OdometryClass Odometry;


	std::vector<wayPoints> path = {
        {0.0, 0.0},
        {10.0, 10.0},
        {54.0, 54.0},
        {90.0, 30.0},
        {100.0, 90.0}
    };

	robotState robot = {
		// 0.0, 0.0
		Odometry.X,
		Odometry.Y,
		Odometry.Theta,
		Odometry.DeltaTheta,
		Odometry.DeltaTheta
	};

	PurePursuit.PurePursuit(path, robot);
	odometryInstance.Odometry();
	//moveForward(3.25);
	
}
//OdometryClass odometryInstance;

void opcontrol() {
	

	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor FrontLeft(15, true);
	pros::Motor FrontRight(16);
	pros::Motor MiddleLeft(13);
	pros::Motor MiddleRight(8, true);
	pros::Motor BackLeft(20);
	pros::Motor BackRight(14, true);

	

	FrontLeft.tare_position();
	FrontRight.tare_position();
	MiddleLeft.tare_position();
	MiddleRight.tare_position();
	BackLeft.tare_position();
	BackRight.tare_position();

	pros::Imu Inertial(12);
	//pros::Vision VisionSensor(9);

	pros::Motor_Group leftMotorGroup ({FrontLeft, MiddleLeft, BackLeft});
	pros::Motor_Group rightMotorGroup ({FrontRight, MiddleRight, BackRight});

	//pros::Task odometryTask(odometryTaskWrapper, nullptr);
	OdometryClass odometryInstance;
	//odometryInstance.Odometry();
    //pros::Task odometryTask(odometryTaskWrapper, &odometryInstance);
    //pros::Task PurePursuitTask(PurePursuitTaskWrapper, &odometryInstance);
    //pros::delay(20);
    //odometryTask.remove();
    //PurePursuitTask.remove();


	while (true) {

    int power = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_RIGHT_X);
    int left = power + turn;
    int right = power - turn;
	
    // int left = master.get_analog(ANALOG_LEFT_Y);
    // int right = master.get_analog(ANALOG_RIGHT_Y);
    // right *= -1;
    
    
	//pros::delay(20);

	
    pros::delay(20);

    pros::Task odometryTask(odometryTaskWrapper, &odometryInstance);
    odometryTask.remove();

	
	
    pros::delay(20);
	leftMotorGroup.move(left);
    rightMotorGroup.move(right);


	}
    
}

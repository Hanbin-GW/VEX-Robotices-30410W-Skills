#include "main.h"
#include "pros/ai_vision.hpp"
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/gps.hpp"
#include "pros/adi.hpp"

#include "lemlib/api.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/chassis/chassis.hpp"

#include "liblvgl/display/lv_display.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/widgets/image/lv_image.h"

// 0: Red Left, 1: Red Right, 2: Blue Left, 3: Blue Right, 4: Skills
int auton_selection = 0;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-19, -20});
pros::MotorGroup rightMotors({1, 2});

pros::Motor intake_1(18);
pros::Motor intake_2(17);

pros::MotorGroup intake({-17, 18}); // Temporary remove intake_1

pros::Motor outtake(3);

pros::Gps gps(12);
pros::Imu imu(5); 
pros::Rotation vertical_rotation_sensor(-15);
pros::Rotation horizontal_rotation_sensor(-16); // should be change later port3 -> port 16
// pros::Rotation vertial_rotation_sensor_2(-14);
pros::adi::DigitalOut pneumatics_1 ('A');
pros::adi::DigitalOut pneumatics_2 ('D');

lemlib::TrackingWheel vertical(
    &vertical_rotation_sensor,
    lemlib::Omniwheel::NEW_2,
    -5.91	// Don't change!
);
// lemlib::TrackingWheel vertical_2(
//     &vertial_rotation_sensor_2,
//     lemlib::Omniwheel::NEW_2,
//     3.937	// Don't change!
// );
lemlib::TrackingWheel horizontal(
    &horizontal_rotation_sensor,
    lemlib::Omniwheel::NEW_2,
    0	// Don't change!
);

// lemlib::Drivetrain drivetrain(&leftMotors,		// left motor group
// 					&rightMotors,		    	// right motor group
// 					10,			    			// 10 in track width
// 					lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
// 					360,				   		// drivetrain rpm is 500 -> 333.33
// 					2				    		// horizontal drift is 2. If we had traction wheels, it would have been 8
// );

lemlib::Drivetrain drivetrain(&leftMotors,		// left motor group
					&rightMotors,		    	// right motor group
					9.645,			    			// 10 in track width
					lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
					480,				   		// drivetrain rpm is 500 -> 333.33
					2				    		// horizontal drift is 2. If we had traction wheels, it would have been 8
);

lemlib::ControllerSettings linearController(
    //      0, 10, 10,   // kP, kI, kD
    10, 0, 180, // kP, kI, kD
    0,	   // anti-windup
    1, 100,	   // smallError,  smallErrorTimeout(ms)
    3, 500,	   // largeError,  largeErrorTimeout(ms)
    0	   // slew rate
);

// lemlib::ControllerSettings linearController(
//     //      0, 10, 10,   // kP, kI, kD
//     10, 0, 15, // kP, kI, kD
//     0,	   // anti-windup
//     1, 100,	   // smallError,  smallErrorTimeout(ms)
//     3, 500,	   // largeError,  largeErrorTimeout(ms)
//     60	   // slew rate
// );

lemlib::ControllerSettings angularController(
	5.0, 0, 100,	// kP, kI, kD
	0,
	1, 100,
	3, 500,
	0);

lemlib::OdomSensors sensors(
	&vertical,
	nullptr,
	&horizontal,
	nullptr,
	&imu);

lemlib::Chassis chassis(
    drivetrain,
    linearController,
    angularController,
    sensors);

// ASSET(proto_txt);


LV_IMAGE_DECLARE(JSOC);

double drivetrainAvgTempC() {
	return (leftMotors.get_temperature() + rightMotors.get_temperature()) / 2.0;
}
pros::Task* controllerHudTaskPtr = nullptr;
pros::Task* screenTaskPtr = nullptr;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	// imu.reset();
	// pros::lcd::register_btn1_cb(on_center_button);
	chassis.calibrate();
	while (imu.is_calibrating()) pros::delay(10);
	// chassis.setPose(0, 0, 0); // There has no reason to reset the pose to (0,0,0)

	// PathJERRYIO start point (VEX Gaming Positioning System) //
	// chassis.setPose(66.464, 13.805, 270);

	pros::lcd::set_text(1, "Skills code initialize!");

    // ---------------- Controller HUD Task ----------------
	if (controllerHudTaskPtr == nullptr) {
        controllerHudTaskPtr = new pros::Task([](void*) {
            while (true) {
                auto pose = chassis.getPose();
                int heading = (int)imu.get_heading();
                int batt = pros::battery::get_capacity();
                double avgTemp = drivetrainAvgTempC();

            //     controller.clear();
                controller.print(0, 0, "X:%3.0f Y:%3.0f H:%3d", pose.x, pose.y, heading);
                controller.print(1, 0, "B:%3d%% T:%2.0fC Th:%3.0f", batt, avgTemp, pose.theta);

                pros::delay(200);
            }
        }, nullptr, "CtrlHUD");
    }

    // ---------------- Brain LCD Task ----------------
	if (screenTaskPtr == nullptr) {
        screenTaskPtr = new pros::Task([](void*) {
            while (true) {
			auto p = chassis.getPose();
			pros::lcd::print(0, "X: %f", p.x);
			pros::lcd::print(1, "Y: %f", p.y);
			pros::lcd::print(2, "Theta: %f", p.theta);
			pros::lcd::print(3, "IMU heading: %.1f", imu.get_heading());

			// int leftVoltage = leftMotors.get_voltage();
			// int rightVoltage = rightMotors.get_voltage();

			// pros::lcd::print(0, "X: %f", p.x);
			// pros::lcd::print(1, "Y: %f", p.y);
			// pros::lcd::print(2, "L Volt: %d", leftVoltage);
			// pros::lcd::print(3, "R Volt: %d", rightVoltage);


                lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

                pros::delay(50);
            }
        }, nullptr, "BrainLCD");
    }

	/* =================== Lv Screen ICON =================== */

	lv_obj_t *img = lv_image_create(lv_screen_active());
	lv_image_set_src(img, &JSOC);

	lv_coord_t sw = lv_display_get_horizontal_resolution(NULL);
	lv_coord_t sh = lv_display_get_vertical_resolution(NULL);

	lv_obj_set_size(img, 100, 100);
	lv_image_set_inner_align(img, LV_IMAGE_ALIGN_STRETCH);
	lv_obj_align(img, LV_ALIGN_TOP_RIGHT, -15, 15);
	
	/* ====================================================== */

	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
}

// void on_center_button() {
//     auton_selection++;
    
//     if (auton_selection > 4) {
//         auton_selection = 0;
//     }
    
// //     update_screen();
// }

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
void competition_initialize() {}

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
void autonomous() {
	pneumatics_1.set_value(false);
	pneumatics_2.set_value(false);
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	//** Test code */
	// chassis.moveToPoint(0, 0, 5000);
	// chassis.moveToPoint(-7.9, 33.982, 5000);
	// chassis.moveToPoint(4.336, 50.296, 5000);


	chassis.moveToPoint(-9.143, 38.258, 2000);
	pros::delay(500);
	//** Intake logic */
	intake.move(100);
	pros::delay(500);
	intake.brake();
	//* ------------- */
	chassis.moveToPoint(-31.762, 36.574, 2500);
	//** Socring logic */
	intake.move(127);
	pros::delay(1500);
	intake.brake();
	//* ------------- */
	chassis.moveToPoint(-31.521, 0, 3000, {.forwards = false});
	//** Loder logic */
	pneumatics_2.set_value(false);
	intake.move(127);
	pros::delay(500);
	intake.brake();
	chassis.moveToPoint(-31.762, 0, 2000);
	pneumatics_2.set_value(true);
	//* ------------- */
	chassis.moveToPoint(-6.015, 13.956, 4000);
	chassis.moveToPoint(62.801, 13.956, 2000);
	chassis.moveToPoint(62.801, 35.13, 3000);
	//** Scoring logic */
	intake.move(127);
	pros::delay(1500);
	intake.brake();
	//* ------------- */
	chassis.moveToPoint(16.362, 34.168, 3000);
	chassis.moveToPoint(16.603, -3.128, 2500, {.forwards = false});
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol()
{
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	pneumatics_1.set_value(true);
	pneumatics_2.set_value(true);
	while (true)
	{
		int dir = controller.get_analog(ANALOG_LEFT_Y);
		int turn = controller.get_analog(ANALOG_RIGHT_X);
		bool in = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		bool out = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
		bool L1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
		bool L2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
		leftMotors.move(dir + turn);	
		rightMotors.move(dir - turn); 
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
		{
			pneumatics_1.set_value(false);
		}
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
		{
			pneumatics_1.set_value(true);
		}
		if (in && !out)
		{
			intake.move(127);
			outtake.move(127);
		}
		else if (out && !in)
		{
			intake.move(-127);
			outtake.move(-127);
		}

		else
		{
			intake.brake();
			outtake.brake();
		}
	
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		{
			pneumatics_2.set_value(false);
		}
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
		{
			pneumatics_2.set_value(true);
		}

		if (L1)
		{
			// 필요: 아래 B안 참고해서 모터 중복 제어 피하기
			outtake.move_voltage(12000);
		}
		else if (L2)
		{
			outtake.move_voltage(-12000);
		}

		else
		{
			// intake_2.move(0);
		}

		// const char* c = detect_color_once();
		// pros::lcd::print(0, "Detect: %s", c);
		// pros::delay(100);

		pros::delay(20); // Run for 20 ms then update
	}
}

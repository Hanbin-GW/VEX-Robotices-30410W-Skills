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

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-1, -2});
pros::MotorGroup rightMotors({19, 20});

pros::Motor intake_1(18);
pros::Motor intake_2(17);

pros::MotorGroup intake({18, -17});

pros::Motor outtake(3);

pros::Vision vision_sensor(14);
pros::Gps gps(12);
pros::Imu imu(5); 
pros::Rotation vertical_rotation_sensor(15);
pros::Rotation horizontal_rotation_sensor(16);
pros::adi::DigitalOut pneumatics_1 ('A');
pros::adi::DigitalOut pneumatics_2 ('B');

lemlib::TrackingWheel vertical(
    &vertical_rotation_sensor,
    lemlib::Omniwheel::NEW_2,
    0.0
);

lemlib::TrackingWheel horizontal(
    &horizontal_rotation_sensor,
    lemlib::Omniwheel::NEW_2,
    0.0
);

lemlib::Drivetrain drivetrain(&leftMotors,		    // left motor group
					&rightMotors,		    // right motor group
					26,				    // 26 inch track width
					lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
					500,				    // drivetrain rpm is 500
					2				    // horizontal drift is 2. If we had traction wheels, it would have been 8
);

lemlib::ControllerSettings linearController(
    //      0, 10, 10,   // kP, kI, kD
    3, 0.5, 9, // kP, kI, kD
    0,	   // anti-windup
    0, 100,	   // smallError,  smallErrorTimeout(ms)
    3, 500,	   // largeError,  largeErrorTimeout(ms)
    100	   // slew rate
);

lemlib::ControllerSettings angularController(
	4.5, 0.5, 13,
	0,
	0, 100,
	3, 500,
	100);

lemlib::OdomSensors sensors(
    nullptr,
    nullptr,
    &vertical,
    nullptr,
    &imu);

lemlib::Chassis chassis(
    drivetrain,
    linearController,
    angularController,
    sensors);

LV_IMAGE_DECLARE(theghost);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	imu.reset();
	chassis.calibrate();
	pros::lcd::set_text(1, "Skills code initialize!");

	/* =================== Lv Screen ICON =================== */

	lv_obj_t *img = lv_image_create(lv_screen_active());
	lv_image_set_src(img, &theghost);

	lv_coord_t sw = lv_display_get_horizontal_resolution(NULL);
	lv_coord_t sh = lv_display_get_vertical_resolution(NULL);

	lv_obj_set_size(img, 75, 75);
	lv_image_set_inner_align(img, LV_IMAGE_ALIGN_STRETCH);
	lv_obj_align(img, LV_ALIGN_TOP_RIGHT, -15, 15);
	
	/* ====================================================== */

	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
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
	pneumatics_1.set_value(true);
	pneumatics_2.set_value(true);

	// chassis.setPose(0, 0, 0);
	chassis.moveToPoint(16.22, 41.505, 5000);
	intake.move_voltage(12000);
	pros::delay(500);
	intake.brake();
	chassis.moveToPoint(-5.486, -59.394, 5000);
	intake.move_voltage(-12000);
	pros::delay(1500);
	intake.brake();
	chassis.moveToPoint(-28.385,  44.844, 5000);
	intake.move_voltage(12000);
	pros::delay(500);
	intake.brake();
	chassis.moveToPoint(-59.394, 41.982, 5000);
	intake.move_voltage(12000);
	pros::delay(2000);
	intake.brake();
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
	pneumatics_1.set_value(true);
	pneumatics_2.set_value(true);
	while (true)
	{
		
		
		
		int dir = controller.get_analog(ANALOG_LEFT_Y);
		int turn = controller.get_analog(ANALOG_RIGHT_X);
		bool in = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		bool out = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
		bool Y = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
		bool A = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
		leftMotors.move(dir + turn);	
		rightMotors.move(dir - turn); 
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
		{
			pneumatics_1.set_value(false);
			pneumatics_2.set_value(false);
		}
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
		{
			pneumatics_1.set_value(true);
			pneumatics_2.set_value(true);
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

		if (Y)
		{
			// 필요: 아래 B안 참고해서 모터 중복 제어 피하기
			intake_2.move_voltage(12000);
		}
		else if (A)
		{
			intake_2.move_voltage(-12000);
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

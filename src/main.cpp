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

pros::MotorGroup leftMotors({1, 2});
pros::MotorGroup rightMotors({-19, -20});

pros::Motor intake_1(18);
pros::Motor intake_2(17);

pros::MotorGroup intake({18, -17});

pros::Motor outtake(3);

pros::Imu imu(5); 
pros::Rotation vertical_rotation_sensor(-15);
pros::Rotation horizontal_rotation_sensor(-16); // should be change later port3 -> port 16
pros::adi::DigitalOut pneumatics_1 ('A');
pros::adi::DigitalOut pneumatics_2 ('B');

lemlib::TrackingWheel vertical(
    &vertical_rotation_sensor,
    lemlib::Omniwheel::NEW_2,
    -5.91	// Don't change!
);

lemlib::TrackingWheel horizontal(
    &horizontal_rotation_sensor,
    lemlib::Omniwheel::NEW_2,
    0	// Don't change!
);


lemlib::Drivetrain drivetrain(&rightMotors,		    // left motor group
					&leftMotors,		    // right motor group
					12.5,				    // 12.5 inch track width
					lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
					360,				    // drivetrain rpm is 500
					2				    // horizontal drift is 2. If we had traction wheels, it would have been 8
);

lemlib::ControllerSettings linearController(
    //      0, 10, 10,   // kP, kI, kD
    14, 0, 52, // kP, kI, kD
    0,	   // anti-windup
    1, 100,	   // smallError,  smallErrorTimeout(ms)
    3, 500,	   // largeError,  largeErrorTimeout(ms)
    30	   // slew rate
);

lemlib::ControllerSettings angularController(
	2.0, 0, 60.0,
	0,
	1, 100,
	3, 500,
	40);

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

LV_IMAGE_DECLARE(JSOC);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	// imu.reset();
	chassis.calibrate();
	pros::lcd::set_text(1, "Skills code initialize!");

	pros::Task screenTask([&]() -> void {
		while(true){
			pros::lcd::print(0, "X: %f",chassis.getPose().x);
			pros::lcd::print(1, "Y: %f",chassis.getPose().y);
			pros::lcd::print(2," Theta: %f",chassis.getPose().theta);
			pros::lcd::print(3, "IMU heading: %.1f", imu.get_heading());
			pros::lcd::print(4, "Pose theta: %.1f", chassis.getPose().theta);
			lemlib::telemetrySink() -> info("Chassis pose: {}", chassis.getPose());

			pros::delay(50);
		}
	});

	/* =================== Lv Screen ICON =================== */

	lv_obj_t *img = lv_image_create(lv_screen_active());
	lv_image_set_src(img, &JSOC);

	lv_coord_t sw = lv_display_get_horizontal_resolution(NULL);
	lv_coord_t sh = lv_display_get_vertical_resolution(NULL);

	lv_obj_set_size(img, 100, 100);
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

	chassis.moveToPoint(0, 0, 5000);
	chassis.moveToPoint(-11.592, 41.077, 5000);
	chassis.moveToPoint(-12.6, 19.656, 5000);
	chassis.moveToPoint(-33.769, 18.396, 5000);
	chassis.moveToPoint(-33.265, 37.297, 5000);
	chassis.moveToPoint(-18.144, 38.557, 5000);
	chassis.moveToPoint(33.517, 40.321, 5000);
	chassis.moveToPoint(59.473, 36.289, 5000);


	// chassis.moveToPoint(10,20,5000);
	// chassis.moveToPose(10,20,90,5000);
	// chassis.moveToPoint(0,0,5000, {.forwards = false,});

	// chassis.moveToPoint(10,10, 5000);
	// chassis.moveToPoint(10,20, 5000);
	// chassis.moveToPoint(0,0, 5000, {.forwards = false,});



	// chassis.follow(proto_txt, 15, 2000);
	// * remember * //
	// chassis.moveToPoint(0, 0, 5000);
	// chassis.moveToPoint(0,20, 15000);
	// chassis.moveToPoint(5, 25, 5000);
	// chassis.turnToHeading(90, 3000);
	// chassis.moveToPose(30,0,90,5000);
	// intake.move_voltage(12000);
	// outtake.move_voltage(12000);
	// pros::delay(8000);
	// intake.brake();
	// outtake.brake();
	// chassis.moveToPoint(-12.95, 31, 5000);
	// chassis.moveToPoint(-1.494, 55.768, 5000);
	// * don't remove *//

	// chassis.follow(auton_new_txt, 10, 5000, true, false);

	// chassis.setPose(66.867, -13.119, 75.692);
	// chassis.follow(decoder["Path"], 15, 2000, true, false); // blocking

	// chassis.moveTo(0, 0, 5000);
	// chassis.moveTo(-11.132, 27.417, 5000);
	// chassis.moveTo(-1.338, 52.499, 5000);



	// chassis.moveToPoint(0, 0, 5000);
	// chassis.moveToPoint(-11.95, 30.623, 5000);
	// chassis.moveToPoint(-2.988, 52.78, 5000);


	// chassis.moveToPoint(0, 0, 5000);
	// chassis.moveToPoint(-4.422, 30.146, 5000);

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
		}
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
		{
			pneumatics_1.set_value(true);
		}
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		{
			pneumatics_2.set_value(false);
		}
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
		{
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

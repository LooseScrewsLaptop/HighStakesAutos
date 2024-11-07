#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "robodash/api.h"
#include "autos.h"

pros::MotorGroup left_motors({-20, -19, -18}); 
pros::MotorGroup right_motors({11, 12, 13}); 
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor intake(10);
pros::Motor chain(9);
pros::adi::DigitalOut mogo('E');

pros::Optical color_sensor(1); 
pros::adi::DigitalOut wall('D');
pros::adi::DigitalOut jaw('A');

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
pros::Imu imu(17);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

rd::Selector selector({
   {"Left Red", &our_autonomous},
   {"Skills", &our_skills},
   {"RedRight", &our_redRight},
   {"soloAWP", &our_soloAWP}
   //{"Auton 1", &simple_auton},
   //{"Skills Run", &skills}
});

rd::Console console;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
   console.println("Initializing robot...");
   // Robot stuff would happen...
  chassis.calibrate();
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
   console.println("Running auton...");
   selector.run_auton();
}

void our_autonomous() {
  //chassis.calibrate();
	// set position to x:0, y:0, heading:0
  chassis.setPose(0, 0, 0);
  wall.set_value(true);
  pros::delay(300);
  wall.set_value(false);
  jaw.set_value(true);

  //chassis.moveToPoint(-2, -12, 4000, {.forwards = false, .maxSpeed = 40},false);

  chain.move(127);
  pros::delay(500);
  chain.move(0);

  chassis.moveToPoint(5, 30, 100, {.forwards = false, .maxSpeed = 40},false);
  

/*
    chassis.moveToPoint(17, -29, 4000, {.forwards = false, .maxSpeed = 40},false);
    //picks up mogo
    mogo.set_value(true);
    //goes to tthe four stack of discs
    chassis.moveToPoint(-1, -40, 4000, {.maxSpeed = 40},true);
    pros::delay(1000);
    //Intakes that disc
    intake.move(-127);
    //pros::delay(2000);
    chain.move(127);
    //pros::delay(2200);
    //chain.move(0); 

    //moves to grab the other dics
    chassis.moveToPoint(-5, -29, 4000, {.maxSpeed = 40},false);
    pros::delay(1000);
    //drops goal
    

    //
    //chassis.moveToPoint(-35, 0, 4000, {.forwards = false, .maxSpeed = 40},false);

    //chassis.moveToPoint(-36, 3, 4000, {.forwards = false, .maxSpeed = 65},false);
    //mogo.set_value(true);
    
    //chassis.turnToPoint(-37, 11, 4000, {.forwards = false, .maxSpeed = 50},false);
    //chassis.turnToPoint(-34, 11, 10, {.forwards = false, .maxSpeed = 50},false);

    //chassis.setPose(0,0,0);

    //chassis.turnToPoint(0, 0, 100000, {.forwards = false, .maxSpeed = 30},false);
    //printf("Test");
    //chain.move(127);
    //pros::delay(1300);
   

    chassis.moveToPoint(16, -39, 4000, {.maxSpeed = 65},false);
    chain.move(0);
    intake.move(0);


    //Do an intake spin and chain spin
    //chassis.moveToPoint(-38, -33, 4000, {.forwards = false, .maxSpeed = 50},false);
*/
	
}
void our_skills(){
  //chassis.calibrate();
  chassis.setPose(0, 0, 0);
  wall.set_value(true);
  pros::delay(300);
  chain.move(127);
  pros::delay(350);
  chain.move(0);
  wall.set_value(false);
  jaw.set_value(true);
  pros::delay(100);



  chassis.moveToPoint(0, 10, 5000, { .maxSpeed = 40},false);

  chassis.moveToPoint(29, 13, 5000, {.forwards = false, .maxSpeed = 40},false);
  mogo.set_value(true);
  intake.move(-127);
  chain.move(127);
  chassis.moveToPoint(30, 30, 5000, { .maxSpeed = 40},false);
  chassis.moveToPoint(50, 30, 5000, { .maxSpeed = 40},false);
  //chassis.moveToPoint(30, 30, 5000, { .maxSpeed = 40},false);
  //chassis.moveToPoint(60, 13, 5000, { .maxSpeed = 40},false);
  chassis.moveToPoint(50, 11, 5000, { .maxSpeed = 40},false);
  chassis.moveToPoint(45, 14, 5000, { .maxSpeed = 40},false);
  chassis.moveToPoint(140, -40, 5000, { .forwards =false, .maxSpeed = 40},false);
  mogo.set_value(false);
  chassis.moveToPoint(30, 30, 5000, { .maxSpeed = 40},false);
  //chassis.moveToPoint(65, 3, 5000, {.forwards = false, .maxSpeed = 40},false);
  
  //OTHER SIDE

  chassis.moveToPoint(0, 0, 5000, {.forwards = false, .maxSpeed = 40},false);
  chassis.moveToPoint(-32, 14, 5000, {.forwards = false, .maxSpeed = 40},false);
  mogo.set_value(true);
  intake.move(-127);
  chain.move(127);
  chassis.moveToPoint(-30, 30, 5000, { .maxSpeed = 40},false);
  chassis.moveToPoint(-50, 34, 5000, { .maxSpeed = 40},false);
  chassis.moveToPoint(-50, 11, 5000, { .maxSpeed = 40},false);
  chassis.moveToPoint(-45, 14, 5000, { .maxSpeed = 40},false);
  chassis.moveToPoint(-140, -90, 5000, { .forwards =false, .maxSpeed = 40},false);
  mogo.set_value(false);
  chassis.moveToPoint(-45, 14, 5000, { .maxSpeed = 40},false);

  //chassis.moveToPoint(65, 3, 5000, {.forwards = false, .maxSpeed = 40},false);
 

  //chassis.moveToPoint(60, 7, 5000, {.forwards = false, .maxSpeed = 40},false);

}
void our_redRight(){
  chassis.setPose(0, 0, 0);
  wall.set_value(true);
  pros::delay(300);
  wall.set_value(false);
  jaw.set_value(true);

  chassis.moveToPoint(-6, -30, 5000, { .forwards = false, .maxSpeed = 40},false);
  //pros::delay(100);
  //chassis.moveToPoint(9, -41, 5000, { .forwards = false, .maxSpeed = 65},false);
  mogo.set_value(true);
  chain.move(127);
  pros::delay(300);
  chain.move(0);

  chassis.moveToPoint(-26, -35, 5000, { .maxSpeed = 65},false);

  intake.move(-127);
  pros::delay(1000);
  intake.move(0);
  chain.move(127);
  pros::delay(1000);
  chain.move(0);
  mogo.set_value(false);

  chassis.moveToPoint(-26, -45, 5000, { .forwards = false, .maxSpeed = 65},false);
  //intake.move(-127);
  //pros::delay(300);
  //intake.move(0);
  //chain.move(-127);
  //pros::delay(300);
  //chain.move(0);

  //intake.move(-127);
  //pros::delay(2000);
  //intake.move(0);
  //chassis.moveToPoint(15, 41, 5000, {.forwards = false, .maxSpeed = 40},false);

}
void our_soloAWP(){
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, -19, 5000, { .forwards = false, .maxSpeed = 65},false);
  chassis.turnToHeading( 90.0,  1000);
  chassis.moveToPoint(-5, -21, 5000, { .forwards = false, .maxSpeed = 65},false);
  wall.set_value(true);
  chain.move(127);
  pros::delay(500);
  wall.set_value(false);
  jaw.set_value(true);
  chain.move(0);
  chassis.moveToPoint(5, -19, 5000, { .forwards = true, .maxSpeed = 65},false);
  chassis.moveToPoint(20, 5, 5000, { .forwards = false, .maxSpeed = 65},false);
  chassis.moveToPoint(30, 7, 5000, { .forwards = false, .maxSpeed = 40},false);
  mogo.set_value(true);
  intake.move(-127);
  pros::delay(300);
  chain.move(127);
  pros::delay(300);
  chassis.moveToPoint(29, 20, 5000, { .forwards = true, .maxSpeed = 65},false);
  pros::delay(500);
  chassis.moveToPoint(40, 23, 5000, { .forwards = true, .maxSpeed = 65},false);
  pros::delay(500);
  chassis.turnToHeading(45, 1000);
  pros::delay(750);
  chassis.moveToPoint(30, -20, 5000, { .forwards = true, .maxSpeed = 65},false);
  chain.move(0);
  intake.move(0);
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
bool piston_on = false; 
bool wpiston_on = true; 
bool jawon = true;

void opcontrol() {
	while (true){
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		chassis.tank(leftY,rightY);

        pros::c::optical_rgb_s_t rgb = color_sensor.get_rgb();
        int red = rgb.red;
        int blue = rgb.blue;
        color_sensor.set_led_pwm(70);
        // Adjust thresholds according to your specific sensor readings
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
          // Toggle pneumatic state
          jawon = !jawon;

          // Set pneumatic based on state
          jaw.set_value(jawon);
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
          // Toggle pneumatic state
          jawon = !jawon;

          // Set pneumatic based on state
          jaw.set_value(jawon);
        }

        


		//CHAIN
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) ) {
		chain.move(127); 
		intake.move(-127);
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			chain.move(45); 
			intake.move(-127);
		}
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		chain.move(-127);
		intake.move(127);
		}
		else {
		chain.move(0);
		intake.move(0);
		}

		//MOGO
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
				// Toggle pneumatic state
				piston_on = !piston_on;

				// Set pneumatic based on state
				mogo.set_value(piston_on);
			}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
				// Toggle pneumatic state
				piston_on = !piston_on;

				// Set pneumatic based on state
				mogo.set_value(piston_on);
			}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
				// Toggle pneumatic state
				wpiston_on = !wpiston_on;

				// Set pneumatic based on state
				wall.set_value(wpiston_on);
			}
    pros::delay(20);
	}
}

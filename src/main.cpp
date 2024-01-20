#include "main.h"
#include "lemlib/api.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor lF(-1);
pros::Motor lM(-9);
pros::Motor lB(-10);
pros::Motor rF(11);
pros::Motor rM(19);
pros::Motor rB(18);

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB});  
pros::MotorGroup rightMotors({rF, rM, rB}); 

//wingpiston
pros::ADIDigitalOut wingPiston('H');

//ratchet
pros::ADIDigitalOut ratchet('G');

// other motors
pros::Motor intake(2);
pros::Motor arm(8);

///////////

// Inertial Sensor
pros::Imu imu(14);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,  // left motor group
                              &rightMotors, // right motor group
                              12.5,         // 10 inch track width
                              lemlib::Omniwheel::NEW_4,
                              280, // drivetrain rpm is 360
                              2    // chase power is 2. If we had traction wheels, it would have been 8
);

lemlib::ControllerSettings linearController(10,  // proportional gain (kP) 10
                                            0,   // integral gain (kI)
                                            30,  // derivative gain (kD) 30
                                            3,   // anti windup
                                            1,   // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3,   // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20   // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2,   // proportional gain (kP)
                                             0,   // integral gain (kI)
                                             10,  // derivative gain (kD)
                                             3,   // anti windup
                                             1,   // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3,   // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0    // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu     // nullptr//// inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate();     // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
// '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */

// Declare paths
ASSET(path1_txt);

void autonomous()
{

//arm go up.
    arm.move_voltage(-12000);
    pros::delay(750);
    arm.brake();
    //alliance ball inital

    chassis.setPose(38.719, -60.963,0);
    chassis.moveToPoint(57.983, -28.608, 1000,true);
    chassis.moveToPoint(38.719, -60.963, 2000,true);
    chassis.turnTo(23.393, -9.238,2000);



//
   // chassis.setPose(38.719, -60.963,0);     maybe 
    chassis.moveToPoint(23.393, -9.238, 2000,true);
    //intake ball 1
    intake.move_voltage(-12000);
    pros::delay(1000);

    chassis.moveToPoint(45, -9.238, 1000,true);
    intake.brake();
 //bal1 1 done 
 chassis.moveToPoint(23.393, -9.238, 2000,false);
    chassis.turnTo(4.768, -0.511, 2000,true);
    chassis.moveToPoint(3.3, -4.5, 2000,true); //mod 10.3, -6.5 almost 5.5,-4.5
    //intake ball 2
    intake.move_voltage(-12000);
    pros::delay(1000);

    chassis.moveToPoint(42, -9, 2000,true);
    intake.brake();
//ball 2 done
chassis.moveToPoint(23.713, -22.861, 2000,false);
chassis.turnTo(5.5, -27.65, 2000,true);
chassis.moveToPoint(5.5, -27.65, 2000,true); //mod 10.5, -22.65  //8.5 almost,-27.65
    //intake ball 3
    intake.move_voltage(-12000);
    pros::delay(1750);

    chassis.moveToPoint(42, -9, 2000,true);
    intake.brake();
//ball 3 done


 



 



 
// chassis.setPose(36.484, -60.431, 0);
// chassis.moveToPoint(23.5, -8.919, 1500, true);
// chassis.turnTo(40.637,-8.919, 1500, true);
// chassis.moveToPoint(40.637, -8.919, 1500, true);
// chassis.moveToPoint(9.877, -8.919, 1500, false);
// chassis.turnTo(4.342, 0, 1500, true);

// chassis.moveToPoint(39.251, 0.5, 1500, true);

// chassis.moveToPoint(12.644, -17.8, 1500, true);
// chassis.moveToPoint(40.637, -8.919, 1500, true);


///first three cubes

// chassis.moveToPoint(40.1, -52.2, 1500, true);
// chassis.moveToPoint(7.535, -51.728, 1500, true);
// chassis.moveToPoint(48.83, -52.302, 1500, false);

// wingPiston.set_value(true);
// pros::delay(500);

// chassis.turnTo(-55.854,-37.655, 1000, true);
// chassis.moveToPoint(-55.854, -37.655, 1000, true);

 
   
}

/**
 * Runs in driver control
 */
void opcontrol(){
    bool value = true;
    bool value2 = true;
    // controller
    // loop to continuously update motors
    while (true)
    {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intake.move_voltage(12000);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intake.move_voltage(-12000);
        }
        else
        {
            intake.brake();
        }
         // arm
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            arm.move_voltage(12000);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            arm.move_voltage(-12000);
        }
        else
        {
            arm.brake();
        }

        //ratchet
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){ 
            ratchet.set_value(value2);
            value2 = !value2;
         } 
        
        //wingpiston
        
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){ 
            wingPiston.set_value(value);
            value = !value;
         } 
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(10);
    }
}
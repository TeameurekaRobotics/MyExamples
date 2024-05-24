package org.firstinspires.ftc.teamcode.ExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of  running a motor based on encoder counts.
 * Hardware configuration will need to match your robots
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Example RunMotor By Encoder", group="Examples")
@Disabled
public class ExampleRunMotorByEncoder extends LinearOpMode {

    //reset runtime counter
    private ElapsedTime runtime = new ElapsedTime();

    //  DON'T FORGET TO RENAME HARDWARE CONFIG FILE NAME HERE!!!!!!
    TestBotHardwareSetup robot = new TestBotHardwareSetup(); //set up remote to robot hardware configuration

    //constructor
    public ExampleRunMotorByEncoder() {}

    //variables for determining distance travels per encoder 'clicks' for given wheel diameter
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder is 1440, AndyMark NevaRest encoders is 1120
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP >1.0 if geared DOWN. If no gearing set to 1.0
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    //Additional variables
    int     armHoldPosition;             // reading of arm position when buttons released to hold
    double  slopeVal         = 2000.0;   // increase or decrease to perfect holding power

    // set positions to drive motor to. Used as target values below
    int positionUp = 750;
    int positionDown = 0;

    // Wait for the game to start (driver presses PLAY)

    @Override
    public  void runOpMode() {
        robot.init(hardwareMap);
        //Reset Encoders
        robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //and hold motorArm in its current position
        armHoldPosition = robot.motorArm.getCurrentPosition();
        robot.motorArm.setPower((double) (armHoldPosition - robot.motorArm.getCurrentPosition()) / slopeVal);
        //display current status
        telemetry.addData("Status", "Encoders reset");
        telemetry.addData("current position", robot.motorArm.getCurrentPosition());    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /************************
         * TeleOp Code Below://
         *************************/
        //move are UP
        if(gamepad2.y) {
            int target =  positionUp;
            robot.motorArm.setTargetPosition(target); // set desired target (determined above)
            robot.motorArm.setPower(1.0); // set motor power
            robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // run motor to that position
            armHoldPosition = target; // update hold to current position
        }

        // move arm DOWN
        else if(gamepad2.b) {
            int target =  positionDown;
            robot.motorArm.setTargetPosition(target); // set desired target (determined above)
            robot.motorArm.setPower(-0.5); // set motor power
            robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // run motor to that position
            armHoldPosition = target; // update hold to current position
        }

        // display arm position while waiting
        if (robot.motorArm.isBusy()) {
            // Send telemetry message to indicate successful Encoder reset to zero
            telemetry.addData("moving to target", "at %7d :%7d", robot.motorArm.getCurrentPosition());
            telemetry.update();
        }
        else {
            // set to hold that position
            robot.motorArm.setPower((double) (armHoldPosition - robot.motorArm.getCurrentPosition()) / slopeVal);
        }


    }//runOpMode
}//MainClass

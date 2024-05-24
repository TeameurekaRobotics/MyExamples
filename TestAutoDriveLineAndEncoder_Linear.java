/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.ExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file illustrates the concept using both line sensing and motor encoder driving
 * together as a single autonomous OpMode
 * The code is structured as a LinearOpMode
 *  SETTINGS:
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on and off the white line and note the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 *   Setting the correct values for wheel diameter, encoder counts
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="TestAuto ODS & Encoder", group="Examples")
@Disabled

public class TestAutoDriveLineAndEncoder_Linear extends LinearOpMode {

    //reset runtime counter
    private ElapsedTime runtime = new ElapsedTime();
    /* //////////
    //Declare OpMode members.
    *////////////

    //motors
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorArm = null;

    //servos
    Servo armServo = null;
    Servo servoHandL = null;
    Servo servoHandR = null;

    //sensors
    OpticalDistanceSensor lightSensor;   //  Modern Robotics ODS sensor

    //light sensor variables
    static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    static final double     APPROACH_SPEED  = 0.5;  // adjust to desired motor speed ( 0.0 - 1.0 )
    //encoder variables
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder is 1440, AndyMark NevaRest encoders is 1120
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP >1.0 if geared DOWN. If no gearing set to 1.0
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() {

       /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         *
         * You could also set up a Hardware Class and create instance here. Thus removing all this
         * hardware set up and mapping here within the program
         */
        motorLeft  = hardwareMap.dcMotor.get("motorL");
        motorRight = hardwareMap.dcMotor.get("motorR");
        motorArm = hardwareMap.dcMotor.get("motorArm");
        armServo = hardwareMap.servo.get("armServo");     //autonomous code example below uses servo for arm instead/in addition to motor
        servoHandL = hardwareMap.servo.get("servoHandL"); //assuming a pushBot configuration of two servo grippers
        servoHandR = hardwareMap.servo.get("servoHandR");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorArm.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration

        // get a reference to our Light Sensor object.
        lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Primary MR ODS sensor.

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // this is used in place of waitForStart()
        // Will display Light Level while waiting
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle();
        }

        ///
        //  Add code here for steps before white line (See example steps below)
        ///
        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout


        // Once Start is pressed the robot begins moving forward, and then enters while loop looking for a white line threshold.
        motorLeft.setPower(APPROACH_SPEED);
        motorRight.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP;
        // will continue to display Light Level values
        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Continue displaying the light level while we are looking for the line threshold value
            telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.update();
        }

        // Stop all motors
        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(500);     // pause for 1/2 sec to allow motors to fully stop

        // after driving until white line is sensed, continue with the following autonomous encoder
        // based path below.
        // these steps can be edited to meet your desired movements along with repeating the above
        // light sensor movement

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
    }

    /* MAINTAIN
 *  Code below is encoderDrive Method used in above autonomous path
 *  which performs relative movements, based on encoder counts.
 *  Note: Encoders are NOT reset as the move is based on the current position.
 *  Move will stop if any of three conditions occur:
 *  1) Move gets to the desired position
 *  2) Move runs out of time  NOTE: Ample time must be given to allow for position to be obtained
 *  3) Driver stops the opmode running.
*/
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            motorLeft.setTargetPosition(newLeftTarget);
            motorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeft.setPower(Math.abs(speed));
            motorRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLeft.isBusy() && motorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorLeft.getCurrentPosition(),
                        motorRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorLeft.setPower(0);
            motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}

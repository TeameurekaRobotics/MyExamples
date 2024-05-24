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
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two possible different light sensors:
 *   The sensor shown in this code is a MR Optical Distance Sensor (called "sensor_ods")
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on and off the white line and note the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="TestAuto Drive To Line", group="Examples")
@Disabled
public class TestAutoDriveToLine_Linear extends LinearOpMode {

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

    //variables
    static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    static final double     APPROACH_SPEED  = 0.5;  // adjust to desired motor speed ( 0.0 - 1.0 )

    @Override
    public void runOpMode() {

       /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
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
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Will display Light Level while waiting
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle();
        }

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
        sleep(1000);     // pause for 1 sec to allow motors to fully stop


        /*///////////////
        // Alternatively you could add additional code here to continue on with Autonomous actions
        *////////////////
    }
}

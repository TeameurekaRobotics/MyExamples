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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear Tele "OpMode".
 *
 * This particular OpMode just executes a basic Tank Drive, Arm and 2 Servos for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Example: TeleOp", group="Examples")  // @Autonomous(...) is the other common choice
@Disabled
public class Example_TeleOp_with_Hardware extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //motors
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorArm = null;

    //servos
    Servo servoHandL = null;
    Servo servoHandR = null;

    //Create and set default hand positions variables. To be determined based on your build
    double CLOSED = 0.2;
    double OPEN = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
         motorLeft  = hardwareMap.dcMotor.get("motorL");
         motorRight = hardwareMap.dcMotor.get("motorR");
         motorArm = hardwareMap.dcMotor.get("motorArm");
         servoHandL = hardwareMap.servo.get("servoHandL"); //assuming a pushBot configuration of two servo grippers
         servoHandR = hardwareMap.servo.get("servoHandR");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
         motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
         motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
         motorArm.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration

        //Set servo hand grippers to open position.
         servoHandL.setPosition(OPEN);
         servoHandR.setPosition(OPEN);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /************************
         * TeleOp Code Below://
         *************************/

        while (opModeIsActive()) {  // run until the end of the match (driver presses STOP)
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // tank drive set to gamepad1 joysticks
            //(note: The joystick goes negative when pushed forwards)
            motorLeft.setPower(gamepad1.left_stick_y);
            motorRight.setPower(gamepad1.right_stick_y);

            // Arm Control - Uses dual buttons to control motor direction
            if(gamepad1.right_bumper)
            {
                motorArm.setPower(-gamepad1.right_trigger); // if both Bumper + Trigger, then negative power, runs arm down
            }
            else
            {
                motorArm.setPower(gamepad1.right_trigger);  // else trigger positive value, runs arm up
            }

            //servo commands
            if(gamepad1.a) //button 'a' will open
            {
                servoHandR.setPosition(OPEN);
                servoHandL.setPosition(OPEN);
            }
            else if (gamepad1.b) //button 'b' will close
            {
                servoHandR.setPosition(CLOSED);
                servoHandL.setPosition(CLOSED);
            }


            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}

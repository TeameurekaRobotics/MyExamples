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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * An example toggle button by Stetson's sweeper button toggle program from Dan
 * Button 'a' will turn sweeper motor on. Consecutive presses will alternate direction of motor
 * Motor stays running until button 'x' is pressed.
 */

@TeleOp(name="buttonToggle", group="Examples")  // @Autonomous(...) is the other common choice
@Disabled
public class TestStetsonToggleButton extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //motors
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor sweeper = null;


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
         sweeper = hardwareMap.dcMotor.get("sweeper");


        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
         motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
         motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
         sweeper.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration

         // Declare some variables
        double  motorDirection = -1.0;   //Keeps track of the direction for the sweeper motor
        boolean buttonPressed  = false;  //Keeps track of whether the button was previously pressed or not so we know when it is released

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


//Toggle Button
            if(gamepad1.a) //button 'a' is pressed
            {
                // Only do the following if this is the first time the button is pressed
                // since the last time it was released
                if (!buttonPressed)
                {
                    //Multiplying the motorDirection by -1 will invert the value and reverse the motor direction
                    motorDirection = motorDirection * -1.0;

                    //Set button pressed to true so that we don't invert it again until the button is released and pressed again
                    buttonPressed = true;
                }
                //Set the sweeper power to whatever the motorDirection value is
                sweeper.setPower(motorDirection);
            }
            else //Button a is not currently pressed so set our variable accordingly
            {
                buttonPressed = false;
            }

            if (gamepad1.x) //button 'x' will stop sweeper and reset the motor direction to initial state
            {
                sweeper.setPower(0.0);
                motorDirection = -1.0;
            }


            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}

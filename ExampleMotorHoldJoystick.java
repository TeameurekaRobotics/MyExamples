package org.firstinspires.ftc.teamcode.ExampleCode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by hsrobotics on 1/26/2022.
 * LINEAR OpMode configuration
 * Concept Using Motor Encoder to set Arm Drive motor upper/lower limits and monitor and hold position
 */

@TeleOp(name="Example: HoldArm w/Joystick", group="Examples")  // @Autonomous(...) is the other common choice
@Disabled
public class ExampleMotorHoldJoystick extends LinearOpMode{

    ExampleHardwareSetupHolonomic bot = new ExampleHardwareSetupHolonomic(); //set up remote to robot hardware configuration

    // variables for arm limits and hold position
    // note: these can be placed in your hardwareSetup Class
    double  armMinPos        = 0.0;      // encoder position for arm at bottom
    double  armMaxPos        = 5380.0;   // encoder position for arm at top
    int     armHoldPosition;             // reading of arm position when buttons released to hold
    double  slopeVal         = 2000.0;   // increase or decrease to perfect holding power

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        /*  ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
        !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                    BUG ALERT!!!

            YOU HAVE TO SET THE HOLD MOTOR TO RUN_WITHOUT_ENCODER

                    DO THIS IF MOTOR IS MOVING UNEXPECTEDLY!!!
        !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! */
        bot.init(hardwareMap);  //Initialize hardware from the HardwareHolonomic Setup

        while(!opModeIsActive()) { // Testing out loop to monitor armPos while waiting for start

            //initialize current position of arm motor to hold at current location once started
            armHoldPosition = bot.armMotor.getCurrentPosition();

            //adds feedback telemetry to DS
            telemetry.addData("Status", "Initialized - Waiting for Start");
            telemetry.addData("armPostion: ", +bot.armMotor.getCurrentPosition());
            telemetry.addData("HoldPostion: ", +armHoldPosition);
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
        }
        runtime.reset();

        while (opModeIsActive()) {  // run until the end of the match (driver presses STOP)
            /************************
             * TeleOp Code Below://
             *************************/

            // Display running time and Encoder value
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("ArmPosition: ", +bot.armMotor.getCurrentPosition());
            telemetry.addData("HoldPostion: ", +armHoldPosition);
            telemetry.update();

            // Arm Control - Uses left_stick_y to control motor direction.
            //          Note:   joystick values are reversed to common thought
            //                  Neg value when stick up, Pos when stick down (to reverse this you'd need to make all gamepad values negative)

            // Uses Encoder values to set upper and lower limits to protect motors from over-driving lift
            // and to hold arm position on decent to account for gravitational inertia
            if (gamepad2.left_stick_y != 0) // && robot.armMotor.getCurrentPositionJ() > 0 && robot.armMotor.getCurrentPosition() < 100 //add this to check encoder within limits
            {
                bot.armMotor.setPower(gamepad2.left_stick_y); // let stick drive UP (note this is positive value on joystick)
                armHoldPosition = bot.armMotor.getCurrentPosition(); // while the lift is moving, continuously reset the arm holding position
            }

            else //joystick is released - try to maintain the current position
            {
                bot.armMotor.setPower((armHoldPosition - bot.armMotor.getCurrentPosition()) / slopeVal);   // Note depending on encoder/motor values it may be necessary to reverse sign for motor power by making neg -slopeVal
                // the difference between hold and current positions will
                // attempt to drive the motor back to be equal with holdPosition.
                // By adjusting slopeVal you can achieved perfect hold power
            }

        }//OpModeIsActive
    }//runOpMode
}//ConceptHoldArm

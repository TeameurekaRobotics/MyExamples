package org.firstinspires.ftc.teamcode.ExampleCode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by hsrobotics on 1/16/2018.
 * LINEAR OpMode configuration
 * Concept Using Motor Encoder to set Arm Drive motor upper/lower limits and monitor and hold position
 */

@TeleOp(name="Example: Concept HoldArm", group="Examples")  // @Autonomous(...) is the other common choice
@Disabled
public class ExampleMotorHold extends LinearOpMode{

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

        bot.init(hardwareMap);  //Initialize hardware from the HardwareHolonomic Setup

        while(!opModeIsActive()) { // Testing out loop to continuted update of armPos while waiting for start
            //init current position of arm motor
            armHoldPosition = bot.armMotor.getCurrentPosition();

            //adds feedback telemetry to DS
            telemetry.addData("Status", "Initialized");
            telemetry.addData("armPostion: ", +bot.armMotor.getCurrentPosition());
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
            telemetry.update();

            // Arm Control - Uses left_stick_y to control motor direction.
            //          Note:   joystick values are reversed to common thought
            //                  Neg value when stick up, Pos when stick down (to reverse this you'd need to make all gamepad values negative)

            // Uses Encoder values to set upper and lower limits to protect motors from over-driving lift
            // and to hold arm position on decent to account for gravitational inertia

            if (gamepad1.left_stick_y < 0.0 && bot.armMotor.getCurrentPosition() > armMinPos) // encoder greater that lower limit
            {
                bot.armMotor.setPower(gamepad1.left_stick_y ); // let stick drive UP (note this is positive value on joystick)
                armHoldPosition = bot.armMotor.getCurrentPosition(); // while the lift is moving, continuously reset the arm holding position
            } else if (gamepad1.left_stick_y > 0.0 && bot.armMotor.getCurrentPosition() < armMaxPos) //encoder less than Max limit
            {
                bot.armMotor.setPower(gamepad1.left_stick_y); //let stick drive DOWN (note this is negative value on joystick)
                armHoldPosition = bot.armMotor.getCurrentPosition(); // while the lift is moving, continuously reset the arm holding position
            } else //triggers are released - try to maintain the current position
            {
                bot.armMotor.setPower((double) (armHoldPosition - bot.armMotor.getCurrentPosition()) / slopeVal);   // Note that if the lift is lower than desired position,
                // the subtraction will be positive and the motor will
                // attempt to raise the lift. If it is too high it will
                // be negative and thus try to lower the lift
                // adjust slopeVal to achieved perfect hold power
            }

        }//OpModeIsActive
    }//runOpMode
}//ConceptHoldArm

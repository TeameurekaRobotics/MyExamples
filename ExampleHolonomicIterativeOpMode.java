package org.firstinspires.ftc.teamcode.ExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


/**
 *
 * This is an Iterative vs Linear program
 * for TeleOp control with a single controller
 */

/*
   Holonomic concepts from:
   http://www.vexforum.com/index.php/12370-holonomic-drives-2-0-a-video-tutorial-by-cody/0
   Robot wheel mapping:
          X FRONT X
        X           X
      X  FL       FR  X
              X
             XXX
              X
      X  BL       BR  X
        X           X
          X       X
*/
    //DON'T FORGET TO RENAME TeleOp NAME HERE IN GREEN
    //AND COMMENT OUT "Disabled" WHEN READY TO HAVE IT SHOW UP ON PHONE
@TeleOp(name = "Example: HolonomicTeleOp Iterative", group = "Examples")
@Disabled
public class ExampleHolonomicIterativeOpMode extends OpMode {

    //  DON'T FORGET TO RENAME HARDWARE CONFIG FILE NAME HERE!!!!!!
    //  ALSO, THE HOLONOMIC CALCULATIONS BELOW ASSUME THE LEFT MOTORS HAVE BEEN REVERSED
    ExampleHardwareSetupHolonomic robot     =   new ExampleHardwareSetupHolonomic();

    /**
     * Constructor
     */
    public ExampleHolonomicIterativeOpMode() { }

    @Override
    public void init() {
      /*
       * Use the hardwareMap to get the dc motors and servos by name. Note
       * that the names of the devices must match the names used when you
       * configured your robot and created the configuration file.
       */
        robot.init(hardwareMap);  //Initialize hardware from the HardwareHolonomic Class
    }

    @Override
    public void loop() {

        //THE GAMEPAD CONFIGURATION - which controls control what!
        // "Y" left stick Y controls drive Forward/Backward
        // "Z" left stick X controls Strafe Direction
        // "X" right stick X controls Spin Direction

        float Y = -gamepad1.left_stick_y;   //drives forwards and backwards (both Y & X gamepad values are set negative as input values from gamepad are reversed i.e. pressing forward give neg value)
        float Z = -gamepad1.left_stick_x;   // strafe direction (side to side)

        float X = gamepad1.right_stick_x;   // drives spin left/right

        // holonomic formulas - combines gamepad values to control direction of motor rotation. NOTE this is with Left motors REVERSED. See http://ftckey.com/programming/advanced-programming/
        float FrontLeft     = Y +  X - Z;
        float FrontRight    = Y - X + Z;
        float BackRight     = Y - X - Z;
        float BackLeft      = Y + X + Z;

        // clip the right/left values so that the values never exceed +/- 1
        FrontRight = Range.clip(FrontRight, -1, 1);
        FrontLeft = Range.clip(FrontLeft, -1, 1);
        BackLeft = Range.clip(BackLeft, -1, 1);
        BackRight = Range.clip(BackRight, -1, 1);

        // /finally, write the calculated values to the motors
        robot.motorFrontRight.setPower(FrontRight);
        robot.motorFrontLeft.setPower(FrontLeft);
        robot.motorBackLeft.setPower(BackLeft);
        robot.motorBackRight.setPower(BackRight);

      /*
       * Telemetry for debugging
       */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Joy XL YL XR",  String.format("%.2f", X) + " " + String.format("%.2f", X) + " " +  String.format("%.2f", Z));
        telemetry.addData("f left pwr",  "front left  pwr: " + String.format("%.2f", FrontLeft));
        telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FrontRight));
        telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BackRight));
        telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BackLeft));

    }

    @Override
    public void stop()
    {

    }

}
package org.firstinspires.ftc.teamcode.ExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Example Auto for holding arm while driving
 */
@Autonomous(name = "danArmHold")
@Disabled
public class danArmHold extends LinearOpMode {

  //Configure Hardware
  private Servo servoL;
  private DcMotor motorArm;
  private DcMotor motorL;
  private DcMotor motorR;
  //Create Global Variables
  int armHold;
  int slopeVal = 2000;
  ElapsedTime timer = new ElapsedTime();

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    //initialize hardware
    servoL = hardwareMap.get(Servo.class, "servoL");
    motorArm = hardwareMap.get(DcMotor.class, "motorArm");
    motorL = hardwareMap.get(DcMotor.class, "motorL");
    motorR = hardwareMap.get(DcMotor.class, "motorR");

    motorL.setDirection(DcMotorSimple.Direction.REVERSE);
    motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    servoL.setPosition(0.1);

    waitForStart();

    if (opModeIsActive()) {
      // Put run blocks here......
      // lift arm drive forward
      armMove(0.5, 1000);
      motorMoveArmHold(2000, 2000, 0.5, 0.5);
      // open servo
      ServoPosition(0.8);
      // back up
      motorMoveArmHold(-2000, -2000, -0.5, -0.5);
      // turn
      MotorTurn(2000, 0.25);
      // Speed forward
      motorMoveArmHold(4000, 4000, 1, 1);
    }
  }

  /**
   * ********************************************
   * ***BELOW ARE MOVEMENT METHODS USED ABOVE****
   * ********************************************
   */
  /**
   * Turn method, uses motorMove, but reverses the left side...
   */
  private void MotorTurn(int posMotor, double powMotor) {
    motorMoveArmHold(-posMotor, posMotor, -powMotor, powMotor);
  }

  /**
   * servo method
   */
  private void ServoPosition(double ServoPosition) {
    servoL.setPosition(ServoPosition);
    sleepAndHold(100);
  }
  /**
   * Arm Sleep/Hold method
   */
  private void sleepAndHold(int iMilliseconds) {
	  timer.reset();
	  while(opModeIsActive() && timer.milliseconds() < iMilliseconds) {
		  armHold();
	  }
  }
  /**
   * Set motorArm power based on hold position
   */
  private void armHold() {
    motorArm.setPower((armHold - motorArm.getCurrentPosition()) / slopeVal);
  }

  /**
   * Run multiple motors while holding position of motorArm
   */
  private void motorMoveArmHold(int posML, int posMR, double powML, double powMR) {
    // reset drive encoders:
    motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	
    // Set Target Positions for drive motors:
    motorL.setTargetPosition(posML);
    motorR.setTargetPosition(posMR);
	
    // Set Motors Mode and Power
    motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	
	// Start motors moving
    motorL.setPower(powML);
    motorR.setPower(powMR);
	
    // Run while motors are moving
    while (motorL.isBusy() || motorR.isBusy()) {
      // Holds Arm until other motor(s) finish
	  armHold();
      //Display all motor positions
      telemetry.addData("armPos", motorArm.getCurrentPosition());
      telemetry.addData("posMotorL", motorL.getCurrentPosition());
      telemetry.addData("PosMotorR", motorR.getCurrentPosition());
      telemetry.update();
      idle();
    }
	// STOP all drive motors
    motorL.setPower(0);
    motorR.setPower(0);
    //run the armHold method for a set period of time to make sure it doesn't drop while going to next step in program
    sleepAndHold(100);
  }
   /**
   * Run motorArm to pos @ power
   */
  private void armMove(double power, int pos) {
    motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorArm.setTargetPosition(pos);
    motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motorArm.setPower(power);
    while (motorArm.isBusy()) {
      telemetry.addData("armPos", motorArm.getCurrentPosition());
      telemetry.update();
      idle();
    }
    motorArm.setPower(0);
    // Set the arm hold position to the final position of the arm
	armHold = motorArm.getCurrentPosition();
    sleep(100);
  }
}
package edu.elon.robotics;
/**
 * A simple teleop OpMode for drive the PushBot.
 * + Arm Controls.
 *
 * @author J. Hollingsworth
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import edu.elon.robotics.RobotHardware;

@TeleOp(name = "Drive the Arm Robot", group = "TeleOp")
public class ArmBot extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);

        // move the arm to a known location
        initializeArm();

        waitForStart();

        while (opModeIsActive()) {
            stickDriving();
            controlArm();
            controlHand();

            /* telemetry */
            showDriveMotorEncoders();
            telemetry.update();
        }
    }

    private void controlArm() {
        double armPower = 0.0;

        // drive the arm up/down
        if (gamepad1.dpad_up && robot.motorArm.getCurrentPosition() < robot.ARM_MAX_HEIGHT) {
            armPower = robot.ARM_POWER_UP;
        } else if (gamepad1.dpad_down && robot.touchSensorArm.getState()) {
            armPower = robot.ARM_POWER_DOWN;
        }

        // reset the encoder when we touch the button
        if (!robot.touchSensorArm.getState()) {
            robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // set the arm power
        robot.motorArm.setPower(armPower);

        // show information to the user
        telemetry.addData("arm power", armPower);
        telemetry.addData("arm ticks", robot.motorArm.getCurrentPosition());
    }

    // starting positions
    private double gripperPos = RobotHardware.GRIPPER_FULLY_OPEN;
    private double wristPos = RobotHardware.WRIST_PICKUP_POS;

    // used for toggling
    private boolean wasAPressed = false;
    private boolean wasBPressed = false;
    private boolean wasXPressed = false;
    private boolean wasYPressed = false;

    private void controlHand() {
        // open/close the gripper
        if (gamepad1.b && !wasBPressed) {
            gripperPos += robot.GRIPPER_INCREMENT;
        } else if (gamepad1.x && !wasXPressed) {
            gripperPos -= robot.GRIPPER_INCREMENT;
        }

        // move the wrist up/down
        if (gamepad1.y && !wasYPressed) {
            wristPos += robot.WRIST_INCREMENT;
        } else if (gamepad1.a && !wasAPressed) {
            wristPos -= robot.WRIST_INCREMENT;
        }

        // remember button presses
        wasAPressed = gamepad1.a;
        wasBPressed = gamepad1.b;
        wasXPressed = gamepad1.x;
        wasYPressed = gamepad1.y;

        // limit the servo to possible gripper positions
        gripperPos = Range.clip(gripperPos, robot.GRIPPER_FULLY_CLOSED, robot.GRIPPER_FULLY_OPEN);
        wristPos = Range.clip(wristPos, robot.WRIST_FULLY_DOWN, robot.WRIST_FULLY_UP);

        // set the servo positions
        robot.servoGripper.setPosition(gripperPos);
        robot.servoWrist.setPosition(wristPos);

        // telemetry
        telemetry.addData("gripper pos", gripperPos);
        telemetry.addData("wrist pos", wristPos);
    }

    private void initializeArm() {

        // move the servo to known good positions
        robot.servoGripper.setPosition(robot.GRIPPER_FULLY_OPEN);
        robot.servoWrist.setPosition(robot.WRIST_PICKUP_POS);

        // let the servos get to their position before moving the arm
        sleep(500);

        // tell the user that the arm is initializing
        telemetry.addData("ARM", "is initializing");
        telemetry.update();

        // initialize the arm
        robot.motorArm.setPower(robot.ARM_INIT_POWER);
        while (robot.touchSensorArm.getState()) {
            telemetry.addData("moving arm", 0);
            telemetry.update();
            // do nothing -- waiting for a button press
        }
        telemetry.addData("done moving arm", 0);
        telemetry.update();
        robot.motorArm.setPower(0);

        // reset the encoder
        robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // tell the user we are done
        telemetry.addData("ARM", "initialization complete");
        telemetry.update();
    }

    /************************************************************************
     * Drive the robot using the left/right joysticks.
     ************************************************************************/
    private void stickDriving() {
        // simple tank drive controls
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        // create dead zones in case the sticks have drift
        if (Math.abs(gamepad1.left_stick_y) < robot.STICK_THRESHOLD) {
            drive = 0;
        }

        if (Math.abs(gamepad1.right_stick_x) < robot.STICK_THRESHOLD) {
            turn = 0;
        }

        robot.startMove(drive, turn, robot.DRIVE_SPEED_NORMAL);
    }

    /************************************************************************
     * Telemetry - messages that show up on the Driver Station
     ************************************************************************/
    private void showDriveMotorEncoders() {
        telemetry.addData(" motorLeft: ", robot.motorLeft.getCurrentPosition());
        telemetry.addData("motorRight: ", robot.motorRight.getCurrentPosition());
    }
}
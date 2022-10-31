package edu.elon.robotics;
/**
 * A simple teleop OpMode for drive the PushBot.
 *
 * @author J. Hollingsworth
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive Robot", group = "TeleOp")
public class Teleop extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            stickDriving();

            /* telemetry */
            showDriveMotorEncoders();
            telemetry.update();
        }

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
        telemetry.addData("motorLeft: ", robot.motorLeft.getCurrentPosition());
        telemetry.addData("motorRight: ", robot.motorRight.getCurrentPosition());
    }
}

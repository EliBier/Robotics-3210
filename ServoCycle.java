package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import edu.elon.robotics.RobotHardware;

@TeleOp(name="Servo Cycle", group="setup")
//@Disabled
public class ServoCycle extends LinearOpMode {

    private RobotHardware robot;
    private boolean prevGamepad = false;
    private boolean prevBumper = false;
    private int which = 0;

    @Override
    public void runOpMode() {

        robot = new RobotHardware(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Servo[] servos    = { robot.servoGripper, robot.servoWrist };
        String[] names     = { "gripper", "wrist" };
        Double[] positions = { 0.5, 0.5 };

        waitForStart();

        while (opModeIsActive()) {
            // servo mod
            if (gamepad1.a && !prevGamepad) positions[which] += .05;
            if (gamepad1.b && !prevGamepad) positions[which] -= .05;
            if (gamepad1.x && !prevGamepad) positions[which] += .01;
            if (gamepad1.y && !prevGamepad) positions[which] -= .01;
            prevGamepad = gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y;

            if (gamepad1.left_bumper && !prevBumper) which--;
            if (which < 0) which = servos.length - 1;
            if (gamepad1.right_bumper && !prevBumper) which++;
            if (which > servos.length - 1) which = 0;
            prevBumper = gamepad1.left_bumper || gamepad1.right_bumper;

            // modify range
            positions[which] = Range.clip(positions[which], 0, 1);

            // set position
            servos[which].setPosition(positions[which]);

            // add telemetry
            telemetry.addLine(names[which]);
            telemetry.addData("Position", positions[which]);
            telemetry.update();
        }
    }
}
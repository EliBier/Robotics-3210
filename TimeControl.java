package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TimeControl", group = "tabs")

public class TimeControl extends AutoCommon {
    protected void driveForTime(double power, long milliseconds) {
        // start the drive motors
        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(power);

        // wait for some time
        sleep(milliseconds);

        //stop the drive motors
        robot.motorLeft.setPower(0);
        robot.motorRight.setPower(0);
    }

    protected void turnForTime(double power, long milliseconds) {
        // start the drive motors
        robot.motorLeft.setPower(-power);
        robot.motorRight.setPower(power);

        // wait for some time
        sleep(milliseconds);

        // stop the drive motors
        robot.motorLeft.setPower(0);
        robot.motorRight.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        double cmPer1000Millisecond = 1782;
        double degreesPer1000Millisecond = 48.648;

        driveForTime(0.25,8403);
        driveForTime(-0.25,2801);
        turnForTime(0.25,1725);
        driveForTime(0.25,5602);
        turnForTime(-0.25,1341);
        driveForTime(-0.25,5961);
        turnForTime(0.25,4691);
        driveForTime(0.25,3563);
        turnForTime(0.25,1725);

    }
}
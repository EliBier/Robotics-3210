package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CountLines", group = "tabs")

public class CountLines extends IMUControl {

    protected void driveUntilTouch(double power){
        while(robot.touchSensor.getState() == true) drive(power, 1);
        robot.stopDriveMotors();
    }

    protected void driveToCalibrateLightSensor() {
        double cm = 20;
        double power = 0.5;
        maxBrightness = robot.colorSensor.alpha();
        minBrightness = robot.colorSensor.alpha();

        // convert cm to encoder ticks (make sure cm is not negative)
        int distance_in_ticks = robot.convertDistanceToTicks(Math.abs(cm));
        // reset the drive encoders (this sets the encoder value to 0)
        robot.resetDriveEncoders();

        // start both drive motors at the same given power
        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(power);

        // Loop until one of the encoders passes the number of ticks for the given cm
        while (Math.abs(robot.motorLeft.getCurrentPosition()) <= distance_in_ticks && opModeIsActive()) {
            if (robot.colorSensor.alpha() > maxBrightness) maxBrightness = robot.colorSensor.alpha();
            if (robot.colorSensor.alpha() < minBrightness) minBrightness = robot.colorSensor.alpha();
        }

        // stop both drive motors
        robot.stopDriveMotors();
    }
    // This is the function that we used for lab 6 to count the number of lines that the robot drives over until it hits an object.
    protected void countLinesTillWall(double power) {
        int whiteLines = 0;
        boolean lineInProgress = false;
        robot.resetDriveEncoders();
        driveToCalibrateLightSensor();

        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(power);

        while(robot.touchSensor.getState() == true) {
            telemetry.addData("White Lines: ", whiteLines);
            telemetry.addData("Alpha: ", robot.colorSensor.alpha());
            telemetry.addData("maxBright: ", maxBrightness);
            telemetry.addData("minBright: ", minBrightness);
            telemetry.update();
            if (robot.colorSensor.alpha() > (maxBrightness+minBrightness)/2 && !lineInProgress) {
                whiteLines = whiteLines + 1;
                lineInProgress = true;
            }
            if (robot.colorSensor.alpha() < (maxBrightness+minBrightness)/2 && lineInProgress) lineInProgress = false;
        }
        robot.stopDriveMotors();
        int ticksTraveled = robot.motorLeft.getCurrentPosition();
        driveIMU(power, -robot.convertTicksToDistance(ticksTraveled)+2);
        telemetry.addData("White Lines: ", whiteLines);
        telemetry.addData("Distance Traveled: ", robot.convertTicksToDistance(ticksTraveled));
        telemetry.update();

        sleep(100000);

    }
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        driveToCalibrateLightSensor();
        countLinesTillWall(0.05);
    }
}
package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TestingLoop", group = "tabs")

public class TestingLoop extends AutoCommon {

    // Constants
    private final int RAMPTICKS = 500;
    private final double BASEPOWER = 0.15;

    // Global Variables
    protected int maxBrightness;
    protected int minBrightness;
    protected double middleBrightness;

    // P Control Variables
    protected double P_LINE_KP = 5.5*Math.pow(10,-5);

    // PID Control Variables
    // PC ~ 2.75
    protected double LINE_KP = 5*Math.pow(10,-5);
    protected double LINE_KI = 3.5*Math.pow(10,-6);
    protected double LINE_KD = 5*Math.pow(10,-7);

    protected double sumError;
    protected double prevError = 0;



    protected void turnToCalibrateLightSensor() {
        int turns[] = {45, -90, 45};
        double power = 0.25;
        maxBrightness = robot.colorSensor.alpha();
        minBrightness = robot.colorSensor.alpha();

        // convert cm to encoder ticks (make sure cm is not negative)
        for (int degrees : turns) {

            int arc_length_in_ticks = robot.convertDegreeToTicks(Math.abs(degrees));
            robot.resetDriveEncoders();

            power = Math.abs(power) * Math.signum(degrees);
            // start both drive motors at the same given power
            robot.motorLeft.setPower(-power);
            robot.motorRight.setPower(power);

            // Loop until one of the encoders passes the number of ticks for the given cm
            while (Math.abs(robot.motorLeft.getCurrentPosition()) <= arc_length_in_ticks && opModeIsActive()) {
                if (robot.colorSensor.alpha() > maxBrightness)
                    maxBrightness = robot.colorSensor.alpha();
                if (robot.colorSensor.alpha() < minBrightness)
                    minBrightness = robot.colorSensor.alpha();
            }
            robot.stopDriveMotors();
            telemetry.addData("maxBright: ", maxBrightness);
            telemetry.addData("minBright: ", minBrightness);
            telemetry.update();
            sleep(300);

            // stop both drive motors
            robot.stopDriveMotors();
            middleBrightness = (minBrightness + maxBrightness)/2.0;
        }
    }

    protected void LineFollowPControl(double power, int cm, int lineSide) {
        telemetry.addData("maxBright: ", maxBrightness);
        telemetry.addData("minBright: ", minBrightness);
        telemetry.update();


        // convert cm to encoder ticks (make sure cm is not negative)
        int distance_in_ticks = robot.convertDistanceToTicks(Math.abs(cm));
        // reset the drive encoders (this sets the encoder value to 0)
        robot.resetDriveEncoders();
        resetAngle();

        double powerOffset;
        double error;

        // sets the requested power and curPower to be correct sign depending on whether the sign of cm is positive or negative
        double reqPower = Math.abs(power) * Math.signum(cm);
        double curPower = BASEPOWER * Math.signum(cm);
        if (distance_in_ticks < RAMPTICKS) {
            drive(BASEPOWER, cm);
        } else {
            int curTicks = robot.motorLeft.getCurrentPosition();

            // Initializes the curTicks to the current position of the encoder
            while (curTicks < RAMPTICKS && Math.abs(robot.motorLeft.getCurrentPosition()) <= distance_in_ticks && opModeIsActive()) {
                error = middleBrightness - robot.colorSensor.alpha();
                powerOffset = (P_LINE_KP * error) * lineSide;


                robot.motorLeft.setPower(curPower + powerOffset);
                robot.motorRight.setPower(curPower - powerOffset);

                curPower = Math.signum(cm) * (BASEPOWER + ((double) curTicks / RAMPTICKS * (Math.abs(reqPower) - BASEPOWER)));
                curTicks = Math.abs(robot.motorLeft.getCurrentPosition());

            }

            // Loop until one of the encoders passes the number of ticks for the given cm
            while (Math.abs(robot.motorLeft.getCurrentPosition()) <= distance_in_ticks && opModeIsActive()) {
                error = middleBrightness - robot.colorSensor.alpha();
                powerOffset = (P_LINE_KP * error) * lineSide;


                robot.motorLeft.setPower(reqPower + powerOffset);
                robot.motorRight.setPower(reqPower - powerOffset);
            }

            // stop both drive motors
            robot.stopDriveMotors();
        }
    }

    protected double PIDComputeOffset(int lineSide) {
        double error = middleBrightness - robot.colorSensor.alpha();
        sumError += error;
        double diffError = error - prevError;

        return (LINE_KP * error + LINE_KI * sumError + LINE_KD * diffError) * lineSide;
    }

    protected void LineFollowPIDControl(double power, int cm, int lineSide) {
        ElapsedTime loopTimer = new ElapsedTime();
        // convert cm to encoder ticks (make sure cm is not negative)
        int distance_in_ticks = robot.convertDistanceToTicks(Math.abs(cm));
        // reset the drive encoders (this sets the encoder value to 0)
        robot.resetDriveEncoders();
        resetAngle();

        double curHeading;
        double powerOffset;
        double error;

        // sets the requested power and curPower to be correct sign depending on whether the sign of cm is positive or negative
        double reqPower = Math.abs(power) * Math.signum(cm);
        double curPower = BASEPOWER * Math.signum(cm);
        if (distance_in_ticks < RAMPTICKS) {
            drive(BASEPOWER, cm);
        } else {

            int curTicks = robot.motorLeft.getCurrentPosition();

            // Initializes the curTicks to the current position of the encoder
            while (curTicks < RAMPTICKS && Math.abs(robot.motorLeft.getCurrentPosition()) <= distance_in_ticks && opModeIsActive()) {
                // PID loop
                loopTimer.reset();
                powerOffset = PIDComputeOffset(lineSide);


                robot.motorLeft.setPower(curPower + powerOffset);
                robot.motorRight.setPower(curPower - powerOffset);

                curPower = Math.signum(cm) * (BASEPOWER + ((double) curTicks / RAMPTICKS * (Math.abs(reqPower) - BASEPOWER)));
                curTicks = Math.abs(robot.motorLeft.getCurrentPosition());
                sleep(30-Math.round(loopTimer.milliseconds()));

            }

            // Loop until one of the encoders passes the number of ticks for the given cm
            while (Math.abs(robot.motorLeft.getCurrentPosition()) <= distance_in_ticks && opModeIsActive()) {
                loopTimer.reset();
                powerOffset = PIDComputeOffset(lineSide);


                robot.motorLeft.setPower(reqPower + powerOffset);
                robot.motorRight.setPower(reqPower - powerOffset);
                sleep(30-Math.round(loopTimer.milliseconds()));
            }

            // stop both drive motors
            robot.stopDriveMotors();
        }
    }
    int counter = 0;
    private int linePresent() {
        double alpha = robot.colorSensor.alpha();
        double red = robot.colorSensor.red();
        double blue = robot.colorSensor.blue();
        if (alpha > 400 && (red > 300 || blue > 200)) {
            // Return 0 indicates blue, return 1 indicates red, return 2 indicates orange
            // The line is either red or orange
            if (red > blue) {
                if (red > 700 || counter == 7) return 2;
                else {
                    counter += 1;
                    return 1;
                }
            } else {
                counter += 1;
                return 0;
            }
        }
        return -1;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();


        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("state: ", robot.colorSensor.red());
            telemetry.update();

        }
    }
}
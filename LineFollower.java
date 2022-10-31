package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "LineFollower", group = "tabs")

public class LineFollower extends AutoCommon {

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

    /* Decent Values
    protected double LINE_KP = 4.8*Math.pow(10,-5);
    protected double LINE_KI = 3*Math.pow(10,-6);
    protected double LINE_KD = 5.9*Math.pow(10,-7);
    */

    protected double LINE_KP = 5.6*Math.pow(10,-5);
    protected double LINE_KI = 3.7*Math.pow(10,-6);
    protected double LINE_KD = 8.0*Math.pow(10,-7);

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
                telemetry.addData("powerOffset: ", powerOffset);
                telemetry.update();

                robot.motorLeft.setPower(curPower + powerOffset);
                robot.motorRight.setPower(curPower - powerOffset);

                curPower = Math.signum(cm) * (BASEPOWER + ((double) curTicks / RAMPTICKS * (Math.abs(reqPower) - BASEPOWER)));
                curTicks = Math.abs(robot.motorLeft.getCurrentPosition());

            }

            // Loop until one of the encoders passes the number of ticks for the given cm
            while (Math.abs(robot.motorLeft.getCurrentPosition()) <= distance_in_ticks && opModeIsActive()) {
                error = middleBrightness - robot.colorSensor.alpha();
                powerOffset = (P_LINE_KP * error) * lineSide;
                telemetry.addData("powerOffset: ", powerOffset);
                telemetry.update();

                robot.motorLeft.setPower(reqPower + powerOffset);
                robot.motorRight.setPower(reqPower - powerOffset);
            }

            // stop both drive motors
            robot.stopDriveMotors();
        }
    }

    protected double PIDComputeOffset(int lineSide) {
        double error = middleBrightness - robot.colorSensor.alpha();
        sumError = 0.8*sumError+error;
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

        double powerOffset;

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
                telemetry.addData("powerOffset: ", powerOffset);
                telemetry.update();

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
                telemetry.addData("powerOffset: ", powerOffset);
                telemetry.update();

                robot.motorLeft.setPower(reqPower + powerOffset);
                robot.motorRight.setPower(reqPower - powerOffset);
                sleep(30-Math.round(loopTimer.milliseconds()));
            }

            // stop both drive motors
            robot.stopDriveMotors();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        turnToCalibrateLightSensor();
        LINE_KP = 5.6*Math.pow(10,-5);
        LINE_KI = 3.7*Math.pow(10,-6);
        LINE_KD = 8.0*Math.pow(10,-7);
        LineFollowPIDControl(0.2,1000,-1);
        LINE_KP = 5.6*Math.pow(10,-5);
        LINE_KI = 3.7*Math.pow(10,-6);
        LINE_KD = 8.0*Math.pow(10,-7);
        LineFollowPIDControl(0.15,2000, -1);

//        LINE_KP = 8.0*Math.pow(10,-5);
//        LINE_KI = 8.0*Math.pow(10,-6);
//        LINE_KD = 1.0*Math.pow(10,-5);
//        LineFollowPIDControl(0.3,200,-1);
//        robot.stopDriveMotors();
//        sleep(200);
//        LINE_KP = 5.6*Math.pow(10,-5);
//        LINE_KI = 3.7*Math.pow(10,-6);
//        LINE_KD = 8.0*Math.pow(10,-7);
//        LineFollowPIDControl(0.2,700,-1);
//        LINE_KP = 5.6*Math.pow(10,-5);
//        LINE_KI = 3.7*Math.pow(10,-6);
//        LINE_KD = 8.0*Math.pow(10,-7);
//        LineFollowPIDControl(0.15,2000,-1);
    }
}
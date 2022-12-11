package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "FinalCourse", group = "tabs")

public class FinalCourse extends AutoCommon {
    // DrivePower
    double drivePower = 0.25;
    double colorDrivePower = 0.2;
    // Constants for turnRamp and turnDrive
    private final double BASEPOWER = 0.15;
    private final int RAMPTICKS = 500;

    // Constants for turnIMU
    private final double RAMP_CUTOFF = 20.0;
    private final double ANGLE_UNDERSHOOT = 5.0;        // how many degrees to adjust the requested degree angle by
    private final double TURN_ENDING_POWER = 0.10;      // slow power of the motor for the final part of the turn
    private final double SLOW_DOWN_DEGREES = 20.0;      // number of degrees that will be done using the slow powers

    // PID variables
    private final double HEADING_KP = 0.05;

    // Global Variables
    protected int maxBrightness = 2350;
    protected int minBrightness = 450;
    protected double middleBrightness = (maxBrightness+minBrightness)/2.0;

    double armLength = 40;
    double armPivotHeight = 35.5;
    double thetaMin = 13;
    double ticksPerArmDeg = 13.8;

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

    protected void turnIMU(double power, double degrees) {
        // reset the orientation to the 0 degrees position
        System.out.println("Turning: " + degrees + " degrees");
        resetAngle();

        //
        //if (degrees + 180 < 0.001) degrees = 180;
        double direction = Math.signum(degrees);
        double degreesMagnitude = Math.abs(degrees);
        // sets the requested power and curPower to be correct sign depending on whether the sign of cm is positive or negative
        double powerMagnitude = Math.abs(power);
        double curPower = BASEPOWER * direction;


        if (degreesMagnitude <= RAMP_CUTOFF + SLOW_DOWN_DEGREES + ANGLE_UNDERSHOOT) {
            System.out.println("small turn; turning slowly");
            while (Math.abs(getHeading()) <= degreesMagnitude) {
                robot.motorLeft.setPower(-BASEPOWER * direction);
                robot.motorRight.setPower(BASEPOWER * direction);
            }

        } else {
            System.out.println("large turn, turning with ramp and slow down");
            while (Math.abs(getHeading()) < RAMP_CUTOFF) {
                robot.motorLeft.setPower(-curPower);
                robot.motorRight.setPower(curPower);

                curPower = direction * (BASEPOWER + (Math.abs(getHeading()) / RAMP_CUTOFF * (powerMagnitude - BASEPOWER)));
            }
            System.out.println("ramped to " + curPower);


            while (((getHeading() * direction > 0) ? Math.abs(getHeading()) : 360 - Math.abs(getHeading())) <= degreesMagnitude - SLOW_DOWN_DEGREES - ANGLE_UNDERSHOOT) {
            }
            System.out.println("finished constant speed turn\nsetting speed to slow down speed");

//            while (Math.abs(getHeading()) >= degreesMagnitude - SLOW_DOWN_DEGREES && (Math.abs(getHeading()) <= degreesMagnitude - ANGLE_UNDERSHOOT)) {
//                curPower = direction*(powerMagnitude - (((degreesMagnitude - Math.abs(getHeading())-ANGLE_UNDERSHOOT) / SLOW_DOWN_DEGREES) * (powerMagnitude - TURN_ENDING_POWER)));
//                System.out.println(curPower);
//                robot.motorLeft.setPower(-curPower);
//                robot.motorRight.setPower(curPower);
//            }

            robot.motorLeft.setPower(-TURN_ENDING_POWER * direction);
            robot.motorRight.setPower(TURN_ENDING_POWER * direction);
            while (((getHeading() * direction > 0) ? Math.abs(getHeading()) : 360 - Math.abs(getHeading())) < (degreesMagnitude - ANGLE_UNDERSHOOT)) {
                continue;
            }
        }
        System.out.println("stopping");
        // stop both drive motors
        robot.stopDriveMotors();
    }

    protected void driveIMU(double power, double cm) {
        // convert cm to encoder ticks (make sure cm is not negative)
        int distance_in_ticks = robot.convertDistanceToTicks(Math.abs(cm));
        // reset the drive encoders (this sets the encoder value to 0)
        robot.resetDriveEncoders();
        resetAngle();

        double curHeading;
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
                curHeading = getHeading();
                powerOffset = HEADING_KP * curHeading;

                robot.motorLeft.setPower(curPower + powerOffset);
                robot.motorRight.setPower(curPower - powerOffset);

                curPower = Math.signum(cm) * (BASEPOWER + ((double) curTicks / RAMPTICKS * (Math.abs(reqPower) - BASEPOWER)));
                curTicks = Math.abs(robot.motorLeft.getCurrentPosition());

            }

            // Loop until one of the encoders passes the number of ticks for the given cm
            while (Math.abs(robot.motorLeft.getCurrentPosition()) <= distance_in_ticks && opModeIsActive()) {
                curHeading = getHeading();
                powerOffset = HEADING_KP * curHeading;

                robot.motorLeft.setPower(reqPower + powerOffset);
                robot.motorRight.setPower(reqPower - powerOffset);
            }

            // stop both drive motors
            robot.stopDriveMotors();
        }
    }
    private int linePresent() {
        double alpha = robot.colorSensor.alpha();
        double red = robot.colorSensor.red();
        double blue = robot.colorSensor.blue();
        if (alpha > 400 && (red > 300 || blue > 200)) {
            // Return 0 indicates blue, return 1 indicates red, return 2 indicates orange
            // The line is either red or orange
            if (red > blue) {
                if (red > 700) return 2;
                else {
                    return 1;
                }
            } else {
                return 0;
            }
        }
        return -1;
    }
    private int RedBlueOrange(double red, double blue, double alpha) {
        if (alpha > 400 && (red > 300 || blue > 200)) {
            // Return 0 indicates blue, return 1 indicates red, return 2 indicates orange
            // The line is either red or orange
            if (red > blue) {
                if (red > 700) return 2;
                else {
                    return 1;
                }
            } else {
                return 0;
            }
        }
        return -1;
    }

    private int getLineColor() {
        int currentPosition = robot.motorLeft.getCurrentPosition();
        int finalPosition = currentPosition + robot.convertDistanceToTicks(2);
        moveForward(0.07);
        double maxRed = 0;
        double maxBlue = 0;
        double maxAlpha = 0;

        while (currentPosition < finalPosition) {
            maxRed = Math.max(maxRed, robot.colorSensor.red());
            maxBlue = Math.max(maxBlue, robot.colorSensor.blue());
            maxAlpha = Math.max(maxAlpha, robot.colorSensor.alpha());
            currentPosition = robot.motorLeft.getCurrentPosition();
        }
        robot.stopDriveMotors();
        sleep(200);
        drive(0.1, -2);
        return RedBlueOrange(maxRed, maxBlue, maxAlpha);
    }
    protected void sensorInformation() {
        telemetry.addData("Red: ", robot.colorSensor.red());
        telemetry.addData("Blue: ", robot.colorSensor.blue());
        telemetry.addData("Alpha: ", robot.colorSensor.alpha());
        telemetry.addData("Wall Distance: ", robot.distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    protected double P_LINE_KP = 3.0*Math.pow(10,-4);


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
            while (curTicks < RAMPTICKS && opModeIsActive()) {
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
            while (robot.touchSensor.getState() == true && opModeIsActive()) {
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

    public void setArm(double cm) {
        double armHeight = Math.acos((armPivotHeight - cm) / armLength)*180/Math.PI;
        int ticks = (int)((armHeight - thetaMin) * ticksPerArmDeg);

        if (ticks > robot.motorArm.getCurrentPosition()) {
            robot.motorArm.setPower(robot.ARM_POWER_UP);
            while (robot.motorArm.getCurrentPosition() < ticks && robot.motorArm.getCurrentPosition() < robot.ARM_MAX_HEIGHT) {
            }
        } else if (ticks < robot.motorArm.getCurrentPosition()) {
            robot.motorArm.setPower(robot.ARM_POWER_DOWN);
            while (robot.motorArm.getCurrentPosition() > ticks && robot.touchSensorArm.getState()) {
            }
        }
        robot.motorArm.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();
        waitForStart();
        initializeArm();

        boolean foundLine = false;
        int lineColor = -1;
        // move past calibration lines
        drive(drivePower,50);
        // start moving forward until we hit colored lines
        moveForward(colorDrivePower);

        // Section 1 color turning
        boolean sectionOne = true;
        while(opModeIsActive() && sectionOne) {
            if (!foundLine) {
                lineColor = linePresent();
                if (lineColor != -1) {
                    lineColor = getLineColor();
                    robot.stopDriveMotors();
                    sleep(200);
                    foundLine = true;
                    if (lineColor == 0) {
                        turnIMU(drivePower, 93);
                        sleep(200);
                    } else if (lineColor == 1) {
                        turnIMU(drivePower, -93);
                        sleep(200);
                    } else {
                        sectionOne = false;
                    }
                }
            }
            else {
                moveForward(colorDrivePower);
                foundLine = false;
            }
        }
        robot.stopDriveMotors();


        // Section 2 keep distance from wall
        sensorInformation();
        // HARD CODED WALL_DIST
        double WALL_DIST = 50.0;
        // AUTO WALL_DIST
        // double WALL_DIST = robot.distanceSensor.getDistance(DistanceUnit.CM);
        double wallError = -WALL_DIST + robot.distanceSensor.getDistance(DistanceUnit.CM);
        double kP = 0.03;
        double powerAdjust = kP * wallError;
        double wallFollowPower = 0.2;
        drive(drivePower,10);
        robot.stopDriveMotors();
        sleep(400);
        moveForward(wallFollowPower);

        // p-controller loop
        boolean sectionTwo = true;
        while (sectionTwo & opModeIsActive()) {
            if (linePresent() > -1) {
                sectionTwo = false;
                robot.stopDriveMotors();
                break;
            }
//            p-control
            wallError = -WALL_DIST + robot.distanceSensor.getDistance(DistanceUnit.CM);
            powerAdjust = kP * wallError;

            robot.motorLeft.setPower(wallFollowPower + powerAdjust);
            robot.motorRight.setPower(wallFollowPower - powerAdjust);

        }

        sleep(2000);
        turnIMU(drivePower,90);
        driveIMU(drivePower,25);




        // section 3 white line follow until touch sensor

        LineFollowPControl(drivePower, 3000, 1);

        // Section 4 Turn Light On

        driveIMU(drivePower, -40);
        turnIMU(drivePower, 160);


        setArm(40);
        robot.servoWrist.setPosition(0.5);
        driveIMU(drivePower, -15);
        setArm(25);
        sleep(500);
        setArm(40);


    }
}
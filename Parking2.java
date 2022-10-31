package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.ArrayList;

@Autonomous(name = "Parking2", group = "tabs")

public class Parking2 extends AutoCommon {
    // global variable for parkingSpotLength
    private static double parkingSpotLengthCM;
    private static double wallDistance;
    private static int distanceTraveledTicks;
    private static double distanceMeasurementError;



    // Constants for turnRamp and turnDrive
    private final double BASEPOWER = 0.15;
    private final int RAMPTICKS = 500;

    // Constants for turnIMU
    private final double RAMP_CUTOFF = 20.0;
    private final double ANGLE_UNDERSHOOT = 5.0;        // how many degrees to adjust the requested degree angle by
    private final double TURN_ENDING_POWER = 0.10;      // slow power of the motor for the final part of the turn
    private final double SLOW_DOWN_DEGREES = 20.0;      // number of degrees that will be done using the slow power

    //Constants for parking
    private final double MIN_PARKING_LENGTH = 53.00;
    private final double MIN_PARKING_DEPTH = 29.00;
    private static ArrayList<Double[]> parkingSpots = new ArrayList<>();

    private final double power = 0.2;


    protected double reportDistanceCM() {
        return robot.distanceSensor.getDistance(DistanceUnit.CM);
    }

    protected void driveTillLine(double power) {
        while(robot.colorSensor.alpha() < midBrightness) {
            robot.motorLeft.setPower(power);
            robot.motorRight.setPower(power);
        }
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
            if (robot.colorSensor.alpha() > maxBrightness)
                maxBrightness = robot.colorSensor.alpha();
            if (robot.colorSensor.alpha() < minBrightness)
                minBrightness = robot.colorSensor.alpha();
        }
        midBrightness = (maxBrightness + minBrightness)/2;
        // stop both drive motors
        robot.stopDriveMotors();
        robot.resetDriveEncoders();
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
            while (Math.abs(getHeading()) <= degreesMagnitude && opModeIsActive()) {
                robot.motorLeft.setPower(-BASEPOWER * direction);
                robot.motorRight.setPower(BASEPOWER * direction);
            }

        } else {
            System.out.println("large turn, turning with ramp and slow down");
            while (Math.abs(getHeading()) < RAMP_CUTOFF && opModeIsActive()) {
                robot.motorLeft.setPower(-curPower);
                robot.motorRight.setPower(curPower);

                curPower = direction * (BASEPOWER + (Math.abs(getHeading()) / RAMP_CUTOFF * (powerMagnitude - BASEPOWER)));
            }
            System.out.println("ramped to " + curPower);


            while (((getHeading() * direction > 0) ? Math.abs(getHeading()) : 360 - Math.abs(getHeading())) <= degreesMagnitude - SLOW_DOWN_DEGREES - ANGLE_UNDERSHOOT && opModeIsActive()) {
            }
            System.out.println("finished constant speed turn\nsetting speed to slow down speed");


            robot.motorLeft.setPower(-TURN_ENDING_POWER * direction);
            robot.motorRight.setPower(TURN_ENDING_POWER * direction);
            while (((getHeading() * direction > 0) ? Math.abs(getHeading()) : 360 - Math.abs(getHeading())) < (degreesMagnitude - ANGLE_UNDERSHOOT) && opModeIsActive()) {
                continue;
            }
        }
        System.out.println("stopping");
        // stop both drive motors
        robot.stopDriveMotors();
    }
    protected void searchForSpace(double power) {
        double currentSpotSize = 0;
        double lookingForDistance = wallDistance * 1.25;
        double currentWallDistance;
        int ticksDriven = robot.motorLeft.getCurrentPosition();
        boolean spotFound = false;
        int deltaTicks;

        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(power);

        while (robot.colorSensor.alpha() < midBrightness && opModeIsActive()) {
            currentWallDistance = reportDistanceCM();
            deltaTicks = robot.motorLeft.getCurrentPosition() - ticksDriven;
            if (currentWallDistance >  lookingForDistance && opModeIsActive()) {
                currentSpotSize += robot.convertTicksToDistance(deltaTicks);
                if (currentSpotSize > 25) {
                    spotFound = true;
                }
            } else {
                if (spotFound) {
                    if (currentSpotSize > MIN_PARKING_LENGTH) {
                        Double[] temp = {currentSpotSize, robot.convertTicksToDistance(ticksDriven)};
                        parkingSpots.add(temp);
                        telemetry.addData("adding parking spot, length", currentSpotSize);
                        telemetry.addData("total dist CM", robot.convertTicksToDistance(ticksDriven));
                        telemetry.update();
                        robot.stopDriveMotors();
                        sleep(2000);
                        robot.motorLeft.setPower(power);
                        robot.motorRight.setPower(power);
                    } else {
                        telemetry.addData("parking spot too small", currentSpotSize);
                        telemetry.update();
                        spotFound = false;
                        sleep(500);
                    }
                    spotFound = false;
                }
                currentSpotSize = 0;
            }
            ticksDriven = robot.motorLeft.getCurrentPosition();
            telemetry.addData("currentSpotSize", currentSpotSize);
            telemetry.addData("wallDist", currentWallDistance);
            telemetry.addData("lookingForDist", lookingForDistance);
            telemetry.addData("distance", robot.convertTicksToDistance(deltaTicks));
            telemetry.update();
        }
    }

    protected void parking(double power, Double[] parkingSpot) {
        sleep(1000);
        double totalDistanceTraveled = robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition());
        drive(power, -(totalDistanceTraveled-parkingSpot[1]));
        sleep(1000);
        drive(power,-parkingSpot[0]/2);
        sleep(100);
        turnIMU(power,-91);
        sleep(100);
        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(power);
        while(robot.colorSensor.alpha() < midBrightness && opModeIsActive()) continue;
        drive(power,37.5);
        sleep(100);
        turnIMU(power,90);
        robot.stopDriveMotors();

    }
    protected void returnToBase(double power) {
        sleep(500);
        drive(power, -(robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition())-2.5));
        turnIMU(power,-91);
        drive(power,-30);
        robot.stopDriveMotors();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        parkingSpots.clear();
        waitForStart();
        driveToCalibrateLightSensor();
        driveTillLine(power);
        sleep(100);
        turnIMU(power,93);
        for (int i = 0; i < 10; i++) {
            wallDistance = (reportDistanceCM()+wallDistance)/2;
        }
        robot.stopDriveMotors();
        robot.resetDriveEncoders();


        while(robot.colorSensor.alpha() < midBrightness && opModeIsActive()) {
            searchForSpace(power);
        }
        robot.stopDriveMotors();
        telemetry.addData("spots length", parkingSpots.size());
        telemetry.update();
        sleep(2000);
        if (parkingSpots.isEmpty())  {
            returnToBase(power+0.15);
        }
        else {
            int minParkingIndex = 0;
            boolean validSpotFoundToParkTheCarIn = false;
            for (int i = 0; i < parkingSpots.size(); i++) {
                telemetry.addData("spot size", parkingSpots.get(i)[0]);
                telemetry.addData("total dist", parkingSpots.get(i)[1]);
                telemetry.update();
                sleep(2000);
                if (parkingSpots.get(i)[0] < parkingSpots.get(minParkingIndex)[0] && parkingSpots.get(i)[0] > MIN_PARKING_LENGTH) {
                    minParkingIndex = i;
                }
            }
            parking(power, parkingSpots.get(minParkingIndex));
        }
    }
}
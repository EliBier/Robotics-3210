package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "IMUControl", group = "tabs")

public class IMUControl extends AutoCommon {
    // Constants for turnRamp and turnDrive
    private final double BASEPOWER = 0.15;
    private final int RAMPTICKS = 500;

    // Constants for turnIMU
    private final double RAMP_CUTOFF = 20.0;
    private final double ANGLE_UNDERSHOOT = 5.0;        // how many degrees to adjust the requested degree angle by
    private final double TURN_ENDING_POWER = 0.10;      // slow power of the motor for the final part of the turn
    private final double SLOW_DOWN_DEGREES = 20.0;      // number of degrees that will be done using the slow power

    // PID variables
    private final double HEADING_KP = 0.05;



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

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        // test turnIMU()
        turnIMU(0.75, 90);
        sleep(1000);   // let robot settle before taking a final heading
        System.out.println("TURNIMU: " + getHeading());

        turnIMU(0.75, -90);
        sleep(1000);
        System.out.println("TURNIMU: " + getHeading());

        turnIMU(0.75, 45);
        sleep(1000);
        System.out.println("TURNIMU: " + getHeading());

        turnIMU(0.75, -45);
        sleep(1000);
        System.out.println("TURNIMU: " + getHeading());

        turnIMU(0.75, 180);
        sleep(1000);
        System.out.println("TURNIMU: " + getHeading());

        turnIMU(0.75, -180);
        sleep(1000);
        System.out.println("TURNIMU: " + getHeading());

        turnIMU(0.75, 270);
        sleep(1000);
        System.out.println("TURNIMU: " + getHeading());

        turnIMU(0.75, -270);
        sleep(1000);
        System.out.println("TURNIMU: " + getHeading());

    }
}

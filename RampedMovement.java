package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RampedMovement", group = "tabs")

public class RampedMovement extends AutoCommon {

    // Constants for turnRamp and turnDrive
    private final double BASEPOWER = 0.15;
    private final int RAMPTICKS = 500;

    // *** RAMPED DRIVE AND TURN ***
    protected void driveRamp(double power, double cm) {
        // convert cm to encoder ticks (make sure cm is not negative)
        int distance_in_ticks = robot.convertDistanceToTicks(Math.abs(cm));
        // reset the drive encoders (this sets the encoder value to 0)
        robot.resetDriveEncoders();

        // sets the requested power and curPower to be correct sign depending on whether the sign of cm is positive or negative
        double reqPower = Math.abs(power) * Math.signum(cm);
        double curPower = BASEPOWER * Math.signum(cm);
        if (distance_in_ticks < RAMPTICKS) {
            drive(BASEPOWER, cm);
        } else {
            int curTicks = robot.motorLeft.getCurrentPosition();

            // Initializes the curTicks to the current position of the encoder
            while (curTicks < RAMPTICKS) {
                robot.motorLeft.setPower(curPower);
                robot.motorRight.setPower(curPower);

                curPower = Math.signum(cm) * (BASEPOWER + ((double) curTicks / RAMPTICKS * (Math.abs(reqPower) - BASEPOWER)));
                curTicks = Math.abs(robot.motorLeft.getCurrentPosition());
            }

            // Loop until one of the encoders passes the number of ticks for the given cm
            while (Math.abs(robot.motorLeft.getCurrentPosition()) <= distance_in_ticks && opModeIsActive()) {
                continue;
            }

            // stop both drive motors
            robot.stopDriveMotors();
        }

    }

    protected void turnRamp(double power, double degrees) {
        // convert cm to encoder ticks (make sure cm is not negative)
        int arc_length_in_ticks = robot.convertDegreeToTicks(Math.abs(degrees));
        // reset the drive encoders (this sets the encoder value to 0)
        robot.resetDriveEncoders();

        // sets the requested power and curPower to be correct sign depending on whether the sign of cm is positive or negative
        double reqPower = Math.abs(power) * Math.signum(degrees);
        double curPower = BASEPOWER * Math.signum(degrees);

        if (arc_length_in_ticks < RAMPTICKS) {
            turn(BASEPOWER, degrees);
        } else {
            // Initializes the curTicks to the current position of the encoder
            int curTicks = Math.abs(robot.motorLeft.getCurrentPosition());

            while (curTicks < RAMPTICKS) {
                robot.motorLeft.setPower(-curPower);
                robot.motorRight.setPower(curPower);

                curPower = Math.signum(degrees) * (BASEPOWER + ((double) curTicks / RAMPTICKS * (Math.abs(reqPower) - BASEPOWER)));
                curTicks = Math.abs(robot.motorLeft.getCurrentPosition());
            }

            // Loop until one of the encoders passes the number of ticks for the given cm
            while (Math.abs(robot.motorLeft.getCurrentPosition()) <= arc_length_in_ticks && opModeIsActive()) {
                continue;
            }

            // stop both drive motors
            robot.stopDriveMotors();

        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
    }
}

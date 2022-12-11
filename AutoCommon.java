package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutoCommon extends LinearOpMode {

    protected RobotHardware robot;

    // color sensor variables
    public int maxBrightness;
    public int minBrightness;
    public int midBrightness;

    // class variables
    private double initialHeading;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
    }

    protected void drive(double power, double cm) {
        // convert cm to encoder ticks (make sure cm is not negative)
        int distance_in_ticks = robot.convertDistanceToTicks(Math.abs(cm));
        // reset the drive encoders (this sets the encoder value to 0)
        robot.resetDriveEncoders();

        // sets power to be correct sign depending on whether the sign of cm is positive or negative
        power = Math.abs(power) * Math.signum(cm);
        // start both drive motors at the same given power
        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(power);

        // Loop until one of the encoders passes the number of ticks for the given cm
        while (Math.abs(robot.motorLeft.getCurrentPosition()) <= distance_in_ticks && opModeIsActive()) {
            continue;
        }

        // stop both drive motors
        robot.stopDriveMotors();


    }

    protected void turn(double power, double degrees) {
        // convert cm to encoder ticks (make sure cm is not negative)
        int arc_length_in_ticks = robot.convertDegreeToTicks(Math.abs(degrees));
        // reset the drive encoders (this sets the encoder value to 0)
        robot.resetDriveEncoders();

        // sets power to be correct sign depending on whether the sign of cm is positive or negative
        power = Math.abs(power) * Math.signum(degrees);
        // start both drive motors at the same given power
        robot.motorLeft.setPower(-power);
        robot.motorRight.setPower(power);

        // Loop until one of the encoders passes the number of ticks for the given cm
        while (Math.abs(robot.motorLeft.getCurrentPosition()) <= arc_length_in_ticks && opModeIsActive()) {
            continue;
        }

        // stop both drive motors
        robot.stopDriveMotors();


    }
    protected void moveForward(double power) {
        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(power);
    }


    protected void resetAngle() {
        // take a magnetometer reading
        Orientation lastAngles = robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        );

        // set initialHeading to the current angle (-180 to 180)
        initialHeading = lastAngles.firstAngle;

    }

    protected double getHeading() {
        // get the current angle
        double firstAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // adjust that angle by the initial to get a heading
        double heading = firstAngle - initialHeading;

        if (heading < -180) {
            heading = heading + 360;
        }
        if (heading > 180) {
            heading = heading - 360;
        }
        // Make heading use 0 -> 360 as opposed to 0 -> 180, -180 -> 0
//        if (heading <= 0) {
//            heading = (360 + heading) % 360;
//        }

        return heading;
    }
}
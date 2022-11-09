package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Block Sort", group = "TeleOp")
public class BlockSort extends AutoCommon {
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

    private void controlArm() {
        double armPower = 0.0;

        // drive the arm up/down
        if (gamepad1.dpad_up && robot.motorArm.getCurrentPosition() < robot.ARM_MAX_HEIGHT) {
            armPower = robot.ARM_POWER_UP;
        } else if (gamepad1.dpad_down && robot.touchSensorArm.getState()) {
            armPower = robot.ARM_POWER_DOWN;
        }

        // reset the encoder when we touch the button
        if (!robot.touchSensorArm.getState()) {
            robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // set the arm power
        robot.motorArm.setPower(armPower);

        // show information to the user
        telemetry.addData("arm power", armPower);
        telemetry.addData("arm ticks", robot.motorArm.getCurrentPosition());
    }

    // starting positions
    private double gripperPos = RobotHardware.GRIPPER_FULLY_OPEN;
    private double wristPos = RobotHardware.WRIST_PICKUP_POS;

    // used for toggling
    private boolean wasAPressed = false;
    private boolean wasBPressed = false;
    private boolean wasXPressed = false;
    private boolean wasYPressed = false;

    private void controlHand() {
        // open/close the gripper
        if (gamepad1.b && !wasBPressed) {
            gripperPos += robot.GRIPPER_INCREMENT;
        } else if (gamepad1.x && !wasXPressed) {
            gripperPos -= robot.GRIPPER_INCREMENT;
        }

        // move the wrist up/down
        if (gamepad1.y && !wasYPressed) {
            wristPos += robot.WRIST_INCREMENT;
        } else if (gamepad1.a && !wasAPressed) {
            wristPos -= robot.WRIST_INCREMENT;
        }

        if (gamepad1.left_bumper) {
            gripperPos = robot.GRIPPER_FULLY_OPEN;
            wristPos = 0.87;
        }

        // remember button presses
        wasAPressed = gamepad1.a;
        wasBPressed = gamepad1.b;
        wasXPressed = gamepad1.x;
        wasYPressed = gamepad1.y;

        // limit the servo to possible gripper positions
        gripperPos = Range.clip(gripperPos, robot.GRIPPER_FULLY_CLOSED, robot.GRIPPER_FULLY_OPEN);
        wristPos = Range.clip(wristPos, robot.WRIST_FULLY_DOWN, robot.WRIST_FULLY_UP);

        // set the servo positions
        robot.servoGripper.setPosition(gripperPos);
        robot.servoWrist.setPosition(wristPos);

        // telemetry
        telemetry.addData("gripper pos", gripperPos);
        telemetry.addData("wrist pos", wristPos);
    }

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

    /************************************************************************
     * Drive the robot using the left/right joysticks.
     ************************************************************************/
    private void stickDriving() {
        // simple tank drive controls
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        // create dead zones in case the sticks have drift
        if (Math.abs(gamepad1.left_stick_y) < robot.STICK_THRESHOLD) {
            drive = 0;
        }

        if (Math.abs(gamepad1.right_stick_x) < robot.STICK_THRESHOLD) {
            turn = 0;
        }

        robot.startMove(drive, turn*0.75, robot.DRIVE_SPEED_NORMAL);
    }
    // This returns true if the block is red else it returns false

    public void returnArmToBase() {
        // initialize the arm
        robot.motorArm.setPower(robot.ARM_POWER_DOWN);
        while (robot.touchSensorArm.getState()) {
            // do nothing -- waiting for a button press
        }
        robot.motorArm.setPower(0);
        // reset the encoder
        robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setArm(int ticks) {
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

    private boolean RedVsBlue() {
        double red = robot.colorSensor.red();
        double blue = robot.colorSensor.blue();
        if (red > blue) return true;
        else return false;
    }

    private void driveRed(int blockDropped) {
        double boxHorizontalDistance = 27;
        double boxVerticalDistance = 15;
        if (blockDropped > 0 ) {
            turnIMU(0.3, 90 * blockDropped);
            driveIMU(0.3, boxHorizontalDistance * blockDropped);
        } else {
            driveIMU(0.3, boxHorizontalDistance * blockDropped);
            turnIMU(0.3, 90 * blockDropped);
        }
    }

    private void driveBlue(int blockDropped) {
        double boxHorizontalDistance = 26;
        double boxVerticalDistance = 45;

    }

    public void blockDropOff() {
        if (gamepad1.right_bumper) {
            int blockDropped = 1;
            boolean color = RedVsBlue();
            if (color) {
                driveRed(blockDropped);
                gripperPos = robot.GRIPPER_FULLY_OPEN;
                // limit the servo to possible gripper positions
                setGripperPos(gripperPos);
                blockDropped = -1;
                driveRed(blockDropped);
            } else {
                driveBlue(blockDropped);
                gripperPos = robot.GRIPPER_FULLY_OPEN;
                // limit the servo to possible gripper positions
                setGripperPos(gripperPos);
                blockDropped = -1;
                driveBlue(blockDropped);
            }
        }
    }

    public void blockDropOff2() {
        if (gamepad1.right_bumper) {
            double armLength = 40;
            double armPivotHeight = 35.5;
            double thetaMin = 13;
            double ticksPerArmDeg = 13.8;

            double blueHorizontal = 85;
            double blueVertical = 30;
            double blueDropHeight = 25;
            double redHorizontal = 120;
            double redVertical = 24;
            double redDropHeight = 30;

            double blueBoxDistance = Math.sqrt(Math.pow(blueVertical, 2) + Math.pow(blueHorizontal, 2));
            double blueDistanceToDrive = blueBoxDistance - 25 - Math.sqrt(Math.pow(armLength, 2) - Math.pow(armPivotHeight - blueDropHeight, 2));
            double redBoxDistance = Math.sqrt(Math.pow(redVertical, 2) + Math.pow(redHorizontal, 2));
            double redDistanceToDrive = redBoxDistance - 25 - Math.sqrt(Math.pow(armLength, 2) - Math.pow(armPivotHeight - redDropHeight, 2));

            double blueThetaBox = 90.0 - Math.atan2(blueVertical, blueHorizontal) * 180 / Math.PI;
            double blueThetaArm = Math.acos((armPivotHeight - blueDropHeight) / armLength)*180/Math.PI;
            double blueWristDropPoint = 1 - (blueThetaArm / 180);
            int blueArmRaiseTicks = (int)((blueThetaArm - thetaMin) * ticksPerArmDeg);
            double redThetaBox = 90.0 - Math.atan2(redVertical, redHorizontal) * 180 / Math.PI;
            double redThetaArm = Math.acos((armPivotHeight - redDropHeight) / armLength)*180/Math.PI;
            double redWristDropPoint = 1 - (redThetaArm / 180);
            int redArmRaiseTicks = (int)((redThetaArm - thetaMin) * ticksPerArmDeg);

            setWristPos(1);
            sleep(200);
            boolean isRed = RedVsBlue();
            if (isRed) {
                turnIMU(0.2, redThetaBox);
                setArm(redArmRaiseTicks);
                driveIMU(0.3,  -1*(redDistanceToDrive));
                sleep(150);
                setWristPos(redWristDropPoint);
                sleep(150);
                setGripperPos(robot.GRIPPER_FULLY_OPEN);
                sleep(150);
                driveIMU(0.5, redDistanceToDrive);
                setWristPos(0.87);
                returnArmToBase();
                turnIMU(0.2, -1* redThetaBox);
            } else {
                turnIMU(0.2, -1* blueThetaBox);
                setArm(blueArmRaiseTicks);
                driveIMU(-0.3,  -1*(blueDistanceToDrive));
                sleep(150);
                setWristPos(blueWristDropPoint);
                sleep(150);
                setGripperPos(robot.GRIPPER_FULLY_OPEN);
                sleep(150);
                driveIMU(0.5, blueDistanceToDrive);
                setWristPos(0.87);
                returnArmToBase();
                turnIMU(0.2, blueThetaBox);
            }
            setGripperPos(robot.GRIPPER_FULLY_OPEN);
            setWristPos(.87);
        }
    }

    public void setWristPos(double position) {
        robot.servoWrist.setPosition(Range.clip(position, robot.WRIST_FULLY_DOWN, robot.WRIST_FULLY_UP));
    }
    public void setGripperPos(double position) {
        robot.servoGripper.setPosition(Range.clip(position, robot.GRIPPER_FULLY_CLOSED, robot.GRIPPER_FULLY_OPEN));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        // move the arm to a known location
        initializeArm();

        waitForStart();

        while (opModeIsActive()) {
            stickDriving();
            controlArm();
            controlHand();
            blockDropOff2();
            telemetry.update();
        }
    }
}
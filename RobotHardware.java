package edu.elon.robotics;

/**
 * Define the robot hardware, some useful constants, and a couple
 * of useful methods.
 *
 * @author J. Hollingsworth
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class RobotHardware {

    private final HardwareMap hardwareMap;

    // drive motors
    public DcMotor motorLeft;
    public DcMotor motorRight;

    // sensors
    public BNO055IMU imu;
    public boolean calibratedIMU;
    public DigitalChannel touchSensor;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;

    // useful constants
    public final double STICK_THRESHOLD = 0.2;
    public final double DRIVE_SPEED_NORMAL = 0.8;
    public final double WHEEL_DIAMETER = 10.16;
    public final double TICKS_PER_ROTATION = 1120;
    public final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public final double TICKS_PER_CM = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
    public final double DISTANCE_BETWEEN_WHEELS_IN_CM = 36.0;
    public final double TURNING_CIRCLE_CIRCUMFERENCE_IN_CM = DISTANCE_BETWEEN_WHEELS_IN_CM * Math.PI;

    public void initializeIMU() {
        //------------------------------------------------------------
        // IMU - BNO055
        // Set up the parameters with which we will use our IMU.
        // + 9 degrees of freedom
        // + use of calibration file (see calibration program)
        //------------------------------------------------------------
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // see the calibration sample opmode
        parameters.calibrationDataFile = "AdafruitImuCalibration.json";
        parameters.loggingEnabled      = false;
        //parameters.loggingTag          = "IMU";
        //parameters.mode                = BNO055IMU.SensorMode.NDOF;

        parameters.accelerationIntegrationAlgorithm = null;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Read the IMU configuration from the data file saved during calibration.
        // Using try/catch allows us to be specific about the error instead of
        // just showing a NullPointer exception that could come from anywhere in the program.
        calibratedIMU = true;
        try {
            File file = AppUtil.getInstance().getSettingsFile(parameters.calibrationDataFile);
            String strCalibrationData = ReadWriteFile.readFile(file);
            BNO055IMU.CalibrationData calibrationData = BNO055IMU.CalibrationData.deserialize(strCalibrationData);
            imu.writeCalibrationData(calibrationData);
        }
        catch (Exception e) {
            calibratedIMU = false;
        }
    }
    public RobotHardware(HardwareMap hardwareMap) {
        // remember the robot configuration
        this.hardwareMap = hardwareMap;

        initializeIMU();
        if(!calibratedIMU) {
            System.out.println("Elon_ROBOTICS: IMU is not calibrated");
        }

        // configure the drive motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // configure sensors
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        stopDriveMotors();
        resetDriveEncoders();
    }

    public void startMove(double drive, double turn, double modifier) {
        double leftPower  = (drive + turn) * modifier;
        double rightPower = (drive - turn) * modifier;

        double max = Math.max(leftPower, rightPower);

        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        motorLeft.setPower(Range.clip(leftPower, -1, 1));
        motorRight.setPower(Range.clip(rightPower, -1, 1));
    }

    public void stopDriveMotors() {
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    public void resetDriveEncoders() {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int convertDistanceToTicks(double cm) {
        double distanceInTicks = cm * TICKS_PER_CM;
        return (int)distanceInTicks;
    }

    public double convertTicksToDistance(int ticks) {
        double distanceInCM = (double)ticks/TICKS_PER_CM;
        return Math.round(distanceInCM*100)/100.0;
    }

    public int convertDegreeToTicks(double degrees) {
        double arc_length = degrees / 360.0 * TURNING_CIRCLE_CIRCUMFERENCE_IN_CM;
        return convertDistanceToTicks(arc_length);



    }

}
// need to commit this
// hi

package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    public DcMotor[] driveMotors = new DcMotor[4];
    public DcMotor[] driveMotors2WheelY = new DcMotor[2];
    public DcMotor[] driveMotors2WheelX = new DcMotor[2];
    public DcMotor[] driveMotorsMode = new DcMotor[3];
    public BNO055IMU imu = null;
    Telemetry telemetry = null;
    static final boolean useIMU = true;
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 2.95276;     // For figuring circumference
    static final double COUNTS_PER_OUTPUT_REVOL = 537.7;
    static final double COUNTS_PER_INCH = (22.0 / 16.0) * (COUNTS_PER_OUTPUT_REVOL) / (WHEEL_DIAMETER_INCHES * Math.PI);
    double clawOpen = 0;
    double clawClose = 0;
    double armGround = 0;
    double armLowBar = 0;
    double armMiddleBar = 0;
    double armHighBar = 0;
    public RevRoboticsCoreHexMotor armMotor = null;
    HardwareMap hwMap = null;


    public void init(HardwareMap ahwMap){

        hwMap = ahwMap;

        front_left = hwMap.get(DcMotor .class, "fldrive");
        front_right = hwMap.get(DcMotor.class, "frdrive");
        back_left = hwMap.get(DcMotor.class, "bldrive");
        back_right = hwMap.get(DcMotor.class, "brdrive");

        if (useIMU) {
            imu = hwMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            //change to default set of parameters go here
            imu.initialize(params);
        }

        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        setDriveStop(0);

        // Set all motors to run without encoders.
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDriveStop(double power) {
        front_left.setPower(power);
        front_right.setPower(power);
        back_left.setPower(power);
        back_right.setPower(power);
    }

    public void setRunMode(DcMotor.RunMode mode){
        front_right.setMode(mode);
        front_left.setMode(mode);
        back_right.setMode(mode);
        back_left.setMode(mode);
    }

    public int convertInchesToCounts(double inches) {
        return (int) ((COUNTS_PER_INCH) * inches);
    }

    public boolean isDriveBusy() {
        boolean motor = false;
        if (front_left.isBusy() || front_right.isBusy() || back_right.isBusy() || back_left.isBusy()) {
            motor = true;
        }
        return motor;
    }

    public void setDrivePower(double[] wheelpowers) {
        double max = Math.abs(wheelpowers[0]);
        for (int i = 0; i < wheelpowers.length; i++) {
            if (max < Math.abs(wheelpowers[i])) max = Math.abs(wheelpowers[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < wheelpowers.length; i++) wheelpowers[i] /= max;
        }

        front_left.setPower(wheelpowers[0]);
        front_right.setPower(wheelpowers[1]);
        back_left.setPower(wheelpowers[2]);
        back_right.setPower(wheelpowers[3]);

        reportDrivePowers(wheelpowers);
    }

    public void reportDrivePowers(double[] powers) {
        telemetry.addLine()
                .addData("XDrive Power FL", powers[0])
                .addData("FR", powers[0])
                .addData("BL", powers[0])
                .addData("BR", powers[0]);
    }
    public void setDriveStop() {
        double[] powers = {0, 0, 0, 0};
        setDrivePower(powers);
    }

    public double[] calculateDrivePowers(double heading, double power, double rotate) {
        double m0, m1, m2, m3;
        m0 = power * -Math.sin(heading - (Math.PI / 4)) + rotate;
        m1 = power * Math.cos(heading + (Math.PI / 4)) + rotate;
        m2 = power * Math.sin(heading + (Math.PI / 4)) + rotate;
        m3 = power * -Math.cos(heading - (Math.PI / 4)) + rotate;
        double[] MecanumMotors = {m0, m1, m2, m3};
        return MecanumMotors;
    }

}

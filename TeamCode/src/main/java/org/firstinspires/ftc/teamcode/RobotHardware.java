package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class RobotHardware {

    // --- Configured Values ---
    // Servo positions
    static final double CARTRIDGE_DROP_OPEN = 0.1;
    static final double CARTRIDGE_DROP_CLOSED = 0.5;
    static final double CARTRIDGE_ROTATE_HOME_LEFT = 0.85;
    static final double CARTRIDGE_ROTATE_HOME_RIGHT = 0.2;
    static final double CARTRIDGE_ROTATE_FORWARD_LEFT = 0.50; //55
    static final double CARTRIDGE_ROTATE_FORWARD_RIGHT = 0.55; //50
    // Motor Limits
    static final int LEFT_LIFT_MIN_BOUND = 0;
    static final int RIGHT_LIFT_MIN_BOUND = 0;
    static final int LEFT_LIFT_MAX_BOUND = 1000;
    static final int RIGHT_LIFT_MAX_BOUND = 1000;
    // -------------------------

    // --- Hardware ---
    HardwareMap hardwareMap;
    // IMU
    IMU imu;
    // Drive motors
    public DcMotorEx backLeftDrive;
    public DcMotorEx backRightDrive;
    public DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;
    // Linear slide motors
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;
    // Cartridge servos
    public Servo cartridgeRotateLeft;
    public Servo cartridgeRotateRight;
    public Servo cartridgeDrop;
    // ----------------

    // --- Runtime Properties ---
    public double forwardHeading;
    public boolean cartridgeRotateForward = false;
    public boolean cartridgeDropClosed = false;
    // --------------------------

    // -- Initialization and Utility Functions --
    public RobotHardware(HardwareMap hw) {
        hardwareMap = hw;

        backLeftDrive = hardwareMap.get(DcMotorEx.class, "bld");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "brd");
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frd");

        backLeftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorEx.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftLift = hardwareMap.get(DcMotorEx.class, "ll");
        rightLift = hardwareMap.get(DcMotorEx.class, "rl");

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cartridgeRotateLeft = hardwareMap.get(Servo.class, "crl");
        cartridgeRotateRight = hardwareMap.get(Servo.class, "crr");
        cartridgeDrop = hardwareMap.get(Servo.class, "cd");
    }

    public void initializeIMU() {
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void resetForwardHeading() {
        forwardHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetEncoders() {
        brake();

        backLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    // ------------------------------------------

    // -- Drive Functions --
    public void drivePower(double blPower, double brPower, double flPower, double frPower) {
        backLeftDrive.setPower(Range.clip(blPower, -1, 1));
        backRightDrive.setPower(Range.clip(brPower, -1, 1));
        frontLeftDrive.setPower(Range.clip(flPower, -1, 1));
        frontRightDrive.setPower(Range.clip(frPower, -1, 1));
    }

    public void drive(double drive, double strafe, double rotate, double speed) {
        final double blPower = speed * (drive - rotate + strafe);
        final double brPower = speed * (drive + rotate - strafe);
        final double flPower = speed * (drive - rotate - strafe);
        final double frPower = speed * (drive + rotate + strafe);
        drivePower(blPower, brPower, flPower, frPower);
    }

    public void fieldDrive(double drive, double strafe, double rotate, double speed) {
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - forwardHeading;
        double targetAngle = Math.toDegrees(Math.atan2(strafe, drive));
        double relativeAngle = Math.toRadians(targetAngle - currentAngle);
        double magnitude = Math.sqrt((drive * drive) + (strafe * strafe));
        double botDrive = magnitude * Math.cos(relativeAngle);
        double botStrafe = magnitude * Math.sin(relativeAngle);
        botStrafe *= 1.1; // Correct strafing?
        drive(botDrive, botStrafe, rotate, speed);
    }

    public void brake() {
        drivePower(0, 0, 0, 0);
    }
    // ---------------------

    // -- Manipulator Functions --
    public void liftPower(double power) {
//        if ((leftLift.getCurrentPosition() <= LEFT_LIFT_MIN_BOUND) || (rightLift.getCurrentPosition() <= RIGHT_LIFT_MIN_BOUND)) {
//            power = Range.clip(power, 0, 1);
//        }
//        if ((leftLift.getCurrentPosition() >= LEFT_LIFT_MAX_BOUND) || (rightLift.getCurrentPosition() >= RIGHT_LIFT_MAX_BOUND)) {
//            power = Range.clip(power, -1, 0);
//        }
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    public void rotateCartridgeForward() {
        cartridgeRotateLeft.setPosition(CARTRIDGE_ROTATE_FORWARD_LEFT);
        cartridgeRotateRight.setPosition(CARTRIDGE_ROTATE_FORWARD_RIGHT);
        cartridgeRotateForward = true;
    }

    public void rotateCartridgeHome() {
        cartridgeRotateLeft.setPosition(CARTRIDGE_ROTATE_HOME_LEFT);
        cartridgeRotateRight.setPosition(CARTRIDGE_ROTATE_HOME_RIGHT);
        cartridgeRotateForward = false;
    }

    public void toggleCartridgeRotate() {
        if (cartridgeRotateForward) {
            rotateCartridgeHome();
        } else {
            rotateCartridgeForward();
        }
    }

    public void openCartridge() {
        cartridgeDrop.setPosition(CARTRIDGE_DROP_OPEN);
        cartridgeDropClosed = false;
    }

    public void closeCartridge() {
        cartridgeDrop.setPosition(CARTRIDGE_DROP_CLOSED);
        cartridgeDropClosed = true;
    }

    public void toggleCartridgeDrop() {
        if (cartridgeDropClosed) {
            openCartridge();
        } else {
            closeCartridge();
        }
    }
    // ---------------------------
}

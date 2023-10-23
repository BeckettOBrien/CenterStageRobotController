package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class RobotHardware {

    HardwareMap hardwareMap;

    // Drive motors
    public DcMotorEx backLeftDrive;
    public DcMotorEx backRightDrive;
    public DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;

    public RobotHardware(HardwareMap hw) {
        hardwareMap = hw;

        backLeftDrive = hardwareMap.get(DcMotorEx.class, "bld");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "brd");
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frd");

        backLeftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        backLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void resetEncoders() {
        brake();

        backLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void drivePower(double blPower, double brPower, double flPower, double frPower) {
        backLeftDrive.setPower(Range.clip(blPower, -1, 1));
        backRightDrive.setPower(Range.clip(brPower, -1, 1));
        frontLeftDrive.setPower(Range.clip(flPower, -1, 1));
        frontRightDrive.setPower(Range.clip(frPower, -1, 1));
    }

    public void drive(double drive, double strafe, double rotate, double speed) {
        final double blPower = speed * (drive - rotate - strafe);
        final double brPower = speed * (drive + rotate + strafe);
        final double flPower = speed * (drive - rotate + strafe);
        final double frPower = speed * (drive + rotate - strafe);
        drivePower(blPower, brPower, flPower, frPower);
    }

    public void brake() {
        drivePower(0, 0, 0, 0);
    }
}

package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
@TeleOp(name="Basic Field Drive", group="Iterative Opmode")
public class FieldDrive extends OpMode {

    public static double INPUT_RATE_EXPONENT = 1.8;
    public static double DRIVE_SPEED_MULTIPLIER = 0.3;
    public static double TURN_SPEED_MULTIPLIER = 0.5;

    RobotHardware robot;

    @Override
    public void init() {
        telemetry = FtcDashboard.getInstance().getTelemetry();
        robot = new RobotHardware(hardwareMap);
        robot.initializeIMU();
    }

    @Override
    public void start() {
        robot.resetForwardHeading();
    }

    @Override
    public void loop() {
        if (gamepad1.options) robot.resetForwardHeading();

        double drive = applyRates(gamepad1.right_stick_y);
        double strafe = applyRates(gamepad1.right_stick_x);
        double rotate = applyRates(gamepad1.left_stick_x);

        robot.fieldDrive(drive, strafe, rotate * TURN_SPEED_MULTIPLIER, ((gamepad1.left_trigger > 0.5) ? 1 : DRIVE_SPEED_MULTIPLIER));
    }

    double applyRates(double input) {
        return (input < 0 ? -1 : 1) * Math.pow(Math.abs(input), INPUT_RATE_EXPONENT);
    }
}

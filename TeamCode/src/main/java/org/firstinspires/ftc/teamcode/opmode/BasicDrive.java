package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
@TeleOp(name="Basic Drive", group="Iterative Opmode")
public class BasicDrive extends OpMode {

    public static double INPUT_RATE_EXPONENT = 1.8;
    public static double DRIVE_SPEED_MULTIPLIER = 0.3;
    public static double TURN_SPEED_MULTIPLIER = 0.5;

    RobotHardware robot;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
    }

    @Override
    public void loop() {
        double drive = Math.pow(gamepad1.right_stick_y, INPUT_RATE_EXPONENT);
        double strafe = Math.pow(gamepad1.right_stick_x, INPUT_RATE_EXPONENT);
        double rotate = Math.pow(gamepad1.left_stick_x, INPUT_RATE_EXPONENT);

        robot.drive(drive, strafe, rotate * TURN_SPEED_MULTIPLIER, ((gamepad1.left_trigger > 0.5) ? 1 : DRIVE_SPEED_MULTIPLIER));
    }
}

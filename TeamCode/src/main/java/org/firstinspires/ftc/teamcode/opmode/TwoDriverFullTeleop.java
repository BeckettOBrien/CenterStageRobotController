package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
@TeleOp(name="Full Two Driver", group="Iterative Opmode")
public class TwoDriverFullTeleop extends OpMode {

    public static double INPUT_RATE_EXPONENT = 1.8;
    public static double DRIVE_SPEED_MULTIPLIER = 0.8;
    public static double TURN_SPEED_MULTIPLIER = 0.5;
    public static double LIFT_SPEED_MULTIPLIER = 0.2;
    public static double INTAKE_SPEED = 0.9;

    RobotHardware robot;

    boolean pressedALast = false;
    boolean pressedBLast = false;

    @Override
    public void init() {
        telemetry = FtcDashboard.getInstance().getTelemetry();
        robot = new RobotHardware(hardwareMap);
//        robot.initializeIMU();
    }

    @Override
    public void start() {
        robot.resetEncoders();
//        robot.resetForwardHeading();
    }

    @Override
    public void loop() {
        if (!gamepad2.a) pressedALast = false;
        if (!gamepad2.b) pressedBLast = false;
//        if (gamepad1.options) robot.resetForwardHeading();

        double drive = applyRates(gamepad1.left_stick_y);
        double strafe = applyRates(gamepad1.left_stick_x);
        double rotate = applyRates(gamepad1.right_stick_x);

        robot.drive(drive, strafe, rotate * TURN_SPEED_MULTIPLIER, ((gamepad1.left_trigger > 0.5) ? 0.3 : DRIVE_SPEED_MULTIPLIER));

        robot.liftPower((gamepad2.right_trigger - gamepad2.left_trigger) * LIFT_SPEED_MULTIPLIER);
        if (gamepad2.a && !pressedALast) {
            robot.toggleCartridgeDrop();
            pressedALast = true;
        }
        if (gamepad2.b && !pressedBLast) {
            robot.toggleCartridgeRotate();
            pressedBLast = true;
        }
        if (gamepad2.right_bumper) robot.rotateCartridgeForward();
        if (gamepad2.left_bumper) robot.rotateCartridgeHome();
        if (gamepad1.y) robot.launchPlane();

        robot.intakePower(INTAKE_SPEED * (gamepad1.right_bumper ? 1 : (gamepad1.left_bumper ? -0.75 : 0)));
    }

    double applyRates(double input) {
        return (input < 0 ? -1 : 1) * Math.pow(Math.abs(input), INPUT_RATE_EXPONENT);
    }
}

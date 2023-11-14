package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
@TeleOp(name="Hardware Testing Utility", group="Iterative Opmode")
public class TestUtil extends OpMode {

    public static double LIFT_MOTOR_POWER_LEFT = 0;
    public static double LIFT_MOTOR_POWER_RIGHT = 0;
    public static double LIFT_POWER_COMBINED = 0;
    public static double CARTRIDGE_ROTATE_POS_LEFT = 0;
    public static double CARTRIDGE_ROTATE_POS_RIGHT = 0;
    public static double CARTRIDGE_DROP_POS = 0;

    RobotHardware robot;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        telemetry = FtcDashboard.getInstance().getTelemetry();
    }

    @Override
    public void start() {
        robot.resetEncoders();
    }

    @Override
    public void loop() {
        if (gamepad1.options) {
            robot.resetForwardHeading();
            robot.resetForwardHeading();
        }

//        robot.liftPower((gamepad1.right_trigger - gamepad1.left_trigger) * 0.2);
        robot.leftLift.setPower(LIFT_MOTOR_POWER_LEFT);
        robot.rightLift.setPower(LIFT_MOTOR_POWER_RIGHT);

        if (gamepad1.a) {
            robot.cartridgeRotateLeft.setPosition(CARTRIDGE_ROTATE_POS_LEFT);
            robot.cartridgeRotateRight.setPosition(CARTRIDGE_ROTATE_POS_RIGHT);
            robot.cartridgeDrop.setPosition(CARTRIDGE_DROP_POS);
        }

        telemetry.addData("Left Lift Encoder", robot.leftLift.getCurrentPosition());
        telemetry.addData("Right Lift Encoder", robot.rightLift.getCurrentPosition());
    }

}

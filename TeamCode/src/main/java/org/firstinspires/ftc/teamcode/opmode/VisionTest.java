package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.vision.PropLocator;

@Config
@TeleOp(name="Vision Test Utility", group="Iterative Opmode")
public class VisionTest extends OpMode {

    RobotHardware robot;
    PropLocator locator;

    @Override
    public void init() {
        telemetry = FtcDashboard.getInstance().getTelemetry();
        robot = new RobotHardware(hardwareMap);
        locator = PropLocator.blueSideLocator();
        locator.initializeVision(hardwareMap);
    }

    @Override
    public void loop() {
        String location;
        switch (locator.getLocation()) {
            case LEFT:
                location = "Left";
                break;
            case MIDDLE:
                location = "Middle";
                break;
            case RIGHT:
                location = "Right";
                break;
            default:
                location = "Unknown";
        }
        telemetry.addData("Location", location);
    }
}

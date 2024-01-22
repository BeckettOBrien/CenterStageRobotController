package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "Plane Loader Utility", group = "Linear Opmode")
public class PlaneLoader extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        waitForStart();
        robot.loadPlane();
        sleep(1000);
    }
}

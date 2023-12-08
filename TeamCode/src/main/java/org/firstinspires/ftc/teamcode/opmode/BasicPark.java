package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
@Autonomous(name="Basic Park Auto", group="Linear Opmode")
public class BasicPark extends LinearOpMode {

    public static int RUN_TIME = 5250;
    public static double SPEED = 0.4;

    RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);

        waitForStart();

        robot.drive(-1.0, 0.0, 0.0, SPEED);
        sleep(RUN_TIME);
        robot.brake();
    }
}

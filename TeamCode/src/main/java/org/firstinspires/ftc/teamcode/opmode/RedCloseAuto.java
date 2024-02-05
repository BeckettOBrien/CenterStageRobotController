package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.vision.PropLocator;

@Autonomous(name="Red Close Auto")
public class RedCloseAuto extends LinearOpMode {

    RobotHardware robot;
    SampleMecanumDrive drive;
    PropLocator locator;

//    enum AUTO_STATE {
//        WAIT,
//        TO_SPIKE_MARK,
//        PLACING_SPIKE_PIXEL,
//        TO_BACKDROP,
//        PLACING_BACKBOARD_PIXEL,
//        TO_PARK
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        drive = new SampleMecanumDrive(robot);
        locator = PropLocator.redSideLocator();
        locator.initializeVision(hardwareMap);
        telemetry = FtcDashboard.getInstance().getTelemetry();

        waitForStart();

        Vector2d spikeMark;
        Vector2d backboardSlot;
        switch (locator.getLocation()) {
            case LEFT:
                spikeMark = new Vector2d(14, -30);
                backboardSlot = new Vector2d(41, -29);
                break;
            default:
            case MIDDLE:
                spikeMark = new Vector2d(23, -24);
                backboardSlot = new Vector2d(40, -36);
                break;
            case RIGHT:
                spikeMark = new Vector2d(36, -30);
                backboardSlot = new Vector2d(40, -44);
                break;
        }

        drive.setPoseEstimate(new Pose2d(12, -61, Math.toRadians(-90)));
        TrajectorySequence toSpikeMark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(10)
                .turn(Math.toRadians(180))
                .splineTo(spikeMark, Math.toRadians(90))
                .turn(Math.toRadians(-90))
                .back(1)
                .build();

        TrajectorySequence toBackdrop = drive.trajectorySequenceBuilder(toSpikeMark.end())
                .splineTo(backboardSlot, Math.toRadians(0))
                .build();

        TrajectorySequence toDrop = drive.trajectorySequenceBuilder(toBackdrop.end())
                .forward(10)
                .build();

        TrajectorySequence startPark = drive.trajectorySequenceBuilder(toDrop.end())
                .back(5)
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence finishPark = drive.trajectorySequenceBuilder(startPark.end())
                .splineTo(new Vector2d(56, -60), Math.toRadians(0))
                .build();

        drive.followTrajectorySequence(toSpikeMark);
        robot.intakePower(-0.7);
        sleep(200);
        robot.intakePower(0);
        drive.followTrajectorySequence(toBackdrop);
        robot.liftPower(0.6);
        sleep(250);
        robot.closeCartridge();
        sleep(400);
        robot.liftPower(0);
        robot.rotateCartridgeForward();
        sleep(500);
        drive.followTrajectorySequence(toDrop);
        robot.openCartridge();
        sleep(250);
        drive.followTrajectorySequence(startPark);
        robot.rotateCartridgeHome();
        sleep(250);
        robot.liftPower(-0.2);
        sleep(1350);
        robot.liftPower(0);
        drive.followTrajectorySequence(finishPark);
//        AUTO_STATE currentState = AUTO_STATE.WAIT;
//        while (opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("State", currentState);
//            switch (currentState) {
//                case WAIT:
//                    if (isStarted()) {
//                        currentState = AUTO_STATE.TO_SPIKE_MARK;
//                        drive.followTrajectory();
//                    }
//            }
//        }
    }
}

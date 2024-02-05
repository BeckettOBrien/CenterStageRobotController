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

@Autonomous(name="Red Far Auto")
public class RedFarAuto extends LinearOpMode {

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
        boolean rightSpike = false;
        switch (locator.getLocation()) {
            case LEFT:
                spikeMark = new Vector2d(-46, -38);
                backboardSlot = new Vector2d(40, -35);
                break;
            default:
            case MIDDLE:
                spikeMark = new Vector2d(-35, -32);
                backboardSlot = new Vector2d(40, -39);
                break;
            case RIGHT:
                spikeMark = new Vector2d(-34, -30);
                backboardSlot = new Vector2d(40, -44);
                rightSpike = true;
                break;
        }

        drive.setPoseEstimate(new Pose2d(-35, -61, Math.toRadians(-90)));
        TrajectorySequence toSpikeMark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true)
                .splineTo(spikeMark, rightSpike ? Math.toRadians(0) : Math.toRadians(90))
                .setReversed(false)
                .build();

//        TrajectorySequence toBackdropStart = drive.trajectorySequenceBuilder(toSpikeMark.end())
//                .splineTo(new Vector2d(-14, -35), Math.toRadians(0))
//                .build();

//        TrajectorySequence toRightSpike = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .setReversed(true)
//                .splineTo(spikeMark, Math.toRadians(90))
//                .setReversed(false)
//                .splineTo(new Vector2d(-14, -35), Math.toRadians(0))
//                .build();

        TrajectorySequence toBackdrop = drive.trajectorySequenceBuilder(toSpikeMark.end())
//                .splineTo(new Vector2d(-23, -36), Math.toRadians(0))
                .splineTo(new Vector2d(-30, -58), Math.toRadians(0))
                .forward(45)
                .splineTo(backboardSlot, Math.toRadians(0))
                .build();

        TrajectorySequence toDrop = drive.trajectorySequenceBuilder(toBackdrop.end())
                .forward(10)
                .build();

        TrajectorySequence toPark = drive.trajectorySequenceBuilder(toDrop.end())
                .back(5)
                .turn(Math.toRadians(180))
                .back(2)
                .build();

        sleep(4000);
        drive.followTrajectorySequence(toSpikeMark);
        robot.intakePower(-0.7);
        sleep(200);
        robot.intakePower(0);
//        if (!rightSpike) {
//            drive.followTrajectorySequence(toSpikeMark);
//            robot.intakePower(-0.75);
//            sleep(200);
//            robot.intakePower(0);
//            drive.followTrajectorySequence(toBackdropStart);
//        } else {
//            drive.followTrajectorySequence(toRightSpike);
//            robot.intakePower(-0.75);
//            sleep(200);
//            robot.intakePower(0);
//        }
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
        drive.followTrajectorySequence(toPark);
        robot.rotateCartridgeHome();
        sleep(250);
        robot.liftPower(-0.2);
        sleep(1350);
        robot.liftPower(0);

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

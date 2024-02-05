package com.firstinspires.ftc.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(-90)))
                                .setReversed(true)
//                                .back(10)
//                                .turn(Math.toRadians(180))
                                .splineTo(new Vector2d(-34, -30), Math.toRadians(0))
                                .setReversed(false)
//                                .back(1)
//                                .setReversed(false)
//                                .splineTo(new Vector2d(-14, -35), Math.toRadians(0))
//                                .forward(30)
                                .splineTo(new Vector2d(-30, -58), Math.toRadians(0))
                                .forward(45)
//                                .back(5)
//                                .turn(Math.toRadians(90))
//                                .splineTo(new Vector2d(56, 60), Math.toRadians(0))
//                                .back(3)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
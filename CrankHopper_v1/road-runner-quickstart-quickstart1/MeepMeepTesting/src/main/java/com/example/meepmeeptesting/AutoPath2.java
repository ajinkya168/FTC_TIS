package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoPath2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(300);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50.46434527240892, 50.46434527240892, Math.toRadians(180), Math.toRadians(180), 9.195)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(new Pose2d(-33,-62,Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-33, -36, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-50, -36, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-58, -36, Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(38,-59,Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(45,-34,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(50,-34,Math.toRadians(0)))


                                //.splineToConstantHeading(new Vector2d(54,-40),0)
//                                .setReversed(true)
//                                .splineToConstantHeading(new Vector2d(10,-10),-Math.PI)
//                                .splineToConstantHeading(new Vector2d(-45,-10),Math.PI)
//                                .splineToConstantHeading(new Vector2d(-58,-12),Math.PI)
//                                .setReversed(false)
//                                .splineToConstantHeading(new Vector2d(10,-10),0)
//                                .splineToConstantHeading(new Vector2d(50,-29),0)
//                                .setReversed(true)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
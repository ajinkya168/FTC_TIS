package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoBlueFarPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50.46434527240892, 50.46434527240892, Math.toRadians(180), Math.toRadians(180), 9.195)
                .followTrajectorySequence(drive ->

                                drive.trajectorySequenceBuilder(new Pose2d(-34.3,60.1,Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(-33, 36, Math.toRadians(90)))
                                        .waitSeconds(1.5)
                                        .splineToLinearHeading(new Pose2d(-54.5 , 35.5, Math.toRadians(0)), 0) //intakePixel
                                        .waitSeconds(1)
                                        .setReversed(true)
                                        .lineToConstantHeading(new Vector2d(-51,35.5))
                                        .splineToConstantHeading(new Vector2d(-36, 10), 0) //close to start
                                        .splineToConstantHeading(new Vector2d(14, 10), 0)
                                        .splineToConstantHeading(new Vector2d(47.5,41.5),0) //backdrop
                                        .waitSeconds(1)
                                        //------------------------------------------------------------------
                                        .splineToConstantHeading(new Vector2d(14, 58), 0)
                                        .splineToConstantHeading(new Vector2d(-36, 59), 0)
                                        .setReversed(true)
                                        .lineToConstantHeading(new Vector2d(-54.5,35))
                                        .waitSeconds(1)
                                        .splineToConstantHeading(new Vector2d(-36, 59), 0)
                                        .splineToConstantHeading(new Vector2d(14, 58), 0)
                                        .splineToConstantHeading(new Vector2d(47.5,41.5),0)
                                        .waitSeconds(1)
                                        .splineToConstantHeading(new Vector2d(14, 58), 0)
                                        .splineToConstantHeading(new Vector2d(-36, 59), 0)
                                        .setReversed(true)
                                        .lineToConstantHeading(new Vector2d(-54.5,35))
                                        .waitSeconds(1)
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}




//  .lineToConstantHeading(new Vector2d(26 ,     -42))
//          .splineToConstantHeading(new Vector2d(53,-34),0)
//          .setReversed(true)
//          .splineToConstantHeading(new Vector2d(10,-12),-Math.PI)
//          .splineToConstantHeading(new Vector2d(-58,-12),Math.PI)
//          .setReversed(false)
//          .splineToConstantHeading(new Vector2d(10,-12),0)
//          .splineToConstantHeading(new Vector2d(53,-34),0)
//          .setReversed(true)
//          .splineToConstantHeading(new Vector2d(10,-12),-Math.PI)
//          .splineToConstantHeading(new Vector2d(-58,-12),Math.PI)
//          .setReversed(false)
//          .splineToConstantHeading(new Vector2d(10,-12),0)
//          .splineToConstantHeading(new Vector2d(53,-34),0)
//          .setReversed(true)
//          .splineToConstantHeading(new Vector2d(10,-12),-Math.PI)
//          .splineToConstantHeading(new Vector2d(-40,-12),Math.PI)
//          .splineToConstantHeading(new Vector2d(-58,-24),Math.PI)
//          .setReversed(false)
//          .splineToConstantHeading(new Vector2d(-40,-12),0)
//          .splineToConstantHeading(new Vector2d(10,-12),0)
//          .splineToConstantHeading(new Vector2d(53,-34),0)
//          .setReversed(true)
//          .splineToConstantHeading(new Vector2d(10,-12),-Math.PI)
//          .splineToConstantHeading(new Vector2d(-40,-12),Math.PI)
//          .splineToConstantHeading(new Vector2d(-58,-24),Math.PI)
//          .setReversed(false)
//          .splineToConstantHeading(new Vector2d(-40,-12),0)
//          .splineToConstantHeading(new Vector2d(10,-12),0)
//          .splineToConstantHeading(new Vector2d(53,-34),0)

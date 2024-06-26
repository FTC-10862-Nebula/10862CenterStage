package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedWing {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34,-60, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-56,-30), Math.toRadians(180))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-60,-36))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(-40, -58))
                                .forward(90)
                                .strafeLeft(25)
                                .waitSeconds(1)
                                . strafeRight(25)
                                .back(90)
                                .lineToConstantHeading(new Vector2d(-60,-36))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(-40,-58))
                                .forward(90)
                                .strafeLeft(25)
                                .waitSeconds(1)
                                . strafeRight(25)


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
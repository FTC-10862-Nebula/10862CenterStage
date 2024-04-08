package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.entity.TrajectorySequenceEntity;

import java.util.Vector;

public class RedBackStage {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -65, Math.toRadians(90)))
                                .splineTo(new Vector2d(56, -40), Math.toRadians(0))
                                //Drop Pixel
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(22, -30), Math.toRadians(180))
                                //Drop Pixel on  backboard
                                .splineTo(new Vector2d(0, -12), Math.toRadians(360))
                                .setReversed(false)
                                .forward(55)
                                //intake
                                .forward(-55)
                                .splineTo(new Vector2d(56, -40), Math.toRadians(360))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}










































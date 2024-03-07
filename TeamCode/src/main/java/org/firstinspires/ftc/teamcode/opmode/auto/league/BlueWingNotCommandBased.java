package org.firstinspires.ftc.teamcode.opmode.auto.league;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequenceBuilder;

//@Autonomous
//public class BlueWingNotCommandBased {
//
//
//    public static void main(String[] args) {
//        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//            .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-35, 60, Math.toRadians(270)))
//                                .splineTo(new Vector2d(-34, 30), Math.toRadians(45))
//                                .setReversed(true)
//                                .lineToLinearHeading(new Pose2d(-50, 35))
//                                .strafeRight(23)
//                                .forward(100)
//                                .strafeLeft(50)
//                                .forward(10)
//
//                                .build()
//                );
//
//    }
//}
//
//public class BlueWingNotCommandBase {
//    public static void main(String[] args) {
//
//        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-35, 60, Math.toRadians(270)))
//                                //  .forward(8)
//                                .setReversed(true)
//                                .lineToLinearHeading(new Pose2d(-54, 46))
//                                .strafeRight(34)
//                                .back(7)
//                                .forward(112)
//                                .strafeLeft(50)
//                                .forward(10)
//
//                                .build()
//                );
//    }
//}

package org.firstinspires.ftc.teamcode.opmode.auto.regional;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence.TrajectorySequenceContainerFollowCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.SixWheel;
import org.firstinspires.ftc.teamcode.subsystems.vision.aprilTag.AprilTagVision;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Back;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.StrafeLeft;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.StrafeRight;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.TrajectorySequenceConstraints;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.TrajectorySequenceContainer;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Turn;
//438247 - Justin's Password bn

@Autonomous
public class RedWing extends MatchOpMode {
    // Subsystems
    private Drivetrain drivetrain;
    private AprilTagVision aprilTagVision;
    @Config
    public static class RightRegionalAutoConstants {

        public static Speed speed;
        public static class Speed {
            public static double baseVel = DriveConstants.MAX_VEL; // value
            public static double baseAccel = DriveConstants.MAX_ACCEL; // value
            public static double turnVel = DriveConstants.MAX_VEL; // value
            public static double turnAccel = DriveConstants.MAX_ANG_ACCEL; // value
            public static double slowVel = baseVel * 0.8; // value
            public static double slowAccel = baseAccel * 0.8; // value
            public static double slowTurnVel = turnVel * 0.8; // value
            public static double slowTurnAccel = turnAccel * 0.8; // value
            static TrajectorySequenceConstraints getPickupConstraints() {
                return new TrajectorySequenceConstraints(
                        (s, a, b, c) -> {
                            if (s > 18) {
                                return baseVel * 0.4;
                            } else {
                                return baseVel;
                            }

                        },
                        (s, a, b, c) -> baseAccel,
                        turnVel,
                        turnAccel
                );
            }
            static TrajectorySequenceConstraints getDropConstraints() {
                return new TrajectorySequenceConstraints(
                        (s, a, b, c) -> {
                            if (s > 20) {
                                return baseVel * 0.65;
                            } else {
                                return baseVel;
                            }

                        },
                        (s, a, b, c) -> baseAccel,
                        turnVel,
                        turnAccel
                );
            }
            static TrajectorySequenceConstraints getPreLoadDropConstraints() {
                return new TrajectorySequenceConstraints(
                        (s, a, b, c) -> {
                            if (s > 48) {
                                return baseVel * 0.6;
                            } else {
                                return baseVel;
                            }

                        },
                        (s, a, b, c) -> baseAccel,
                        turnVel,
                        turnAccel
                );
            }
            static TrajectorySequenceConstraints getParkConstraint() {
                return new TrajectorySequenceConstraints(
                        (s, a, b, c) -> {
                            return baseVel * 0.6;
                        },
                        (s, a, b, c) -> baseAccel,
                        turnVel,
                        turnAccel
                );
            }
            static TrajectorySequenceConstraints getBaseConstraints() {
                return new TrajectorySequenceConstraints(baseVel, baseAccel, turnVel, turnAccel);
            }
            static TrajectorySequenceConstraints getSlowConstraints() {
                return new TrajectorySequenceConstraints(slowVel, slowAccel, slowTurnVel, slowTurnAccel);
            }
        }

        public static Path path;
        public static class Path {
            public static PreLoad apreLoad;
            public static class PreLoad {
                public static Pose2dContainer startPose = new Pose2dContainer(-34, -52, Math.toRadians(270));
                public static Forward a = new Forward(-20);
                public static Forward b =  new Forward(28);
                public static Turn c = new Turn(-90);
                public static Forward d = new Forward(-90);
                static TrajectorySequenceContainer preload = new TrajectorySequenceContainer(Speed::getPreLoadDropConstraints, a, b, c,d);
            }

//            public static Cycle1Pickup bcycle1Pickup;
//            public static class Cycle1Pickup {
//                public static SetReversed a = new SetReversed(true);
//                public static SplineTo b = new SplineTo(48, -9.5, 0);
//                public static Back c = new Back(12.6);
//                static TrajectorySequenceContainer cycle1Pickup = new TrajectorySequenceContainer(Speed::getPickupConstraints, a, b, c);
//            }
//
//            public static Cycle1Drop ccycle1Drop;
//            public static class Cycle1Drop {
//                public static SetReversed a = new SetReversed(true);
//                public static Forward b = new Forward(16);
//                public static SplineTo c = new SplineTo(34.75, -14.15, -138);
//                static TrajectorySequenceContainer cycle1Drop = new TrajectorySequenceContainer(Speed::getDropConstraints, a, b, c);
//            }
//
//            public static Cycle2Pickup dcycle2Pickup;
//            public static class Cycle2Pickup {
//                public static SetReversed a = new SetReversed(true);
//                public static SplineTo b = new SplineTo(49.1, -9.5, 0);
//                public static Back c = new Back(13.69);
//                static TrajectorySequenceContainer cycle2Pickup = new TrajectorySequenceContainer(Speed::getPickupConstraints, a, b, c);
//            }
//
//            public static Cycle2Drop ecycle2Drop;
//            public static class Cycle2Drop {
//                public static SetReversed a = new SetReversed(true);
//                public static Forward b = new Forward(16);
//                public static SplineTo c = new SplineTo(34.75, -13.95, -135);//255
//                static TrajectorySequenceContainer cycle2Drop = new TrajectorySequenceContainer(Speed::getDropConstraints, a, b, c);
//            }
//
//            public static Cycle3Pickup fcycle3Pickup;
//            public static class Cycle3Pickup {
//                public static SetReversed a = new SetReversed(true);
//                public static SplineTo b = new SplineTo(51.5, -9.1, 0);
//                public static Back c = new Back(13.08);
//                static TrajectorySequenceContainer cycle3Pickup = new TrajectorySequenceContainer(Speed::getPickupConstraints, a, b, c);
//            }
//
//            public static Cycle3Drop gcycle3Drop;
//            public static class Cycle3Drop {
//                public static SetReversed a = new SetReversed(true);
//                public static Forward b = new Forward(15.5);//16
//                public static SplineTo c = new SplineTo(34.75, -13.95, -135);
//                static TrajectorySequenceContainer cycle3Drop = new TrajectorySequenceContainer(Speed::getDropConstraints, a, b, c);
//            }
            public static Park jpark;
            public static class Park {
                public static double leftX = 7;
                public static double midX = 36;
                public static double rightX = 64;
                public static double y = -15.2;
                public static double heading = -90;
                public static double endHeading = -180;
                public enum AutoPosition {
                    lEFT,
                    MID,
                    RIGHT
                }
                public static AutoPosition autoPosition = AutoPosition.lEFT;
                static TrajectorySequenceContainer getPark(double x) {
                    switch (autoPosition) {
                        case lEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new Back(4),
                                    new Turn(45),

                                    new Back(2),
                                    new StrafeRight(27.),
                                    new Forward(8)
                            );
                        case MID:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new Back(6),
                                    new Turn(45),
                                    new Forward(8)
                            );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new Back(6),
                                    new Turn(45),

                                    new Back(2),
                                    new StrafeLeft(31),
                                    new Forward(8)
//                                    Speed::getParkConstraint,
//                                    new Back(5),
//                                    new SplineToSplineHeading(x, y, heading, endHeading)
                            );
                    }
                }
            }
        }
    }

    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(new SixWheel(hardwareMap), telemetry);
        drivetrain.init();
        aprilTagVision = new AprilTagVision(hardwareMap, telemetry);
        while (!isStarted() && !isStopRequested()) {
            aprilTagVision.updateTagOfInterest();
            aprilTagVision.tagToTelemetry();
            telemetry.update();
        }
        this.matchStart();
    }

    public void matchStart() {
        double finalX = 0;
        switch (aprilTagVision.getTag()) {
            case 1:
//                finalX = RightRegionalAutoConstants.Path.Park.leftX;
                RightRegionalAutoConstants.Path.Park.autoPosition = RightRegionalAutoConstants.Path.Park.AutoPosition.lEFT;
                break;
            case 2:
//                finalX = RightRegionalAutoConstants.Path.Park.midX;
                RightRegionalAutoConstants.Path.Park.autoPosition = RightRegionalAutoConstants.Path.Park.AutoPosition.MID;
                break;
            case 3:
//                finalX = RightRegionalAutoConstants.Path.Park.rightX;
                RightRegionalAutoConstants.Path.Park.autoPosition = RightRegionalAutoConstants.Path.Park.AutoPosition.RIGHT;
                break;
        }
        drivetrain.setPoseEstimate(RightRegionalAutoConstants.Path.PreLoad.startPose.getPose());
        PoseStorage.trajectoryPose = RightRegionalAutoConstants.Path.PreLoad.startPose.getPose();
        schedule(
                new SequentialCommandGroup(
                        /* Preload */
                        new ParallelCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.PreLoad.preload)
                        ),
                        /* Park */
                        new SequentialCommandGroup(//Y is this not working...
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Park.getPark(finalX))
                        ),
                        new SequentialCommandGroup(
                        ),
                        run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),

                        /* Save Pose and end opmode*/

                        run(this::stop)
                )
        );
    }
}
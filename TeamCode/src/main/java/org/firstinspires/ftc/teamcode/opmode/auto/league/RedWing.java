package org.firstinspires.ftc.teamcode.opmode.auto.league;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence.TrajectorySequenceContainerFollowCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.Speed;
import org.firstinspires.ftc.teamcode.subsystems.drive.mec.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.mec.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Back;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.TrajectorySequenceContainer;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Turn;
@Disabled
@Autonomous
public class RedWing extends MatchOpMode {
    // Subsystems
    private Drivetrain drivetrain;
    @Config
    private static class RedWingConstants {
        private static Path path;
        private static class Path {
            public static DropSpikeMark aDropSpikeMark;
            public static class DropSpikeMark {
                public static Pose2dContainer startPose = new Pose2dContainer(-34, -52, Math.toRadians(270));
                public static Back a = new Back(20);
                public static Forward b =  new Forward(28);
                public static Turn c = new Turn(-90);
                public static Forward d = new Forward(-90);
                static TrajectorySequenceContainer dropSpikeMark =
                    new TrajectorySequenceContainer(Speed::getBaseConstraints, a, b, c,d);
            }

            public static d vvvv;
            public static class d  {
                public static Forward a =  new Forward(28);
                public static Turn b = new Turn(-90);
                public static Forward c = new Forward(-90);
                static TrajectorySequenceContainer cycle1Pickup =
                    new TrajectorySequenceContainer(Speed::getBaseConstraints, a, b, c);
            }
        }
    }

    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry), telemetry);
    }

    public void matchStart() {
        drivetrain.setPoseEstimate(RedWingConstants.Path.DropSpikeMark.startPose.getPose());
        PoseStorage.trajectoryPose = RedWingConstants.Path.DropSpikeMark.startPose.getPose();
        schedule(
                new SequentialCommandGroup(
                        /* Preload */
                        new ParallelCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain,
                                    RedWingConstants.Path.DropSpikeMark.dropSpikeMark)
                        ),
                        /* Park */
                        new SequentialCommandGroup(//Y is this not working...
                                new TrajectorySequenceContainerFollowCommand(drivetrain,
                                    RedWingConstants.Path.d.cycle1Pickup)
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
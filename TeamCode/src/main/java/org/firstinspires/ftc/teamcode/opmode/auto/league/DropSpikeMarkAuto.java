package org.firstinspires.ftc.teamcode.opmode.auto.league;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence.TrajectorySequenceContainerFollowCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.Speed;
import org.firstinspires.ftc.teamcode.subsystems.AutoDropper;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.climber.Climber;
import org.firstinspires.ftc.teamcode.subsystems.drive.mec.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.mec.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.TeamMarkerPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.FFVision;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.misc.Util;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.TrajectorySequenceContainer;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Turn;

import java.util.logging.Level;

@Autonomous
public class DropSpikeMarkAuto extends MatchOpMode {
    // Subsystems
    private Drivetrain drivetrain;
    private FFVision vision;
    private PowerIntake intake;
    private Climber climber;
    private Arm arm;
    private Shooter shooter;
    private Slide slide;
    private Claw claw;

    private AutoDropper dropper;
    @Config
    private static class RedBackstageConstants {
        public static Path path;
        public static class Path {
            public static DropSpikeMark aDropSpikeMark;
            public static class DropSpikeMark {
                public static Pose2dContainer startPose = new Pose2dContainer(10, -65, (90));
                public static Forward a = new Forward(-33.5);
                static TrajectorySequenceContainer preload =
                    new TrajectorySequenceContainer(Speed::getBaseConstraints, a);
                
                static TrajectorySequenceContainer getTurn(TeamMarkerPipeline.FFPosition position) {
                    switch (position) {
                        case LEFT:
                            return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Turn(90)
                            );
                        case MIDDLE:
                            return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Forward(1)
                            );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Turn(-90)
                            );
                    }
                }
            }
        }
    }

    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry), telemetry);
//        drivetrain.init();
        vision = new FFVision(hardwareMap, telemetry);
        intake = new PowerIntake(telemetry, hardwareMap, true);
        arm = new Arm(telemetry, hardwareMap, true);
        climber = new Climber(telemetry, hardwareMap, true);
        claw = new Claw(telemetry, hardwareMap, true);
        slide = new Slide(telemetry, hardwareMap, true);
        climber.setSetPointCommand(Climber.ClimbEnum.REST);
        dropper = new AutoDropper(telemetry, hardwareMap, true);
    }

    @Override
    public void disabledPeriodic() {
        vision.setPosition(vision.getPosition());
        Util.logger(this, telemetry, Level.INFO, "Current Position", vision.getFinalPosition());
    }

    public void matchStart() {
        TeamMarkerPipeline.FFPosition position = vision.getPosition();

        drivetrain.setPoseEstimate(RedBackstageConstants.Path.DropSpikeMark.startPose.getPose());
        PoseStorage.trajectoryPose = RedBackstageConstants.Path.DropSpikeMark.startPose.getPose();

        schedule(
            new SequentialCommandGroup( //TODO:TEST!
                /* YellowPixel */
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        RedBackstageConstants.Path.DropSpikeMark.preload),
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                    RedBackstageConstants.Path.DropSpikeMark.getTurn(position)),
                    new ParallelCommandGroup(
                            dropper.dropperSetPositionCommand(AutoDropper.DropPos.DROP)
                    ),
//                intake.setSetPointCommand(PowerIntake.IntakePower.OUTTAKE_PURPLE),

                /* Save Pose and end opmode*/
                run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),
                run(this::stop)
            )
        );
    }
}
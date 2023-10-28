package org.firstinspires.ftc.teamcode.opmode.auto.league;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.arm.position.LowCommand;
import org.firstinspires.ftc.teamcode.commands.arm.position.ResetCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence.DisplacementCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence.TrajectorySequenceContainerFollowCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.Speed;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.climber.Climber;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.SixWheel;
import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.TeamMarkerPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.Vision;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Back;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.TrajectorySequenceContainer;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Turn;
@Disabled
@Autonomous
public class RedBackstage extends MatchOpMode {
    // Subsystems
    private Drivetrain drivetrain;
//    private AprilTagVision aprilTagVision;
    private Vision vision;
    private PowerIntake intake;
    private Climber climber;
    private Arm arm;
    private Shooter shooter;
    private Slide slide;
    private Claw claw;
    @Config
    private static class RedBackstageConstants {
        public static Path path;
        public static class Path {
            public static DropSpikeMark aDropSpikeMark;
            public static class DropSpikeMark {
                public static Pose2dContainer startPose = new Pose2dContainer(10, -65, (90));
                public static Forward a = new Forward(20);
                static TrajectorySequenceContainer preload =
                    new TrajectorySequenceContainer(Speed::getPreLoadDropConstraints, a);
                
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

            public static DropPurplePixel dropPurplePixel;
            public static class DropPurplePixel {
                static TrajectorySequenceContainer getDrop(TeamMarkerPipeline.FFPosition position) {
                    switch (position) {
                        case LEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new Turn(90),
                                    new Forward(3),
                                    new Turn(90),
                                    new Back(20)
                            );
                        case MIDDLE:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new Forward(0),
                                    new Turn(90),
                                    new Back(20)
                                    );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new Turn(90),
                                    new Forward(-3),
                                    new Turn(90),
                                    new Back(20)
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
        vision = new Vision(hardwareMap, telemetry);
        intake = new PowerIntake(telemetry, hardwareMap, true);
        arm = new Arm(telemetry, hardwareMap, true);
        climber = new Climber(telemetry, hardwareMap, true);
        claw = new Claw(telemetry, hardwareMap, true);
        slide = new Slide(telemetry, hardwareMap, true);
        shooter = new Shooter(telemetry, hardwareMap, true);

        climber.setSetPointCommand(Climber.ClimbEnum.REST);
        shooter.ready();
        while (!isStarted() && !isStopRequested()) {
            vision.periodic();
            telemetry.update();
        }
        this.matchStart();
    }

    public void matchStart() {
        TeamMarkerPipeline.FFPosition position = vision.getPosition();

        drivetrain.setPoseEstimate(RedBackstageConstants.Path.DropSpikeMark.startPose.getPose());
        PoseStorage.trajectoryPose = RedBackstageConstants.Path.DropSpikeMark.startPose.getPose();

        schedule(
            new SequentialCommandGroup(
                /* YellowPixel */
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        RedBackstageConstants.Path.DropSpikeMark.preload),
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                    RedBackstageConstants.Path.DropSpikeMark.getTurn(position)),
                intake.setSetPointCommand(PowerIntake.IntakePower.OUTTAKE_YELLOW),

                /* PurplePixel/Drop */
                new SequentialCommandGroup(//Y is this not working...
                    new TrajectorySequenceContainerFollowCommand(drivetrain,
                            RedBackstageConstants.Path.DropPurplePixel.getDrop(position)),
                    new DisplacementCommand(30, new LowCommand(slide, arm, claw))
                ),
                new ResetCommand(slide,arm, claw),
                

                /* Save Pose and end opmode*/
                run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),
                run(this::stop)
            )
        );
    }
}
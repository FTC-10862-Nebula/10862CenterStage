package org.firstinspires.ftc.teamcode.opmode.auto.league;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.drive.mec.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.mec.MecanumDrive;
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
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.StrafeLeft;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.StrafeRight;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.TrajectorySequenceContainer;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Turn;
@Disabled
@Autonomous
public class BlueBackstage extends MatchOpMode {
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
    private static class BlueBackstageConstants {
        public static class Path {
            public static DropSpikeMark aDropSpikeMark;

            public static class DropSpikeMark {
                public static Pose2dContainer startPose = new Pose2dContainer(10, 65, 270);
                public static Forward a = new Forward(30);

                static TrajectorySequenceContainer getTurnDrop(TeamMarkerPipeline.FFPosition position) {
                    switch (position) {
                        case LEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Turn(-90)
                            );
                        case MIDDLE:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Forward(2)
                            );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Turn(90)
                            );
                    }
                }
                static TrajectorySequenceContainer preload =
                        new TrajectorySequenceContainer(Speed::getPreLoadDropConstraints, a);

                static TrajectorySequenceContainer getTurn(TeamMarkerPipeline.FFPosition position) {
                    switch (position) {
                        case LEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Back(2),
                                    new Turn(90)
                            );
                        case MIDDLE:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Forward(-2)
                            );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Back(2),
                                    new Turn(-90)
                            );
                    }
                }
            }

            public static DropPurplePixel dropPurplePixel;

            public static class DropPurplePixel {
                public static Back a = new Back(20);
                public static Turn b = new Turn(90);
                public static Back c = new Back(30);

                static TrajectorySequenceContainer getDrop(TeamMarkerPipeline.FFPosition position) {
                    switch (position) {
                        case LEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new StrafeLeft(20)
                            );
                        case MIDDLE:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new StrafeLeft(15)
                            );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new StrafeLeft(10)
                            );
                    }
                }
                public static Back d =  new Back(2);
                public static Forward e = new Forward(2);
                static TrajectorySequenceContainer dropPurplePixel =
                        new TrajectorySequenceContainer(Speed::getPreLoadDropConstraints, a,b,c);
            }
        }
    }

    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry), telemetry);
//        drivetrain.init();
        vision = new Vision(hardwareMap, telemetry);
        intake = new PowerIntake(telemetry, hardwareMap, true);
        arm = new Arm(telemetry, hardwareMap, true);
        climber = new Climber(telemetry, hardwareMap, true);
        claw = new Claw(telemetry, hardwareMap, true);
        slide = new Slide(telemetry, hardwareMap, true);
        climber.setSetPointCommand(Climber.ClimbEnum.REST);
    }

    public void matchStart() {
        TeamMarkerPipeline.FFPosition position = vision.getPosition();
    
        drivetrain.setPoseEstimate(BlueBackstageConstants.Path.DropSpikeMark.startPose.getPose());
        PoseStorage.trajectoryPose = BlueBackstageConstants.Path.DropSpikeMark.startPose.getPose();
    
        drivetrain.setPoseEstimate(BlueBackstageConstants.Path.DropSpikeMark.startPose.getPose());
        PoseStorage.trajectoryPose = BlueBackstageConstants.Path.DropSpikeMark.startPose.getPose();
        schedule(
                /* YellowPixel */
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        BlueBackstage.BlueBackstageConstants.Path.DropSpikeMark.preload),
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        BlueBackstage.BlueBackstageConstants.Path.DropSpikeMark.getTurnDrop(position)),
                intake.setSetPointCommand(PowerIntake.IntakePower.OUTTAKE_PURPLE),
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        BlueBackstage.BlueBackstageConstants.Path.DropSpikeMark.getTurn(position)),

                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        BlueBackstage.BlueBackstageConstants.Path.DropPurplePixel.dropPurplePixel),
                /* PurplePixel/Drop */
                new SequentialCommandGroup(
                        new TrajectorySequenceContainerFollowCommand(drivetrain,
                                BlueBackstage.BlueBackstageConstants.Path.DropPurplePixel.getDrop(position)),
                        new DisplacementCommand(5, new LowCommand(slide, arm, claw))
                ),
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        new TrajectorySequenceContainer(Speed::getPreLoadDropConstraints,
                                BlueBackstage.BlueBackstageConstants.Path.DropPurplePixel.d)),
                new WaitCommand(200),
                new InstantCommand(()->claw.setBothClaw(Claw.ClawPos.OPEN_POS)),
                new WaitCommand(500),
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        new TrajectorySequenceContainer(Speed::getPreLoadDropConstraints,
                                BlueBackstage.BlueBackstageConstants.Path.DropPurplePixel.e)),
                new ResetCommand(slide,arm, claw),

                /* Save Pose and end opmode*/
                run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),
                run(this::stop)
        );
    }
}
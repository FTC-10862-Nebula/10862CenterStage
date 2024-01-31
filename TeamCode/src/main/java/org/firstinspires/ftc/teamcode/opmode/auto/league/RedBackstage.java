package org.firstinspires.ftc.teamcode.opmode.auto.league;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.arm.position.ResetCommand;
import org.firstinspires.ftc.teamcode.commands.arm.position.SlideCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence.DisplacementCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence.TrajectorySequenceContainerFollowCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.Speed;
import org.firstinspires.ftc.teamcode.subsystems.AutoDropper;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.drive.mec.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.mec.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.TeamMarkerPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.FFVision;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.misc.Util;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Back;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.StrafeLeft;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.StrafeRight;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.TrajectorySequenceContainer;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Turn;

import java.util.logging.Level;

//@Disabled
@Autonomous
    (preselectTeleOp = "TeleOpMain")
public class RedBackstage extends MatchOpMode {
    // Subsystems
    private Drivetrain drivetrain;
//    private AprilTagVision aprilTagVision;
    private FFVision FFVision;
    private PowerIntake intake;
//    private Climber climber;
    private Arm arm;
//    private Shooter shooter;
    private Slide slide;
    private Claw claw;
    private AutoDropper dropper;
//    @Config
            public static DropSpikeMark dropSpikeMark;
            public static class DropSpikeMark {
                public static Pose2dContainer startPose = new Pose2dContainer(10, 65, 270);
//                public static Forward a = new Forward(27);
                static TrajectorySequenceContainer preload =
                        new TrajectorySequenceContainer(Speed::getBaseConstraints, new Forward(27));
                static TrajectorySequenceContainer getTurnDrop(TeamMarkerPipeline.FFPosition position) {
                    switch (position) {
                        case LEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                   new StrafeLeft(16),
                                    new Forward(1.5)
                            );
                        case MIDDLE:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Back(4.5)
                            );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Back(5),
                                    new Forward(0.6),
                                    new Turn(90),
                                    new Back(4)
                            );
                    }
                }


//                public static Back b = new Back(4.5);
                static TrajectorySequenceContainer getTurn(TeamMarkerPipeline.FFPosition position) {
                    switch (position) {
                        case LEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                        new Back(0));

                        case MIDDLE:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Forward(10),
                                    new Turn(-90)
                            );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new StrafeLeft(.5),
                                    new Forward(7.5),
                                    new Turn(180)
                            );
                    }
                }
            }

            public static DropYellowPixel dropYellowPixel;

            public static class DropYellowPixel {
                static TrajectorySequenceContainer getDropPurplePixel(TeamMarkerPipeline.FFPosition position) {
                    switch (position) {
                        case LEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Back(30)
                            );
                        case MIDDLE:
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Back(36)
                            );
                    }
                }
              //  public static Back a = new Back(27);
                //public static Turn b = new Turn(90);
               // public static Back c = new Back(38);

                static TrajectorySequenceContainer getDrop(TeamMarkerPipeline.FFPosition position) {
                    switch (position) {
                        case LEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new StrafeLeft(10)
                            );
                        case MIDDLE:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new StrafeRight(30)
                            );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new StrafeRight(24.5)
                            );
                    }
                }
                public static Back d =  new Back(9);
                public static Forward e = new Forward(5.5);
                static TrajectorySequenceContainer dropPurplePixel =
                        new TrajectorySequenceContainer(Speed::getBaseConstraints);
            }


    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry), telemetry);
//        drivetrain.init();
        FFVision = new FFVision(hardwareMap, telemetry);
        intake = new PowerIntake(telemetry, hardwareMap, true);
        arm = new Arm(telemetry, hardwareMap, true);
//        climber = new Climber(telemetry, hardwareMap, true);
        claw = new Claw(telemetry, hardwareMap, true);
        slide = new Slide(telemetry, hardwareMap, true);
        dropper = new AutoDropper(telemetry, hardwareMap, true);
//        climber.setSetPointCommand(Climber.ClimbEnum.REST);
    }

    @Override
    public void disabledPeriodic() {
        FFVision.setPosition(FFVision.getPosition());
        Util.logger(this, telemetry, Level.INFO, "Current Position", FFVision.getFinalPosition());
    }
    public void matchStart() {
        TeamMarkerPipeline.FFPosition position  = TeamMarkerPipeline.FFPosition.LEFT;

        drivetrain.setPoseEstimate(DropSpikeMark.startPose.getPose());
        PoseStorage.trajectoryPose = DropSpikeMark.startPose.getPose();

        schedule(
            new SequentialCommandGroup(
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                    new TrajectorySequenceContainer(Speed::getBaseConstraints, new Forward(27))),
//                new TrajectorySequenceContainerFollowCommand(drivetrain,
//                        DropSpikeMark.getTurnDrop(position)),
             //   new WaitCommand(2000),
 //               dropper.dropperSetPositionCommand(AutoDropper.DropPos.DROP),
                new WaitCommand(500),

                /* YellowPixel/Drop */
                new ParallelCommandGroup(
                    new TrajectorySequenceContainerFollowCommand(drivetrain,
                            DropYellowPixel.dropPurplePixel),
                    new WaitCommand(500),
                    claw.setBClaw(Claw.ClawPos.CLOSE_POS)
                ),
                new SequentialCommandGroup(
                    new TrajectorySequenceContainerFollowCommand(drivetrain,
                            DropYellowPixel.getDrop(position)),
                    new DisplacementCommand(3.5,
                            new SlideCommand(slide, arm, claw, Slide.SlideEnum.AUTO_LOW))
                ),
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        new TrajectorySequenceContainer(Speed::getBaseConstraints,
                                DropYellowPixel.d)),
                new WaitCommand(500),
                claw.setBothClaw(Claw.ClawPos.OPEN_POS),
                new WaitCommand(2000),
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        new TrajectorySequenceContainer(Speed::getBaseConstraints,
                                DropYellowPixel.e)),
                new ResetCommand(slide,arm, claw),

                /* Save Pose and end opmode*/
                run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),
                run(this::stop)

                )
            );
    }
}
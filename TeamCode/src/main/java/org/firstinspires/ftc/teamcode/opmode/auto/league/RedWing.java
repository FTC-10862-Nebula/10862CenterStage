package org.firstinspires.ftc.teamcode.opmode.auto.league;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AutoIntakeCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.sensor.SensorColor;
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
@Config
public class RedWing extends MatchOpMode {
    // Subsystems
    private Drivetrain drivetrain;
    //    private AprilTagVision aprilTagVision;
    private FFVision vision;
    private PowerIntake intake;
    //    private Climber climber;
    private Arm arm;
    //    private Shooter shooter;
    private Slide slide;
    private Claw claw;
    private AutoDropper dropper;
    private SensorColor sensorColor;


    private static Path path;
    private static class Path {
        public static Path.DropSpikeMark aDropSpikeMark;
        public static class DropSpikeMark {
            public static Pose2dContainer startPose = new Pose2dContainer(10, 65, 270);
            //^^ Has Wrong Coordinates
            static TrajectorySequenceContainer getTurnDrop(TeamMarkerPipeline.FFPosition position) {
                switch (position) {
                    case LEFT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Turn(90),
                                new Back(3)
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
                                new Turn(-90),
                            new Back(2)
                        );
                }
            }

            static TrajectorySequenceContainer getTurn(TeamMarkerPipeline.FFPosition position) {
                switch (position) {
                    case LEFT:
                        return new TrajectorySequenceContainer(
                            Speed::getBaseConstraints,
                            new StrafeLeft(34),
                            new Turn(180),
                            new Forward(19)
                        );
                    case MIDDLE:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Forward(2),
                                new StrafeRight(28),
                                new Turn(-90),
                                new StrafeRight(38.5)

                        );
                    default:
                    case RIGHT:
                        return new TrajectorySequenceContainer(
                            Speed::getBaseConstraints,
                            new Forward(21.5),
                            new StrafeRight(31)
                        );
                }
            }
        }

        public static Path.GetPixel getPixel;
        public static class GetPixel {
           //          public static StrafeLeft a = new StrafeLeft(30.);
            public static Forward b = new Forward(7.5);

            static TrajectorySequenceContainer getPixel =
                    new TrajectorySequenceContainer(Speed::getBaseConstraints, b);
        }

        public static Path.DropPixel dropPixel;
        public static class DropPixel {
            static TrajectorySequenceContainer getDrop(TeamMarkerPipeline.FFPosition position) {
                switch (position) {
                    case LEFT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new StrafeLeft(28)
                        );
                    case MIDDLE:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new StrafeLeft(33)
                        );
                    default:
                    case RIGHT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new StrafeLeft(36.5)
                        );
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
//        climber = new Climber(telemetry, hardwareMap, true);
        claw = new Claw(telemetry, hardwareMap, true);
        slide = new Slide(telemetry, hardwareMap, true);
//        climber.setSetPointCommand(Climber.ClimbEnum.REST);
        dropper = new AutoDropper(telemetry, hardwareMap, true);
        sensorColor = new SensorColor(telemetry,hardwareMap );


    }

    @Override
    public void disabledPeriodic() {
        vision.setPosition(vision.getPosition());
        Util.logger(this, telemetry, Level.INFO, "Current Position", vision.getFinalPosition());
    }
    public void matchStart() {
        TeamMarkerPipeline.FFPosition position = vision.getPosition();

        drivetrain.setPoseEstimate(Path.DropSpikeMark.startPose.getPose());
        PoseStorage.trajectoryPose = Path.DropSpikeMark.startPose.getPose();

        schedule(
                new SequentialCommandGroup(
                        /* YellowPixel */
                    new TrajectorySequenceContainerFollowCommand(drivetrain,
                        new TrajectorySequenceContainer(Speed::getBaseConstraints, new Back(33))),
                    new TrajectorySequenceContainerFollowCommand(drivetrain,
                           Path.DropSpikeMark.getTurnDrop(position)),
                        dropper.dropperSetPositionCommand(AutoDropper.DropPos.DROP),
                    new TrajectorySequenceContainerFollowCommand(drivetrain,
                            Path.DropSpikeMark.getTurn(position)),
                        new InstantCommand(intake::setFive),
                    /*** get pixel ***/
                    new ParallelCommandGroup(
                            claw.setFClaw(Claw.ClawPos.OPEN_POS),
                            new TrajectorySequenceContainerFollowCommand(drivetrain,
                                    Path.GetPixel.getPixel)
                    ),
                        new AutoIntakeCommand(claw, intake, sensorColor),
                    /**drop pixel**/
                    new ParallelCommandGroup(
                            new TrajectorySequenceContainerFollowCommand(drivetrain,
                                new TrajectorySequenceContainer(Speed::getFastConstraints,
                                        new Back(120))
//                                new DisplacementCommand(14,
//                                intake.setSetPointCommand(PowerIntake.IntakePower.STOP)),
//                                new DisplacementCommand(20, claw.setBothClaw(Claw.ClawPos.CLOSE_POS))
                            )
//                            intake.setSetPointCommand(PowerIntake.IntakePower.STOP),
//                            claw.setBothClaw(Claw.ClawPos.CLOSE_POS)
                    ),

                    new SequentialCommandGroup(
                            new TrajectorySequenceContainerFollowCommand(drivetrain,
                                    Path.DropPixel.getDrop(position)),
                            new DisplacementCommand(3.5,
                                    new SlideCommand(slide, arm, claw, Slide.SlideEnum.AUTO_LOW))
                    ),
                    new TrajectorySequenceContainerFollowCommand(drivetrain,
                            new TrajectorySequenceContainer(Speed::getBaseConstraints,
                                    new Back(9))),
                    new WaitCommand(300),
                    claw.setBothClaw(Claw.ClawPos.OPEN_POS),
                    new WaitCommand(500),
                    new TrajectorySequenceContainerFollowCommand(drivetrain,
                            new TrajectorySequenceContainer(Speed::getBaseConstraints,
                                    new Forward(6))),
                    new ResetCommand(slide,arm, claw),

                    /* Save Pose and end opmode*/
                    run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),
                    run(this::stop))
        );
    }
}
package org.firstinspires.ftc.teamcode.opmode.auto.league;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.Vision;
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
    private Vision vision;
    private PowerIntake intake;
    //    private Climber climber;
    private Arm arm;
    //    private Shooter shooter;
    private Slide slide;
    private Claw claw;
    private AutoDropper dropper;

    private static Path path;
    private static class Path {
        public static DropSpikeMark aDropSpikeMark;
        public static class DropSpikeMark {
            public static Pose2dContainer startPose = new Pose2dContainer(10, 65, 270);
            //^^ Has Wrong Coordinates
            public static Back a = new Back(35);
            static TrajectorySequenceContainer preload =
                    new TrajectorySequenceContainer(Speed::getBaseConstraints, a);

            static TrajectorySequenceContainer getTurnDrop(TeamMarkerPipeline.FFPosition position) {
                switch (position) {
                    case LEFT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Turn(90),
                                new Back(8)
                        );
                    case MIDDLE:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Forward(3),
                                new Back(3.5)
                        );
                    default:
                    case RIGHT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Turn(90)
                        );
                }
            }


            //                public static Back b = new Back(4.5);
            static TrajectorySequenceContainer getTurn(TeamMarkerPipeline.FFPosition position) {
                switch (position) {
                    case LEFT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Back(0.001)
                        );
                    case MIDDLE:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Turn(90),
                                new Back(15)
                        );
                    default:
                    case RIGHT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Back(15)
                        );
                }
            }
        }

        public static GetPixel getPixel;
        public static class GetPixel {
            public static StrafeRight a = new StrafeRight(15);//MOVDE THIS
            public static Forward b = new Forward(5);

            static TrajectorySequenceContainer getPixel =
                    new TrajectorySequenceContainer(Speed::getBaseConstraints, a,b);
        }

        public static DropPixel dropPixel;
        public static class DropPixel {
            public static Back a = new Back(78);

            static TrajectorySequenceContainer getDrop(TeamMarkerPipeline.FFPosition position) {
                switch (position) {
                    case LEFT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new StrafeLeft(36.)
                        );
                    case MIDDLE:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new StrafeLeft(30)
                        );
                    default:
                    case RIGHT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new StrafeLeft(24.5)
                        );
                }
            }
            public static Back b =  new Back(9);
            public static Forward c = new Forward(5.5);
        }
    }


    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry), telemetry);
//        drivetrain.init();
        vision = new Vision(hardwareMap, telemetry);
        intake = new PowerIntake(telemetry, hardwareMap, true);
        arm = new Arm(telemetry, hardwareMap, true);
//        climber = new Climber(telemetry, hardwareMap, true);
        claw = new Claw(telemetry, hardwareMap, true);
        slide = new Slide(telemetry, hardwareMap, true);
//        climber.setSetPointCommand(Climber.ClimbEnum.REST);
        dropper = new AutoDropper(telemetry, hardwareMap, true);


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
                        new ParallelCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain,
                                        Path.DropSpikeMark.preload)
//                    new DisplacementCommand(14, new InstantCommand(()->intake.setDown())),
//                    new InstantCommand(()->intake.setDown())
                        ),
                        new TrajectorySequenceContainerFollowCommand(drivetrain,
                                Path.DropSpikeMark.getTurnDrop(position)),
                        new ParallelCommandGroup(
                                dropper.dropperSetPositionCommand(AutoDropper.DropPos.DROP)
                        ),
                        new WaitCommand(1000),
                        new TrajectorySequenceContainerFollowCommand(drivetrain,
                                Path.DropSpikeMark.getTurn(position)),


                        /*** get pixel ***/
                        new ParallelCommandGroup(
                                claw.setFClaw(Claw.ClawPos.OPEN_POS),
                                new TrajectorySequenceContainerFollowCommand(drivetrain,
                                        Path.GetPixel.getPixel),
                                new DisplacementCommand(25,new SequentialCommandGroup(
                                        new InstantCommand(intake::setFive),
                                        intake.setSetPointCommand(PowerIntake.IntakePower.AUTO_INTAKE)
                                ))
                        ),
                        new WaitCommand(1000),

                        /**drop pixel**/
                        new ParallelCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain,
                                        new TrajectorySequenceContainer(Speed::getFastConstraints,
                                                Path.DropPixel.a)
                                ),
                                intake.setSetPointCommand(PowerIntake.IntakePower.STOP),
                                claw.setBothClaw(Claw.ClawPos.CLOSE_POS)
                        ),

                        new SequentialCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain,
                                        Path.DropPixel.getDrop(position)),
                                new DisplacementCommand(3.5,
                                        new SlideCommand(slide, arm, claw, Slide.SlideEnum.AUTO_LOW))
                        ),
                        new TrajectorySequenceContainerFollowCommand(drivetrain,
                                new TrajectorySequenceContainer(Speed::getBaseConstraints,
                                        Path.DropPixel.b)),
                        new WaitCommand(500),
                        claw.setBothClaw(Claw.ClawPos.OPEN_POS),
                        new WaitCommand(1500),
                        new TrajectorySequenceContainerFollowCommand(drivetrain,
                                new TrajectorySequenceContainer(Speed::getBaseConstraints,
                                        Path.DropPixel.c)),
                        new ResetCommand(slide,arm, claw),

                        /* Save Pose and end opmode*/
                        run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),
                        run(this::stop)
                )
        );
    }
}
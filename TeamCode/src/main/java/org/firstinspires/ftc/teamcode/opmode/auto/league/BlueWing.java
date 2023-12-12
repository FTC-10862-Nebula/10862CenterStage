package org.firstinspires.ftc.teamcode.opmode.auto.league;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence.TrajectorySequenceContainerFollowCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.Speed;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.climber.Climber;
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
        (preselectTeleOp = "TeleOpMain")

public class BlueWing extends MatchOpMode {
    // Subsystems
    private Drivetrain drivetrain;
    //    private AprilTagVision aprilTagVision;
    private Vision vision;
    private PowerIntake intake;
    private Climber climber;
    private Arm arm;
    //    private Shooter shooter;
    private Slide slide;
    private Claw claw;

    @Config
    private static class Path {
        public static DropSpikeMark aDropSpikeMark;

        public static class DropSpikeMark {
            public static Pose2dContainer startPose = new Pose2dContainer(10, 65, 270);
            public static Forward a = new Forward(33);
            static TrajectorySequenceContainer preload =
                    new TrajectorySequenceContainer(Speed::getPreLoadDropConstraints, a);

            static TrajectorySequenceContainer getTurnDrop(TeamMarkerPipeline.FFPosition position) {
                switch (position) {
                    case LEFT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Forward(1.5),
                                new Turn(90),
                                new Back(0.8)
                        );
                    case MIDDLE:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Forward(4),
                                new Back(4.5)
                        );
                    default:
                    case RIGHT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Forward(0.8),
                                new Turn(-90),
                                new Forward(8),
                                new Back(7)
//                                    new Back(2)
                        );
                }
            }


            //                public static Back b = new Back(4.5);
            static TrajectorySequenceContainer getTurn(TeamMarkerPipeline.FFPosition position) {
                switch (position) {
                    case LEFT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new StrafeLeft(2),
                                new Back(4.1),
                                new Turn(-90),
                                new Back(2.5),
                                new StrafeRight(2)
                        );
                    case MIDDLE:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new Back(0.5),
                                new Turn(0.01)
                        );
                    default:
                    case RIGHT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new StrafeRight(.5),
                                new Back(3.8),
                                new Turn(90),
                                new Back(3.5)
                        );
                }
            }
        }

//        public static Park park;

//        public static class Park {
//            public static Back a = new Back(28);
//            public static Turn b = new Turn(90);
//            public static Back c = new Back(70);
//
//            static TrajectorySequenceContainer park =
//                    new TrajectorySequenceContainer(Speed::getPreLoadDropConstraints, a, b, c);
//        }
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
                                intake.setSetPointCommand(PowerIntake.IntakePower.OUTTAKE_PURPLE)
//                    new TrajectorySequenceContainerFollowCommand(drivetrain,
//                            new TrajectorySequenceContainer(Speed::getPreLoadDropConstraints,
//                                    Path.DropSpikeMark.b))
                        ),
                        new WaitCommand(1000),
                        new ParallelCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain,
                                        Path.DropSpikeMark.getTurn(position)),
                                intake.setSetPointCommand(PowerIntake.IntakePower.STOP)
                        ),




                        /* PurplePixel/Drop */
//                        new ParallelCommandGroup(
//                                new TrajectorySequenceContainerFollowCommand(drivetrain,
//                                        Path.Park.park)
//                        ),
//                        intake.setSetPointCommand(PowerIntake.IntakePower.OUTTAKE),
                        /* Save Pose and end opmode*/
                        run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),
                        run(this::stop)
                ));
    }
}
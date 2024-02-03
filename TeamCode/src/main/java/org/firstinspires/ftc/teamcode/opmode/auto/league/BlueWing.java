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
public class BlueWing extends MatchOpMode {
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


    private static Path path;
    private static class Path {
        public static DropSpikeMark aDropSpikeMark;
            public static class DropSpikeMark {
                public static Pose2dContainer startPose = new Pose2dContainer(10, 65, 270);
                //^^ Has Wrong Coordinates
                public static                Back a = new Back(33);
                static TrajectorySequenceContainer preload =
                        new TrajectorySequenceContainer(Speed::getBaseConstraints, a);

                static TrajectorySequenceContainer getTurnDrop(TeamMarkerPipeline.FFPosition position) {
                    switch (position) {
                        case LEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Turn(90),
                                    new Back(3)
//                                    new Forward(6)
                            );
                        case MIDDLE:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Back(1.5)
//                                    new Forward(3),
//                                    new Back(3.5)
                            );
                        default:
                       case RIGHT:
                           return new TrajectorySequenceContainer(
                                   Speed::getBaseConstraints,
//                                    new StrafeLeft(8)
                                    new Turn(-90)

//                                    new Forward(19)
                            );
//                        case RIGHT:


                    }
                }


                //                public static Back b = new Back(4.5);
                static TrajectorySequenceContainer getTurn(TeamMarkerPipeline.FFPosition position) {
                    switch (position) {
                        case LEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Forward(24.5),
                                    new StrafeLeft(32.5)
                                  //  new Forward(1)
                            );
                        case MIDDLE:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Forward(4),
//                                    new Forward(4),
                                   new StrafeLeft(28),
                                   new Turn(90),
                                  new Forward(0.5),
                                    new StrafeLeft(36.5)

                            );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new StrafeRight(31),
                                    new Turn(180),
                        new Forward(20)
                            );
                    }
                }
            }

            public static GetPixel getPixel;
            public static class GetPixel {
//                public static StrafeLeft a = new StrafeLeft(30.);
                public static Forward b = new Forward(7);

                static TrajectorySequenceContainer getPixel =
                        new TrajectorySequenceContainer(Speed::getBaseConstraints, b);
            }

        public static DropPixel dropPixel;
        public static class DropPixel {
            public static Back a = new Back(122);

            static TrajectorySequenceContainer getDrop(TeamMarkerPipeline.FFPosition position) {
                switch (position) {
                    case LEFT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new StrafeRight(44)
                        );
                    case MIDDLE:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new StrafeRight(33)
      //                          new Back(6)
                        );
                    default:
                    case RIGHT:
                        return new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,
                                new StrafeRight(23)
                        );
                }
            }
            public static Back b =  new Back(6);
            public static Forward c = new Forward(6);
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
    }

    @Override
    public void disabledPeriodic() {
        vision.setPosition(vision.getPosition());
        Util.logger(this, telemetry, Level.INFO, "Current Position", vision.getFinalPosition());
    }
    public void matchStart() {
        TeamMarkerPipeline.FFPosition position = vision.getPosition();
      //  position = TeamMarkerPipeline.FFPosition.LEFT;

        drivetrain.setPoseEstimate(Path.DropSpikeMark.startPose.getPose());
        PoseStorage.trajectoryPose = Path.DropSpikeMark.startPose.getPose();

        schedule(
                new SequentialCommandGroup(
                        /* YellowSPixel */
                        new ParallelCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain,
                                        Path.DropSpikeMark.preload)

//                    new DisplacementCommand(14, new InstantCommand(()->intake.setDown())),
//                    new InstantCommand(()->intake.setDown())
                        ),
                       new TrajectorySequenceContainerFollowCommand(drivetrain,
                                Path.DropSpikeMark.getTurnDrop(position)),
                        dropper.dropperSetPositionCommand(AutoDropper.DropPos.DROP),
                        new WaitCommand(1000),
                        new TrajectorySequenceContainerFollowCommand(drivetrain,
                                Path.DropSpikeMark.getTurn(position)),
                            //    dropper.dropperSetPositionCommand(AutoDropper.DropPos.DROP),


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
                        new WaitCommand(2000),
                        intake.setSetPointCommand(PowerIntake.IntakePower.AUTO_OUTTAKE),
                        //When sensors work, do the conditional command please

                        /**drop pixel**/
                        new ParallelCommandGroup(
                            new TrajectorySequenceContainerFollowCommand(drivetrain,
                                    new TrajectorySequenceContainer(Speed::getFastConstraints,
                                            Path.DropPixel.a),
                                    new DisplacementCommand(14,
                                            intake.setSetPointCommand(PowerIntake.IntakePower.STOP)),
                                    //maybe also hAve bot outtake
//                                    new AutoIntakeCommand(claw, intake)
                                    new DisplacementCommand(20, claw.setBothClaw(Claw.ClawPos.CLOSE_POS))
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
                                        Path.DropPixel.b)),
                        new WaitCommand(300),
                        claw.setBothClaw(Claw.ClawPos.OPEN_POS),
                        new WaitCommand(500),
                        dropper.dropperSetPositionCommand(AutoDropper.DropPos.HOLD),
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
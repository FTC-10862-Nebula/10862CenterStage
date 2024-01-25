package org.firstinspires.ftc.teamcode.opmode.auto.league;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.arm.position.ResetCommand;
import org.firstinspires.ftc.teamcode.commands.arm.position.SlideCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence.DisplacementCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence.TrajectorySequenceContainerFollowCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.Speed;
import org.firstinspires.ftc.teamcode.subsystems.AutoDropper;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.climber.Climber;
import org.firstinspires.ftc.teamcode.subsystems.drive.mec.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.mec.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.TeamMarkerPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.FFVision;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.misc.Util;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Back;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.LineToSplineHeading;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.StrafeLeft;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.StrafeRight;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.TrajectorySequenceContainer;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Turn;

import java.util.logging.Level;

//@Disabled
@Autonomous(preselectTeleOp = "TeleOpMain")

public class BlueBackstage extends MatchOpMode {
    // Subsystems
    private Drivetrain drivetrain;
    //    private AprilTagVision aprilTagVision;
    private FFVision FFVision;
    private PowerIntake intake;
    private Climber climber;
    private Arm arm;
    private AutoDropper dropper;
//    private Shooter shooter;
    private Slide slide;
    private Claw claw;
    
    @Config
        private static class Path {
            public static DropSpikeMark aDropSpikeMark;

            public static class DropSpikeMark {
                public static Pose2dContainer startPose = new Pose2dContainer(10, 65, 270);
                public static Back a = new Back(27);
                static TrajectorySequenceContainer preload =
                        new TrajectorySequenceContainer(Speed::getBaseConstraints, a);

                TeamMarkerPipeline.FFPosition position = TeamMarkerPipeline.FFPosition.LEFT;

                static TrajectorySequenceContainer getTurnDrop(TeamMarkerPipeline.FFPosition position) {
                    switch (position) {
                        case LEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new StrafeRight(16),
                                    new Forward(1.5)
                                  //  new Turn(90)
  //                                  new Back(0.8)
                            );
                        case MIDDLE:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Back(4.5)
                                 //   new Forward(10)
                              //      new Back(4.5)
                            );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new Back(5),
                                    new Forward(0.6),
                                    new Turn(-90),
                                    new Back(4)
//                                    new Back(2)
                            );
                    }
                }


                //                public static Back b = new Back(4.5);
                static TrajectorySequenceContainer getTurn(TeamMarkerPipeline.FFPosition position) {
                    switch (position) {
                        case LEFT:
                           // return new TrajectorySequenceContainer(
                                   // Speed::getBaseConstraints,
                                  //  new StrafeLeft(2),
                                   // new Back(4.1),
                                   // new Turn(-90),
                                  //  new Back(2.5),
                            //new Forward(1.5),
                          //  new Turn(90),
                                 //   new StrafeRight(5)
                                   // new StrafeRight(2)
                        //   );
                        case MIDDLE:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
       //                             new Back(0.5),
                                    new Forward(10),
                                    new Turn(90)
                            );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new StrafeRight(.5),
                                    new Forward(7.5),
                                    new Turn(-180)
                            );
                    }
                }
            }

            public static DropYellowPixel dropYellowPixel;

            public static class DropYellowPixel {
     //           public static Back a = new Back(5);
  //              public static Turn b = new Turn(-90);

            //    public static Back c = new Back(36);
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

                static TrajectorySequenceContainer getDrop(TeamMarkerPipeline.FFPosition position) {
                    switch (position) {
                        case LEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new StrafeRight(10)
                            );
                        case MIDDLE:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new StrafeLeft(15)
                            );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getBaseConstraints,
                                    new StrafeLeft(-18)
                            );
                    }
                }

                public static Back d = new Back (6);
                public static Forward e = new Forward(5);
                static TrajectorySequenceContainer dropPurplePixel =
                        new TrajectorySequenceContainer(Speed::getBaseConstraints);
            }
        }

    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry), telemetry);
//        drivetrain.init();
        FFVision = new FFVision(hardwareMap, telemetry);
        intake = new PowerIntake(telemetry, hardwareMap, true);
        arm = new Arm(telemetry, hardwareMap, true);
        climber = new Climber(telemetry, hardwareMap, true);
        claw = new Claw(telemetry, hardwareMap, true);
        slide = new Slide(telemetry, hardwareMap, true);
        dropper = new AutoDropper(telemetry, hardwareMap, true);
        climber.setSetPointCommand(Climber.ClimbEnum.REST);
    }

    @Override
    public void disabledPeriodic() {
      //  TeamMarkerPipeline.FFPosition position = TeamMarkerPipeline.FFPosition.LEFT;

  //      FFVision.setPosition(FFVision.getPosition());
        Util.logger(this, telemetry, Level.INFO, "Current Position", FFVision.getFinalPosition());
    }
    public void matchStart() {
 //       TeamMarkerPipeline.FFPosition position = FFVision.getPosition();
        TeamMarkerPipeline.FFPosition position = TeamMarkerPipeline.FFPosition.LEFT;

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
                     //   new ParallelCommandGroup(
      //                          intake.setSetPointCommand(PowerIntake.IntakePower.OUTTAKE_PURPLE)
                                new ParallelCommandGroup(
                                        dropper.dropperSetPositionCommand(AutoDropper.DropPos.HOLD),
                                        dropper.dropperSetPositionCommand(AutoDropper.DropPos.DROP)),
                                new WaitCommand(1000)
//                    new TrajectorySequenceContainerFollowCommand(drivetrain,
//                            new TrajectorySequenceContainer(Speed::getPreLoadDropConstraints,
//                                    Path.DropSpikeMark.b))
                        ),
                        new WaitCommand(1000),
                        new ParallelCommandGroup(
                 //               new TrajectorySequenceContainerFollowCommand(drivetrain,
                   //                     Path.DropSpikeMark.getTurn(position)),
                     //           intake.setSetPointCommand(PowerIntake.IntakePower.STOP)
                        ),




                        /* PurplePixel/Drop */
                        new ParallelCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain,
                                        Path.DropYellowPixel.dropPurplePixel),
                                new WaitCommand(500),
                                claw.setBClaw(Claw.ClawPos.CLOSE_POS)
                        ),
                        new SequentialCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain,
                                        Path.DropYellowPixel.getDrop(position)),
                               new DisplacementCommand(3.5,
                                        new SlideCommand(slide, arm, claw, Slide.SlideEnum.AUTO_LOW))
                        ),
                        new TrajectorySequenceContainerFollowCommand(drivetrain,
                                new TrajectorySequenceContainer(Speed::getBaseConstraints,
                                        Path.DropYellowPixel.d)),
                        new WaitCommand(500),
                        claw.setBothClaw(Claw.ClawPos.OPEN_POS),
                        new WaitCommand(2000),
                        new TrajectorySequenceContainerFollowCommand(drivetrain,
                                new TrajectorySequenceContainer(Speed::getBaseConstraints,
                                        Path.DropYellowPixel.e)),
                        new ResetCommand(slide,arm, claw),

                        /* Save Pose and end opmode*/
                        run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),
                        run(this::stop)
                );
    }
}
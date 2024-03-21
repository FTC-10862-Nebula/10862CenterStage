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


    public static DropSpikeMark aDropSpikeMark;

    public static class DropSpikeMark {
        public static Pose2dContainer startPose = new Pose2dContainer(10, 65, 270);

        //^^ Has Wrong Coordinates
        static TrajectorySequenceContainer getTurnDrop(TeamMarkerPipeline.FFPosition position) {
            switch (position) {
                default:
                case LEFT:
                    return new TrajectorySequenceContainer(
                            Speed::getBaseConstraints,
                            new Back(31),
                            new Turn(90),
                            new Back(3)
                    );
                case MIDDLE:
                    return new TrajectorySequenceContainer(
                            Speed::getBaseConstraints,
                            new Back(30)
                    );

                case RIGHT:
                    return new TrajectorySequenceContainer(
                            Speed::getBaseConstraints,
                            new Back(30),
                            new Turn(-90),
                            new Back(3)
                    );
            }
        }

        static TrajectorySequenceContainer getTurn(TeamMarkerPipeline.FFPosition position) {
            switch (position) {
                default:
                case LEFT:
                    return new TrajectorySequenceContainer(
                            Speed::getBaseConstraints,
                            new StrafeLeft(31.5)
                            //new Turn(180),
                           // new Forward(23)
                            // new Forward(5)
                    );
                case MIDDLE:
                    return new TrajectorySequenceContainer(
                            Speed::getBaseConstraints,
                            new StrafeRight(29.5),
                            new Turn(90),
                            new StrafeRight(33)

                    );

                case RIGHT:
                    return new TrajectorySequenceContainer(
                            Speed::getBaseConstraints,
                            new Forward(3),
                            new StrafeRight(33)
                    );
            }
        }
    }

    public static DropPixel dropPixel;

    public static class DropPixel {
        static TrajectorySequenceContainer getBack(TeamMarkerPipeline.FFPosition position) {
            switch (position) {
                default:
                    case LEFT:
                        return new TrajectorySequenceContainer(
                                Speed::getFastConstraints,
                                new Turn(180),
                                new Back(97)
                        );
                case MIDDLE:
                    return new TrajectorySequenceContainer(
                            Speed::getFastConstraints,
                            new Back(120)
                    );
                case RIGHT:
                    return new TrajectorySequenceContainer(
                            Speed::getFastConstraints,
                            new Back(93)
                    );
            }
        }

        static TrajectorySequenceContainer getDrop(TeamMarkerPipeline.FFPosition position) {
            switch (position) {
                case LEFT:
                default:
                    return new TrajectorySequenceContainer(
                            Speed::getFastConstraints,
                            new StrafeLeft(23.5),
                            new Back(7.5)
                    );
                case MIDDLE:
                    return new TrajectorySequenceContainer(
                            Speed::getFastConstraints,
                            new StrafeLeft(33),
                            new Back(7.5)

                    );
                case RIGHT:
                    return new TrajectorySequenceContainer(
                            Speed::getFastConstraints,
                            new StrafeLeft(39.5),
                            new Back(7.5)
                    );
            }
        }

        static TrajectorySequenceContainer getPark(TeamMarkerPipeline.FFPosition position) {
            switch (position) {
                case LEFT:
                    return new TrajectorySequenceContainer(
                            Speed::getFastConstraints,
                            new StrafeRight(26.5),
                            new Back(10)
                    );
                case MIDDLE:
                    return new TrajectorySequenceContainer(
                            Speed::getFastConstraints,
                            new StrafeRight(34),
                            new Back(5)
                    );
                default:
                case RIGHT:
                    return new TrajectorySequenceContainer(
                            Speed::getFastConstraints,
                            new StrafeRight(39),
                           new Back(10)
                    );
            }
        }
    }


    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry), telemetry);
        vision = new FFVision(hardwareMap, telemetry);
        intake = new PowerIntake(telemetry, hardwareMap, true);
        arm = new Arm(telemetry, hardwareMap, true);
//        climber = new Climber(telemetry, hardwareMap, true);
//        climber.setSetPointCommand(Climber.ClimbEnum.REST);
        claw = new Claw(telemetry, hardwareMap, true);
        slide = new Slide(telemetry, hardwareMap, true);

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
      //  TeamMarkerPipeline.FFPosition position = TeamMarkerPipeline.FFPosition.RIGHT;

        drivetrain.setPoseEstimate(DropSpikeMark.startPose.getPose());
        PoseStorage.trajectoryPose = DropSpikeMark.startPose.getPose();

        schedule(
            new SequentialCommandGroup(
                /*** YellowPixel ***/
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                       DropSpikeMark.getTurnDrop(position)),
                    dropper.dropperSetPositionCommand(AutoDropper.DropPos.DROP),

                /*** get pixel ***/
                new ParallelCommandGroup(
                    new TrajectorySequenceContainerFollowCommand(drivetrain,
                        DropSpikeMark.getTurn(position))
                ),
          //      new AutoIntakeCommand(claw, intake, sensorColor, drivetrain),

                /**drop pixel**/
                new SequentialCommandGroup(
                    new TrajectorySequenceContainerFollowCommand(drivetrain,
                            DropPixel.getBack(position)) ,
                    new ParallelCommandGroup(
                    new TrajectorySequenceContainerFollowCommand(drivetrain,
                        DropPixel.getDrop(position)),
                    new DisplacementCommand(3.5,
                        new SlideCommand(slide, arm, claw, Slide.SlideEnum.AUTO_LOW))
                    )
                ),

//                new WaitCommand(300),
                claw.setBothClaw(Claw.ClawPos.OPEN_POS),
                new WaitCommand(800),
                new ParallelCommandGroup(
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                    new TrajectorySequenceContainer(Speed::getBaseConstraints,
                        new Forward(7))),
                new ResetCommand(slide,arm, claw)
                        ),

                new TrajectorySequenceContainerFollowCommand(drivetrain,
                    DropPixel.getPark(position)),

                /* Save Pose and end opmode*/
                run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),
                run(this::stop)
            )
        );
    }
}
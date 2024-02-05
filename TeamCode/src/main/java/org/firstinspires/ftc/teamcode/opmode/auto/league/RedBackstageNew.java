package org.firstinspires.ftc.teamcode.opmode.auto.league;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
@Autonomous(preselectTeleOp = "TeleOpMain")
public class RedBackstageNew extends MatchOpMode {
    // Subsystems
    private Drivetrain drivetrain;
    private FFVision vision;
    private PowerIntake intake;
    private Arm arm;
    private Slide slide;
    private Claw claw;
    private AutoDropper dropper;

    public static class DropSpikeMark {
        public static Pose2dContainer startPose = new Pose2dContainer(10, 65, (270));

        static TrajectorySequenceContainer getTurn(TeamMarkerPipeline.FFPosition position) {
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
                            new Back(4.4),
                            new Turn(90),
                            new Back(4)
                    );
            }
        }

        static TrajectorySequenceContainer getTurnDrop(TeamMarkerPipeline.FFPosition position) {
            switch (position) {
                case LEFT:
                default:
                    return new TrajectorySequenceContainer(
                            Speed::getBaseConstraints,
                            new Back(0.01)
                    );
                case MIDDLE:
                    return new TrajectorySequenceContainer(
                            Speed::getBaseConstraints,
                            new Forward(10),
                            new Turn(-90)
                    );
                case RIGHT:
                    return new TrajectorySequenceContainer(
                            Speed::getBaseConstraints,
                            new StrafeLeft(0.5),
                            new Forward(7.5),
                            new Turn(180)
                    );
            }
        }
    }
 public static class DropYellowPixel {
     static TrajectorySequenceContainer getToBack(TeamMarkerPipeline.FFPosition position) {
         switch (position) {
             case LEFT:
             default:
                 return new TrajectorySequenceContainer(
                         Speed::getBaseConstraints,
                         new Back(30)
                 );
             case MIDDLE:
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
             default:
                 return new TrajectorySequenceContainer(
                         Speed::getBaseConstraints,
                         new StrafeLeft(10)
                 );
             case MIDDLE:
                 return new TrajectorySequenceContainer(
                         Speed::getBaseConstraints,
                         new StrafeRight(30)
                 );
             case RIGHT:
                 return new TrajectorySequenceContainer(
                         Speed::getBaseConstraints,
                         new StrafeRight(24.5)
                 );
         }
     }

//     public static Back b = new Back(9);
 }


    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry), telemetry);
//        drivetrain.init();
        vision = new FFVision(hardwareMap, telemetry);
        intake = new PowerIntake(telemetry, hardwareMap, true);
        arm = new Arm(telemetry, hardwareMap, true);
        //      climber = new Climber(telemetry, hardwareMap, true);
        claw = new Claw(telemetry, hardwareMap, true);
        slide = new Slide(telemetry, hardwareMap, true);
        //    climber.setSetPointCommand(Climber.ClimbEnum.REST);
        dropper = new AutoDropper(telemetry, hardwareMap, true);
    }

    @Override
    public void disabledPeriodic() {
        vision.setPosition(vision.getPosition());
        Util.logger(this, telemetry, Level.INFO, "Current Position", vision.getFinalPosition());
    }

    public void matchStart() {
        TeamMarkerPipeline.FFPosition position = TeamMarkerPipeline.FFPosition.LEFT;
        drivetrain.setPoseEstimate(RedBackstage.DropSpikeMark.startPose.getPose());
        PoseStorage.trajectoryPose = RedBackstage.DropSpikeMark.startPose.getPose();

        schedule(
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        new TrajectorySequenceContainer(Speed::getBaseConstraints,
                                new Back(27))),
                new WaitCommand(1000),
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        DropSpikeMark.getTurn(position)),
                new WaitCommand(500),
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        DropSpikeMark.getTurnDrop(position)),
                new WaitCommand(500),
                dropper.dropperSetPositionCommand(AutoDropper.DropPos.DROP),
                new WaitCommand(500),
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        DropYellowPixel.getToBack(position)),
                new WaitCommand(500),
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        DropYellowPixel.getDrop(position)),
                new TrajectorySequenceContainerFollowCommand(drivetrain,
                        new TrajectorySequenceContainer(
                                Speed::getBaseConstraints,new Back(9))),

                /* Save Pose and end opmode*/
                run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),
                run(this::stop)


    );

    }
}
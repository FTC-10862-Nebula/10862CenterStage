package org.firstinspires.ftc.teamcode.opmode.auto.misc;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.arm.position.HighCommand;
import org.firstinspires.ftc.teamcode.commands.arm.position.ResetCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.tank.paths.DropYellowPixel;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.climber.Climber;
import org.firstinspires.ftc.teamcode.subsystems.drive.tank.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.Vision;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;
@Disabled
@Autonomous
public class TestAutonWithoutCam extends MatchOpMode {
    private Drivetrain drivetrain;
    //    private AprilTagVision aprilTagVision;
    private Vision vision;
    private PowerIntake intake;
    private Climber climber;
    private Arm arm;
    //    private Shooter shooter;
    private Slide slide;
    private Claw claw;
    @Override
    public void robotInit() {
    
        drivetrain = new Drivetrain(hardwareMap, true);
//        drivetrain.init();
        vision = new Vision(hardwareMap, telemetry);
        intake = new PowerIntake(telemetry, hardwareMap, true);
        arm = new Arm(telemetry, hardwareMap, true);
        climber = new Climber(telemetry, hardwareMap, true);
        claw = new Claw(telemetry, hardwareMap, true);
        slide = new Slide(telemetry, hardwareMap, true);
//        shooter = new Shooter(telemetry, hardwareMap, true);
    
        climber.setSetPointCommand(Climber.ClimbEnum.REST);
//        shooter.ready();
        while (!isStarted() && !isStopRequested()) {
            vision.periodic();
            telemetry.update();
        }
        this.matchStart();
    }


    public void matchStart() {new SequentialCommandGroup(
        new DriveForwardCommand(drivetrain, 20),
    
        new TurnToCommand(drivetrain, 90, true),
        new DriveForwardCommand(drivetrain, 1),
    
        intake.setSetPointCommand(PowerIntake.IntakePower.OUTTAKE_PURPLE),
        new WaitCommand(2000),
        intake.setSetPointCommand(PowerIntake.IntakePower.STOP),
    
    
        new DriveForwardCommand(drivetrain, 3),
    
        new DropYellowPixel(drivetrain,vision,intake,climber, arm,slide,claw),
    
        new TurnToCommand(drivetrain, 270),
        new DriveForwardCommand(drivetrain, -15),
    
        new HighCommand(slide, arm, claw),
        new DriveForwardCommand(drivetrain, -2),
        claw.setClawPos(Claw.ClawPos.OPEN_POS),
        new WaitCommand(1000),
    
        new ResetCommand(slide, arm, claw)
    );
    
    }


};
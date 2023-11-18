package org.firstinspires.ftc.teamcode.opmode.auto.tank;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.arm.position.LowCommand;
import org.firstinspires.ftc.teamcode.commands.arm.position.ResetCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.tank.paths.DropPurplePixel;
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
@Autonomous//(preselectTeleOp = "TeleOpMain")
public class TankRedBackstage extends MatchOpMode {
    
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
//        while (!isStarted() && !isStopRequested()) {
//
//        }
//        this.matchStart();
    }

@Override
public void disabledPeriodic() {
    vision.setPosition(vision.getPosition());
    vision.periodic();
    telemetry.update();
}

@Override
public void matchStart() {
    schedule(
        new SequentialCommandGroup(
            new DropPurplePixel(drivetrain, vision,intake),
            new DropYellowPixel(drivetrain,vision,intake,climber),

            new TurnCommand(drivetrain, 90),
            new DriveForwardCommand(drivetrain, -6),

            new LowCommand(slide, arm, claw),
            new WaitCommand(200),
            new DriveForwardCommand(drivetrain, -14),
            new WaitCommand(1500),
            claw.setClawPos(Claw.ClawPos.OPEN_POS),
            new WaitCommand(1000),

            new ResetCommand(slide, arm, claw)
//            new DriveForwardCommand(drivetrain, -4)
        )
        
    );
    }
}
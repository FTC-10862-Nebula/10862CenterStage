package org.firstinspires.ftc.teamcode.opmode.auto.tank.preload;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.drive.DriveCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.tank.paths.DropPurplePixel;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.climber.Climber;
import org.firstinspires.ftc.teamcode.subsystems.drive.tank.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.Vision;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;

@Autonomous//(preselectTeleOp = "TeleOpMain")
public class TankBlueWingPreload extends MatchOpMode {
    
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
            new DropPurplePixel(drivetrain, vision,intake,climber, arm,slide,claw),
            new DriveForwardCommand(drivetrain, -20),
            new TurnCommand(drivetrain, 270),
            new DriveForwardCommand(drivetrain, -20)
    
    
            )
        
    );
    }
}
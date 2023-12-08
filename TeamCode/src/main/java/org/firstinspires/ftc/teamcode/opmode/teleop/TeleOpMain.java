package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ClimberMoveManual;
import org.firstinspires.ftc.teamcode.commands.arm.ArmMoveManual;
import org.firstinspires.ftc.teamcode.commands.arm.position.HighCommand;
import org.firstinspires.ftc.teamcode.commands.arm.position.LowCommand;
import org.firstinspires.ftc.teamcode.commands.arm.position.MiddleCommand;
import org.firstinspires.ftc.teamcode.commands.arm.position.ResetCommand;
import org.firstinspires.ftc.teamcode.commands.arm.slide.SlideMoveManual;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.climber.Climber;
import org.firstinspires.ftc.teamcode.subsystems.drive.mec.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.mec.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;
import org.firstinspires.ftc.teamcode.util.teleop.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;

@Config
@TeleOp
public class TeleOpMain extends MatchOpMode {
    //TODO: Add a on/off switch for drivetrain
    private GamepadEx driverGamepad, operatorGamepad;


   //  Subsystems
        private Drivetrain drivetrain;
    private Slide slide;
    private PowerIntake intake;
    private Arm arm;
    private Claw claw;
//    private Shooter shooter;
//    private PowerClimber climb;
    private Climber climb;
//    private CycleTracker cycleTracker;
    public TeleOpMain() {}

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        claw = new Claw(telemetry, hardwareMap, true);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry), telemetry);  //Works
        intake = new PowerIntake(telemetry, hardwareMap, true);
      //  climb = new PowerClimber(telemetry, hardwareMap, true);
        climb = new Climber(telemetry,hardwareMap, true);
        arm = new Arm(telemetry, hardwareMap, true);
////        shooter = new Shooter(telemetry, hardwareMap, true);
        slide = new Slide(telemetry, hardwareMap, true);
    }


    @Override
    public void configureButtons() {
        //Claw
        Button closeF = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
                .whenPressed(claw.setBothClaw(Claw.ClawPos.CLOSE_POS)));
        Button closeB= (new GamepadTrigger(operatorGamepad,  GamepadKeys.Trigger.RIGHT_TRIGGER)
                .whenPressed(claw.setBothClaw(Claw.ClawPos.OPEN_POS)));

//        Button openF = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
//               .whenPressed(claw.setFClaw(Claw.ClawPos.OPEN_POS)));
//       Button openB= (new GamepadButton(operatorGamepad,  GamepadKeys.Button.RIGHT_BUMPER)
//               .whenPressed(claw.setBClaw(Claw.ClawPos.OPEN_POS)));

        //Arm
 //       Button armTransfer = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN))
//               .whenPressed(arm.armSetPositionCommand(Arm.ArmPos.TRANSFER));
//        Button armOuttake = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP))
//                .whenPressed(arm.armSetPositionCommand(Arm.ArmPos.OUTTAKE));
//        arm.setDefaultCommand(new ArmMoveManual(arm, operatorGamepad::getLeftX));//TODO:Test

        //Intake
        Trigger INTAKE = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
            .whenPressed(new InstantCommand(intake::setDown))
            .whileHeld(intake.setSetPointCommand(PowerIntake.IntakePower.INTAKE)))
            .whenReleased(intake.setSetPointCommand(PowerIntake.IntakePower.STOP))
            .whenReleased(new InstantCommand(intake::setUp));
        Trigger OUTTAKE = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)
            .whenPressed(new InstantCommand(intake::setDown))
            .whileHeld(intake.setSetPointCommand(PowerIntake.IntakePower.OUTTAKE)))
//                .whenPressed(cycleTracker.trackCycle())
            .whenReleased(intake.setSetPointCommand(PowerIntake.IntakePower.STOP))
            .whenReleased(new InstantCommand(intake::setUp));

        //Shooter
//        Button shoot = (new GamepadButton(operatorGamepad, Button.RIGHT_BUMPER))
//                .whenPressed(shooter.shoot());
    
        //Climber
        Button climbButton  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP))
                .whileHeld(climb.setSetPointCommand(Climber.ClimbEnum.CLIMB))
                .whenReleased(climb.setSetPointCommand(Climber.ClimbEnum.REST));
//        Button climbUp  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP))
//                .whileHeld(climb.setSetPointCommand(Climber.ClimbEnum.CLIMB));
//        Button climbDown  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN))
//                .whileHeld(climb.setSetPointCommand(Climber.ClimbEnum.REST));
        climb.setDefaultCommand(new ClimberMoveManual(climb, operatorGamepad::getRightY));
        Button resetClimb  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
                .whenPressed(new InstantCommand(()->climb.resetEncoder()));

        //Slide
        Button slideRest  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A))
            .whenPressed(new ResetCommand(slide, arm, claw));
        Button slideLow  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X))
            .whenPressed(new LowCommand(slide,arm,claw));
        Button slideMid  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B))
            .whenPressed(new MiddleCommand(slide,arm, claw));
        Button slideHigh  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y))
            .whenPressed(new HighCommand(slide,arm, claw));
        slide.setDefaultCommand(new SlideMoveManual(slide, operatorGamepad::getLeftY));
        Button resetSlide  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.BACK))
                .whenPressed(new InstantCommand(()->slide.resetEncoder()));

        //Driver
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, true));
//        Button slowModeBumper = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER))
//            .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad));
//        Button recenterIMU = (new GamepadButton(driverGamepad, GamepadKeys.Button.A))
//                .whenPressed(new InstantCommand(drivetrain::reInitializeIMU));


        //y - up/dowm
        //x- right left
    }

    @Override
    public void matchLoop() {}
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){
    }
}

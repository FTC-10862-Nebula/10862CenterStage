package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.arm.position.HighCommand;
import org.firstinspires.ftc.teamcode.commands.arm.position.LowCommand;
import org.firstinspires.ftc.teamcode.commands.arm.position.MiddleCommand;
import org.firstinspires.ftc.teamcode.commands.arm.position.ResetCommand;
import org.firstinspires.ftc.teamcode.commands.arm.slide.SlideMoveManual;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.tank.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.tank.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.climber.PowerClimber;
import org.firstinspires.ftc.teamcode.subsystems.drive.tank.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;
import org.firstinspires.ftc.teamcode.util.teleop.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;

@Config
@TeleOp
public class TeleOpMain extends MatchOpMode {

    // Gamepad
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;


    // Subsystems
    private Drivetrain drivetrain;
    private Slide slide;
    private PowerIntake intake;
    private Arm arm;
    private Claw claw;
//    private Shooter shooter;
    private PowerClimber climb;
    
    public TeleOpMain() {
    }
//    private CycleTracker cycleTracker;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        claw = new Claw(telemetry, hardwareMap, true);
        drivetrain = new Drivetrain(hardwareMap, true);  //Works
//        drivetrain.init();
        intake = new PowerIntake(telemetry, hardwareMap, true);
        climb = new PowerClimber(telemetry, hardwareMap, true);
        arm = new Arm (telemetry, hardwareMap, true);
//        shooter = new Shooter(telemetry, hardwareMap, true);
        slide = new Slide(telemetry, hardwareMap, true);
    }


    @Override
    public void configureButtons() {
        //        //Claw
        Button up = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
                .whenPressed(claw.setClawPos(Claw.ClawPos.OPEN_POS)));
        Button close= (new GamepadTrigger(operatorGamepad,  GamepadKeys.Trigger.RIGHT_TRIGGER)
                .whenPressed(claw.setClawPos(Claw.ClawPos.CLOSE_POS)));
//
//        //Arm
//        Button armTransfer = (new GamepadButton(operatorGamepad, Button.DPAD_DOWN))
//                .whenPressed(arm.armSetPositionCommand(Arm.ArmPos.TRANSFER));
//        Button armOuttake = (new GamepadButton(operatorGamepad, Button.DPAD_UP))
//                .whenPressed(arm.armSetPositionCommand(Arm.ArmPos.OUTTAKE));
//
//        //Intake
        Trigger INTAKE = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)
            .whenPressed(new InstantCommand(intake::setDown))
            .whileHeld(intake.setSetPointCommand(PowerIntake.IntakePower.INTAKE)))
            .whenReleased(intake.setSetPointCommand(PowerIntake.IntakePower.STOP));
        
        Trigger outtake = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
            .whileHeld(intake.setSetPointCommand(PowerIntake.IntakePower.OUTTAKE)))
//                .whenPressed(cycleTracker.trackCycle())
            .whenReleased(intake.setSetPointCommand(PowerIntake.IntakePower.STOP));
//
//
//        //Shooter
//        Button shoot = (new GamepadButton(operatorGamepad, Button.RIGHT_BUMPER))
//                .whenPressed(shooter.shoot());
    
        //Climber
        Button moveUp  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP))
            .whileHeld(climb.setPowerCommand(PowerClimber.ClimbPower.UP))
            .whenReleased(climb.setPowerCommand(0));
        Button moveDown  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN))
            .whileHeld(climb.setPowerCommand(PowerClimber.ClimbPower.DOWN))
            .whenReleased(climb.setPowerCommand(0));
    
        //Slide
        //TODO m ake the commands close the claw and stuff
    
    
        Button slideRest  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A))
            .whenPressed(new ResetCommand(slide, arm, claw));
        Button slideLow  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X))
            .whenPressed(new LowCommand(slide,arm,claw));
        Button slideMid  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B))
            .whenPressed(new MiddleCommand(slide,arm, claw));
        Button slideHigh  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y))
            .whenPressed(new HighCommand(slide,arm, claw));


        /*
         *  DRIVER
         */
    
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad));
        Button slowModeBumper = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER))
            .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad));

//        slide.setDefaultCommand(slide.slideMoveManual(operatorGamepad::getRightY));
        slide.setDefaultCommand(new SlideMoveManual(slide, operatorGamepad::getRightY));
//        pivot.setDefaultCommand(new PivotMoveManual(pivot, operatorGamepad::getLeftY));
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

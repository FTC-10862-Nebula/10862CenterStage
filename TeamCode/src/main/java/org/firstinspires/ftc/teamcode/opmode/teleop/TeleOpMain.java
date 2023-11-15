package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultTankDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.SlowTankDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.climber.PowerClimber;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
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

//        claw = new Claw(telemetry, hardwareMap, true);
//        drivetrain = new Drivetrain(new SixWheel(hardwareMap), telemetry);  //Works
//        drivetrain.init();
//        intake = new PowerIntake(telemetry, hardwareMap, false);
//        climb = new PowerClimber(telemetry, hardwareMap, false);
//        arm = new Arm (telemetry, hardwareMap, true);
//        shooter = new Shooter(telemetry, hardwareMap, true);
        slide = new Slide(telemetry, hardwareMap, true);
    }


    @Override
    public void configureButtons() {
        //        //Claw
//        Button up = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
//                .whenPressed(claw.setClawPos(Claw.ClawPos.OPEN_POS)));
//        Button close= (new GamepadButton(driverGamepad,  Button.DPAD_DOWN)
//                .whenPressed(claw.setClawPos(Claw.ClawPos.CLOSE_POS)));
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
        Button moveUp  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A))
            .whenPressed(climb.setPowerCommand(PowerClimber.ClimbPower.UP));
        Button moveDown  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B))
            .whenPressed(climb.setPowerCommand(PowerClimber.ClimbPower.DOWN));
    
        //Slide
        Button slideRest  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A))
                .whenPressed(slide.setSetPointCommand(Slide.SlideEnum.TRANSFER));
        Button slideLow  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X))
                .whenPressed(slide.setSetPointCommand(Slide.SlideEnum.LOW));
        Button slideMid  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B))
                .whenPressed(slide.setSetPointCommand(Slide.SlideEnum.MID));
        Button slideHigh  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y))
                .whenPressed(slide.setSetPointCommand(Slide.SlideEnum.HIGH));


        /*
         *  DRIVER
         */
    
        drivetrain.setDefaultCommand(new DefaultTankDriveCommand(drivetrain, driverGamepad));
        Button slowModeBumper = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER))
            .whileHeld(new SlowTankDriveCommand(drivetrain, driverGamepad));

//
//        /*
//         * OPERATOR
//         */
//
//        slide.setDefaultCommand(slide.slideMoveManual(operatorGamepad::getRightY));
//        slide.setDefaultCommand(new SlideMoveManual(slide, operatorGamepad::getRightY));
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

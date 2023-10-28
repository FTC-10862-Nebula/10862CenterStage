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
import org.firstinspires.ftc.teamcode.subsystems.climber.PowerClimber;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.SixWheel;
import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;
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
//    private Slide slide;
    private PowerIntake intake;
//    private Arm arm;
//    private Claw claw;
//    private Shooter shooter;
    private PowerClimber climb;
//    private CycleTracker cycleTracker;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

//        claw = new Claw(telemetry, hardwareMap, true);
        drivetrain = new Drivetrain(new SixWheel(hardwareMap), telemetry);  //Works
        drivetrain.init();
        intake = new PowerIntake(telemetry, hardwareMap, true);
        climb = new PowerClimber(telemetry, hardwareMap, true);
//        arm = new Arm (telemetry, hardwareMap, true);
//        shooter = new Shooter(telemetry, hardwareMap, true);
//        slide = new Slide(telemetry, hardwareMap, false);
    }


    @Override
    public void configureButtons() {
        //Intake
        Trigger INTAKE = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)
                .whenPressed(new InstantCommand(intake::setDown))
                .whileHeld(intake.setSetPointCommand(PowerIntake.IntakePower.INTAKE)))
                .whenReleased(intake.setSetPointCommand(PowerIntake.IntakePower.STOP));
        Trigger outtake = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
                .whileHeld(intake.setSetPointCommand(PowerIntake.IntakePower.OUTTAKE)))
//                .whenPressed(cycleTracker.trackCycle())
                .whenReleased(intake.setSetPointCommand(PowerIntake.IntakePower.STOP));

        //Climber
        Button moveUp  = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP))
            .whenPressed(climb.setPowerCommand(PowerClimber.ClimbPower.UP));
        Button moveDown  = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN))
            .whenPressed(climb.setPowerCommand(PowerClimber.ClimbPower.DOWN));

        drivetrain.setDefaultCommand(new DefaultTankDriveCommand(drivetrain, driverGamepad));
        Button slowModeBumper = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER))
            .whileHeld(new SlowTankDriveCommand(drivetrain, driverGamepad));
        
        
        
        
//        /**Operator**/
//        //Intake
//        Trigger INTAKE2 = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)
//            .whenPressed(new InstantCommand(intake::setDown))
//            .whileHeld(intake.setSetPointCommand(PowerIntake.IntakePower.INTAKE)))
//            .whenReleased(intake.setSetPointCommand(PowerIntake.IntakePower.STOP));
//        Trigger outtake2 = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
//            .whileHeld(intake.setSetPointCommand(PowerIntake.IntakePower.OUTTAKE)))
////                .whenPressed(cycleTracker.trackCycle())
//            .whenReleased(intake.setSetPointCommand(PowerIntake.IntakePower.STOP));
//
//        //Climber
//        Button moveUp2  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP))
//            .whenPressed(climb.setPowerCommand(PowerClimber.ClimbPower.UP));
//        Button moveDown2  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN))
//            .whenPressed(climb.setPowerCommand(PowerClimber.ClimbPower.DOWN));
    
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

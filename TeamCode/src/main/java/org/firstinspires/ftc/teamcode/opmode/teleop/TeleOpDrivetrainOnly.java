package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultTankDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.SixWheel;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;

@Config
@TeleOp(group = "Subsystem test")
public class TeleOpDrivetrainOnly extends MatchOpMode {

    // Gamepad
    private GamepadEx driverGamepad;


    // Subsystems
    private Drivetrain drivetrain;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        drivetrain = new Drivetrain(new SixWheel(hardwareMap),telemetry);
        drivetrain.init();
        drivetrain.setDefaultCommand(new DefaultTankDriveCommand(drivetrain, driverGamepad));
    }


    @Override
    public void configureButtons() {
    }

    @Override
    public void matchLoop() {
    }
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){
    }
}
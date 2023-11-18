package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.mec.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.mec.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;
@Disabled
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
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry),telemetry);
        drivetrain.init();
    }


    @Override
    public void configureButtons() {
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, true));
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

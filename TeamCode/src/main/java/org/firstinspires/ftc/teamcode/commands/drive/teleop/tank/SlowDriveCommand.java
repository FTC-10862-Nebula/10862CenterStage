package org.firstinspires.ftc.teamcode.commands.drive.teleop.tank;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.tank.Drivetrain;


public class SlowDriveCommand extends DefaultDriveCommand {
    public SlowDriveCommand(Drivetrain drive, GamepadEx driverGamepad) {
        super(drive, driverGamepad);
        this.multiplier = 0.5;
    }
}
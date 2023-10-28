package org.firstinspires.ftc.teamcode.commands.drive.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

public class SlowTankDriveCommand extends DefaultTankDriveCommand {
    public SlowTankDriveCommand(Drivetrain drive, GamepadEx driverGamepad) {
        super(drive, driverGamepad);
        this.multiplier = 0.3; //Change in the TankModeCommand too; idk which one is working rn
    }
}
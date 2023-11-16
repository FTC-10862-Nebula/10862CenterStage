package org.firstinspires.ftc.teamcode.commands.drive.teleop.tank;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.tank.Drivetrain;

public class DefaultDriveCommand extends CommandBase {
    private Drivetrain drive;
    private GamepadEx driverGamepad;

    protected double multiplier;

    public DefaultDriveCommand(Drivetrain drive, GamepadEx driverGamepad) {
        addRequirements(drive);

        this.drive = drive;
        this.driverGamepad = driverGamepad;

        this.multiplier = 1;
    }

    @Override
    public void execute() {
        // Arcade Drive
        drive.arcadeDrive(driverGamepad.getLeftY() * multiplier, driverGamepad.getRightX() * multiplier);

        // Tank Drive
        //https://github.com/FTCLib/RoadRunner-FTCLib-Quickstart/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/MecanumDriveCommand.java
        //drive.tankDrive(driverGamepad.getLeftY() * multiplier, driverGamepad.getRightY() * multiplier);
        // Mecanum drive
        //drive.drive(-driverGamepad.getLeftY() * multiplier, driverGamepad.getLeftX() * multiplier, driverGamepad.getRightX() * multiplier);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}

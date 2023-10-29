package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;

public class DriveForward2 extends CommandBase {
    private final Drivetrain drive;
    private final double forward, turn;

    public DriveForward2(Drivetrain drive, double forward, double turn) {
        this.forward = forward;
        this.turn = turn;
        this.drive = drive;
        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        drive.arcadeDrive(forward, turn);
//        drive.tankDrive(lY * multiplier, rY * multiplier);
    }


    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}

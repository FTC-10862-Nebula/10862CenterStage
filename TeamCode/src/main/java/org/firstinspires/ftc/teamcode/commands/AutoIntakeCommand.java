package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.sensor.SensorColor;

public class AutoIntakeCommand extends SequentialCommandGroup{
    public AutoIntakeCommand(Claw claw, PowerIntake intake, SensorColor sensorColor){
        addCommands(
                intake.setSetPointCommand(PowerIntake.IntakePower.AUTO_INTAKE),
                new WaitUntilCommand(sensorColor::grabbedFrontPixel).withTimeout(3000),
                claw.setBothClaw(Claw.ClawPos.CLOSE_POS),
                intake.setSetPointCommand(PowerIntake.IntakePower.AUTO_OUTTAKE),
                new WaitCommand(3000),
                intake.setSetPointCommand(PowerIntake.IntakePower.STOP)
        );
    }
}
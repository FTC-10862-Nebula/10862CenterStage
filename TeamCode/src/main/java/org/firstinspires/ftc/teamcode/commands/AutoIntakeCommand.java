package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;

public class AutoIntakeCommand extends SequentialCommandGroup{
    public AutoIntakeCommand(Claw claw, PowerIntake intake){
        addCommands(
            intake.setSetPointCommand(PowerIntake.IntakePower.AUTO_OUTTAKE),
            new WaitCommand(500),
            intake.setSetPointCommand(PowerIntake.IntakePower.AUTO_INTAKE),
            new WaitCommand(500),
            new ParallelCommandGroup(
                intake.setSetPointCommand(PowerIntake.IntakePower.STOP),
                claw.setBothClaw(Claw.ClawPos.CLOSE_POS)
            )
        );
    }
}
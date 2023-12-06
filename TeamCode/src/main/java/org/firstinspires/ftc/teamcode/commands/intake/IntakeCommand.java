package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(PowerIntake intake){
        addRequirements();    //Add Subsystems that you need to run this Command - not necessary
        addCommands(
        
        );
    }
}
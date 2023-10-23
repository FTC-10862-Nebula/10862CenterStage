package org.firstinspires.ftc.teamcode.commands.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.old.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.old.Claw;
import org.firstinspires.ftc.teamcode.subsystems.old.Slide;
@Deprecated
public class AutoDropCommand extends SequentialCommandGroup {

    public AutoDropCommand(Claw claw, Slide slide, Pivot pivot, boolean auto){
//        if(auto){
            addCommands(
                    new InstantCommand(pivot::dropArmAuto),
                    new WaitCommand(40),
                    new InstantCommand(claw::clawOpen)
                    );
    }

}

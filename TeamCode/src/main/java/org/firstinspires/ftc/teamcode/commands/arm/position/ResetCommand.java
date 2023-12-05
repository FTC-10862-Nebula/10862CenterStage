package org.firstinspires.ftc.teamcode.commands.arm.position;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;

public class ResetCommand extends SequentialCommandGroup {
    public ResetCommand(Slide slide, Arm arm, Claw claw) {
        addCommands(
            claw.setBothClaw(Claw.ClawPos.CLOSE_POS),
            new WaitCommand(1200),
            slide.setSetPointCommand(Slide.SlideEnum.TRANSFER),
//            new ParallelCommandGroup(
            new WaitCommand(600),
                arm.armSetPositionCommand(Arm.ArmPos.TRANSFER)
                
//            )
//            new WaitCommand(2500),
//            claw.setClawPos(Claw.ClawPos.OPEN_POS)
        );
    }
}

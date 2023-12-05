package org.firstinspires.ftc.teamcode.commands.arm.position;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;

public class HighCommand extends SequentialCommandGroup {
    public HighCommand(Slide slide, Arm arm, Claw claw) {
        addCommands(
            claw.setBothClaw(Claw.ClawPos.CLOSE_POS),
            new ParallelCommandGroup(
                arm.armSetPositionCommand(Arm.ArmPos.OUTTAKE),
                slide.setSetPointCommand(Slide.SlideEnum.HIGH)
            )
        );
    }
}

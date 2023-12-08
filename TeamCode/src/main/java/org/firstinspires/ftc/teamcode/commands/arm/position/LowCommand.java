package org.firstinspires.ftc.teamcode.commands.arm.position;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;

public class LowCommand extends SequentialCommandGroup {
    public LowCommand(Slide slide, Arm arm, Claw claw) {
        addCommands(
            claw.setBothClaw(Claw.ClawPos.CLOSE_POS),
//            new ParallelCommandGroup(
            slide.setSetPointCommand(Slide.SlideEnum.LOW),
            arm.armSetPositionCommand(Arm.ArmPos.OUTTAKE)
//            )
        );
    }
}

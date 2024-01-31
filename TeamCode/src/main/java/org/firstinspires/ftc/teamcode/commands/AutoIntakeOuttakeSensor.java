package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.sensor.SensorColor;

public class AutoIntakeOuttakeSensor extends SequentialCommandGroup {
    public AutoIntakeOuttakeSensor(Claw claw, SensorColor sensorColor) {
        addRequirements(claw, sensorColor);
        addCommands(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                claw.setBClaw(Claw.ClawPos.CLOSE_POS),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                claw.setFClaw(Claw.ClawPos.CLOSE_POS)
                                        ),
                                        new InstantCommand(),
                                        sensorColor::grabbedFrontPixel
                                )
                        ),
                        new SequentialCommandGroup(),
                        sensorColor::grabbedBackPixel)
        );

    }
}


package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.climber.Climber;
import org.firstinspires.ftc.teamcode.subsystems.sensor.SensorColor;

import java.util.function.Supplier;

public class ClawColorSensorCommand extends CommandBase {
    private final Claw claw;
//    private final SensorColor sensorColor;
    private final Supplier<Boolean> back, front;

    public ClawColorSensorCommand(Claw claw, Supplier<Boolean> back, Supplier<Boolean>front) {
        this.claw = claw;
//        this.sensorColor = sensorColor;
        this.back = back;
        this.front = front;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        if (back.get()) {
            claw.setBClaw(Claw.ClawPos.CLOSE_POS);
            if (front.get()) {
                claw.setFClaw(Claw.ClawPos.CLOSE_POS);
            }
        }
    }
}
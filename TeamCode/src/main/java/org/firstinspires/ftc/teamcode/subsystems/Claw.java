package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;
import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaServo;

@Config
public class Claw extends SubsystemBase
{
    public enum ClawPos {
        CLOSE_POS(0.12),
//        AUTO_CLOSE (0.5),
//        AUTO_OPEN(0.24),
        OPEN_POS(0.21);

        public final double clawPos;
        ClawPos(double clawPos) {
            this.clawPos = clawPos;
        }
    }

    Telemetry telemetry;
    private final NebulaServo claw;     //Claw

    public Claw(Telemetry tl, HardwareMap hw, boolean isEnabled) {
        claw = new NebulaServo(hw,
            NebulaConstants.Claw.clawName,
            NebulaConstants.Claw.clawDirection,
            NebulaConstants.Claw.minAngle,
            NebulaConstants.Claw.maxAngle,
            isEnabled);
        setClawPos(ClawPos.CLOSE_POS);

        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        telemetry.addData("Claw Servo 1 Pos: ", claw.getPosition());
    }

    public Command setClawPos(ClawPos pos){
        return new InstantCommand(()->{
            claw.setPosition(pos.clawPos);});
    }
//    public void clawClose() {
//        claw.setPosition(ClawPos.CLOSE_POS.clawPos);
//    }
//    public void clawOpen() {
//        claw.setPosition(ClawPos.OPEN_POS.clawPos);
//    }

}
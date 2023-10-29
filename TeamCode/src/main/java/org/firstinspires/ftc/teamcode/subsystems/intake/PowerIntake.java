package org.firstinspires.ftc.teamcode.subsystems.intake;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;
import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaMotor;
import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaServo;

//@Deprecated
@Config
public class PowerIntake extends SubsystemBase {
    public enum IntakePower {
        OUTTAKE(-0.7),
        INTAKE(0.7,true),
        STOP(0),
        OUTTAKE_YELLOW(0.1);

        public final double power;
        public final boolean reset;
        IntakePower(double power) {
            this.power = power;
            this.reset = false;
        }
        IntakePower(double power, boolean reset) {
            this.power = power;
            this.reset = reset;
        }
    }
    
    public enum IntakePos {
        UP(0.2,0),
        DOWN(0.5,0.3);
        
        public final double rPos, lPos;
        IntakePos(double rPos, double lPos) {
            this.rPos = rPos;
            this.lPos = lPos;
        }
    }
//    IntakePower shooterRPM = IntakePower.STOP;
    Telemetry telemetry;
    public final NebulaMotor motor;
    private final NebulaServo intakeServoR,intakeServoL;
    

    public PowerIntake(Telemetry tl, HardwareMap hw, Boolean isEnabled) {
        motor = new NebulaMotor(hw, NebulaConstants.Intake.intakeMName,
            NebulaConstants.Intake.intakeType, NebulaConstants.Intake.intakeDirection,
            NebulaConstants.Intake.intakeIdleMode, isEnabled);
        intakeServoR = new NebulaServo(hw,
            NebulaConstants.Intake.intakeRName,
            NebulaConstants.Intake.intakeRDirection,
            NebulaConstants.Intake.minAngle,
            NebulaConstants.Intake.maxAngle,
            isEnabled);
        intakeServoL = new NebulaServo(hw,
            NebulaConstants.Intake.intakeLName,
            NebulaConstants.Intake.intakeLDirection,
            NebulaConstants.Intake.minAngle,
            NebulaConstants.Intake.maxAngle,
            isEnabled);
        setUp();
        this.telemetry = tl;
    }

    @Override
    public void periodic() {
//        telemetry.addData("Intake Speed:", motor.getVelocity());
    }

    public void setPower(double power, boolean reset) {
        motor.setPower(power);
        if(reset){
            NebulaConstants.Intake.intakeTime.reset();
        }
    }

    //TODO: Test!
    public Command setSetPointCommand(double power, boolean reset) {
        return new InstantCommand(()->{
            setPower(power, reset);});
    }
    public Command setSetPointCommand(IntakePower pos) {
        return setSetPointCommand(pos.power, pos.reset);
//        return new InstantCommand(()->{setSetPoint(pos);});
    }

    public void encoderReset() {//Motors wouldn't need reset
        motor.resetEncoder();
    }

    @Deprecated //To FInd Alternative or Not Use
    public boolean isIntaked(){//TODO:Needs to have something where it times
        if(NebulaConstants.Intake.intakeTime.seconds()>2){
//            return controller.getVelocityError()>100;//Whatever the Number is
        }
        return false;
    }
    
    public void setDown(){
        intakeServoR.setPosition(IntakePos.DOWN.rPos);
        intakeServoL.setPosition(IntakePos.DOWN.lPos);
    }
    public void setUp(){
        intakeServoR.setPosition(IntakePos.UP.rPos);
        intakeServoL.setPosition(IntakePos.UP.lPos);
    }
}

package org.firstinspires.ftc.teamcode.subsystems.slide;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;
import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaMotor;

import java.util.function.Supplier;

@Config
public class Slide extends SubsystemBase {
    protected Telemetry telemetry;
    protected NebulaMotor slideR, slideL;
    protected PIDFController slideController;
    protected double output = 0;

    //TODO: Should the Slide even drop?

    public enum SlideEnum {
        TRANSFER(0.0),

        LOW(-150),
        MID(-300),
        HIGH(-400),

        MANUAL(0.0);
        public final double slidePos;
        SlideEnum(double slidePos) {
            this.slidePos = slidePos;
        }
    }


    protected static SlideEnum slidePos;

    public Slide(Telemetry tl, HardwareMap hw, boolean isEnabled) {
        slideR = new NebulaMotor(hw,
            NebulaConstants.Slide.slideRName,
            NebulaConstants.Slide.slideType,
            NebulaConstants.Slide.slideRDirection,
            NebulaConstants.Slide.slideIdleMode,
            isEnabled);
        slideL = new NebulaMotor(hw,
            NebulaConstants.Slide.slideLName,
            NebulaConstants.Slide.slideType,
            NebulaConstants.Slide.slideLDirection,
            NebulaConstants.Slide.slideIdleMode,
            isEnabled);

//        motorGroup = new NebulaMotorGroup(slideR, slideL);
        slideR.setDistancePerPulse(NebulaConstants.Slide.slideDistancePerPulse);

        slideController = new PIDFController(
                NebulaConstants.Slide.slidePID.p,
            NebulaConstants.Slide.slidePID.i,
            NebulaConstants.Slide.slidePID.d,
            NebulaConstants.Slide.slidePID.f,
            getEncoderDistance(),
            getEncoderDistance());
        slideController.setTolerance(NebulaConstants.Slide.slideTolerance);

        this.telemetry = tl;
        slidePos = SlideEnum.TRANSFER;
    }

    @Override
    public void periodic() {
        slideController.setF(NebulaConstants.Slide.slidePID.f * Math.cos(Math.toRadians(slideController.getSetPoint())));
        output = slideController.calculate(getEncoderDistance());
        setPower(output);//TODO: Probably shouldn't be like this

        telemetry.addData("Slide Motor Output:", output);
        telemetry.addData("Slide1 Encoder: ", slideR.getPosition());
        telemetry.addData("Slide2 Encoder: ", slideL.getPosition());
        telemetry.addData("Slide Pos:", getSetPoint());
    }

    public double getEncoderDistance() {
        return slideR.getDistance();
//        return slideR.getPosition();
        //TODO:Does this work?
    }


    public void setPower(double power) {
        slideR.setPower(power);
        slideL.setPower(-power);
//        slideM1.setPower(power);
//        slideM2.setPower(power);//Instead of putting -power, maybe reverse the motor
    }

    public void stopSlide() {
        slideR.stop();
        slideController.setSetPoint(getEncoderDistance());
    }
    /****************************************************************************************/


    public void resetEncoder() {
        slideR.resetEncoder();
        slideL.resetEncoder();
    }


    public void setSetPoint(double setPoint) {
        //TODO: Maybe should remove all Safety Stuff
//        if(NebulaConstants.GamePad.overrideSafety){
//            if(setPoint>NebulaConstants.Slide.MAX_POSITION ||
//                setPoint<NebulaConstants.Slide.MIN_POSITION){
//                slideM1.stop();
//                return;
//            }
//        }

        slideController.setSetPoint(setPoint);
    }

    //TODO: Test!
    public Command setSetPointCommand(double setPoint) {
        slidePos = SlideEnum.MANUAL;    //WTH is this; The booleans don't match
        return new InstantCommand(()->{this.setSetPoint(setPoint);});
    }
    public Command setSetPointCommand(SlideEnum pos) {
        return new InstantCommand(()->{setSetPoint(pos.slidePos);});
    }

    public double getSetPoint() {
        return slideController.getSetPoint();
    }
    
    //TODO: test
    public Command slideMoveManual(Supplier<Double> doubleSupplier) {
        double position = doubleSupplier.get();
//        if (Math.abs(position) > 0.1) {
            return setSetPointCommand(getSetPoint() + position * -25);
//        }
    }
}
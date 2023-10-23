package org.firstinspires.ftc.teamcode.subsystems.old;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Deprecated
//@Config
public class  Slide extends SubsystemBase {
    private final Telemetry telemetry;
    private final MotorEx slideM1, slideM2;

    public static PIDFCoefficients pidfUpCoefficients = new PIDFCoefficients(0.005, 0.00, 0,0);

    private PIDFController upController;

    public static double CPR = 751.8;

    private final double encoderOffset = 0;
    private final double encoderOffset2 = 0;
    
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
    
    double output = 0;
    public Slide( Telemetry tl, HardwareMap hw) {

        slideM1 = new MotorEx(hw, "lift");
        slideM2 = new MotorEx(hw, "lift2");

        //Reverse lift motor
        slideM1.setInverted(true);
        //slideMotor2.setInverted(true);

        slideM1.resetEncoder();
        slideM2.resetEncoder();

        slideM1.setDistancePerPulse(360 / CPR);
        slideM2.setDistancePerPulse(360 / CPR);

        upController = new PIDFController(
            pidfUpCoefficients.p, pidfUpCoefficients.i, pidfUpCoefficients.d, pidfUpCoefficients.f, getAngle(), getAngle());
        upController.setTolerance(10);

        this.telemetry = tl;
        setOffset();
    }

    @Override
    public void periodic() {
            upController.setF(pidfUpCoefficients.f * Math.cos(Math.toRadians(upController.getSetPoint())));

            output = upController.calculate(getAngle());
//            if (output >= 1) output = 1;
//            if (output <= -1) output = -1;

            slideM1.set(output);
            slideM2.set(-output);//TODO: Probably shouldn't be like this
//            if (lowBool) {
//                slideM1.set(output * LOW_POWER);
//                slideM2.set(output * LOW_POWER);
//            }
//            else {
//                slideM1.set(output * POWER);
//                slideM2.set(output * POWER);
//            }
        telemetry.addLine("Slide - ");
        telemetry.addData("     Lift Motor Output:", output);
//        telemetry.addData("     Lift Motor 1 Power", slideM1.getVelocity());
//        telemetry.addData("     Lift Motor 2 Power:", slideM2.getVelocity());

        telemetry.addData("     Lift1 Encoder: ", slideM1.getCurrentPosition());
        telemetry.addData("     Lift2 Encoder: ", slideM2.getCurrentPosition());
    }

    private double getEncoderDistance() {
        return slideM1.getDistance() - encoderOffset;
    }

    private double getEncoderDistance2(){
        return slideM2.getDistance() - encoderOffset2;
    }

//    public void upSlideManual(){
//        slideM1.set(UP_SPEED);
//        slideM2.set(-UP_SPEED);
//    }
//    public void downSlideManual() {
//        slideM1.set(DOWN_SPEED);
//        slideM2.set(-DOWN_SPEED);
//    }

    public void setPower(double power) {
        slideM1.set(power);
        slideM2.set(power);
    }
    public void stopSlide() {
        upController.setSetPoint(getAngle());
        slideM1.stopMotor();
        slideM2.stopMotor();
    }

    public double getAngle() {
        return getEncoderDistance();
    }


    /****************************************************************************************/


    public void slideResting() {
        upController.setSetPoint(SlideEnum.TRANSFER.slidePos);
    }

    public void resetEncoder() {
        slideM1.resetEncoder();
        slideM2.resetEncoder();
        telemetry.addLine("SLIDE RESET");
    }
    public void setOffset() {
        resetEncoder();
        upController.setSetPoint(getAngle());
    }
    public void setSetPoint(double position) {
        upController.setSetPoint(position);
    }
    public double getPosition() {
        return upController.getSetPoint();
    }
    
    public Command setSetPointCommand(SlideEnum pos) {
        return new InstantCommand(()->{setSetPoint(pos.slidePos);});
    }
}
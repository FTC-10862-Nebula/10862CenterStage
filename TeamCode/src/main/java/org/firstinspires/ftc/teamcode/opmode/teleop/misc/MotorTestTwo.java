package org.firstinspires.ftc.teamcode.opmode.teleop.misc;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MotorTestTwo extends OpMode {
    private  MotorEx motor;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        motor = new MotorEx(hardwareMap, "climb");
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        if(gamepad1.dpad_down){
            motor.set(1);
        }
        else if(gamepad1.dpad_up){
            motor.set(-1);
        }
        else{
            motor.stopMotor();
        }
//        telemetry.addData("Motor: ", motor.getRate());
//        telemetry.update();

    }
}
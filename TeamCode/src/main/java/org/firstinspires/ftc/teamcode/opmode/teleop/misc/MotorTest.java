package org.firstinspires.ftc.teamcode.opmode.teleop.misc;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Disabled
@TeleOp(group = "Subsystem test")
public class MotorTest extends OpMode {
    private final MotorEx motor = new MotorEx(hardwareMap, "climb");

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        if(gamepad1.dpad_down){
            motor.set(0.25);
        }
        else if(gamepad1.dpad_up){
            motor.set(-0.25);
        }
//        telemetry.addData("Motor: ", motor.getRate());
//        telemetry.update();

    }
}
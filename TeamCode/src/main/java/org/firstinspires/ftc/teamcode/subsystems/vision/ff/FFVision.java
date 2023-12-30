package org.firstinspires.ftc.teamcode.subsystems.vision.ff;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FFVision extends SubsystemBase {
    private final Telemetry telemetry;
    private final FFRectDetector duckDetector;
    private TeamMarkerPipeline.FFPosition finalPos;

    public FFVision(HardwareMap hw, Telemetry tl) {
        duckDetector = new FFRectDetector(hw, tl);
        duckDetector.init();
//y-
//y+
        
        //x- <---> x+

        duckDetector.setLeftRectangle(0.5, 0.5);
        duckDetector.setCenterRectangle(.45, 0.5);
        duckDetector.setRightRectangle(0.6, 0.5);
//        duckDetector.setLeftRectangle(.02, .42);
//        duckDetector.setCenterRectangle(.45, .40);
//        duckDetector.setRightRectangle(0.999, .45);
        duckDetector.setRectangleSize(40,50);
        telemetry = tl;
//        currentPos = duckDetector.getPosition();
    }


    @Override
    public void periodic() {
//        currentPos = duckDetector.getPosition();
//        telemetry.addData("Position", duckDetector.getPosition());
        
        telemetry.addData("Current Position", getPosition());
        telemetry.addData("Final Position", getFinalPosition());
        
    }

    public TeamMarkerPipeline.FFPosition getPosition() {
        return duckDetector.getPosition();
    }
    public void setPosition(TeamMarkerPipeline.FFPosition position) {
        finalPos = position;
    }
    
    public TeamMarkerPipeline.FFPosition getFinalPosition() {
        return finalPos;
    }
    
}
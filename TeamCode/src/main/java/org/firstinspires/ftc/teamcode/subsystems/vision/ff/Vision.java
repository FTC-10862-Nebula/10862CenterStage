package org.firstinspires.ftc.teamcode.subsystems.vision.ff;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Vision extends SubsystemBase {
    private final Telemetry telemetry;
    private final FFRectDetector duckDetector;
    private TeamMarkerPipeline.FFPosition finalPos;

    public Vision(HardwareMap hw, Telemetry tl) {
        duckDetector = new FFRectDetector(hw, tl);
        duckDetector.init();
//y-
//y+
        
        //x- <---> x+
        duckDetector.setLeftRectangle(.04999, .42);
        duckDetector.setCenterRectangle(.5, .40);
        duckDetector.setRightRectangle(0.9, .45);
        duckDetector.setRectangleSize(40,40);
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
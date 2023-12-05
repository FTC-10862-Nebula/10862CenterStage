package org.firstinspires.ftc.teamcode.opmode.auto.misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.vision.ff.Vision;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;
@Autonomous
public class VisionAuto extends MatchOpMode {
    private Vision vision;
    
    @Override
    public void robotInit() {
        vision = new Vision(hardwareMap, telemetry);
    }
    
    @Override
    public void disabledPeriodic() {
        vision.setPosition(vision.getPosition());
        vision.periodic();
        telemetry.update();
    }
    
    public void matchStart() {
//        TeamMarkerPipeline.FFPosition position = vision.getPosition();
    }
}
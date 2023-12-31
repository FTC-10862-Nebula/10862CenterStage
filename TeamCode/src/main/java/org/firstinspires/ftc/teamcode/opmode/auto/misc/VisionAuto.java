package org.firstinspires.ftc.teamcode.opmode.auto.misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.vision.ff.FFVision;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;
@Autonomous
public class VisionAuto extends MatchOpMode {
    private FFVision FFVision;
    
    @Override
    public void robotInit() {
        FFVision = new FFVision(hardwareMap, telemetry);
    }
    
    @Override
    public void disabledPeriodic() {
        FFVision.setPosition(FFVision.getPosition());
        FFVision.periodic();
        telemetry.update();
    }
    
    public void matchStart() {
//        TeamMarkerPipeline.FFPosition position = vision.getPosition();
    }
}
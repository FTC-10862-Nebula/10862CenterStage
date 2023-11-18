package org.firstinspires.ftc.teamcode.opmode.auto.tank.paths;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.drive.DriveCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.climber.Climber;
import org.firstinspires.ftc.teamcode.subsystems.drive.tank.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.TeamMarkerPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.Vision;

import java.util.HashMap;


public class DropYellowPixel extends SequentialCommandGroup {
    public DropYellowPixel(Drivetrain drivetrain,
                           //    private AprilTagVision aprilTagVision;
                           Vision vision,
                           PowerIntake intake,
                           Climber climber,
                           Arm arm,
                           Slide slide,
                           Claw claw){
        //declare variables here
        addCommands(
            new SelectCommand(new HashMap<Object, Command>() {{
                put(TeamMarkerPipeline.FFPosition.LEFT, new SequentialCommandGroup(
                    new DriveForwardCommand(drivetrain, 7.8)
                ));
                put(TeamMarkerPipeline.FFPosition.MIDDLE, new SequentialCommandGroup(
                    new DriveForwardCommand(drivetrain, 3.6)
                ));
                put(TeamMarkerPipeline.FFPosition.RIGHT, new SequentialCommandGroup(
                    new DriveForwardCommand(drivetrain, -3)
                ));
            }}, vision::getPosition)
        );
    }
}
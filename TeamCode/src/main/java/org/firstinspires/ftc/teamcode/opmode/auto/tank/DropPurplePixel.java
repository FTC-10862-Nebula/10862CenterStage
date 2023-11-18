package org.firstinspires.ftc.teamcode.opmode.auto.tank;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.drive.DriveCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.climber.Climber;
import org.firstinspires.ftc.teamcode.subsystems.drive.tank.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.TeamMarkerPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.Vision;

import java.util.HashMap;


public class DropPurplePixel extends SequentialCommandGroup {
    public DropPurplePixel(Drivetrain drivetrain,
                           //    private AprilTagVision aprilTagVision;
                           Vision vision,
                           PowerIntake intake,
                           Climber climber,
                           Arm arm,
                           Slide slide,
                           Claw claw){
        addCommands(
            new SelectCommand(
                new HashMap<Object, Command>() {{
                put(TeamMarkerPipeline.FFPosition.LEFT, new SequentialCommandGroup(
                    new TurnCommand(drivetrain, -90),
                    new DriveForwardCommand(drivetrain, 1)
                ));
                put(TeamMarkerPipeline.FFPosition.MIDDLE, new SequentialCommandGroup(
                    new DriveForwardCommand(drivetrain, -4)
                ));
                put(TeamMarkerPipeline.FFPosition.RIGHT, new SequentialCommandGroup(
                    new TurnCommand(drivetrain, 90),
                    new DriveForwardCommand(drivetrain, 1)
                ));
            }}, vision::getFinalPosition),
            
            
            new TurnCommand(drivetrain, 90),
            new DriveForwardCommand(drivetrain, 1),
            
            
            new ParallelCommandGroup(
                intake.setSetPointCommand(PowerIntake.IntakePower.OUTTAKE_PURPLE),
                new DriveForwardCommand(drivetrain, -3)
            ),
            new WaitCommand(4000),
            intake.setSetPointCommand(PowerIntake.IntakePower.STOP),

            new SelectCommand(new HashMap<Object, Command>() {{
                put(TeamMarkerPipeline.FFPosition.LEFT, new SequentialCommandGroup(
                    new TurnCommand(drivetrain, 90)
                ));
                put(TeamMarkerPipeline.FFPosition.MIDDLE, new SequentialCommandGroup(
                    new TurnCommand(drivetrain, 0.1)
                ));
                put(TeamMarkerPipeline.FFPosition.RIGHT, new SequentialCommandGroup(
                    new TurnCommand(drivetrain, -90)
                ));
            }}, vision::getFinalPosition),
    
            new TurnToCommand(drivetrain, 0),
            new DriveForwardCommand(drivetrain, -15)
        );
    }
}
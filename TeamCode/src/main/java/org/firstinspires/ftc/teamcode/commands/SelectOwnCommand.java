package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.vision.ff.TeamMarkerPipeline;

public class SelectOwnCommand extends SequentialCommandGroup{
    public SelectOwnCommand(TeamMarkerPipeline.FFPosition position, Command command1,
                            Command command2, Command command3){
        if(position==TeamMarkerPipeline.FFPosition.LEFT){
            addCommands(
                command1
            );
        } else if(position==TeamMarkerPipeline.FFPosition.MIDDLE){
            addCommands(
                command2
            );
        } else if(position==TeamMarkerPipeline.FFPosition.RIGHT){
            addCommands(
                command3
            );
        }
        
    }
}

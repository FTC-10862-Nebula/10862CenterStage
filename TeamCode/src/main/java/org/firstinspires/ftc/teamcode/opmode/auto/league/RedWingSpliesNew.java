package org.firstinspires.ftc.teamcode.opmode.auto.league;

import org.firstinspires.ftc.teamcode.opmode.auto.Speed;
import org.firstinspires.ftc.teamcode.subsystems.vision.ff.TeamMarkerPipeline;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Back;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.LineToConstantHeading;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.SplineTo;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.SplineToSplineHeading;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.container.TrajectorySequenceContainer;

public class RedWingSpliesNew extends MatchOpMode {
    public static Pose2dContainer startPose = new Pose2dContainer(10, 65, 270);

    static TrajectorySequenceContainer getDropSpike(TeamMarkerPipeline.FFPosition position) {
        switch (position) {
            case LEFT:
            case MIDDLE:
            default:
            case RIGHT:
                return new TrajectorySequenceContainer(
                        Speed::getFastConstraints,
                new SplineTo(-32,-34,45)
                );
        }
    }
    static TrajectorySequenceContainer getPlus(TeamMarkerPipeline.FFPosition position) {
        switch (position) {
            case LEFT:
            case MIDDLE:
            default:
            case RIGHT:
                return new TrajectorySequenceContainer(
                        Speed::getFastConstraints,
                        new SplineToSplineHeading(-60,-35,45,180)
                );
        }
    }
    static TrajectorySequenceContainer getBack(TeamMarkerPipeline.FFPosition position) {
        switch (position) {
            case LEFT:
            case MIDDLE:
            default:
            case RIGHT:
                return new TrajectorySequenceContainer(
                        Speed::getFastConstraints,
                        new LineToConstantHeading()
                        );
        }
    }
}

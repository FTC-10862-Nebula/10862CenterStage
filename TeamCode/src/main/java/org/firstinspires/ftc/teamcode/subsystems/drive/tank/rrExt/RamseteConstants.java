package org.firstinspires.ftc.teamcode.subsystems.drive.tank.rrExt;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RamseteConstants {
    //horizontal error adjustment
    public static double b = 13;
    public static double zeta = 2.5;

    //minimize overshoot
    public static double kLinear = 0.1;
    public static double kHeading = 6.65;
    
}
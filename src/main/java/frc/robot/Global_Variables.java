package frc.robot;

import frc.robot.subsystems.Limelight;

public class Global_Variables
{
    static{limelightInstance = Limelight.getInstance();}
    public static Limelight limelightInstance;

    public static double yawTarget = 0.0;
    public static boolean isBoost = false;
}
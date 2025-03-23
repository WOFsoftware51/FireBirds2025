// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight {

    private final String name = "limelight";
    private static Limelight instance = null;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable(name);

    private double tx = 0.0;
    public double ty = 0.0;
    public double tv = 0.0;
    public int id = 0;
    // public double distanceY = 0.0;
    // public double distanceX = 0.0;
    // public double distanceXY = 0.0;
    public double txCenterRobot = 0.0;  
    public double yawFixed = 0.0;

    private Pose2d startingPose2d;
    private Pose2d visionPoseStartClone;
    /** Creates a new Limelight. */
    private Limelight() {
        startingPose2d = new Pose2d();
        visionPoseStartClone = getVisionPoseEstimate2d().pose;
    }
    public static Limelight getInstance(){
        if(instance == null){
            instance = new Limelight();
        }
        return instance;
    }

    public PoseEstimate getVisionPoseEstimate2d(){
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }
    private Pose2d updateStartingPose2d(){
        Transform2d t = getVisionPoseEstimate2d().pose.minus(visionPoseStartClone);
        return startingPose2d = new Pose2d(t.getTranslation(), t.getRotation());
    }
    public Pose3d getTargetPoseEstimate3d(){
        return LimelightHelpers.getTargetPose3d_CameraSpace(name);
    }
    public double getTX(){
        return tx;
    }
    public double getTY(){
        return ty;
    }
    public double getTV(){
        return tv;
    }
    public double getDesiredAngle(){
        double yawTarget = 0;
        if(id==6){
           yawTarget = 120;
        }
        else if(id==7){
            yawTarget = 0;
        }
        else if(id==8){
            yawTarget = -120;
        }
        else if(id==9){
            yawTarget = -60;
        }
        else if(id==10){
            yawTarget = 0;
        }
        else if(id==11){
            yawTarget = 60;
        }
        else if(id==17){
            yawTarget  = -120;
        }
        else if(id==18){
            yawTarget  = 180;
        }
        else if(id==19){
            yawTarget = 120;
        }
        else if(id==20){
            yawTarget = 60;
        }
        else if(id==21){
            yawTarget = 0;
        }
        else if(id==22){
            yawTarget = -60;
        }
        else{
            yawTarget = 0.0;
        }
        return yawTarget;
    }

    public double getVisibleTags(){
        return LimelightHelpers.getFiducialID(name);
    }
    public double getLatency(){
        return LimelightHelpers.getLatency_Capture(name);
    }

    public double getBestTarget(){
        return LimelightHelpers.getFiducialID(name);
    }

  public void periodic() {
    tv = table.getEntry("tv").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    tx = table.getEntry("tx").getDouble(0);
    id = (int)table.getEntry("tid").getInteger(0);

    // distanceY = (Constants.APRIL_TAG_HEIGHT-Constants.LIMELIGHT_HEIGHT)/(Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE+ty)));
     // distanceX = distanceY/(Math.tan(Math.toRadians(tx)));

    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("April Tag ID", id);

    SmartDashboard.putNumber("Vision Pose X", getVisionPoseEstimate2d().pose.getX());
    SmartDashboard.putNumber("Vision Pose Y", getVisionPoseEstimate2d().pose.getY());

    // SmartDashboard.putNumber("Vision Pose X From Start ", Global_Variables.visionPoseStart.getX());
    // SmartDashboard.putNumber("Vision Pose Y Fromt Start", Global_Variables.visionPoseStart.getY());
}
}

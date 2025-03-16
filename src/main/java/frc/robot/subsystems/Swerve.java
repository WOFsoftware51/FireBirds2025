package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.lang.Thread.State;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator3d swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public Limelight limeLight = Global_Variables.limelightInstance;
    public Optional<Alliance> ally = DriverStation.getAlliance();

    private static Matrix<N4, N1> driveTrainStandardDeviation = VecBuilder.fill(0.045, 0.045, 0, 0.0);//VecBuilder.fill(0.1, 0.1, 0.1);
    private static Matrix<N4, N1> visionStandardDeviation = Constants.Vision.kSingleTagStdDevs;

    public Swerve() {
        limeLight = Global_Variables.limelightInstance;
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANIVORE_NAME);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDrivePoseEstimator3d(Constants.Swerve.swerveKinematics, getGyroRotation3d(), getModulePositions(), new Pose3d(), driveTrainStandardDeviation, visionStandardDeviation);
        swerveOdometry = new SwerveDrivePoseEstimator3d(Constants.Swerve.swerveKinematics, getGyroRotation3d(), getModulePositions(), new Pose3d(), driveTrainStandardDeviation, visionStandardDeviation);
        configureAuton();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    
        /** Robot Relative Drive
     * <P>Drive command used in auton */
    public void driveRelative(ChassisSpeeds driveMSupplier) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(driveMSupplier);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }   

  

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }
    public ChassisSpeeds getChassisSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(this.getModuleStates()); //TODO IS THIS ROBOT RELATIVE OR FIELD RELATIVE?
        // ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeeds(), getGyroYaw())
    }    

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }
    
    public void zeroGyro(){
        gyro.setYaw(0, 0.1);
    }

    public Pose3d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void setPose(Supplier<Pose3d> pose) {
        swerveOdometry.resetPosition(getGyroRotation3d(), getModulePositions(), pose.get());
    }


    public Rotation2d getHeading(){
        return getPose().getRotation().toRotation2d();
    }


    public void setHeading(Rotation3d heading){
        swerveOdometry.resetPosition(getGyroRotation3d(), getModulePositions(), new Pose3d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroRotation3d(), getModulePositions(), new Pose3d(getPose().getTranslation(), new Rotation3d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }
    public Rotation3d getGyroRotation3d() {
        return gyro.getRotation3d();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    private void configureAuton(){
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        AutoBuilder.configure(
            ()->getPose().toPose2d(), // Robot pose supplier
            pose2d -> setPose(()-> new Pose3d(pose2d)), // Robot pose supplier
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController(
                new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants 
                new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0) // Rotation PID constants
            ),
            config,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }
    /**Directly follows a Pathplanner path
     * <p> NOTE: May not work because pose get weird. Sidestepped the issue by following a Pathplanner Auton and overriding the Pose in there instead.
     * <p> ISSUE SIDESTEPPED: If you reset the post by following a Pathplanner Auton for the first path, the rest of the paths can use this function. 
     * This ensures that pose is only reset when you run an auton, not everytime you run a path
     */
    public Command followTrajectoryCommand(String pathString) {
        try{
            PathPlannerPath path = PathPlannerPath.fromPathFile("pathString");    
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big Auton oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
    public Command followTrajectoryCommand(PathPlannerPath path) {
        try{
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big Auton oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    /**Aiming Swerve with Limelight Towards Set Yaw Angle */
    public double goToAprilTag_Yaw(Rotation2d rotTarget){
        var targetState = new PathPlannerTrajectoryState();
        targetState.pose = new Pose2d(getPose().getX(), getPose().getY(), rotTarget);
        return Constants.Swerve.ppHolonomicController.calculateRobotRelativeSpeeds(getPose().toPose2d(), targetState).omegaRadiansPerSecond;
    }  
    /**Aiming Swerve with Limelight Towards Set Yaw Angle */
    public Translation2d goToAprilTag_XY(Translation2d transTarget){
        var targetState = new PathPlannerTrajectoryState();
        targetState.pose = new Pose2d(transTarget, getHeading());
        return new Translation2d(Constants.Swerve.ppHolonomicController.calculateRobotRelativeSpeeds(getPose().toPose2d(), targetState).vxMetersPerSecond, Constants.Swerve.ppHolonomicController.calculateRobotRelativeSpeeds(getPose().toPose2d(), targetState).vyMetersPerSecond);
    }  
    public ChassisSpeeds holonomicController(Pose2d targetPose, Rotation2d targetRot)
    {
       return new HolonomicDriveController(Constants.Swerve.positionControllerX, Constants.Swerve.positionControllerY, Constants.Swerve.positionControllerTheta).
            calculate(getPose().toPose2d(), targetPose, Constants.Swerve.maxSpeed * 0.6, targetRot);
    }
    public void addVisionToPoseEstimator(){
        Pose2d visionPose3d = limeLight.getVisionPoseEstimate2d().pose;
        boolean rejectVision = false;
        
        // if(Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        if(Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            rejectVision = true;
        }
        if(limeLight.getVisionPoseEstimate2d().tagCount == 0)
        {
            rejectVision = true;
        }

        if(!rejectVision){
            swerveOdometry.addVisionMeasurement(new Pose3d(visionPose3d), limeLight.getLatency());
        }
    }

    // /**Everything is always in meters. */
    // public double getSwerveToAverageVisionPoseDistance(){

    //     return limeLight.getVisionPoseEstimate2d().pose.relativeTo(getPose().toPose2d()).getTranslation().getNorm();
    // }

    // public void setSwervePoseToAverageVisionPose()
    // {
    //     if(limeLight.getVisionPoseEstimate2d().tagCount != 0 && getSwerveToAverageVisionPoseDistance() > 0.0175 && cameraSystem.getBestCamAmbiguity() < 0.1)
    //     {
    //         setSwervePoseToVisionPose();
    //     }
    // }

    // public void setSwerveHeadingToVisionHeading(){
    //     if(!hasUpdatedVisionHeading && Math.abs(getHeading().minus(cameraSystem.getBestRobotPose().getRotation().toRotation2d()).getDegrees()) > 5.0 && cameraSystem.hasTargets() && cameraSystem.getBestCamAmbiguity() < 0.15){
    //         swerveOdometry.resetRotation(cameraSystem.getBestRobotPose().getRotation().rotateBy(new Rotation3d(0,0,Math.PI)));
    //         hasUpdatedVisionHeading = true;
    //     }
    // }

    /** */
    public HashMap<Integer, Pose3d> transformApriltagPose(Map<Integer, AprilTag> aprilTagList, Transform3d trasnform1, Transform3d transform2 ){
        HashMap<Integer, Pose3d> newPoses = new HashMap<>();
        for(AprilTag apriltag : Constants.Vision.aprilTagList.values()){
            Transform3d transform = new Transform3d(new Translation3d(), new Rotation3d());
            newPoses.put(apriltag.ID, apriltag.pose.transformBy(transform));
        } 
        return newPoses;
    }
    /**
     * xDistance is how far away you want to transform in the x direction.
     * yDistance is the same for the y direction. <li>
     * Units: METERS
     */
    public Pose3d transformBestApriltagPose(double xDistance, double yDistance){
        Transform3d transform1 = 
            new Transform3d(new Translation3d(xDistance, Rotation3d.kZero), Rotation3d.kZero);
        Transform3d transform2 = 
            new Transform3d(new Translation3d(yDistance, new Rotation3d(0,0, Math.PI / 2.0)), Rotation3d.kZero);
        return getClosestAprilTag(getAllianceAccurateReefTagList()).pose
            .transformBy(transform1).transformBy(transform2);
    }

    public double getTargetToPoseMeters(int apritagId){
        return Constants.Vision.aprilTagList.get(apritagId).pose.relativeTo(getPose()).getTranslation().getNorm();
    }
    /**Gets norm of closest apriltag with robot as origin.*/
    public double getTargetToPoseMeters(){
        return getTargetToPoseMeters(getClosestAprilTag().ID);
    }
    /**Returns pose of closest apriltag. From all apriltags.*/
    public AprilTag getClosestAprilTag(){
        double shortestDistance = Double.MAX_VALUE;
        int closestAprilTagID = -1;

        for(AprilTag apriltag : Constants.Vision.aprilTagList.values())
        {
            double distance = apriltag.pose.relativeTo(getPose()).getTranslation().getNorm();
            if(shortestDistance > distance){
                shortestDistance = distance;
                closestAprilTagID = apriltag.ID;
            }
        }

        if(closestAprilTagID == -1)
        {
            return new AprilTag(-1, Pose3d.kZero);
        }
        else
        {
            return Constants.Vision.aprilTagList.getOrDefault(closestAprilTagID, new AprilTag(-1, Pose3d.kZero));
        }
    }
    /**Returns pose of closest apriltag from List.*/
    public AprilTag getClosestAprilTag(Map<Integer, AprilTag> aprilTagList){
        double shortestDistance = Double.MAX_VALUE;
        int closestAprilTagID = -1;

        for(AprilTag apriltag : aprilTagList.values())
        {
            double distance = apriltag.pose.relativeTo(getPose()).getTranslation().getNorm();
            if(shortestDistance > distance){
                shortestDistance = distance;
                closestAprilTagID = apriltag.ID;
            }
        }

        if(closestAprilTagID == -1)
        {
            return new AprilTag(-1, Pose3d.kZero);
        }
        else
        {
            return aprilTagList.getOrDefault(closestAprilTagID, new AprilTag(-1, Pose3d.kZero));
        }
    }

    public Pose3d getClosestAprilTagToRobot(Map<Integer, AprilTag> aprilTagList)
    {
        return getClosestAprilTag(aprilTagList).pose.relativeTo(getPose());
        // return getPose().relativeTo(getClosestAprilTag(aprilTagList).pose);
    }

    

    public Map<Integer, AprilTag> getAllianceAccurateReefTagList()
    {
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                return Constants.Vision.aprilTagListReefRed;

            }
            else if (ally.get() == Alliance.Blue) {
                return Constants.Vision.aprilTagListReefBlue;
            }
            else{
                return Constants.Vision.aprilTagListReef;
            }

        }
        else{
            return Constants.Vision.aprilTagListReef;
        }
    }
    
    @Override
    public void periodic(){
        ally = DriverStation.getAlliance();
        swerveOdometry.update(getGyroRotation3d(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getRotations());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}
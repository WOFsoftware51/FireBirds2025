package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    public double speedModifier = Constants.Swerve.DRIVE_SPEED;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private ChassisSpeeds aimSpeeds = new ChassisSpeeds();
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier isAimingLeft;
    private BooleanSupplier isAimingRight;

    private double translationVal = 0; 
    private double strafeVal = 0; 
    private double rotationVal = 0; 


    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier isAimingLeft, BooleanSupplier isAimingRight) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.isAimingLeft = isAimingLeft;
        this.isAimingRight = isAimingRight;
    }

    @Override
    public void execute() {
        aimSpeeds = s_Swerve.holonomicController(Global_Variables.limelightInstance.getTargetPoseEstimate3d().toPose2d(), new Rotation2d(Global_Variables.yawTarget * Math.PI / 180.0));
        
        /* Get Values, Deadband*/
        if(Math.abs(translationSup.getAsDouble())>0.1 || Math.abs(strafeSup.getAsDouble())>0.1 || Math.abs(rotationSup.getAsDouble())>0.1){
            translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
            strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);    
        }
        else if(isAimingLeft.getAsBoolean() || isAimingRight.getAsBoolean()){
            if(Global_Variables.limelightInstance.getVisionPoseEstimate2d().tagCount != 0){
                rotationVal = aimSpeeds.omegaRadiansPerSecond;
                strafeVal = aimSpeeds.vyMetersPerSecond;
                translationVal = aimSpeeds.vxMetersPerSecond;
            }
        }
        else{
            translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
            strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);    
        }

        
        if (Global_Variables.isBoost)
        {
            speedModifier = 1.0;
        }
        else
        {
            speedModifier = Constants.Swerve.DRIVE_SPEED;
        }
        
        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed).times(speedModifier), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}
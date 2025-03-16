package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve2 extends Command {    
    private Swerve s_Swerve;    
    public double speedModifier = Constants.Swerve.DRIVE_SPEED;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;


    public TeleopSwerve2(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if (Global_Variables.isBoost)
        {
            speedModifier = 1.0;
        }
        else if (Global_Variables.right_Bumper_Slow)
        {
            speedModifier = 0.3;
        }
        else
        {
            speedModifier = Constants.Swerve.DRIVE_SPEED;
        }
        
        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed).times(speedModifier), 
            rotationVal * Constants.Swerve.maxAngularVelocity*speedModifier, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}
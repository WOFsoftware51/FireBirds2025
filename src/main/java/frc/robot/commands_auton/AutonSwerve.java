package frc.robot.commands_auton;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class AutonSwerve extends Command 
{    
    private Swerve s_Swerve;    
    private double xspeed = 0.0;
    private double yspeed = 0.0;
    private double rotation;
    private int count = 0;
    private int time = 0;
    private double turn_error;
    private double rotationPercent;
    private boolean endCommand = false;


    public AutonSwerve(Swerve swerve, double ySpeed, double xSpeed, double xRotation, int time) {
        this.s_Swerve = swerve;
        addRequirements(s_Swerve);

        this.rotation = xRotation;
        this.yspeed = ySpeed;
        this.xspeed = xSpeed;
        this.time = time;
    }

    @Override
    public void initialize()
    {
        endCommand = false;
        count = 0;
    }
    
    @Override
    public void execute() 
    {
      turn_error = rotation - s_Swerve.getGyroYaw().getDegrees();


        /* Drive */
        s_Swerve.drive(
            new Translation2d(yspeed, xspeed).times(Constants.Swerve.maxSpeed), 
            rotationPercent * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
        );

        count++;
        if (count > time)
        {
            count = 0;
            endCommand = true;
        }

        /*
        if(time>count)
        { 
            s_Swerve.drive(
                new Translation2d(yspeed, xspeed).times(Constants.Swerve.maxSpeed), 
                xRotation * Constants.Swerve.maxAngularVelocity*s_Swerve.SpeedModifier, 
                false, 
                true
            );
        count++;
        }
        else
        {
            endCommand = true;
        }
         */
    }
    @Override
    public boolean isFinished() 
    { 
      return endCommand;

    }  
}

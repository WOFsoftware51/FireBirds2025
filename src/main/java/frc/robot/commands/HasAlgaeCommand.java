// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Global_Variables;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HasAlgaeCommand extends Command {
  /** Creates a new ClawIntakeCommand. */
  private AlgaeIntake m_AlgaeIntake;
  private int count = 0;
  public HasAlgaeCommand(AlgaeIntake mAlgaeIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_AlgaeIntake = mAlgaeIntake;

    addRequirements(m_AlgaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    Global_Variables.hasAlgae = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_AlgaeIntake.getVelocityRPS()*60 < 480.0 && m_AlgaeIntake.getCurrent() > 20.0){
      count++;
      if(count > 10) {
        Global_Variables.hasAlgae = true;
      }
    }
    else {
      count = 0;
    }

    
    if(Global_Variables.hasAlgae)
      m_AlgaeIntake.runSmallVoltage();
    else
      m_AlgaeIntake.off();
    } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AlgaeIntake.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

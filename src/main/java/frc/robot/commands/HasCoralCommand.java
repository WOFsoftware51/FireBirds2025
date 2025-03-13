// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Global_Variables;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralScorer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HasCoralCommand extends Command {
  /** Creates a new ClawIntakeCommand. */
  private CoralScorer m_CoralScorer;
  private int count = 0;
  public HasCoralCommand(CoralScorer coralScorer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_CoralScorer = coralScorer;

    addRequirements(m_CoralScorer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    Global_Variables.hasCoral = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_CoralScorer.getSpeed()*60 < 480.0 && m_CoralScorer.getCurrent() > 20.0){
      count++;
      if(count > 10) {
        Global_Variables.hasCoral = true;
      }
    }
    else {
      count = 0;
    }

    
    if(Global_Variables.hasCoral)
      m_CoralScorer.runSmallVoltage();
    else
      m_CoralScorer.coralOff();
    } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CoralScorer.coralOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

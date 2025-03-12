// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BlinkIn;
import frc.robot.subsystems.CoralScorer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BlinkIn_Command extends Command {
  /** Creates a new BlinkIn_Command. */
  private BlinkIn mBlinkIn;
  private int m_color;
  private double mColor;
  private CoralScorer mCoralScorer;
  public BlinkIn_Command(BlinkIn blinkIn, double color) { 
    this.mBlinkIn = blinkIn;
    mColor = color;
    addRequirements(mBlinkIn);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mBlinkIn.setColor(mColor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    /*
    switch (m_color) {
    case Constants.WHITE:
      mBlinkInTarget = Constants.BlinkinClass.BLINKIN_WHITE;
      break;
   
    default:
      mBlinkInTarget = Constants.BlinkinClass.BLINKIN_PURPLE_DEFAULT;
      break;
    }
      */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // mBlinkIn.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

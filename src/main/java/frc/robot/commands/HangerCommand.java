// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hanger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HangerCommand extends Command {
  /** Creates a new HangerCommand. */
  private  Hanger mHanger;
  private double mSpeed = 0;
  private double mTarget = 0;
  public HangerCommand(Hanger hanger, double speed, double target) {
    this.mHanger = hanger;
    this.mSpeed = speed;
    this.mTarget = target;
    addRequirements(mHanger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  if(mHanger.getHangerPosition() > mTarget && mSpeed<0){
  mHanger.hangerActive(mSpeed);
  }
  else if(mHanger.getHangerPosition() < mTarget && mSpeed>0){
    mHanger.hangerActive(mSpeed);
    }
    else{
      mHanger.hangerActive(0);
    }
}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mHanger.hangerOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

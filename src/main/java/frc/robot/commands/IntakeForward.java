// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeForward extends Command {
  public Intake mIntake;
  private int count = 0;

  public IntakeForward(Intake Intake) {
    this.mIntake = Intake;
    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Global_Variables.topIntakeHasPiece = false;
    int count = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mIntake.getVelocityRPS() < 8.0 && mIntake.getCurrent() > 20.0){
      count++;
      if(count > 10) {
        Global_Variables.topIntakeHasPiece = true;
      }
    }
    else{
      count = 0;
    }
    if(Global_Variables.topIntakeHasPiece){
      mIntake.runSmallVoltage();
    }
    else{
      mIntake.powerOn();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.powerOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake_Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeWrist_Manual extends Command {
  private AlgaeIntake_Wrist mAlgaeWrist;
  private DoubleSupplier m_joystickSupplier;
  public AlgaeWrist_Manual(AlgaeIntake_Wrist algaeWrist, DoubleSupplier joystickSupplier) { 
    this.mAlgaeWrist = algaeWrist;
    addRequirements(mAlgaeWrist);
    m_joystickSupplier = joystickSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mAlgaeWrist.onPercent(m_joystickSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mAlgaeWrist.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

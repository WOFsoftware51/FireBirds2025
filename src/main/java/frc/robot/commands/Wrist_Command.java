// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake_Wrist;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Wrist_Command extends Command {
  private Wrist mCoralWrist;
  private DoubleSupplier m_joystickSupplier;
  public Wrist_Command(Wrist coralWrist, DoubleSupplier joystickSupplier) { 
    this.mCoralWrist = coralWrist;
    addRequirements(mCoralWrist);
    m_joystickSupplier = joystickSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joyStickFixed = MathUtil.applyDeadband(m_joystickSupplier.getAsDouble(), Constants.stickDeadband);
    mCoralWrist.WristSetSpeed(joyStickFixed * -0.20);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mCoralWrist.WristSetSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake_Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeGoTo extends Command {
  /** Creates a new WristGoToPositionCommand. */
  private AlgaeIntake_Wrist m_algaeIntake;
  private int m_button;
  private double algaeIntakeTarget;
 
  public AlgaeIntakeGoTo(AlgaeIntake_Wrist algaeIntake, int button) {
    this.m_button = button; 
    this.m_algaeIntake = algaeIntake;
    addRequirements(m_algaeIntake);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }  
      
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_button) {
      case 1: 
        algaeIntakeTarget = Constants.AlgaeIntakeClass.ALGAE_INTAKE_HOME;
        break;
   
      case Constants.A_BUTTON:
        algaeIntakeTarget = Constants.AlgaeIntakeClass.ALGAE_INTAKE_POSITION;
        break;
    }
   m_algaeIntake.goToPosition(algaeIntakeTarget);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

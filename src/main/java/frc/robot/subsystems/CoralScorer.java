// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralScorer extends SubsystemBase {
  private TalonFX mCoralMotor = new TalonFX(Constants.CoralScorerClass.CORALSCORERID, Constants.CANIVORE_NAME);
  /** Creates a new CoralScorer. */
  public CoralScorer() 
  {
    mCoralMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
  }
  public void coralOn(){
    mCoralMotor.set(0.35); 
  }
  public void coralOnSlow(){
    mCoralMotor.set(0.15); 
  }
  public void coralOff(){
    mCoralMotor.set(0.0);
  }
  public void coralReverse(){
    mCoralMotor.set(-0.25);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

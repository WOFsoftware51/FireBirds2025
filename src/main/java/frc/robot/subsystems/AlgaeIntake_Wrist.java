// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.Constable;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake_Wrist extends SubsystemBase {
  private TalonFX mAlgaeWrist = new TalonFX(Constants.AlgaeIntakeClass.ALGAEWRISTID, Constants.CANIVORE_NAME);
  private double m_target = 0.0;
  /** Creates a new AlgaeIntake. */
  public AlgaeIntake_Wrist() {
    mAlgaeWrist.setNeutralMode(NeutralModeValue.Brake);
  } 

  public void on(){
    mAlgaeWrist.set(0.3);
  }
  public void off(){
    mAlgaeWrist.set(0.0);
  }
  public void reverse(){
    mAlgaeWrist.set(-0.3);
  }
  public void onPercent(double speed){
    mAlgaeWrist.set(-speed * 0.35);
  }
  public void setPosition(double newPosition){
    mAlgaeWrist.setPosition(newPosition);
  }
  public double getWristPosition(){
    return mAlgaeWrist.getPosition().getValueAsDouble()*360/Constants.ArmClass.ARM_GEAR_RATIO;
  }

  public double getWristVelocity(){
    return mAlgaeWrist.getVelocity().getValueAsDouble()*360/Constants.ArmClass.ARM_GEAR_RATIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Wrist Position", getWristPosition());
    SmartDashboard.putNumber("Algae WristSpeed", getWristVelocity());
    SmartDashboard.putNumber("Algae Wrist Target Position", m_target);
  }
}

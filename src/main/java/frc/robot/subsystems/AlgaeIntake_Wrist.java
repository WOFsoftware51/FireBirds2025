// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.Constable;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake_Wrist extends SubsystemBase {
  private TalonFX mAlgaeWrist = new TalonFX(Constants.AlgaeIntakeClass.ALGAEWRISTID, Constants.CANIVORE_NAME);

  private MotionMagicDutyCycle magicDutyCycle = new MotionMagicDutyCycle(0);

  private double m_target = 0.0;
  /** Creates a new AlgaeIntake. */
  public AlgaeIntake_Wrist() {
    TalonFXConfiguration algaeIntakeConfig = new TalonFXConfiguration();
    MotionMagicConfigs algaeIntakeMotionMagic = algaeIntakeConfig.MotionMagic;
    
    algaeIntakeMotionMagic.MotionMagicCruiseVelocity = 600; //400// 5 rotations per second cruise
    algaeIntakeMotionMagic.MotionMagicAcceleration = 300; //160 // Take approximately 0.5 seconds %to reach max vel
    algaeIntakeMotionMagic.MotionMagicJerk = 2400;//1600// Take approximately 0.2 seconds to reach max accel 
    algaeIntakeConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    algaeIntakeConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -90*Constants.AlgaeIntakeClass.ALGAE_INTAKE_GEAR_RATIO/360;
    algaeIntakeConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    algaeIntakeConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90*Constants.AlgaeIntakeClass.ALGAE_INTAKE_GEAR_RATIO/360;
    
    Slot0Configs slot0 = algaeIntakeConfig.Slot0;
    slot0.kP = 1.92;
    slot0.kI = 0;
    slot0.kD = 0.0;
    slot0.kV = 0.0;
    slot0.kS = 0.375; // Approximately 0.375V to get the mechanism moving

    algaeIntakeConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    algaeIntakeConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
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

  private void resetEncoder(){
    mAlgaeWrist.setPosition(0.0);
  }

  public void goToPosition(double target) {
    m_target = target;
    mAlgaeWrist.setControl(magicDutyCycle.withPosition(target));
  }

  @Override
  public void periodic() {
    if (mAlgaeWrist.getForwardLimit().getValue().value == 0) {
      resetEncoder();
    }

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Forward Limit", mAlgaeWrist.getForwardLimit().getValue().value);
    SmartDashboard.putNumber("Algae Wrist Position", getWristPosition());
    SmartDashboard.putNumber("Algae WristSpeed", getWristVelocity());
    SmartDashboard.putNumber("Algae Wrist Target Position", m_target);
  }
}

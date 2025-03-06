// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.signals.NeutralModeValue; 
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Coral_Reef_Arm_Claw_Thing. */
  private TalonFX mArmMotor = new TalonFX(Constants.ArmClass.ARMID, Constants.CANIVORE_NAME);
  private CANcoder mArmCANcoder = new CANcoder(Constants.ArmClass.ARMCANCODERID, Constants.CANIVORE_NAME);

  
  private TalonFXConfiguration armConfig = new TalonFXConfiguration();
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private PositionDutyCycle armPositionDutyCycle = new PositionDutyCycle(0);
  private MotionMagicDutyCycle armMotionMagic = new MotionMagicDutyCycle(0);
  private double m_target = 0;

  public Arm() {
    mArmMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    MotionMagicConfigs armMotionMagic = armConfig.MotionMagic;
    armMotionMagic.MotionMagicCruiseVelocity = 20; //400// 5 rotations per second cruise
    armMotionMagic.MotionMagicAcceleration = 60; //160 // Take approximately 0.5 seconds %to reach max vel
    armMotionMagic.MotionMagicJerk = 0.0;//1600// Take approximately 0.2 seconds to reach max accel 
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -97.0*Constants.ArmClass.ARM_GEAR_RATIO/360;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5.0*Constants.ArmClass.ARM_GEAR_RATIO/360;
    
    Slot0Configs slot0 = armConfig.Slot0;
    slot0.kP = 1.92;
    slot0.kI = 0;
    slot0.kD = 0.0;
    slot0.kV = 0.0;
    slot0.kS = 0.375; // Approximately 0.375V to get the mechanism moving
     

    armConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    armConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    mArmMotor.getConfigurator().apply(armConfig);
    updateEncoderPosition();
  }


  public void ArmSetSpeed(double x){
    mArmMotor.setControl(dutyCycleOut.withOutput(x));
  }
  public void ArmToPostion(double target) {
    m_target = target;
    mArmMotor.setControl(armMotionMagic.withPosition(target * Constants.ArmClass.ARM_GEAR_RATIO/360));
  }
  public void armActive(double speed ){
    mArmMotor.set(speed); 
  }
  public void armOff(){
   mArmMotor.set(0);
  }
  public void armSetPosition(double newPosition){
    mArmMotor.setPosition(newPosition * Constants.ArmClass.ARM_GEAR_RATIO / 360);
  }
  public double getArmPosition(){
    return mArmMotor.getPosition().getValueAsDouble()*360/Constants.ArmClass.ARM_GEAR_RATIO;
  }
  public void updateEncoderPosition(){
      armSetPosition(mArmCANcoder.getPosition().getValueAsDouble() * 360 - Constants.ArmClass.ARMCANCODEROFFSET);
  }
  public double getArmVelocity(){
    return mArmMotor.getVelocity().getValueAsDouble()*360/Constants.ArmClass.ARM_GEAR_RATIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Position", getArmPosition());
    SmartDashboard.putNumber("Arm Speed", getArmVelocity());
    SmartDashboard.putNumber("Arm Target Position", m_target);
    SmartDashboard.putNumber("ARM Cancoder Position", mArmCANcoder.getAbsolutePosition().getValueAsDouble() * 360);
  }
}


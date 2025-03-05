 
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue; 
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private TalonFX mWristMotor = new TalonFX(Constants.WristClass.WRISTID, Constants.CANIVORE_NAME);
  private CANcoder mWristCANcoder = new CANcoder(Constants.WristClass.WRISTCANCODERID);

  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private PositionDutyCycle wristPositionDutyCycle = new PositionDutyCycle(0);
  private MotionMagicDutyCycle wristMotionMagic = new MotionMagicDutyCycle(0);

  private double m_target = 0;

  public Wrist() {
    TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    MotionMagicConfigs wristMotionMagic = wristConfig.MotionMagic;
    
    wristMotionMagic.MotionMagicCruiseVelocity = 600; //400// 5 rotations per second cruise
    wristMotionMagic.MotionMagicAcceleration = 300; //160 // Take approximately 0.5 seconds %to reach max vel
    wristMotionMagic.MotionMagicJerk = 2400;//1600// Take approximately 0.2 seconds to reach max accel 
    wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -90*Constants.WristClass.WRIST_GEAR_RATIO/360;
    wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90*Constants.WristClass.WRIST_GEAR_RATIO/360;
    
    Slot0Configs slot0 = wristConfig.Slot0;
    slot0.kP = 1.92;
    slot0.kI = 0;
    slot0.kD = 0.0;
    slot0.kV = 0.0;
    slot0.kS = 0.375; // Approximately 0.375V to get the mechanism moving

    wristConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    wristConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    mWristMotor.getConfigurator().apply(wristConfig);
    updateEncoderPosition();
  }


  public void WristSetSpeed(double x){
    mWristMotor.setControl(dutyCycleOut.withOutput(x));
  }
  public void WristToPostion(double target) {
    m_target = target;
    mWristMotor.setControl(wristMotionMagic.withPosition(target));
  }
  public void wristSetPosition(double newPosition){
    mWristMotor.setPosition(newPosition * Constants.ArmClass.ARM_GEAR_RATIO / 360);
  }
  public void updateEncoderPosition(){
    if(mWristCANcoder.isConnected())
      wristSetPosition(mWristCANcoder.getPosition().getValueAsDouble() - Constants.WristClass.WRISTCANCODEROFFSET);
  }
    //wristOnPercent
  public void wristActive(double speed ){
    mWristMotor.set(speed); 
  }
  public double getWristPosition(){
    return mWristMotor.getPosition().getValueAsDouble()*360/Constants.ArmClass.ARM_GEAR_RATIO;
  }

  public double getWristVelocity(){
    return mWristMotor.getVelocity().getValueAsDouble()*360/Constants.ArmClass.ARM_GEAR_RATIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Position", getWristPosition());
    SmartDashboard.putNumber("Wrist Speed", getWristVelocity());
    SmartDashboard.putNumber("Wrist Target Position", m_target);
  }
}
    
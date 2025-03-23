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

public class Hanger extends SubsystemBase {
  /** Creates a new hanger_elevator_Thing. */
  private TalonFX mHangerMotor = new TalonFX(Constants.HangerClass.HANGERID, Constants.CANIVORE_NAME);

  
  private TalonFXConfiguration hangerConfig = new TalonFXConfiguration();
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private PositionDutyCycle hangerPositionDutyCycle = new PositionDutyCycle(0);
  private MotionMagicDutyCycle hangerMotionMagic = new MotionMagicDutyCycle(0);
  private double m_target = 0;

  public Hanger() {
    // mhangerMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    MotionMagicConfigs hangerMotionMagic = hangerConfig.MotionMagic;
    hangerMotionMagic.MotionMagicCruiseVelocity = 20; //400// 5 rotations per second cruise
    hangerMotionMagic.MotionMagicAcceleration = 55; //160 // Take approximately 0.5 seconds %to reach max vel
    hangerMotionMagic.MotionMagicJerk = 0.0;//1600// Take approximately 0.2 seconds to reach max accel 
    hangerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    hangerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;//-97.0*Constants.HangerClass.Hanger_GEAR_RATIO/360;
    hangerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    hangerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;//5.0*Constants.hangerClass.hanger_GEAR_RATIO/360;
    
    Slot0Configs slot0 = hangerConfig.Slot0;
    slot0.kP = 1.92;
    slot0.kI = 0;
    slot0.kD = 0.0;
    slot0.kV = 0.0;
    slot0.kS = 0.375; // Approximately 0.375V to get the mechanism moving
     

    hangerConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    hangerConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
   // mArmMotor.getConfigurator().apply(armConfig);
   // updateEncoderPosition();
  }


  public void hangerSetSpeed(double x){
    mHangerMotor.setControl(dutyCycleOut.withOutput(x));
  }
  public void hangerToPostion(double target) {
    m_target = target;
    mHangerMotor.setControl(hangerMotionMagic.withPosition(target * Constants.HangerClass.HANGER_GEAR_RATIO/360));
  }
  public void hangerActive(double speed ){
    mHangerMotor.set(speed); 
  }
  public void hangerOff(){
   mHangerMotor.set(0);
  }
  public void hangerSetPosition(double newPosition){
    mHangerMotor.setPosition(newPosition * Constants.HangerClass.HANGER_GEAR_RATIO / 360);
  }
  public double getHangerPosition(){
    return mHangerMotor.getPosition().getValueAsDouble()*360/Constants.HangerClass.HANGER_GEAR_RATIO;
  }
  public void resetEncoder(){
    mHangerMotor.setPosition(0);
  }
 // public void updateEncoderPosition(){
   //   HangerSetPosition(mHangerCANcoder.getPosition().getValueAsDouble() * 360 - Constants.HangerClass.HangerCANCODEROFFSET);
 // }
  public double getHangerVelocity(){
    return mHangerMotor.getVelocity().
  getValueAsDouble()*360/Constants.HangerClass.HANGER_GEAR_RATIO;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("HANGER Position", getHangerPosition());
    SmartDashboard.putNumber("HANGER Speed", getHangerVelocity());
    SmartDashboard.putNumber("HANGER Target Position", m_target);
  }
}


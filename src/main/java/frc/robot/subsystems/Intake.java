// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
private TalonFX mIntakeMotor = new TalonFX(Constants.IntakeClass.INTAKEID, Constants.CANIVORE_NAME);
public Intake() {
  
}
public void powerOn(){
  mIntakeMotor.set(0.5);
}
public void runSmallVoltage(){
  mIntakeMotor.setControl(new VoltageOut(1.3));
}
public void reverse(){
  mIntakeMotor.set(-0.20);
}
public void powerOff(){
  mIntakeMotor.set(0);
}
public double getVelocityRPS(){
  return mIntakeMotor.getVelocity().getValueAsDouble();
}
public double getCurrent(){
  return mIntakeMotor.getStatorCurrent().getValueAsDouble();
}
@Override
  public void periodic() {
    SmartDashboard.putNumber("Top Intake Speed RPS", getVelocityRPS());
    SmartDashboard.putNumber("Top Intake CURRENT AMPS", getCurrent());
  }
}

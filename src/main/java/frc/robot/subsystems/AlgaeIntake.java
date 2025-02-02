// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.Constable;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
  private TalonFX mAlgaeIntake = new TalonFX(Constants.AlgaeIntakeClass.ALGAEINTAKEID, Constants.CANIVORE_NAME);
  private TalonFX mAlgaeWrist = new TalonFX(Constants.AlgaeIntakeClass.ALGAEWRISTID, Constants.CANIVORE_NAME);

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {} 

  public void setIntakeSpeed() {
    mAlgaeIntake.set(0.75);
  }
  public void setIntakeRevese(){
    mAlgaeIntake.set(-0.75);
  }
  public void intakeOff(){
      mAlgaeIntake.set(0.0);
  }

  public void setWristSpeed(){
    mAlgaeWrist.set(0.3);
  }
  public void wristOff(){
    mAlgaeWrist.set(0.0);
  }
  public void wristReverse(){
    mAlgaeWrist.set(-0.3);
  }
  public void wristOnPercent(double speed){
    mAlgaeWrist.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.Constable;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
  private TalonFX mAlgaeIntake = new TalonFX(Constants.AlgaeIntakeClass.ALGAEINTAKEID, Constants.CANIVORE_NAME);

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    mAlgaeIntake.setNeutralMode(NeutralModeValue.Brake);
  } 

  public void on(){
    mAlgaeIntake.set(1.0);
  }
  public void off(){
    mAlgaeIntake.set(0.0);
  }
  public void reverse(){
    mAlgaeIntake.set(-1.0);
  }
  public void onPercent(double speed){
    mAlgaeIntake.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

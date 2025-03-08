// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkIn extends SubsystemBase {
  private final Spark BlinkIn;

  /** Creates a new BlinkIn. */
  public BlinkIn() {
    BlinkIn = new Spark(0);
  }

  public void on(){
    BlinkIn.set(0.91);
  }
  public void onAlternate(){
    BlinkIn.set(0.93);
  }

  public void onReverse(){
    BlinkIn.set(-0.59);
  }
  public void off(){
    BlinkIn.set(0.0);
  }
  @Override
  public void periodic() {

  }
}

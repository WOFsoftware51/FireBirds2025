// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Auton_Functions {
  private static HashMap<String, Command> autonsHashMap = new HashMap<String, Command>();
  /** Creates a new Auton_Subsystem. */
  public Auton_Functions() 
  {
    initAutons();
  }
  private static void initAutons()
  {
    try {
      AutoBuilder. getAllAutoNames().forEach((autoName) -> {autonsHashMap.put(autoName, new PathPlannerAuto(autoName));});
    } catch (Exception e) {
      DriverStation.reportError("You Have No Autons :( " + e.getMessage(), e.getStackTrace());
    }
  }
  public static Command getAuton(String autonName)
  {
    try {
      return autonsHashMap.get(autonName);
    } 
    catch (Exception e) {
      DriverStation.reportError("Auton Doesn't Exist :(: " + e.getMessage(), e.getStackTrace());
      return new InstantCommand();
    }
  }

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
}

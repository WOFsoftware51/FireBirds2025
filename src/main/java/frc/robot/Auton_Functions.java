// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArmGoToPositionCommand;
import frc.robot.commands.IntakeForward;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.WristGoToPositionCommand;
import frc.robot.commands_auton.Auton_Wait;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class Auton_Functions {
  /** Creates a new Auton_Subsystem. */
  public Auton_Functions() 
  {
  }
  public static Command autonScore(CoralScorer coralScorer){ 
    return Commands.run(()->coralScorer.coralOn()).raceWith(autonWait(0.4).finallyDo((()->coralScorer.coralOn())));
  }
  public static Command autonScoreTop(Intake intake,Wrist wrist,Arm arm, int scorePosition ){
    return new ArmGoToPositionCommand(arm,scorePosition).alongWith(new WristGoToPositionCommand(wrist,scorePosition)).
      finallyDo(()-> intake.reverse());
  }
  public static Command autonIntakeTop(Intake intake){
    return new IntakeForward(intake).raceWith(autonWait(0.4));
  }
  public static Command autonWait(double timerSeconds)
  {
    return new Auton_Wait(timerSeconds*50.0);
  }
}

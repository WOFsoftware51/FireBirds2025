// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaeIntakeGoTo;
import frc.robot.commands.AlgaeWrist_Manual;
import frc.robot.commands.ArmGoToPositionCommand;
import frc.robot.commands.IntakeForward;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.WristGoToPositionCommand;
import frc.robot.commands_auton.Auton_Wait;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake_Wrist;
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
    return Commands.run(()->coralScorer.coralOn()).raceWith(autonWait(0.5).finallyDo((()->coralScorer.coralOn())));
  }
  public static Command autonScoreTop(Intake intake,Wrist wrist,Arm arm, int scorePosition ){
    return autonGoToPosition(arm, wrist, scorePosition).
      finallyDo(()-> intake.reverse());
  }
  public static Command autonGoToPosition(Arm arm, Wrist wrist, int scorePosition){
    return new ArmGoToPositionCommand(arm,scorePosition).alongWith(new WristGoToPositionCommand(wrist,scorePosition)).raceWith(autonWait(2.5));
  }


  public static Command autonIntakeTop(Intake intake){
    return new IntakeReverse(intake).raceWith(autonWait(0.8));
  }
  public static Command autonWait(double timerSeconds)
  {
    return new Auton_Wait(timerSeconds*50.0);
  }
  public static Command algaeLow(Intake intake,Wrist wrist,Arm arm ){
    return autonScoreTop(intake, wrist ,arm, Constants.ARM_HIGH_ALGAE_LVL3).andThen(new IntakeReverse(intake).raceWith(autonWait(5)));
  }
  public static Command autonAlgaePosition(AlgaeIntake algaeIntake,AlgaeIntake_Wrist algaeIntake_Wrist, CoralScorer coralScorer){
    return Commands.run(()->algaeIntake_Wrist.on()).raceWith(autonWait(0.5).finallyDo((()->algaeIntake_Wrist.on())));
  }

  public static Command autonAlgaeCommand(AlgaeIntake algaeIntake,AlgaeIntake_Wrist algaeIntake_Wrist, CoralScorer coralScorer){
    return new SequentialCommandGroup(
      Commands.run(()->algaeIntake.on()).raceWith(autonWait(3.0)),
      new AlgaeIntakeGoTo(algaeIntake_Wrist, Constants.Level_1).raceWith(autonWait(3.0).finallyDo((()->algaeIntake_Wrist.off()))));
  }
  public static Command autonStopCoral(CoralScorer coralScorer){
    return Commands.runOnce(()->coralScorer.coralOff());
  }
  public static Command autonStopAlgae(AlgaeIntake algaeIntake){
    return Commands.runOnce(()->algaeIntake.off());
}
public static Command autonAlgaeBack(AlgaeIntake_Wrist algaeIntake_Wrist){
  return new SequentialCommandGroup(
  new AlgaeWrist_Manual(algaeIntake_Wrist, ()-> -0.4).raceWith(autonWait(2.0)),
  autonStopAlgaeBack(algaeIntake_Wrist)
  );
}
public static Command autonStopAlgaeBack(AlgaeIntake_Wrist algaeIntake_Wrist){
  return Commands.runOnce(()->algaeIntake_Wrist.off());

  }
  public static Command autonStopAlgaeManualBack(AlgaeIntake_Wrist algaeIntake_Wrist){
    return Commands.runOnce(()->algaeIntake_Wrist.off());
}
public static Command ArmHome(Intake intake,Wrist wrist,Arm arm ){
  return autonScoreTop(intake, wrist ,arm, Constants.HOME).raceWith(autonWait(3.0)).finallyDo(()->intake.powerOff());
}
public static Command autonStopTopScorer(Wrist wrist, Arm arm){
  return Commands.runOnce(()->arm.armOff()).alongWith(Commands.runOnce(()->wrist.wristOff()));
}
}
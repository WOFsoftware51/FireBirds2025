// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristGoToPositionCommand extends Command {
  /** Creates a new WristGoToPositionCommand. */
  private Wrist mWrist;
  private int m_button;
  private double wristTarget;

  public WristGoToPositionCommand(Wrist wrist, int button) {
    this.m_button = button; 
    this.mWrist = wrist;
    addRequirements(mWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_button) {
   // case 1: 
     // wristTarget = Constants.WristClass.WRIST_LVL1_POSITION;
    //  break;
   
    //  case 3: 
    //  wristTarget = Constants.WristClass.WRIST_LVL2_POSITION;
    //  break;
      
    case Constants.Level_2: 
      wristTarget = Constants.WristClass.WRIST_LVL2_SCORE;
      break;
      
    case Constants.Level_3: 
        wristTarget = Constants.WristClass.WRIST_LVL3_SCORE;
        break;
        case Constants.HUMAN_PLAYER:
      wristTarget = Constants.WristClass.WRIST_HP_INTAKE;
      break;

    case Constants.HOME:
      wristTarget = Constants.WristClass.WRIST_HOME;
      break;
      case Constants.WRIST_HIGH_ALGAE_LVL3: 
      wristTarget = Constants.WristClass.WRIST_HIGH_ALGAE_LVL3;
      break;
      case Constants.HANGER: 
      wristTarget = Constants.WristClass.WRIST_HANG;
      break;
    default:
      wristTarget = Constants.WristClass.WRIST_HOME;
      break;
    }

    mWrist.WristToPostion(wristTarget);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mWrist.wristOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

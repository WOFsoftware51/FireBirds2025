// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake_Wrist;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmGoToPositionCommand extends Command {
  /** Creates a new ArmGoToPositions. */
  private Arm mArm;
  private int m_button;
  private double armTarget;
  public ArmGoToPositionCommand( Arm arm, int button) {
    this.m_button = button; 
    this.mArm = arm;
    addRequirements(mArm);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mArm.updateEncoderPosition();
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   switch (m_button) {
    // case 1: 
    //   armTarget = Constants.ArmClass.ARM_LVL1_POSITION;
    //   break;

    case Constants.Level_1: 
      armTarget = Constants.ArmClass.ARM_LVL1_SCORE;
      break;
    
    // case 3: 
    //   armTarget = Constants.ArmClass.ARM_LVL2_POSITION;
    //   break;
    
    case Constants.Level_2: 
      armTarget = Constants.ArmClass.ARM_LVL2_SCORE;
      break;
      
    case Constants.Level_3: 
      armTarget = Constants.ArmClass.ARM_LVL3_SCORE;
      break;

    case Constants.HUMAN_PLAYER: 
      armTarget = Constants.ArmClass.ARM_HP_INTAKE;
      break;

    case Constants.HOME: 
      armTarget = Constants.ArmClass.ARM_HOME;
      break;
    
      case Constants.ARM_HIGH_ALGAE_LVL3: 
      armTarget = Constants.ArmClass.ARM_HIGH_ALGAE_LVL3;
      break;

    default:
      armTarget = Constants.ArmClass.ARM_HOME;
      break;
   }
   mArm.ArmToPostion(armTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mArm.armOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

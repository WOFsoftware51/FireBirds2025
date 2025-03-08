package frc.robot.autos;

import frc.robot.AutoPath;
import frc.robot.Auton_Functions;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class New_Left_1_Piece extends SequentialCommandGroup {
    public New_Left_1_Piece(Intake intake, Wrist wrist, Arm arm){

        AutoPath L_R5 =  AutoPath.PP("L_R5");
      
        
    
        addCommands(
            L_R5.resetOdometryToStart(),
            L_R5.follow(),
            Auton_Functions.autonScoreTop(intake, wrist, arm, Constants.Level_2)
        );
    }
}
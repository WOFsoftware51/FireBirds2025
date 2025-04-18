package frc.robot.autos;

import frc.robot.AutoPath;
import frc.robot.Auton_Functions;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class New_Middle_1_Piece extends SequentialCommandGroup {
    public New_Middle_1_Piece(Intake intake, Wrist wrist, Arm arm){

        

        AutoPath C_R0 =  AutoPath.PP("C_R0");
      
        
    
        addCommands(
            C_R0.resetOdometryToStart(),
            C_R0.follow(),
           Auton_Functions.autonScoreTop(intake, wrist, arm, Constants.Level_1)

            );
    }
}

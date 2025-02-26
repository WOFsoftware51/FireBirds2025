package frc.robot.autos;

import frc.robot.AutoPath;
import frc.robot.Auton_Functions;
import frc.robot.subsystems.CoralScorer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Left_1_Piece extends SequentialCommandGroup {
    public Left_1_Piece(CoralScorer coralScorer){

        AutoPath L_R5 =  AutoPath.PP("L_R5");
      
        
    
        addCommands(
            L_R5.resetOdometryToStart(),
            L_R5.follow(),
            Auton_Functions.autonScore(coralScorer) );
    }
}
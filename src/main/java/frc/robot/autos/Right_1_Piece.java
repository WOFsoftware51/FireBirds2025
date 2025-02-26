package frc.robot.autos;

import frc.robot.AutoPath;
import frc.robot.Auton_Functions;
import frc.robot.subsystems.CoralScorer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Right_1_Piece extends SequentialCommandGroup {
    
    public Right_1_Piece(CoralScorer coralScorer){

        AutoPath R_R1 =  AutoPath.PP("R_R1");
      
        
    
        addCommands(
            R_R1.resetOdometryToStart(),
            R_R1.follow(),
            Auton_Functions.autonScore(coralScorer)  );
    }
}
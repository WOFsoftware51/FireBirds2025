package frc.robot.autos;

import frc.robot.AutoPath;
import frc.robot.Auton_Functions;
import frc.robot.subsystems.CoralScorer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Middle_Right_4_Piece extends SequentialCommandGroup {
    public Middle_Right_4_Piece(CoralScorer coralScorer){

        AutoPath C_R0 =  AutoPath.PP("C_R0");  
        AutoPath R0_HPR =  AutoPath.PP("R0_HPR");
        AutoPath HPR_R2 =  AutoPath.PP("HPR_R2");
        AutoPath R2_HPR =  AutoPath.PP("R2_HPR");
        // AutoPath HPR_R2 =  AutoPath.PP("HPR_R2");
        // AutoPath R2_HPR =  AutoPath.PP("R2_HPR");
        AutoPath HPR_R1 =  AutoPath.PP("HPR_R1");
      
    
        addCommands(
            C_R0.resetOdometryToStart(),
            C_R0.follow(),
            Auton_Functions.autonScore(coralScorer)
            HPR_R2.follow(),
            HPR_R2.follow()

            );
    }
}
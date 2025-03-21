package frc.robot.autos;

import frc.robot.AutoPath;
import frc.robot.Auton_Functions;
import frc.robot.commands_auton.Auton_Wait;
import frc.robot.subsystems.CoralScorer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Right_3_Piece extends SequentialCommandGroup {
    public Right_3_Piece(CoralScorer coralScorer){

        AutoPath R_R1   =  AutoPath.PP("R_R1");
        AutoPath R1_HPR =  AutoPath.PP("R1_HPR");
        AutoPath HPR_R2 =  AutoPath.PP("HPR_R2");
        AutoPath R2_HPR =  AutoPath.PP("R2_HPR");
        AutoPath HPR_R2_2 =  AutoPath.PP("HPR_R2_2");

        //AutoPath HPR_R2 =  AutoPath.PP("HPR_R2");
    
        addCommands(
            R_R1.resetOdometryToStart(),
            R_R1.follow(),
            Auton_Functions.autonScore(coralScorer),
            Auton_Functions.autonStopCoral(coralScorer),
            R1_HPR.follow(),
            //HP intake
            new Auton_Wait(50),
            HPR_R2.follow(),
            Auton_Functions.autonScore(coralScorer),
            Auton_Functions.autonStopCoral(coralScorer),
            R2_HPR.follow(),
            //HP intake
            new Auton_Wait(50),
            HPR_R2_2.follow(),
            Auton_Functions.autonScore(coralScorer),
            Auton_Functions.autonStopCoral(coralScorer));
    }
}
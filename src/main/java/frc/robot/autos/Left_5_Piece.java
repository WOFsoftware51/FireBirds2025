package frc.robot.autos;

import frc.robot.AutoPath;
import frc.robot.Auton_Functions;
import frc.robot.commands_auton.Auton_Wait;
import frc.robot.subsystems.CoralScorer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Left_5_Piece extends SequentialCommandGroup {
    public Left_5_Piece(CoralScorer coralScorer){

        AutoPath L_R5 =  AutoPath.PP("L_R5");  
        AutoPath R5_HPL =  AutoPath.PP("R5_HPL");
        AutoPath HPL_R4 =  AutoPath.PP("HPL_R4");
        AutoPath R4_HPL =  AutoPath.PP("R4_HPL");
        AutoPath HPL_R4_2 =  AutoPath.PP("HPL_R4_2");
        //AutoPath R4_HPL_2 =  AutoPath.PP("R4_HPL_2");
        //AutoPath HPL_R3 =  AutoPath.PP("HPL_R3");
       // AutoPath R3_HPL =  AutoPath.PP("R3_HPL");
        //AutoPath HPL_R3 =  AutoPath.PP("HPL_R3");
        addCommands(
            L_R5.resetOdometryToStart(),
            L_R5.follow(),
            Auton_Functions.autonScore(coralScorer),
            Auton_Functions.autonStopCoral(coralScorer),
            R5_HPL.follow(),
            //HP intake
            new Auton_Wait(75),
            HPL_R4.follow(),
            Auton_Functions.autonScore(coralScorer),
            Auton_Functions.autonStopCoral(coralScorer),
            R4_HPL.follow(),
            //HP intake
            new Auton_Wait(50),
            HPL_R4_2.follow(),
            Auton_Functions.autonScore(coralScorer),
            Auton_Functions.autonStopCoral(coralScorer)
            //R4_HPL_2.follow(),
            //HP intake
            //new Auton_Wait(50),
           // HPL_R3.follow(),
            //Auton_Functions.autonScore(coralScorer),
            //Auton_Functions.autonStopCoral(coralScorer));
           // R3_HPL.follow(),
            //HP intake
           // new Auton_Wait(50),
           // HPL_R3.follow(),
           // Auton_Functions.autonScore(coralScorer),
           // Auton_Functions.autonStopCoral(coralScorer) 
           );
    }
}
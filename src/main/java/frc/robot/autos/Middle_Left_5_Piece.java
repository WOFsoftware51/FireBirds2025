package frc.robot.autos;

import frc.robot.AutoPath;
import frc.robot.Auton_Functions;
import frc.robot.commands_auton.Auton_Wait;
import frc.robot.subsystems.CoralScorer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Middle_Left_5_Piece extends SequentialCommandGroup {
    public Middle_Left_5_Piece(CoralScorer coralScorer){

        AutoPath C_R0 =  AutoPath.PP("C_R0");
        AutoPath R0_HPL =  AutoPath.PP("R0_HPL");
        AutoPath HPL_R4=  AutoPath.PP("HPL_R4");
        AutoPath R4_HPL=  AutoPath.PP("R4_HPL");
        //AutoPath HPL_R4=  AutoPath.PP("HPL_R4");
        //AutoPath R4_HPL=  AutoPath.PP("R4_HPL");
        AutoPath HPL_R3=  AutoPath.PP("HPL_R3");
        AutoPath R3_HPL=  AutoPath.PP("R3_HPL");
        //AutoPath HPL_R3=  AutoPath.PP("HPL_R3");
        
        addCommands(
            C_R0.resetOdometryToStart(),
            C_R0.follow(),
            Auton_Functions.autonScore(coralScorer),
            R0_HPL.follow(),
            //HP intake
            new Auton_Wait(50),
            HPL_R4.follow(),
            Auton_Functions.autonScore(coralScorer),
            R4_HPL.follow(),
            //HP intake
            new Auton_Wait(50),
            HPL_R4.follow(),
            Auton_Functions.autonScore(coralScorer),
            R4_HPL.follow(),
            //HP intake
            new Auton_Wait(50),
            HPL_R3.follow(),
            Auton_Functions.autonScore(coralScorer),
            R3_HPL.follow(),
            //HP intake
            new Auton_Wait(50),
            HPL_R3.follow(),
            Auton_Functions.autonScore(coralScorer)  );
    }
}
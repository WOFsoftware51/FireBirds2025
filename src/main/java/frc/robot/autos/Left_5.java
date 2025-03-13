package frc.robot.autos;

import frc.robot.AutoPath;
import frc.robot.Auton_Functions;
import frc.robot.commands_auton.Auton_Wait;
import frc.robot.subsystems.CoralScorer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Left_5 extends SequentialCommandGroup {
    public Left_5(CoralScorer coralScorer){

        AutoPath R5_HPL =  AutoPath.PP("R5_HPL");
        AutoPath HPL_R4 =  AutoPath.PP("HPL_R4");
        AutoPath R4_HPL =  AutoPath.PP("R4_HPL");
        AutoPath HPL_R4_2 =  AutoPath.PP("HPL_R4_2");
        AutoPath R4_HPL_2 =  AutoPath.PP("R4_HPL_2");
         AutoPath HPL_R3 =  AutoPath.PP("HPL_R3");
        // AutoPath R3_HPL =  AutoPath.PP("R3_HPL");
        //AutoPath HPL_R3 =  AutoPath.PP("HPL_R3");
        addCommands(
            R5_HPL.resetOdometryToStart(),
            Auton_Functions.autonScore(coralScorer),
            Auton_Functions.autonStopCoral(coralScorer),
            R5_HPL.follow(),
            //HP intake
            new Auton_Wait(50),
            HPL_R4.follow(),
            Auton_Functions.autonScore(coralScorer),
            Auton_Functions.autonStopCoral(coralScorer),
            R4_HPL.follow(),
            //HP intake
            new Auton_Wait(50),
            HPL_R4_2.follow(),
            Auton_Functions.autonScore(coralScorer),
            Auton_Functions.autonStopCoral(coralScorer),
            R4_HPL_2.follow(),
            //HP intake
            new Auton_Wait(50),
            HPL_R3.follow(),
             Auton_Functions.autonScore(coralScorer),
             Auton_Functions.autonStopCoral(coralScorer)
         );
    }
}
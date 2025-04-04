package frc.robot.autos;

import frc.robot.AutoPath;
import frc.robot.Auton_Functions;
import frc.robot.subsystems.CoralScorer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Middle_1_Piece_LVL_2 extends SequentialCommandGroup {
    public Middle_1_Piece_LVL_2(CoralScorer coralScorer){

        AutoPath NP1 =  AutoPath.PP("NP1");
    
        addCommands(
            NP1.resetOdometryToStart(),
            NP1.follow(),
            Commands.waitSeconds(0.5),
            // Auton_Functions.armLVL2(intake,wrist,arm),
            Auton_Functions.autonStopCoral(coralScorer)
            );
    }
}
package frc.robot.autos;

import frc.robot.AutoPath;
import frc.robot.Auton_Functions;
import frc.robot.subsystems.CoralScorer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Middle_1_Piece extends SequentialCommandGroup {
    public Middle_1_Piece(CoralScorer coralScorer){

        AutoPath C_R0 =  AutoPath.PP("C_R0");
    
        addCommands(
            C_R0.resetOdometryToStart(),
            C_R0.follow(),
            Commands.waitSeconds(0.5),
            Auton_Functions.autonScore(coralScorer));
    }
}
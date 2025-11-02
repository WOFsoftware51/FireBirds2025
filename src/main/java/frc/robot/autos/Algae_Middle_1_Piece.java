package frc.robot.autos;

import frc.robot.AutoPath;
import frc.robot.Auton_Functions;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake_Wrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Algae_Middle_1_Piece extends SequentialCommandGroup {
    public Algae_Middle_1_Piece(CoralScorer coralScorer, AlgaeIntake_Wrist algaeIntake_Wrist, AlgaeIntake algaeIntake, Arm arm, Wrist wrist, Intake intake){

        AutoPath C_R0 =  AutoPath.PP("C_R0");
        AutoPath R0_C_RO =  AutoPath.PP("R0_C_RO");
        AutoPath C_R0_AG =  AutoPath.PP("C_R0_AG");
        AutoPath R0_0_5_C =  AutoPath.PP("R0_0_5_C");

    
        addCommands(
            C_R0.resetOdometryToStart(),
            C_R0.follow(),
            Commands.waitSeconds(0.5),
            Auton_Functions.autonScore(coralScorer),
            Auton_Functions.autonStopCoral(coralScorer),
            R0_C_RO.follow(),
            C_R0_AG.follow(),
            Commands.waitSeconds(0.2),
            Auton_Functions.autonGoToPosition(arm, wrist, Constants.ARM_HIGH_ALGAE_LVL3),
            Auton_Functions.autonIntakeTop(intake),
            Auton_Functions.ArmHome( intake, wrist, arm),
            Auton_Functions.autonStopTopScorer(wrist, arm));
}
}

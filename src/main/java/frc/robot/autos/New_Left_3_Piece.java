package frc.robot.autos;

import frc.robot.AutoPath;
import frc.robot.Auton_Functions;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class New_Left_3_Piece extends SequentialCommandGroup {
    public New_Left_3_Piece(Intake intake, Wrist wrist, Arm arm){

        AutoPath L_R5 =  AutoPath.PP("L_R5");
        AutoPath R5_HPL =  AutoPath.PP("R5_HPL");
        AutoPath HPL_R4=  AutoPath.PP("HPL_R4");
        AutoPath R4_HPL=  AutoPath.PP("R4_HPL");
        //AutoPath HPL_R4=  AutoPath.PP("HPL_R4");
        addCommands(
            L_R5.resetOdometryToStart(),
            L_R5.follow(),
            Auton_Functions.autonScoreTop(intake, wrist, arm, Constants.B_BUTTON),
            R5_HPL.follow(),
            Auton_Functions.autonIntakeTop(intake),
            HPL_R4.follow(),
            Auton_Functions.autonScoreTop(intake, wrist, arm, Constants.B_BUTTON),
            R4_HPL.follow(),
            Auton_Functions.autonIntakeTop(intake),
            HPL_R4.follow(),
            Auton_Functions.autonScoreTop(intake, wrist, arm, Constants.B_BUTTON) );
    }
}
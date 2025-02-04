package frc.robot.autos;

import frc.robot.AutoPath;
import frc.robot.subsystems.Auton_Functions;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Middle_1_Piece extends SequentialCommandGroup {
    public Middle_1_Piece(){

        AutoPath C_R0 = AutoPath.PP("C_R0");
        AutoPath R0_C = AutoPath.PP("R0_C");

        addCommands(
            C_R0.resetOdometryToStart(),
            C_R0.follow(),
            R0_C.follow()
        );
    }
}
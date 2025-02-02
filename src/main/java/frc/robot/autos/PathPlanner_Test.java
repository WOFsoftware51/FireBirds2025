package frc.robot.autos;

import frc.robot.subsystems.Auton_Subsystem;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathPlanner_Test extends SequentialCommandGroup {
    public PathPlanner_Test(Swerve s_Swerve, Auton_Subsystem m_Auton){

        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro())
            // Auton_Subsystem.getAuton("")
        );
    }
}
package frc.robot.autos;

import frc.robot.AutoPath;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathPlanner_Test extends SequentialCommandGroup {
    public PathPlanner_Test(){

        AutoPath examplePath = AutoPath.PP("Example Path");

        addCommands(
            // new InstantCommand(() -> s_Swerve.zeroGyro())
            examplePath.resetOdometryToStart(),
            examplePath.follow()
        );
    }
}
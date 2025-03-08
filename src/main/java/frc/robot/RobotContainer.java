package frc.robot;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import pabeles.concurrency.IntOperatorTask.Min;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandXboxController TestController = new CommandXboxController(2);
    


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver.getHID(), XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver.getHID(), XboxController.Button.kLeftBumper.value);

    /*Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();
    private final AlgaeIntake_Wrist m_AlgaeIntake_Wrist = new AlgaeIntake_Wrist();
    private final CoralScorer m_CoralScorer = new CoralScorer();
    private final Arm m_Arm = new Arm();
    private final Wrist m_Wrist = new Wrist();
    private final Intake m_Intake = new Intake();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false
            )
        );
        m_Wrist.setDefaultCommand(new Wrist_Command(m_Wrist, ()-> operator.getLeftY()));
        m_AlgaeIntake_Wrist.setDefaultCommand(new AlgaeWrist_Manual(m_AlgaeIntake_Wrist, ()-> TestController.getRightY()));
        m_Arm.setDefaultCommand(new Arm_Command(m_Arm, ()-> operator.getRightY()));
        m_Intake.setDefaultCommand(new IntakeDefaultCommand(m_Intake));
            
        // Configure the button bindings
        configureButtonBindings();
    } 

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.start().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        operator.back().whileTrue(Commands.runOnce(()-> m_Wrist.wristSetPosition(0)));
        operator.back().whileTrue(Commands.runOnce(()-> m_AlgaeIntake_Wrist.setPosition(0)));
        operator.back().whileTrue(Commands.runOnce(()-> m_Arm.armSetPosition(0)));
        
        // driver.leftTrigger(0.8).whileTrue(Commands.run(()-> m_CoralScorer.coralOnSlow()).finallyDo(()-> m_CoralScorer.coralOff()));
        // driver.rightBumper().whileTrue(Commands.run(()->{Global_sVariables.isBoost = true;}).finallyDo(()->{Global_Variables.isBoost = false;}));
      
        /*Driver O */
        driver.rightBumper().whileTrue(new AlgaeWrist_Manual (m_AlgaeIntake_Wrist, ()-> -0.25));
        driver.leftBumper().whileTrue(new AlgaeIntakeGoTo(m_AlgaeIntake_Wrist, Constants.HOME));
        driver.leftTrigger(0.8).whileTrue(new AlgaeIntakeGoTo(m_AlgaeIntake_Wrist, Constants.Level_1));
        driver.leftTrigger(0.8).whileTrue(Commands.run(()->m_AlgaeIntake.reverse()).finallyDo(()->m_AlgaeIntake.off()));
        // driver.leftBumper().whileTrue(Commands.run(()->m_AlgaeIntake.reverse()).finallyDo(()->m_AlgaeIntake.off()));
        driver.x().whileTrue(new AlgaeIntakeGoTo(m_AlgaeIntake_Wrist, Constants.ALGAE_GO_WRONG));
        operator.leftTrigger(0.8).whileTrue(new CoralScorerCommand(m_CoralScorer));
        operator.rightTrigger(0.8).whileTrue(new ArmGoToPositionCommand(m_Arm, Constants.ARM_HIGH_ALGAE_LVL3));
        operator.rightTrigger(0.8).whileTrue(new WristGoToPositionCommand(m_Wrist, Constants.WRIST_HIGH_ALGAE_LVL3));

        /*Operator O */
        operator.x().whileTrue(new ArmGoToPositionCommand(m_Arm, Constants.HUMAN_PLAYER));
        operator.x().whileTrue(new WristGoToPositionCommand(m_Wrist, Constants.HUMAN_PLAYER));
        operator.x().whileTrue(new IntakeReverse(m_Intake)); //Intake IN

        /**Scoring*/
        driver.rightTrigger(0.8).whileTrue(new AlgaeIntakeCommand(m_AlgaeIntake));
        operator.rightBumper().whileTrue(new IntakeForward(m_Intake)); //INTAKE OUT / SCORE
        operator.leftBumper().whileTrue(new IntakeReverse(m_Intake)); //Intake INz

        // driver.leftTrigger(0.8).whileTrue(Commands.run(()-> m_CoralScorer.coralReverse()).finallyDo(()-> m_CoralScorer.coralOff()));
        
        operator.y().whileTrue(new ArmGoToPositionCommand(m_Arm, Constants.HOME));
        operator.y().whileTrue(new WristGoToPositionCommand(m_Wrist, Constants.HOME));
        operator.a().whileTrue(new ArmGoToPositionCommand(m_Arm, Constants.Level_2));
        operator.a().whileTrue(new WristGoToPositionCommand(m_Wrist, Constants.Level_2));
        operator.b().whileTrue(new ArmGoToPositionCommand(m_Arm, Constants.Level_3));
        operator.b().whileTrue(new WristGoToPositionCommand(m_Wrist, Constants.Level_3));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new PathPlanner_Test();
    }
}

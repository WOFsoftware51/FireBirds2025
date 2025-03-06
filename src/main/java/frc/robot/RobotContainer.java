package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
    
    private SendableChooser<Double> wristPostionChooser = new SendableChooser<>(); 
    private SendableChooser<Double> armPostionChooser = new SendableChooser<>();  

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
                () -> robotCentric.getAsBoolean()
            )
        );
        m_AlgaeIntake_Wrist.setDefaultCommand(new AlgaeWrist_Manual(m_AlgaeIntake_Wrist, ()-> operator.getLeftY()));
        m_Wrist.setDefaultCommand(new Wrist_Command(m_Wrist, ()-> operator.getRightY()));
        m_Arm.setDefaultCommand(new Arm_Command(m_Arm, ()-> TestController.getRightY()));
        // Configure the button bindings
        configureButtonBindings();
        print();

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

        operator.start().whileTrue(new AlgaeIntakeGoTo(m_AlgaeIntake_Wrist, 2));

        driver.rightTrigger(0.8).whileTrue(new CoralScorerCommand(m_CoralScorer));
        driver.leftTrigger(0.8).whileTrue(Commands.run(()-> m_CoralScorer.coralReverse()).finallyDo(()-> m_CoralScorer.coralOff()));
        // driver.leftTrigger(0.8).whileTrue(Commands.run(()-> m_CoralScorer.coralOnSlow()).finallyDo(()-> m_CoralScorer.coralOff()));
        // driver.rightBumper().whileTrue(Commands.run(()->{Global_Variables.isBoost = true;}).finallyDo(()->{Global_Variables.isBoost = false;}));
        
        /**Scoring*/
        operator.rightTrigger(0.8).whileTrue(new AlgaeIntakeCommand(m_AlgaeIntake));
        operator.leftTrigger(0.8).whileTrue(Commands.run(()->m_AlgaeIntake.reverse()).finallyDo(()->m_AlgaeIntake.off()));
        operator.rightBumper().whileTrue(new IntakeForward(m_Intake));
        operator.leftBumper().whileTrue(new IntakeReverse(m_Intake));

        operator.a().whileTrue(new ArmGoToPositionCommand(m_Arm, Constants.A_BUTTON));
        operator.a().whileTrue(new WristGoToPositionCommand(m_Wrist, Constants.A_BUTTON));
        operator.b().whileTrue(new ArmGoToPositionCommand(m_Arm, Constants.B_BUTTON));
        operator.b().whileTrue(new WristGoToPositionCommand(m_Wrist, Constants.B_BUTTON));
        operator.x().whileTrue(new ArmGoToPositionCommand(m_Arm, Constants.X_BUTTON));
        operator.x().whileTrue(new WristGoToPositionCommand(m_Wrist, Constants.X_BUTTON));
        operator.y().whileTrue(new ArmGoToPositionCommand(m_Arm, Constants.Y_BUTTON));
        operator.y().whileTrue(new WristGoToPositionCommand(m_Wrist, Constants.Y_BUTTON));
    }
    private void print(){
        SmartDashboard.putData("Wrist Position", wristPostionChooser);
        wristPostionChooser.addOption("180 W Degrees",  180.0);
        wristPostionChooser.addOption("90 W Degrees",  90.0);
        wristPostionChooser.addOption("45 W Degrees",  45.0);
        wristPostionChooser.setDefaultOption("0 W Degrees",  0.0);
        wristPostionChooser.addOption("-180 W Degrees",  -180.0);
        wristPostionChooser.addOption("-90 W Degrees",  -90.0);
        wristPostionChooser.addOption("-45 W Degrees",  -45.0);

        SmartDashboard.putData("Arm Position", armPostionChooser);
        // armPostionChooser.addOption("180 Arm Degrees",  360.0);
        // armPostionChooser.addOption("180 Arm Degrees",  270.0);
        armPostionChooser.addOption("180 Arm Degrees",  180.0);
        armPostionChooser.addOption("90 Arm Degrees",  90.0);
        armPostionChooser.setDefaultOption("0 Arm Degrees",  0.0);
        armPostionChooser.setDefaultOption("-90 Arm Degrees",  -90.0);
        armPostionChooser.setDefaultOption("-180 Arm Degrees",  -180.0);
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

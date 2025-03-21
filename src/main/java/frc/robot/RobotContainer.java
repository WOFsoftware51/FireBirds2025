package frc.robot;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
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
    
    private SendableChooser<Integer> m_AutoChooser = new SendableChooser<>();

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
    private final BlinkIn m_BlinkIn = new BlinkIn();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        Global_Variables.limeLight = Limelight.getInstance();
        printAutos();
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> driver.getHID().getRightBumperButton()
            )
        );
        m_Wrist.setDefaultCommand(new Wrist_Command(m_Wrist, ()-> operator.getLeftY()));
        m_AlgaeIntake_Wrist.setDefaultCommand(new AlgaeWrist_Manual(m_AlgaeIntake_Wrist, ()-> TestController.getRightY()));
        m_Arm.setDefaultCommand(new Arm_Command(m_Arm, ()-> operator.getRightY()));
        m_Intake.setDefaultCommand(new IntakeDefaultCommand(m_Intake));
        m_BlinkIn.setDefaultCommand(new BlinkIn_Command(m_BlinkIn, Constants.BlinkinClass.BLINKIN_PURPLE_DEFAULT));
        m_AlgaeIntake.setDefaultCommand(new HasAlgaeCommand(m_AlgaeIntake));

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
        driver.a().whileTrue(new AlgaeWrist_Manual (m_AlgaeIntake_Wrist, ()-> -0.50));
        driver.leftBumper().whileTrue(new AlgaeIntakeGoTo(m_AlgaeIntake_Wrist, Constants.HOME));
        driver.leftTrigger(0.8).whileTrue(new AlgaeIntakeGoTo(m_AlgaeIntake_Wrist, Constants.Level_1));
        driver.leftTrigger(0.8).whileTrue(Commands.run(()->m_AlgaeIntake.reverse()).finallyDo(()->m_AlgaeIntake.off()));
        driver.b().whileTrue(new TeleopSwerve(
            s_Swerve, 
            ()-> (Global_Variables.limeLight.getTV()* 0.05*(Global_Variables.limeLight.getTY()+6)), 
            ()-> (Global_Variables.limeLight.getTV()* 0.015* (Global_Variables.limeLight.getTX()+17)), 
            ()-> (Global_Variables.limeLight.getTV()* 0.05* (Global_Variables.limeLight.getDesiredAngle() - s_Swerve.getHeading().getDegrees())), 
            ()->true));

            driver.x().whileTrue(new TeleopSwerve(
            s_Swerve, 
            ()-> (Global_Variables.limeLight.getTV()* 0.05*(Global_Variables.limeLight.getTY()+6)), 
            ()-> (Global_Variables.limeLight.getTV()* 0.015* (Global_Variables.limeLight.getTX()-17)), 
            ()-> (Global_Variables.limeLight.getTV()* 0.05* (Global_Variables.limeLight.getDesiredAngle() - s_Swerve.getHeading().getDegrees())), 
            ()->true));

        // driver.leftBumper().whileTrue(Commands.run(()->m_AlgaeIntake.reverse()).finallyDo(()->m_AlgaeIntake.off()));
        // driver.x().whileTrue(new AlgaeIntakeGoTo(m_AlgaeIntake_Wrist, Constants.ALGAE_GO_WRONG));
        operator.leftTrigger(0.8).whileTrue(new CoralScorerCommand(m_CoralScorer));
        operator.rightTrigger(0.8).whileTrue(new ArmGoToPositionCommand(m_Arm, Constants.ARM_HIGH_ALGAE_LVL3));
        operator.rightTrigger(0.8).whileTrue(new WristGoToPositionCommand(m_Wrist, Constants.WRIST_HIGH_ALGAE_LVL3));
        operator.leftTrigger(0.8).whileTrue(new BlinkIn_Command(m_BlinkIn, Constants.BlinkinClass.BLINKIN_WHITE));
        driver.leftTrigger(0.8).whileTrue(new BlinkIn_Command(m_BlinkIn, Constants.BlinkinClass.BLINKIN_OCEAN));

        /*Operator O */
        operator.x().whileTrue(new ArmGoToPositionCommand(m_Arm, Constants.HUMAN_PLAYER));
        operator.x().whileTrue(new WristGoToPositionCommand(m_Wrist, Constants.HUMAN_PLAYER));
        operator.x().whileTrue(new IntakeForward(m_Intake)); //Intake IN

        /**Scoring*/
        driver.rightTrigger(0.8).whileTrue(new AlgaeIntakeCommand(m_AlgaeIntake));
        operator.rightBumper().whileTrue(new IntakeForward(m_Intake)); //INTAKE OUT / SCORE
        operator.leftBumper().whileTrue(new IntakeReverse(m_Intake)); //Intake INz
        operator.leftBumper().whileTrue(new BlinkIn_Command(m_BlinkIn, Constants.BlinkinClass.BLINKIN_WHITE));


        // driver.leftTrigger(0.8).whileTrue(Commands.run(()-> m_CoralScorer.coralReverse()).finallyDo(()-> m_CoralScorer.coralOff()));
        
        operator.y().whileTrue(new ArmGoToPositionCommand(m_Arm, Constants.HOME));
        operator.y().whileTrue(new WristGoToPositionCommand(m_Wrist, Constants.HOME));
        operator.a().whileTrue(new BlinkIn_Command(m_BlinkIn, Constants.BlinkinClass.BLINKIN_WHITE_SLOW));
        operator.a().whileTrue(new ArmGoToPositionCommand(m_Arm, Constants.Level_2));
        operator.a().whileTrue(new WristGoToPositionCommand(m_Wrist, Constants.Level_2));
        operator.b().whileTrue(new BlinkIn_Command(m_BlinkIn, Constants.BlinkinClass.BLINKIN_WHITE_SLOW));
        operator.b().whileTrue(new ArmGoToPositionCommand(m_Arm, Constants.Level_3));
        operator.b().whileTrue(new WristGoToPositionCommand(m_Wrist, Constants.Level_3));
    }
    private void printAutos(){
        SmartDashboard.putData("Auto Chooser", m_AutoChooser);
        m_AutoChooser.setDefaultOption("DO NOTHING", 0);
        m_AutoChooser.addOption("Middle 1 Piece", 1);
        m_AutoChooser.addOption("Path Planner Test", 2);
        m_AutoChooser.addOption("Algae Middle 1 Piece", 3);
        m_AutoChooser.addOption("copy", 4);
        m_AutoChooser.addOption("Left 5", 5);
        m_AutoChooser.addOption("Left_5_Piece", 6);
        m_AutoChooser.addOption("Right_3_Piece", 7);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        switch (m_AutoChooser.getSelected()) {
            case 0:
                return Commands.print("Default Auto");
            case 1:
                return new Middle_1_Piece(m_CoralScorer);
            case 2:
                return new PathPlanner_Test();
            case 3:
                return new Algae_Middle_1_Piece(m_CoralScorer, m_AlgaeIntake_Wrist, m_AlgaeIntake, m_Arm, m_Wrist, m_Intake);
            case 4:
            return new copy(m_CoralScorer);
            case 5:
            return new Left_5(m_CoralScorer);
            case 6:
            return new Left_5_Piece(m_CoralScorer);
            case 7:
            return new Right_3_Piece(m_CoralScorer);
            default:
                break;
        }
        // An ExampleCommand will run in autonomous
        return new PathPlanner_Test();

           //     s_Swerve.drive(
       // new Translation2d(-0.05*s_Swerve.ty, -0.015*s_Swerve.tx).times(Constants.Swerve.maxSpeed), 
       // rotationPercent * Constants.Swerve.maxAngularVelocity, 
       // true, 
       // true
      //  );
       
    }
}

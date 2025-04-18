package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final String CANIVORE_NAME = "CANivore";

    public static final int Level_1 = 1;
    public static final int Level_2 = 2;
    public static final int Level_3 = 3;
    public static final int HOME = 4;
    public static final int HUMAN_PLAYER = 5;
    public static final int ARM_HIGH_ALGAE_LVL3 = 6;
    public static final int WRIST_HIGH_ALGAE_LVL3 = 7;
    public static final int ALGAE_GO_WRONG = 8;
    public static final String BlinkInClass = null;
    public static final int WHITE = 9;
    public static final int DEFAULT_PURPLE = 10;
    public static final int HANGER = 11;

    public static final class CoralScorerClass {
        public static final int CORALSCORERID = 3;
    }

    public static final class AlgaeIntakeClass {
        public static final int ALGAEINTAKEID = 1;
        public static final int ALGAEWRISTID = 2;
        public static final double ALGAE_INTAKE_POSITION = -55.0;
        public static final double ALGAE_INTAKE_AUTON_POSITION = -25.0;
        public static final double ALGAE_INTAKE_HOME = 0.0;
        public static final double ALGAE_INTAKE_GEAR_RATIO = 100.0;
        public static final double ALGAEINTAKECANCODERID = 3;
        public static final double ALGAE_LIMIT_SWITCH = 55.0;
    }
    public static final class ArmClass {
        public static final int ARMID = 4;
        public static final int ARMCANCODERID = 7;
        public static final double ARMCANCODEROFFSET = -166.0;
        public static final double ARM_GEAR_RATIO = 144.83;

        public static final double ARM_LVL1_SCORE = -76.0;//-84.0;
        public static final double ARM_LVL2_SCORE = -107;//-84.0;105;
        public static final double ARM_LVL3_SCORE = -74;//-65.0;
        public static final double ARM_HP_INTAKE = 0.0;
        public static final double ARM_HOME = -4.0;
        public static final double ARM_HIGH_ALGAE_LVL3 = -105.0;
        public static final double ARM_HANG = -50.0;
    }
   
    public static final class IntakeClass {
        public static final int INTAKEID = 5;
    }
    public static final class WristClass {  
        public static final int WRISTID = 6;
        public static final int WRISTCANCODERID = 8;
        public static final int WRISTCANCODEROFFSET = 0;
        public static final double WRIST_GEAR_RATIO = 32.0;

        public static final double WRIST_LVL2_SCORE = 45.0;//6.0; 52.0; 33.0; 42.0;
        public static final double WRIST_LVL3_SCORE = 60.0;// 55.0; 57.0;
        public static final double WRIST_HP_INTAKE = 117.0;//133.0;
        public static final double WRIST_HOME = 80.0; /// 68.19
        public static final double WRIST_HIGH_ALGAE_LVL3 = 85.0;
        public static final double WRIST_HANG = 140.0;
    }
    public static final class BlinkinClass {
        public static final double BLINKIN_PURPLE_DEFAULT = 0.91;
        public static final double BLINKIN_WHITE = -0.05;
        public static final double BLINKIN_AQUA = 0.81;
        public static final double BLINKIN_ORANGE = 0.65; 
        public static final double BLINKIN_OCEAN = -0.37;
        public static final double BLINKIN_WHITE_SLOW = -0.21;
    }
    public static final class HangerClass {
        public static final double HANGERUP= -0.50;
        public static final double HANGERDOWN = -0.0;
        public static final double HANGER_GEAR_RATIO = 101.0;
        public static final int HANGERID = 8;


    }
    public static final class Swerve {
        public static final double DRIVE_SPEED = 0.6;
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.0); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(24.0); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 40;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        public static final double angleKS = 0.1;//(0.667 / 12);//0.32; //TODO: This must be tuned to specific robot
        public static final double angleKV = 1.59;//(1.51 / 12);//1.51;
        public static final double angleKA = 0.0;//(0.27 / 12);//0.27;


        /* Drive Motor PID Values */
        public static final double driveKP = 0.1;//0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.0;//0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 0.0;//1.51;
        public static final double driveKA = 0.124;//0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.73; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 30;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.258056640625);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 21;
            public static final int angleMotorID = 31;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.3740234375+0.5);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 22;
            public static final int angleMotorID = 32;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.17529296875);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 23;
            public static final int angleMotorID = 33;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.08349609375+0.5);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }


    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 4.73;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
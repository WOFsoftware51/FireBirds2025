package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.AutoConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final String CANIVORE_NAME = "CANivore";

    public static final class CoralScorerClass {
        public static final int CORALSCORERID = 3;
    }

    public static final class AlgaeIntakeClass {
        public static final int ALGAEINTAKEID = 1;
        public static final int ALGAEWRISTID = 2;
    }


    public static final class Swerve {
        public static PIDController positionControllerX = new PIDController(AutoConstants.kPXController, 0.0, 0.0);
        public static PIDController positionControllerY = new PIDController(AutoConstants.kPYController, 0.0, 0.0);
        public static ProfiledPIDController positionControllerTheta = new ProfiledPIDController(AutoConstants.kPThetaController, 0.0, 0.0, AutoConstants.kThetaControllerConstraints);
        public static final double DRIVE_SPEED = 0.6;
        public static final int pigeonID = 1;
        public static final PPHolonomicDriveController ppHolonomicController = 
            new PPHolonomicDriveController(               
                 new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants 
                new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0) // Rotation PID constants
        );

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
    public static class Vision {        
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N4, N1> kSingleTagStdDevs = VecBuilder.fill(0.045, 0.045, Double.MAX_VALUE, Double.MAX_VALUE); //TODO FIND ACTUAL VALUES
        public static final Matrix<N4, N1> kMultiTagStdDevs = VecBuilder.fill(0.045, 0.045,  Double.MAX_VALUE, Double.MAX_VALUE);
        
        public static final Map<Integer, AprilTag> aprilTagListReefBlue = kTagLayout.getTags().stream().filter(tag-> tag.ID > 16 && tag.ID < 23).collect(Collectors.toMap(apriltag -> apriltag.ID, apriltag -> apriltag));
        public static final Map<Integer, AprilTag> aprilTagListReefRed = kTagLayout.getTags().stream().filter(tag-> tag.ID >= 6 && tag.ID <= 11).collect(Collectors.toMap(apriltag -> apriltag.ID, apriltag -> apriltag));
        public static final HashMap<Integer, AprilTag> aprilTagListReef = new HashMap<>(aprilTagListReefRed); 
                                                                        static {aprilTagListReef.putAll(aprilTagListReefBlue);}
        public static final Map<Integer, AprilTag> aprilTagListHumanPlayerBlue = kTagLayout.getTags().stream().filter(tag-> tag.ID == 12 && tag.ID == 13).collect(Collectors.toMap(apriltag -> apriltag.ID, apriltag -> apriltag));
        public static final Map<Integer, AprilTag> aprilTagListHumanPlayerRed = kTagLayout.getTags().stream().filter(tag-> tag.ID == 1 && tag.ID == 2).collect(Collectors.toMap(apriltag -> apriltag.ID, apriltag -> apriltag));
        public static final Map<Integer, AprilTag> aprilTagList = kTagLayout.getTags().stream().collect(Collectors.toMap(apriltag -> apriltag.ID, apriltag -> apriltag));
    }



    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 4.73;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
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
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DRIVETRAIN_PIGEON_ID;
import static frc.robot.Constants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DRIVETRAIN_WHEELBASE_METERS;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  public static final double MAX_VOLTAGE = 12.0;

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
      SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
  public static final double MAX_VELOCITY_METERS_PER_SECOND_TRAJECTORY = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
      SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
  WPI_Pigeon2 gyroscope = new WPI_Pigeon2(DRIVETRAIN_PIGEON_ID);
  

  private final SwerveDriveOdometry odometry;

  // Field object to keep track of location of bot.
  private final Field2d field = new Field2d();

  // These are our modules. We initialize them in the constructor.
  public SwerveModule frontLeftModule;
  public SwerveModule frontRightModule;
  public SwerveModule backLeftModule;
  public SwerveModule backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private SwerveModuleState[] states;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
    // zeroGyroscope();

    frontLeftModule = new MkSwerveModuleBuilder()
        .withLayout(shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0))
        .withGearRatio(SdsModuleConfigurations.MK4I_L2)
        .withDriveMotor(MotorType.NEO, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, Constants.FRONT_LEFT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(Constants.FRONT_LEFT_MODULE_STEER_ENCODER)
        .withSteerOffset(Constants.FRONT_LEFT_MODULE_STEER_OFFSET)
        .build();

    frontRightModule = new MkSwerveModuleBuilder()
        .withLayout(shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0))
        .withGearRatio(SdsModuleConfigurations.MK4I_L2)
        .withDriveMotor(MotorType.NEO, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER)
        .withSteerOffset(Constants.FRONT_RIGHT_MODULE_STEER_OFFSET)
        .build();

    backLeftModule = new MkSwerveModuleBuilder()
        .withLayout(shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0))
        .withGearRatio(SdsModuleConfigurations.MK4I_L2)
        .withDriveMotor(MotorType.NEO, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, Constants.BACK_LEFT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(Constants.BACK_LEFT_MODULE_STEER_ENCODER)
        .withSteerOffset(Constants.BACK_LEFT_MODULE_STEER_OFFSET)
        .build();

    backRightModule = new MkSwerveModuleBuilder()
        .withLayout(shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0))
        .withGearRatio(SdsModuleConfigurations.MK4I_L2)
        .withDriveMotor(MotorType.NEO, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, Constants.BACK_RIGHT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(Constants.BACK_RIGHT_MODULE_STEER_ENCODER)
        .withSteerOffset(Constants.BACK_RIGHT_MODULE_STEER_OFFSET)
        .build();

    odometry = new SwerveDriveOdometry(
        kinematics,
        Rotation2d.fromDegrees(getHeading()), getPositions());

    // Potential fix to wheels not aligning properly
    Timer.delay(1.0);
    frontLeftModule.resetToAbsolute();
    frontRightModule.resetToAbsolute();
    backLeftModule.resetToAbsolute();
    backRightModule.resetToAbsolute();


    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getSpeeds, 
      this::drive, 
      Constants.pathFollowerConfig,
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
}

  public double getHeading() {
    return Math.IEEEremainder(gyroscope.getYaw(), 360);
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] swervePositions = {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
    };
    return swervePositions;
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] swerveStates = {
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState()
    };
    return swerveStates;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    field.setRobotPose(getPose());
  }

  public Rotation2d getGyroscopeRotation() {
    // Don't Remove Follwoing if you are using a Pigeon
    // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
    return Rotation2d.fromDegrees(gyroscope.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
    states = kinematics.toSwerveModuleStates(m_chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[3].angle.getRadians());
  }
  public void zeroGyro(){
    gyroscope.setYaw(0.0);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(gyroscope.getRotation2d(), getPositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }
//* possibly will need from the pathplanner example project */

//   public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
//     driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
//   }

//   public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
//     ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

//     SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
//     setStates(targetStates);
//   }

//   public void setStates(SwerveModuleState[] targetStates) {
//     SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Swerve.maxModuleSpeed);

//     for (int i = 0; i < states.length; i++) {
//       states[i].setTargetState(targetStates[i]);
//     }
//   }

//   public SwerveModuleState[] getModuleStates() {
//     SwerveModuleState[] states = new SwerveModuleState[states.length];
//     for (int i = 0; i < states.length; i++) {
//       states[i] = states[i].getStates();
//     }
//     return states;
//   }

//   public SwerveModulePosition[] getPosition() {
//     SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
//     for (int i = 0; i < states.length; i++) {
//       positions[i] = states[i].getPositions();
//     }
//     return positions;
//   }
}

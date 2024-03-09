// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TargetConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(I2C.Port.kMXP);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getHeadingOdometry(),
      getModulePositions(),
      new Pose2d());

  private final ProfiledPIDController m_rotationPID;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(Supplier<Translation2d> aimingVectorSupplier) {
    // Reset and calibrate
    resetGyro();
    // m_gyro.setAngleAdjustment(180);
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        AutoConstants.AutoPathFollowerConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
    PPHolonomicDriveController.setRotationTargetOverride(() -> {
      return Optional.of(aimingVectorSupplier.get().getAngle());
    });

    m_rotationPID = new ProfiledPIDController(
        DriveConstants.kRotationPID.kP, DriveConstants.kRotationPID.kI, DriveConstants.kRotationPID.kD,
        new TrapezoidProfile.Constraints(
            DriveConstants.kAutoAimMaxAngularSpeed,
            DriveConstants.kAutoAimMaxAngularAccel),
        0.02);

    m_rotationPID.setIntegratorRange(-DriveConstants.kRotationPID.iZone, DriveConstants.kRotationPID.iZone);
    m_rotationPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void turnTo(double degrees) {
    /*
     * m_frontLeft.m_drivingPIDController.setReference(rot,
     * CANSparkMax.ControlType.kPosition);
     * m_frontRight.m_drivingPIDController.setReference(rot,
     * CANSparkMax.ControlType.kPosition);
     * m_rearLeft.m_drivingPIDController.setReference(rot,
     * CANSparkMax.ControlType.kPosition);
     * m_rearRight.m_drivingPIDController.setReference(rot,
     * CANSparkMax.ControlType.kPosition);
     */
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("robot heading", getHeading().getDegrees());

    SmartDashboard.putNumber("Velocity (RPM)", m_frontLeft.m_drivingSparkMax.getEncoder().getVelocity());

    updateOdometry();
  }

  /**
   * Reset the forward direction of the robot
   */
  public void resetGyro() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Pose2d newPose = new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(alliance == Alliance.Blue ? 0 : 180));
    resetPose(newPose);
  }

  /**
   * Update the odometry with an estimation from the vision system
   */
  public void updateOdometryWithVision(Pose2d pose, double timestamp) {
    m_odometry.addVisionMeasurement(pose, timestamp);
  }

  private void updateOdometry() {
    // Update the odometry in the periodic block
    m_odometry.update(
        getHeadingOdometry(),
        getModulePositions());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
        getHeadingOdometry(),
        getModulePositions(),
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotSpeed      Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;
    double rotSpeedCommanded;

    if (rateLimit) {
      ChassisSpeeds limitedSpeeds = limitSlewRate(xSpeed, ySpeed, rotSpeed);

      xSpeedCommanded = limitedSpeeds.vxMetersPerSecond;
      ySpeedCommanded = limitedSpeeds.vyMetersPerSecond;
      rotSpeedCommanded = limitedSpeeds.omegaRadiansPerSecond;
    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      rotSpeedCommanded = rotSpeed;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotSpeedDelivered = rotSpeedCommanded * DriveConstants.kMaxAngularSpeed;

    setSwerveSpeeds(xSpeedDelivered, ySpeedDelivered, rotSpeedDelivered, fieldRelative);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed         Speed of the robot in the x direction (forward).
   * @param ySpeed         Speed of the robot in the y direction (sideways).
   * @param targetRotation Rotation to set the robot to in radians
   * @param fieldRelative  Whether the provided x and y speeds are relative to the
   *                       field.
   * @param rateLimit      Whether to enable rate limiting for smoother control.
   */
  public void driveWithHeading(double xSpeed, double ySpeed, Rotation2d targetRotation, boolean fieldRelative,
      boolean rateLimit) {
    double rotSpeed = m_rotationPID.calculate(
        getHeading().getRadians(),
        new TrapezoidProfile.State(targetRotation.getRadians(), 0));

    double xSpeedCommanded;
    double ySpeedCommanded;
    double rotSpeedCommanded;

    if (rateLimit) {
      ChassisSpeeds limitedSpeeds = limitSlewRate(xSpeed, ySpeed, rotSpeed);

      xSpeedCommanded = limitedSpeeds.vxMetersPerSecond;
      ySpeedCommanded = limitedSpeeds.vyMetersPerSecond;
      rotSpeedCommanded = limitedSpeeds.omegaRadiansPerSecond;
    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      rotSpeedCommanded = rotSpeed;
    }

    // Convert the translational speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotSpeedDelivered = rotSpeedCommanded;

    setSwerveSpeeds(xSpeedDelivered, ySpeedDelivered, rotSpeedDelivered, fieldRelative);
  }

  /**
   * Limit the slew rate of the robot to reduce wear
   * 
   * @param xSpeed   Speed of the robot in the x direction (forward).
   * @param ySpeed   Speed of the robot in the y direction (sideways).
   * @param rotSpeed Angular rate of the robot.
   * @return a new limited ChassisSpeeds with the updated values
   */
  private ChassisSpeeds limitSlewRate(double xSpeed, double ySpeed, double rotSpeed) {
    // Convert XY to polar for rate limiting
    double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
    double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

    // Calculate the direction slew rate based on an estimate of the lateral
    // acceleration
    double directionSlewRate;
    if (m_currentTranslationMag != 0.0) {
      directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
    } else {
      directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
    }

    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
    if (angleDif < 0.45 * Math.PI) {
      m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
    } else if (angleDif > 0.85 * Math.PI) {
      if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
        // keep currentTranslationDir unchanged
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      } else {
        m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
    } else {
      m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.calculate(0.0);
    }
    m_prevTime = currentTime;
    m_currentRotation = m_rotLimiter.calculate(rotSpeed);

    double limitedXSpeed = inputTranslationMag * Math.cos(m_currentTranslationDir);
    double limitedYSpeed = inputTranslationMag * Math.sin(m_currentTranslationDir);
    double limitedRotSpeed = m_currentRotation;

    return new ChassisSpeeds(limitedXSpeed, limitedYSpeed, limitedRotSpeed);
  }

  /**
   * Set the speeds of the swerve modules
   * 
   * @param xSpeed        Forward speed of the robot in m/s.
   * @param ySpeed        Sideways speed of the robot in m/s.
   * @param rotSpeed      Angular rate of the robot in rad/s.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  private void setSwerveSpeeds(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
                getHeading())
            : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  /**
   * Drive the robot using a robot relative ChasisSpeeds
   * 
   * @param speeds The robot relative ChasisSpeeds
   */
  private void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  public Translation2d getTarget() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? TargetConstants.kBlueSpeakerTarget
        : TargetConstants.kRedSpeakerTarget;
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  /**
   * Get the robot's current speed relative to the robot
   * 
   * @return The robot relative ChasisSpeeds
   */
  private ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  // Get heading for odometry
  private Rotation2d getHeadingOdometry() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(
        m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0) - DriveConstants.kGyroAdjustment, 360));
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  private void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetdrivingEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Returns the translation of the robot.
   *
   * @return the robot's translation
   */
  public Translation2d getTranslation() {
    return getPose().getTranslation();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}

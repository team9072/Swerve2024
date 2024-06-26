// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TargetConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AttachmentCoordinator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.attachment.FeederSubsystem;
import frc.robot.subsystems.attachment.PivotSubsystem;
import frc.robot.subsystems.attachment.PivotSubsystem.PivotPosition;
import frc.robot.subsystems.attachment.ShooterSubsystem;
import frc.robot.subsystems.attachment.UTBIntakerSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Autogenerated chooser with all the auto routes
  public final SendableChooser<Command> autoChooser;

  // Other (tests)
  private boolean m_autoAim = false;
  private double m_adjust = 0;
  private boolean m_vision = true;

  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem(() -> {
    if (m_autoAim) {
      return Optional.of(getTargetVector().getAngle());
    } else {
      return Optional.empty();
    }
  });

  public final AttachmentCoordinator m_attatchment = new AttachmentCoordinator(
      new UTBIntakerSubsystem(),
      new FeederSubsystem(),
      new ShooterSubsystem(),
      new PivotSubsystem());

  // The driver's controllers
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_attachmentController = new CommandXboxController(OIConstants.kAttatchmentsControllerPort);

  // Fields for visualization and testing
  private final Field2d m_field = new Field2d();
  private final Field2d m_estimationField = new Field2d();  
  private final Field2d m_calibrationField = new Field2d();


  public RobotContainer() {
    registerPathplannerCommands();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("Pose Estimation", m_estimationField);    
    SmartDashboard.putData("Calibration Testing", m_calibrationField);
    SmartDashboard.putNumber("Cali X", 0);
    SmartDashboard.putNumber("Cali Y", 0);


    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        Commands.run(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(invertIfRed(m_driverController.getLeftY()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(invertIfRed(m_driverController.getLeftX()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, false),
            m_robotDrive));

             new PathfindHolonomic(
            new Pose2d(15.0, 4.0, Rotation2d.fromDegrees(180)),
            new PathConstraints(4, 3, 4, 4),
            () -> new Pose2d(1.5, 4, new Rotation2d()),
            ChassisSpeeds::new,
            (speeds) -> {},
            new HolonomicPathFollowerConfig(4.5, 0.4, new ReplanningConfig()))
        .andThen(Commands.print("[PathPlanner] PathfindingCommand finished warmup"))
        .ignoringDisable(true).schedule();
  }

  /**
   * Register named commands used in pathplanner autos
   */
  private void registerPathplannerCommands() {
    NamedCommands.registerCommand("startContinuousFire", m_attatchment.getStartContinuousFireCommand());
    NamedCommands.registerCommand("stopContinuousFire", m_attatchment.getStopContinuousFireCommand());

    // TODO: maybe find a cleaner way to implement this. also includes running the
    // pivot function constantly during auto
    NamedCommands.registerCommand("startAutoAim", Commands.runOnce(() -> {
      m_autoAim = true;
    }).asProxy());

    NamedCommands.registerCommand("stopAutoAim", Commands.runOnce(() -> {
      m_autoAim = false;
    }).asProxy());

    
    NamedCommands.registerCommand("enableVision", Commands.runOnce(() -> {
      m_vision = true;
    }).asProxy());

    NamedCommands.registerCommand("disableVision", Commands.runOnce(() -> {
      m_vision = false;
    }).asProxy());

    NamedCommands.registerCommand("adjustYes", Commands.runOnce(() -> {
      m_adjust = 1;
    }).asProxy());

    NamedCommands.registerCommand("adjustNo", Commands.runOnce(() -> {
      m_adjust = 0;
    }).asProxy());

    NamedCommands.registerCommand("zeroGyro", Commands.runOnce(() -> {
      m_robotDrive.resetGyro();
    }).asProxy());

    NamedCommands.registerCommand("setX", Commands.runOnce(() -> {
      m_robotDrive.setX();
    }).asProxy());

    NamedCommands.registerCommand("pivotW1", Commands.runOnce(() -> {
      m_attatchment.setCustomPosition(7.7);
    }).asProxy());

    NamedCommands.registerCommand("pivotW2", Commands.runOnce(() -> {
      m_attatchment.setCustomPosition(9.27);
    }).asProxy());

    NamedCommands.registerCommand("pivotW3", Commands.runOnce(() -> {
      m_attatchment.setCustomPosition(6.95);
    }).asProxy());

    NamedCommands.registerCommand("beamBreak", m_attatchment.getBeamBreakCommand());

    NamedCommands.registerCommand("pivotSubwoofer",
        m_attatchment.getSetPivotPositionCommand(PivotPosition.kSubwooferPosition));
    NamedCommands.registerCommand("pivotIntake",
        m_attatchment.getSetPivotPositionCommand(PivotPosition.kIntakePosition));

    NamedCommands.registerCommand("startFeeders", m_attatchment.getStartShootCommand());
    NamedCommands.registerCommand("startShooter", m_attatchment.getSpinShooterAutoCommand());
    NamedCommands.registerCommand("stopShooter", m_attatchment.getStopShootCommand());
    NamedCommands.registerCommand("startIntakers", m_attatchment.getIntakeAutoCommand().asProxy());

    NamedCommands.registerCommand("startIntaker", m_attatchment.getIntakeAutoCommand());
    NamedCommands.registerCommand("stopIntaker", m_attatchment.getStopIntakeAutoCommand());

    // will not stop
    NamedCommands.registerCommand("intake", m_attatchment.getIntakeCommand());

    NamedCommands.registerCommand("stopFeeders", m_attatchment.getStopShootCommand());
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    // Rumble on intake note
    m_attatchment.bindControllerRumble(m_driverController);
    m_attatchment.bindControllerRumble(m_attachmentController);

    // Base controls

    // Set X wheels
    m_driverController.rightBumper().whileTrue(Commands.run(
        () -> m_robotDrive.setX(),
        m_robotDrive));

    // Auto aiming
    m_attachmentController.rightTrigger().whileTrue(Commands.run(() -> {
      autoAimDrive(getAimingVector(getTarget()).getAngle());
      autoAimPivot(0);
    }));
    
    // adjusted vision
    /* m_attachmentController.leftTrigger().whileTrue(Commands.run(() -> {
      autoAimDrive(getAimingVector(getTarget()).getAngle());
      autoAimPivot(5);
    })); */

    // Reset field oriented
    m_driverController.x().onTrue(Commands.runOnce(() -> {
      m_robotDrive.resetGyro();
    }));

    // D-pad turning
    m_driverController.povUp().whileTrue(Commands.run(() -> autoAimDrive(Rotation2d.fromDegrees(getFromAlliance(0, 180))), m_robotDrive));

    m_driverController.povRight().whileTrue(Commands.run(() -> autoAimDrive(Rotation2d.fromDegrees(getFromAlliance(-90, 90))), m_robotDrive));

    m_driverController.povDown().whileTrue(Commands.run(() -> autoAimDrive(Rotation2d.fromDegrees(getFromAlliance(180, 0))), m_robotDrive));

    m_driverController.povLeft().whileTrue(Commands.run(() -> autoAimDrive(Rotation2d.fromDegrees(getFromAlliance(90, -90))), m_robotDrive));

    m_driverController.leftTrigger().whileTrue(Commands.run(() -> autoAimDrive(Rotation2d.fromDegrees(getFromAlliance(-90, -90))), m_robotDrive));

    // Attatchment controls

    // Intake
    m_attachmentController.b().whileTrue(m_attatchment.getIntakeCommand());

    // Unjam
    m_attachmentController.y().whileTrue(m_attatchment.getUnjamIntakersCommand());

    // Spin up shooter
    m_attachmentController.leftBumper().whileTrue(m_attatchment.getSpinShooterCommand());

    // Shoot
    m_driverController.rightTrigger()
        .whileTrue(m_attatchment.getStartShootCommand())
        .whileFalse(m_attatchment.getStopShootCommand());
       
// Pose2d targetPose = new Pose2d(2.21, 7.82, Rotation2d.fromDegrees(-88.767));

   /*  m_driverController.y().onTrue(
      AutoBuilder.pathfindThenFollowPath(
        new PathPlannerPath(
          PathPlannerPath.bezierFromPoses(
      m_robotDrive.getPose(),
        new Pose2d(2.411, 8.133, Rotation2d.fromDegrees(-90.0))),
          new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
          new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  ),
        new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720))
    ));*/

    m_driverController.y().onTrue(
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("Amp"))
    );

    String x = """
        
        """;

    
    // Amp
    m_attachmentController.a().whileTrue(m_attatchment.getAmpCommand())
    .onFalse(m_attatchment.getCancelAmpCommand());

    // Arm/pivot positioning

    m_attachmentController.povUp().onTrue(m_attatchment.getSetPivotPositionCommand(PivotPosition.kSubwooferPosition));

    m_attachmentController.povDown().onTrue(m_attatchment.getSetPivotPositionCommand(PivotPosition.kIntakePosition));

    m_attachmentController.povLeft().onTrue(m_attatchment.getSetCustomPivotPositionCommand(20));

    m_attachmentController.povRight().onTrue(m_attatchment.getSetCustomPivotPositionCommand(17));
  }

  // TODO: try this adjustment if long shots are bad (distance^(1.161))-4
  public void autoAimPivot(float adjustment) {
    double angle = 15;
    double targetDistance = getAimingVector(getTarget()).getNorm();
    if (isBlueAlliance()) {
      angle = (35.8266 * Math.pow(.7037, targetDistance));
    } else {
      angle = (35.8266 * Math.pow(.7037, targetDistance));
      angle += m_adjust;
    }

    angle -= 4; // adjustment
    angle += adjustment; // other adjustment
    
    /*if (targetDistance >= 4) {
      angle += Math.pow(targetDistance, 1.05) - 4;
    }*/

    if (angle < 30 && angle > 2) {
      m_attatchment.setCustomPosition(angle);
    }
  }

  public void autoAimDrive(Rotation2d angle) {
    // Auto aiming left-right (offset is 5 degrees for alignment)
    m_robotDrive.driveWithHeading(
        -MathUtil.applyDeadband(invertIfRed(m_driverController.getLeftY()), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(invertIfRed(m_driverController.getLeftX()), OIConstants.kDriveDeadband),
        angle,
        true, false);
  }

  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  public <T> T getFromAlliance(T blueVal, T redVal) {
    return isBlueAlliance() ? blueVal : redVal;
  }

  public double invertIfRed(double num) {
     return num * getFromAlliance(1, -1);
  }

  public Translation2d getTarget() {
    return TargetConstants.AimingTarget.kSpeaker.getTarget(isBlueAlliance());
  }

  public Translation2d getAimingVector(Translation2d target) {
    return m_robotDrive.getPose().getTranslation().minus(target);
  }

  public Translation2d getTargetVector() {
    if (m_autoAim) {
      autoAimPivot(0);
    }
    return getAimingVector(getTarget());
  }

  public void periodic() {
    SmartDashboard.putNumber("Auto Aim Distance", getAimingVector(getTarget()).getNorm());
    SmartDashboard.putNumber("Bumper to Sub (In)", Units.metersToInches(getAimingVector(getTarget()).getNorm()) - 35.25 - (33/2));

    SmartDashboard.putBoolean("Beam Break", m_attatchment.getBeamBreakState());
    SmartDashboard.putBoolean("Vision", VisionConstants.rearCam.isConnected());

    m_field.setRobotPose(m_robotDrive.getPose());

    var pose = VisionConstants.rearCamPoseEstimator.update();

    SmartDashboard.putNumber("x", m_robotDrive.getPose().getX());
        SmartDashboard.putNumber("y", m_robotDrive.getPose().getY());


    if (pose.isPresent() && m_vision) {
      Pose2d estimatedPose = pose.get().estimatedPose.toPose2d();
      double timestamp = pose.get().timestampSeconds;

      m_estimationField.setRobotPose(estimatedPose);
      m_robotDrive.updateOdometryWithVision(estimatedPose, timestamp);

      SmartDashboard.putBoolean("Tag", true);
      //System.out.println("[vision] I see a tag");
    } else {
      SmartDashboard.putBoolean("Tag", false);
      m_estimationField.setRobotPose(new Pose2d());
      //System.out.println("[vision] I do NOT see a tag");
    }

    // CALIBRATION TESTING START
    /*
    double caliX = SmartDashboard.getNumber("Cali X", 0);   
    double caliY = SmartDashboard.getNumber("Cali Y", 0);

    VisionConstants.calibrationPoseEstimator.setRobotToCameraTransform(
      new Transform3d(
        new Translation3d(Units.inchesToMeters(caliX), Units.inchesToMeters(caliY), -Units.inchesToMeters(-11)),
        new Rotation3d(0, Units.degreesToRadians(-35.5), Math.PI))
    );

    var caliPose = VisionConstants.calibrationPoseEstimator.update();

    if (caliPose.isPresent()) {
      m_calibrationField.setRobotPose(caliPose.get().estimatedPose.toPose2d());
    } else {
      m_calibrationField.setRobotPose(new Pose2d());
    }
    // CALIBRATION TESTING END */
  }

  public void prepareTeleop() {
      m_attatchment.stopContinuousFire();
      m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0);    
      m_attachmentController.getHID().setRumble(RumbleType.kBothRumble, 0);
      m_autoAim = false;
      m_vision = true;

      SmartDashboard.putNumber("Cali X", 0);
      SmartDashboard.putNumber("Cali Y", 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Stop continuous fire and auto aim after auto ends
    return autoChooser.getSelected().finallyDo(() -> {
      // shooter stops too early for the third note sometimes
      //m_attatchment.stopContinuousFire();
     // m_autoAim = false;
    });
  }
}

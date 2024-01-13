// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_armController = new CommandXboxController(OIConstants.kArmControllerPort);

  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        Commands.run(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    // m_arm.setDefaultCommand(
    // Commands.run(() -> m_arm.setRotationSpeed(m_armController.getLeftY()), m_arm)
    // );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.rightBumper().whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive));
    m_armController.b().onTrue(m_arm.getToggleIntakerCommand());
    m_armController.povUp().onTrue(m_arm.getShootCommand(ArmSubsystem.IntakerMode.SHOOT_TOP));
    m_armController.povLeft().onTrue(m_arm.getShootCommand(ArmSubsystem.IntakerMode.SHOOT_MIDDLE));
    m_armController.povDown().onTrue(m_arm.getShootCommand(ArmSubsystem.IntakerMode.SHOOT_BOTTOM));

    m_armController.y().onTrue(m_arm.setArmPosition(-1));
    m_armController.x().onTrue(m_arm.setArmPosition(-44));

    // Drive speeds
    m_driverController.rightTrigger().whileTrue(new RunCommand(
        () -> {
          Constants.DriveConstants.kMaxSpeedMetersPerSecond = 4.8 * 0.5;
        }))
        .whileFalse(new RunCommand(() -> {
          Constants.DriveConstants.kMaxSpeedMetersPerSecond = 4.8 * 1.3;
        }));

    // Reset field oriented
    m_driverController.leftTrigger().onTrue(new RunCommand(() -> {
      // m_robotDrive.m_gyro.reset();
    }));

    // D-Pad turning (def doesnt work)
    m_driverController.povUp().onTrue(new RunCommand(() -> {
      ExecutorService executor = Executors.newFixedThreadPool(9); // Just 1?
      executor.submit(() -> {
        // Normalize the degree
        double angle = m_robotDrive.m_gyro.getAngle() % 360;

        while (true) {
          if (angle < 180) {
            m_robotDrive.drive(0, 0, -.25, false, false);
          } else {
            m_robotDrive.drive(0, 0, .25, false, false);
          }

          if (Math.abs(angle) > 175 && Math.abs(angle) < 185) {
            m_robotDrive.drive(0, 0, 0, false, false);
            break;
          };

          angle = m_robotDrive.m_gyro.getAngle() % 360;
        }

        executor.shutdown();
      });
    }));
  }
}

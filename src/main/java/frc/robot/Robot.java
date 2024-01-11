// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  // Temp: auto selection here
  SendableChooser<String> chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Temp: auto selection here
    chooser.addOption("Balance", "paths/Balance.wpilib.json");
    chooser.addOption("Three Cubes", "paths/ThreeCubes.wpilib.json");
    chooser.addOption("Two Cubes", "paths/TwoCubes.wpilib.json");
    chooser.addOption("Community", "paths/Community.wpilib.json");
    SmartDashboard.putData("Auto Path", chooser);

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Temp: auto selection here
    m_robotContainer.autoPath = chooser.getSelected();

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Auto: Shoots preloaded cube, goes down, goes to other cube, comes back and shoots (and parks)
    Command shootTopCube = m_robotContainer.m_arm.getShootCommand(ArmSubsystem.IntakerMode.SHOOT_TOP);
    Command shootMiddleCube = m_robotContainer.m_arm.getShootCommand(ArmSubsystem.IntakerMode.SHOOT_MIDDLE);
    Command resetArm = m_robotContainer.m_arm.setArmPosition(-44);

    if (!m_robotContainer.autoPath.contains("Cubes")) {
      // Balance path just shoots its preloaded cube and balances
      m_autonomousCommand = shootTopCube
      .andThen(m_robotContainer.getAutonomousCommand());
    } else if (m_robotContainer.autoPath.equals("paths/TwoCubes.wpilib.json")) {
      m_autonomousCommand = shootTopCube
      .andThen(
        resetArm,
        new WaitCommand(0.5),
        m_robotContainer.m_arm.getToggleIntakerCommand(),
        Commands.parallel(
          m_robotContainer.getAutonomousCommand(),
          Commands.sequence(
            new WaitCommand(3),
            m_robotContainer.m_arm.getToggleIntakerCommand(),
            m_robotContainer.m_arm.setArmPosition(-1),
            new WaitCommand(2),
            shootMiddleCube
          )
        )
      );
    } else if (m_robotContainer.autoPath.equals("paths/ThreeCubes.wpilib.json")) {
      /*m_autonomousCommand = shootTopCube
      .andThen(
        resetArm,
        new WaitCommand(0.5),
        m_robotContainer.m_arm.getToggleIntakerCommand(),
        Commands.parallel(
          m_robotContainer.getAutonomousCommand(),
          Commands.sequence(
            new WaitCommand(3),
            m_robotContainer.m_arm.getToggleIntakerCommand(),
            m_robotContainer.m_arm.setArmPosition(-1),
            new WaitCommand(2),
            shootMiddleCube,
            new WaitCommand(.75),
            m_robotContainer.m_arm.getToggleIntakerCommand(),
            m_robotContainer.m_arm.setArmPosition(-1),
            new WaitCommand(2)
          )
        )
      );*/
    }

    // Schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_robotContainer.m_arm.setArmPosition(-1).andThen(m_autonomousCommand).schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Data
    //SmartDashboard.putNumber("Encoder Position", m_robotContainer.m_arm.getEncoderValue());
    //SmartDashboard.putNumber("Encoder Velocity", m_robotContainer.m_armController.getAbsoluteEncoder(Type.kDutyCycle).getVelocity());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

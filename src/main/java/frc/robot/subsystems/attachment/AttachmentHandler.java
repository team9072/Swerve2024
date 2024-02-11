package frc.robot.subsystems.attachment;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.attachment.ShooterSubsystem.ShooterState;

public class AttachmentHandler extends SubsystemBase {
    private final UTBIntakerSubsystem m_UTBIntaker;
    private final FeederSubsystem m_feeder;
    private final ShooterSubsystem m_shooter;

    // state variables
    private boolean hasNote;

    public AttachmentHandler(UTBIntakerSubsystem utbIntaker, FeederSubsystem feeder, ShooterSubsystem shooter) {
        m_UTBIntaker = utbIntaker;
        m_feeder = feeder;
        m_shooter = shooter;
    }

    @Override
    public void periodic() {
        if (m_feeder.getBeamBreakState()) {
            if (!hasNote) {
                // on get note stop intake
                m_UTBIntaker.stop();
                m_feeder.stop();
            }

            hasNote = true;
        } else {
            hasNote = false;
        }
    }

    /**
     * Set the new state only if the shoter is not currently shooting
     * @return true if new state was set, false otherwise
     */
    private boolean trySetShooterState(ShooterState state) {
        boolean canSetState = m_shooter.getState() != ShooterState.kShooting;
        if (canSetState) {
            m_shooter.setState(state);
        }

        return canSetState;
    }

    /**
     * Start the intakers
     * Will not start the intakers if the feeder is shooting
     */
    public void startIntakers() {
        getStartIntakersCommand().schedule();
    }

    /**
     * Start the intakers
     * Will not start the intakers if the feeder is shooting
     * @return A command to start the intakers
     */
    public Command getStartIntakersCommand() {
        return Commands.parallel(
            m_UTBIntaker.getIntakeCommand(),
            //TODO: OTB Intaker
            m_feeder.getIntakeCommand()
        );
    }

    /**
     * Stop the intakers
     * Will stop the intakers even if the feeder is shooting
     */
    public void stopIntakers() {
        getStartIntakersCommand().schedule();
    }

    /**
     * Stop the intakers
     * Will stop the intakers even if the feeder is shooting
     * @return A command to start the intakers
     */
    public Command getStopIntakersCommand() {
        return Commands.parallel(
            m_UTBIntaker.getStopCommand(),
            //TODO: OTB Intaker
            m_feeder.getStopCommand().asProxy()
        );
    }

    /**
     * Reverse the intakers
     * Will not reverse the intakers if the feeder is shooting
     */
    public void reverseIntakers() {
        getReverseIntakersCommand().schedule();
    }

    /**
     * Reverse the intakers
     * Will not reverse the intakers if the feeder is shooting
     * @return A command to reverse the intakers
     */
    public Command getReverseIntakersCommand() {
        return Commands.parallel(
            m_UTBIntaker.getReverseCommand(),
            //TODO: OTB Intaker
            m_feeder.getReverseCommand()
        );
    }

    /**
     * Start spinning up the shooter
     */
    public void spinShooter() {
        trySetShooterState(ShooterState.kSpinning);
    }

    /**
     * Start spinning up the shooter
     * @return A command to start spinning the shooter
     */
    public Command getSpinShooterCommand() {
        return Commands.runOnce(() -> spinShooter(), m_shooter);
    }

    /**
     * Stop the shooter
     */
    public void stopShooter() {
        trySetShooterState(ShooterState.kStopped);
    }

    /**
     * Stop the shooter
     * @return A command to stop the shooter
     */
    public Command getStopShooterCommand() {
        return Commands.runOnce(() -> stopShooter(), m_shooter);
    }

    /**
     * Shoot the note in the feeder out of the shooter
     */
    public void shoot() {
        getShootCommand().schedule();
    }

    /**
     * Shoot the note in the feeder out of the shooter
     * @return A commnd t shoot the loaded note
     */
    public Command getShootCommand() {
        return Commands.sequence(
            new WaitUntilCommand(m_shooter::isShooterReady),
            m_shooter.getSetStateCommand(ShooterState.kShooting),
            m_feeder.getstartShootingCommand(),
            new WaitCommand(ShooterConstants.kShootTime),
            m_shooter.getStopCommand(),
            m_feeder.getStopCommand()
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
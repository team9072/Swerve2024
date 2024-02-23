package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TargetConstants.AimingTarget;
import frc.robot.subsystems.attachment.FeederSubsystem;
import frc.robot.subsystems.attachment.ShooterSubsystem;
import frc.robot.subsystems.attachment.UTBIntakerSubsystem;
import frc.robot.subsystems.attachment.PivotSubsystem.PivotPosition;
import frc.robot.subsystems.attachment.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.attachment.FeederSubsystem.FeederState;
import frc.robot.subsystems.attachment.Intaker.IntakerState;
import frc.robot.subsystems.attachment.PivotSubsystem;

public class AttachmentCoordinator {
    public enum AttatchmentState {
        kIntake,
        kAiming,
        kShooting
    }

    private final UTBIntakerSubsystem m_UTBIntaker;
    private final FeederSubsystem m_feeder;
    private final ShooterSubsystem m_shooter;
    private final PivotSubsystem m_pivot;
    private final Trigger m_beamBreak;

    // state variables
    private AttatchmentState m_state = AttatchmentState.kIntake;
    private AimingTarget m_target = AimingTarget.kSpeaker;

    public AttachmentCoordinator(UTBIntakerSubsystem utbIntaker, FeederSubsystem feeder, ShooterSubsystem shooter, PivotSubsystem pivot) {
        m_UTBIntaker = utbIntaker;
        m_feeder = feeder;
        m_shooter = shooter;
        m_pivot = pivot;

        m_beamBreak = new Trigger(m_feeder::getBeamBreakState).debounce(0.1, DebounceType.kBoth);

        m_beamBreak.onTrue(new InstantCommand(() -> handleGetNote(), m_UTBIntaker, m_feeder));
        m_beamBreak.onFalse(new InstantCommand(() -> handleLoseNote(), m_UTBIntaker, m_feeder));
    }

    private void setState(AttatchmentState state) {
        if (m_state == state) { return; }

        m_state = state;

        switch (m_state) {
            case kIntake -> {
                m_pivot.setPosition(PivotPosition.kIntakePosition);
            }
            case kAiming -> {
                m_pivot.setPosition(switch (m_target) {
                    case kSpeaker -> PivotPosition.kSpeakerPosition;
                    case kAmp -> PivotPosition.kAmpPosition;
                });

                Command utbIntakerCommand = m_UTBIntaker.getCurrentCommand();
                Command feederCommand = m_feeder.getCurrentCommand();
                if (utbIntakerCommand != null) { utbIntakerCommand.cancel(); }
                if (feederCommand != null) { feederCommand.cancel(); }

                m_UTBIntaker.setState(IntakerState.kStopped);
                m_feeder.setState(FeederState.kStopped);
            }
            case kShooting -> {
                m_pivot.setPosition(switch (m_target) {
                    case kSpeaker -> PivotPosition.kSpeakerPosition;
                    case kAmp -> PivotPosition.kAmpPosition;
                });
            }
        }
    }

    private void handleGetNote() {
        switch (m_state) {
            // Startaiming on get note
            case kIntake -> setState(AttatchmentState.kAiming);
            // Keep aiming I guess? Not sure how we got here
            case kAiming -> {}
            // Keep shooting, something odd is happening tho ._.
            case kShooting -> {}
        }
    }

    private void handleLoseNote() {
        switch (m_state) {
            // Stay in intake state
            case kIntake -> {}
            // Go back to intake on unjam or lose note
            case kAiming -> setState(AttatchmentState.kIntake);
            // Go back to intake after a delay
            case kShooting -> {
                
                setState(AttatchmentState.kIntake);
            }
        }
    }

    /**
     * Intake if in intake mode
     */
    private void startIntaking() {
        if (m_state == AttatchmentState.kIntake) {
            m_UTBIntaker.setState(IntakerState.kIntaking);
            m_feeder.setState(FeederState.kIntaking);
        }
    }

    /**
     * Reverse intakers if not shooting
     * Acts as an unjam feature
     */
    private void unjamIntakers() {
        if (m_state != AttatchmentState.kShooting) {
            m_UTBIntaker.setState(IntakerState.kReversed);
            m_feeder.setState(FeederState.kReversed);
        }
    }

    /**
     * Stop intaking, but not shooting
     */
    private void stopIntaking() {
        if (m_state != AttatchmentState.kShooting) {
            m_feeder.setState(FeederState.kStopped);
        }

        m_UTBIntaker.setState(IntakerState.kStopped);
    }

    /**
     * Set the shooter state only if it woukd not interupt shooting
     */
    private void softSetShooterState(ShooterState state) {
        if (m_state != AttatchmentState.kShooting) {
          m_shooter.setState(state);
        }
    }

    /**
     * Set the target for auto aiming This should be the same target that the drivebase is targeting
     * @param target the target to aim for
     */
    public void setTarget(AimingTarget target) {
        m_target = target;
        if (m_state != AttatchmentState.kIntake) {
            m_pivot.setPosition(switch (m_target) {
                case kSpeaker -> PivotPosition.kSpeakerPosition;
                case kAmp -> PivotPosition.kAmpPosition;
            });
        }
    }

    /**
     * Intake untul the returned command is canceled
     * @return a command to intake
     */
    public Command getIntakeCommand() {
        return Commands.startEnd(() -> startIntaking(), () -> stopIntaking(), m_UTBIntaker, m_feeder);
    }

    /**
     * Reverse the intakers to unjam, until the command is cancelled
     * @return a command to unjam
     */
    public Command getUnjamIntakersCommand() {
        return Commands.startEnd(() -> unjamIntakers(), () -> stopIntaking(), m_UTBIntaker, m_feeder);
    }

    /**
     * Spin the shooter until the command is cancelled
     * @return a command to spin the shooter
     */
    public Command getSpinShooterCommand() {
        return Commands.startEnd(() -> softSetShooterState(ShooterState.kSpinning), () -> softSetShooterState(ShooterState.kStopped), m_shooter);
    }

    /**
     * Get ready and shoot once the shooter is at speed and the pivot is at the right angle
     * @return a command to shoot
     */
    public Command getShootCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> m_shooter.setState(ShooterState.kShooting), m_shooter, m_pivot),
            Commands.parallel(
                Commands.waitUntil(m_shooter::isShooterReady),
                Commands.waitUntil(m_pivot::isPivotReady)
            ),
            Commands.runOnce(() -> m_feeder.setState(FeederState.kShooting), m_feeder),
            Commands.race(
                Commands.waitSeconds(ShooterConstants.kMaxShootTime),
                Commands.sequence(
                    Commands.waitUntil(m_beamBreak.negate()),
                    Commands.waitSeconds(ShooterConstants.kBeamBreakEndLag)
                )
            )
        ).finallyDo(() -> {
            m_shooter.setState(ShooterState.kStopped);
            m_feeder.setState(FeederState.kStopped);
        });
    }

    /**
     * Set the position of the povot for speaker shots at different distances
     * @param rotations the pivot angle
     */
    public void setSpeakerRotations(double rotations) {
        m_pivot.setPrecisePosition(rotations);
    }

    /**W
     * Set the position of the povot for speaker shots at different distances
     * @param rotations the pivot angle
     * @return a command to set the pivot angle
     */
    public Command getSetSpeakerRotationsCommand(double rotations) {
        return Commands.runOnce(() -> m_pivot.setPrecisePosition(rotations), m_pivot);
    }
}

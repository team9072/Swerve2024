package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeederConstants;
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
        kAiming,
        kShooting,
        kContinuousFire;
    }

    private final UTBIntakerSubsystem m_UTBIntaker;
    private final FeederSubsystem m_feeder;
    private final ShooterSubsystem m_shooter;
    private final PivotSubsystem m_pivot;
    private final Trigger m_beamBreak;

    // state variables
    private AttatchmentState m_state = AttatchmentState.kAiming;
    private AimingTarget m_target = AimingTarget.kSpeaker;

    public AttachmentCoordinator(UTBIntakerSubsystem utbIntaker, FeederSubsystem feeder, ShooterSubsystem shooter,
            PivotSubsystem pivot) {
        m_UTBIntaker = utbIntaker;
        m_feeder = feeder;
        m_shooter = shooter;
        m_pivot = pivot;

        m_beamBreak = new Trigger(m_feeder::getBeamBreakState);

        m_beamBreak.onTrue(getHandleGetNoteCommand());
    }

    private void setState(AttatchmentState state) {
        if (m_state == state) {
            return;
        }

        m_state = state; 

        switch (m_state) {
            // do nothing
            case kAiming, kShooting -> {
            }
            case kContinuousFire -> {
                m_pivot.setPosition(PivotPosition.kIntakePosition);
            }
        }
    }

    private void goToIntake() {
        setState(AttatchmentState.kAiming);
        m_pivot.setPosition(PivotPosition.kIntakePosition);
        m_shooter.setState(ShooterState.kStopped);
        m_feeder.setState(FeederState.kStopped);
        m_UTBIntaker.setState(IntakerState.kStopped);
    }

    public AttatchmentState getState() {
        return this.m_state;
    }

    public boolean getBeamBreakState() {
        return m_beamBreak.getAsBoolean();
    }

    private Command getHandleGetNoteCommand() {
        return Commands.either(
                Commands.sequence(
                        Commands.runOnce(() -> {
                            setState(AttatchmentState.kAiming);
                            m_feeder.setState(FeederState.kAlignReverse);
                        }, m_UTBIntaker, m_feeder),
                        Commands.race(Commands.waitUntil(m_beamBreak.negate()),
                                Commands.waitSeconds(FeederConstants.kNotePullbackMaxTime)),
                        Commands.runOnce(() -> m_feeder.setState(FeederState.kStopped), m_UTBIntaker, m_feeder)),
                Commands.none(),
                () -> m_pivot.getPosition() == PivotPosition.kIntakePosition && m_state == AttatchmentState.kAiming);
    }

    /**
     * Intake if in intake mode
     */
    private void startIntaking() {
        if (m_pivot.getPosition() == PivotPosition.kIntakePosition) {
            m_UTBIntaker.setState(IntakerState.kIntaking);
            m_feeder.setState(FeederState.kIntaking);
        }
    }

    /**
     * Reverse intakers if not shooting
     * Acts as an unjam feature
     */
    private void unjamIntakers() {
        if (m_pivot.getPosition() == PivotPosition.kIntakePosition) {
            m_UTBIntaker.setState(IntakerState.kReversed);
            m_feeder.setState(FeederState.kReversed);
        }
    }

    /**
     * Stop intaking, but not shooting
     */
    private void stopIntaking() {
        m_UTBIntaker.setState(IntakerState.kStopped);
        softSetFeederState(FeederState.kStopped);
    }

    /**
     * Set the feeder state only if it woukd not interupt shooting
     */
    private void softSetFeederState(FeederState state) {
        if (m_state != AttatchmentState.kShooting) {
            m_feeder.setState(state);
        }
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
     * Set the target for auto aiming This should be the same target that the
     * drivebase is targeting
     * 
     * @param target the target to aim for
     */
    public void setTarget(AimingTarget target) {
        m_target = target;
        if (m_pivot.getPosition() != PivotPosition.kIntakePosition) {
            m_pivot.setPosition(switch (m_target) {
                case kSpeaker -> PivotPosition.kCustomSpeakerPosition;
                case kAmp -> PivotPosition.kAmpPosition;
            });
        }
    }

    /**
     * Intake untul the returned command is canceled
     * 
     * @return a command to intake
     */
    public Command getIntakeCommand() {
        return Commands.startEnd(() -> startIntaking(), () -> stopIntaking(), m_UTBIntaker, m_feeder);
    }

    /**
     * Reverse the intakers to unjam, until the command is cancelled
     * 
     * @return a command to unjam
     */
    public Command getUnjamIntakersCommand() {
        return Commands.startEnd(() -> unjamIntakers(), () -> stopIntaking(), m_UTBIntaker, m_feeder);
    }

    /**
     * Spin the shooter until the command is cancelled
     * 
     * @return a command to spin the shooter
     */
    public Command getSpinShooterCommand() {
        return Commands.startEnd(() -> softSetShooterState(ShooterState.kSpinning),
                () -> softSetShooterState(ShooterState.kStopped), m_shooter);
    }

    /**
     * Get ready and shoot once the shooter is at speed and the pivot is at the
     * right angle
     * 
     * @return a command to shoot
     */
    public Command getShootCommand() {
        return Commands.runOnce(() -> m_feeder.setState(FeederState.kShooting), m_feeder);
    }

    /**
     * Stop shooting.
     * @return a command to stop shooting
     */
    public Command getStopShootCommand() {
        return Commands.runOnce(() -> m_feeder.setState(FeederState.kStopped), m_feeder);
    }

    /**
     * Get a command to feed notes through the whole attatchment without stopping
     * This mode ends when the command ends
     * 
     * @return the continuous fire command
     */
    public Command getContinuousFireCommand() {
        return Commands.startEnd(() -> {
            setState(AttatchmentState.kContinuousFire);
            m_UTBIntaker.setState(IntakerState.kIntaking);
            m_feeder.setState(FeederState.kShooting);
            m_shooter.setState(ShooterState.kShooting);
        }, () -> {
            goToIntake();
        }, m_UTBIntaker, m_feeder, m_shooter);
    }

    /**
     * Set the position of the pivot
     * @param position the pivot position
     * @return a command to set the pivot position
     */
    public Command getSetPivotPositionCommand(PivotPosition position) {
        return Commands.runOnce(() -> {
            m_pivot.setPosition(position);
        }, m_pivot);
    }

    /**
     * Set the roattion of the pivot to a custom value
     * @param rotations the pivot angle in rotations of the neo motor
     * @return a command to set the pivot angle
     */
    public Command getSetCustomPivotPositionCommand(double rotations) {
        return Commands.runOnce(() -> {
            m_pivot.setPosition(PivotPosition.kCustomSpeakerPosition);
            m_pivot.setPrecisePosition(rotations);
        }, m_pivot);
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.PivotConstants;
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

        m_beamBreak = new Trigger(m_feeder::getBeamBreakState).negate();
    }

    // Starts the beam break trigger for teleop
    public void bindControllerRumble(CommandXboxController driveController) {
        // Rumble on intake (this mess makes it so it doesn't rumble while shooting and
        // only for intaking)
        m_beamBreak.onTrue(
                Commands.parallel(
                        Commands.either(
                                Commands.sequence(
                                        Commands.runOnce(
                                                () -> driveController.getHID().setRumble(RumbleType.kBothRumble, 1)),
                                        Commands.waitSeconds(1),
                                        Commands.runOnce(
                                                () -> driveController.getHID().setRumble(RumbleType.kBothRumble, 0))),
                                Commands.none(), () -> m_state == AttatchmentState.kAiming)));
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
            // do nothing again
            case kContinuousFire -> {
            }
        }
    }

    public AttatchmentState getState() {
        return this.m_state;
    }

    public boolean getBeamBreakState() {
        return m_beamBreak.getAsBoolean();
    }

    /**
     * Intake if in intake mode
     */
    private void startIntaking() {
        m_pivot.setPosition(PivotPosition.kIntakePosition);

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
     * Set the shooter state
     */
    private void softSetShooterState(ShooterState state) {
        m_shooter.setState(state);
    }

    private Command getAlignNoteCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> m_feeder.setState(FeederState.kAlignReverse), m_UTBIntaker, m_feeder),
                Commands.race(
                        Commands.waitUntil(m_beamBreak.negate()),
                        Commands.waitSeconds(FeederConstants.kNotePullbackMaxTime)),
                Commands.runOnce(this::stopIntaking, m_UTBIntaker, m_feeder));
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
     * If you want to intake based on a joystick button or other trigger, use
     * {@link #bindIntakeTrigger() bindIntakeTrigger}
     * 
     * @return a command to intake
     */
    public Command getIntakeCommand() {
        return Commands.sequence(
                Commands.runOnce(this::startIntaking, m_UTBIntaker, m_feeder),
                Commands.waitUntil(m_beamBreak),
                getAlignNoteCommand()).finallyDo(this::stopIntaking);
    }

    /**
     * Bind intaking to a trigger so it can cancel intaking but not note alignment
     * This should only be called once. For multiple triggers use the trigger
     * composition methods.
     */
    public void bindIntakeTrigger(Trigger intakeTrgger) {
        Command intakeCommand = Commands.sequence(
                Commands.runOnce(this::startIntaking, m_UTBIntaker, m_feeder),
                // Run until beam break datects note or intake trigger becomes false
                Commands.waitUntil(m_beamBreak.or(intakeTrgger.negate())),
                // If the intake trigger is true, then the previous command
                // stopped due to the beam break sensor
                Commands.either(getAlignNoteCommand(), Commands.none(), intakeTrgger::getAsBoolean))
                .finallyDo(this::stopIntaking);

        intakeTrgger.onTrue(intakeCommand);
    }

    public Command getIntakeAutoCommand() {
        return Commands.runOnce(this::startIntaking, m_UTBIntaker, m_feeder);
    }

    /**
     * Reverse the intakers to unjam, until the command is cancelled
     * 
     * @return a command to unjam
     */
    public Command getUnjamIntakersCommand() {
        return Commands.startEnd(this::unjamIntakers, this::stopIntaking, m_UTBIntaker, m_feeder);
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

    public Command getSpinShooterAutoCommand() {
        return Commands.runOnce(() -> softSetShooterState(ShooterState.kSpinning), m_shooter);
    }

    // Starts shooting and stops when the command ends
    public Command getShootCommand() {
        return Commands.startEnd(() -> {
            setState(AttatchmentState.kShooting);
            m_feeder.setState(FeederState.kShooting);
        },
                () -> {
                    setState(AttatchmentState.kAiming);
                    m_feeder.setState(FeederState.kStopped);
                    m_pivot.setPosition(PivotPosition.kIntakePosition);
                });
    }

    // Starts shooting without stopping
    public Command getStartShootCommand() {
        return Commands.runOnce(() -> {
            setState(AttatchmentState.kShooting);
            m_feeder.setState(FeederState.kShooting);
        });
    }

    // Stops shooting
    public Command getStopShootCommand() {
        return Commands.sequence(
                Commands.waitSeconds(0.2),
                Commands.runOnce(() -> {
                    setState(AttatchmentState.kAiming);
                    m_feeder.setState(FeederState.kStopped);
                    if (m_pivot.getPrecisePosition() != PivotConstants.kAmpPos) {
                        m_pivot.setPosition(PivotPosition.kIntakePosition);
                    }
                }));
    }

    // Starts continuous fire without stopping
    public Command getStartContinuousFireCommand() {
        return Commands.runOnce(() -> {
            setState(AttatchmentState.kContinuousFire);
            m_UTBIntaker.setState(IntakerState.kIntaking);
            m_feeder.setState(FeederState.kShooting);
            m_shooter.setState(ShooterState.kShooting);
        }, m_UTBIntaker, m_feeder, m_shooter);
    }

    public void stopContinuousFire() {
        setState(AttatchmentState.kAiming);
        m_pivot.setPosition(PivotPosition.kIntakePosition);
        m_shooter.setState(ShooterState.kStopped);
        m_feeder.setState(FeederState.kStopped);
        m_UTBIntaker.setState(IntakerState.kStopped);
    }

    // Stops continuous fire
    public Command getStopContinuousFireCommand() {
        return Commands.runOnce(() -> {
            stopContinuousFire();
        }, m_UTBIntaker, m_feeder, m_shooter);
    }

    /**
     * Set the position of the pivot
     * 
     * @param position the pivot position
     * @return a command to set the pivot position
     */
    public Command getSetPivotPositionCommand(PivotPosition position) {
        return Commands.runOnce(() -> m_pivot.setPosition(position), m_pivot);
    }

    /**
     * Set the roattion of the pivot to a custom value
     * 
     * @param rotations the pivot angle in rotations of the neo motor
     * @return a command to set the pivot angle
     */
    public Command getSetCustomPivotPositionCommand(double rotations) {
        return Commands.runOnce(() -> {
            m_pivot.setPosition(PivotPosition.kCustomSpeakerPosition);
            m_pivot.setPrecisePosition(rotations);
        }, m_pivot);
    }

    // TODO: testing
    public void setCustomPosition(double rotations) {
        m_pivot.setPosition(PivotPosition.kCustomSpeakerPosition);
        m_pivot.setPrecisePosition(rotations);
    }
}

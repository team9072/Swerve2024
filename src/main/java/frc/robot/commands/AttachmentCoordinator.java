package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TargetConstants.AimingTarget;
import frc.robot.subsystems.attachment.FeederSubsystem;
import frc.robot.subsystems.attachment.ShooterSubsystem;
import frc.robot.subsystems.attachment.UTBIntakerSubsystem;
import frc.robot.subsystems.attachment.FeederSubsystem.FeederPivotPosition;
import frc.robot.subsystems.attachment.FeederSubsystem.FeederState;
import frc.robot.subsystems.attachment.Intaker.IntakerMotorState;

public class AttachmentCoordinator {
    public enum AttatchmentState {
        kIntake,
        kAiming,
        kShooting
    }

    private final UTBIntakerSubsystem m_UTBIntaker;
    // TODO: Make private
    public final FeederSubsystem m_feeder;
    private final ShooterSubsystem m_shooter;
    private final Trigger m_beamBreak;

    // state variables
    private AttatchmentState m_state = AttatchmentState.kIntake;
    private AimingTarget m_target = AimingTarget.kSpeaker;

    public AttachmentCoordinator(UTBIntakerSubsystem utbIntaker, FeederSubsystem feeder, ShooterSubsystem shooter) {
        m_UTBIntaker = utbIntaker;
        m_feeder = feeder;
        m_shooter = shooter;

        m_beamBreak = new Trigger(m_feeder::getBeamBreakState).debounce(0.1, DebounceType.kBoth);

        m_beamBreak.onTrue(new InstantCommand(() -> handleSensorUpdate(true), m_UTBIntaker, m_feeder));
        m_beamBreak.onFalse(new InstantCommand(() -> handleSensorUpdate(false), m_UTBIntaker, m_feeder));
    }

    private void updateState() {
        switch (m_state) {
            case kIntake -> {
                m_feeder.setPosition(FeederPivotPosition.kIntakePosition);
            }
            case kAiming -> {
                m_feeder.setPosition(switch (m_target) {
                    case kSpeaker -> FeederPivotPosition.kSpeakerPosition;
                    case kAmp -> FeederPivotPosition.kAmpPosition;
                });

                m_UTBIntaker.setMotorState(IntakerMotorState.kStopped);
                m_feeder.setState(FeederState.kStopped);
            }
            case kShooting -> {
                m_feeder.setPosition(switch (m_target) {
                    case kSpeaker -> FeederPivotPosition.kSpeakerPosition;
                    case kAmp -> FeederPivotPosition.kAmpPosition;
                });
            }
        }
    }

    private void setState(AttatchmentState state) {
        m_state = state;
        updateState();
    }

    private void handleSensorUpdate(boolean noteDetected) {
        // start aiming on intake nte
        if (m_state == AttatchmentState.kIntake && noteDetected) {
            setState(AttatchmentState.kAiming);
        }

        // go back to intaking if lose note somehow
        if (m_state == AttatchmentState.kAiming && !noteDetected) {
            setState(AttatchmentState.kIntake);
        }

        if (m_state == AttatchmentState.kShooting) {

        }
    }

    public void setTarget(AimingTarget target) {
        m_target = target;
        if (m_state != AttatchmentState.kIntake) {
            m_feeder.setPosition(switch (m_target) {
                case kSpeaker -> FeederPivotPosition.kSpeakerPosition;
                case kAmp -> FeederPivotPosition.kAmpPosition;
            });
        }
    }
}

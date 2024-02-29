package frc.robot.subsystems.attachment;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

    public enum FeederState {
        kAlignReverse,
        kReversed,
        kStopped,
        kIntaking,
        kShooting;
    }

    private final CANSparkMax m_feederMotor;
    private final DigitalInput m_beamBreakSensor;

    private FeederState m_state = FeederState.kStopped;

    /**
     * Create a new feeder subsystem
     */
    public FeederSubsystem() {
        m_feederMotor = new CANSparkMax(FeederConstants.kFeederMotorCANId, MotorType.kBrushless);

        m_beamBreakSensor = new DigitalInput(FeederConstants.kBeamBreakDIOId);

        m_feederMotor.restoreFactoryDefaults();
        m_feederMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Set the state of the feeder
     * 
     * @param state the new state for the feeder
     */
    public void setState(FeederState state) {
        m_state = state;

        double speed = switch (m_state) {
            case kAlignReverse -> FeederConstants.kReverseAlignNoteSpeed;
            case kReversed -> FeederConstants.kReverseSpeed;
            case kStopped -> 0;
            case kIntaking -> FeederConstants.kIntakeSpeed;
            case kShooting -> FeederConstants.kShootSpeed;
        };

        m_feederMotor.set(speed);
    }

    /**
     * Get the state of the beam break sensor
     * 
     * @return true if a note is detected, or fale otherwise
     */
    public boolean getBeamBreakState() {
        return !m_beamBreakSensor.get();
    }

    /*
     * Get the current state of the feeder
     */
    public FeederState getState() {
        return m_state;
    }
}

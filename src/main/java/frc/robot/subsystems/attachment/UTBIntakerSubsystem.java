package frc.robot.subsystems.attachment;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.UTBIntakerConstants;

public class UTBIntakerSubsystem extends Intaker {
    
    private final CANSparkMax m_intakeMotor1;
    private final CANSparkMax m_intakeMotor2;

    private IntakerState m_state = IntakerState.kStopped;

    public UTBIntakerSubsystem() {
        m_intakeMotor1 = new CANSparkMax(UTBIntakerConstants.kIntakeMotor1CANId, MotorType.kBrushless);
        m_intakeMotor2 = new CANSparkMax(UTBIntakerConstants.kIntakeMotor2CANId, MotorType.kBrushless);

        m_intakeMotor1.restoreFactoryDefaults();
        m_intakeMotor2.restoreFactoryDefaults();

        // set coast mode to prevent damage
        m_intakeMotor1.setIdleMode(IdleMode.kCoast);
        m_intakeMotor2.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public IntakerPosition getIntakerPosition() {
        // intaker is always down
        return IntakerPosition.kDown;
    }

    @Override
    public void setIntakerPosition(IntakerPosition position) {
        // UTB is always down and can't be raised;
        return;
    }

    @Override
    public IntakerState getMotorState() {
        return m_state;
    }

    public void setState(IntakerState state) {
        m_state = state;

        double speed = switch (m_state) {
            case kReversed -> UTBIntakerConstants.kReverseMotorSpeed;
            case kStopped -> 0;
            case kIntaking -> UTBIntakerConstants.kIntakeMotorSpeed;
        };

        m_intakeMotor1.set(speed);
        m_intakeMotor2.set(speed);
    }
}

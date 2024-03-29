package frc.robot.subsystems.attachment;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    public enum ShooterState {
        kStopped,
        kSpinning,
        kShooting,
        kAmp
    }

    private CANSparkFlex m_motor1;
    private CANSparkFlex m_motor2;
    private ShooterState m_state = ShooterState.kStopped;
    private double m_speed = ShooterConstants.kShootSpeed;

    public ShooterSubsystem() {
        m_motor1 = new CANSparkFlex(ShooterConstants.kRightShooterMotorCANId, MotorType.kBrushless);
        m_motor2 = new CANSparkFlex(ShooterConstants.kLeftShooterMotorCANId, MotorType.kBrushless);

        m_motor1.restoreFactoryDefaults();
        m_motor2.restoreFactoryDefaults();

        // set coast mode to prevent damage
        m_motor1.setIdleMode(IdleMode.kBrake);
        m_motor2.setIdleMode(IdleMode.kBrake);
    }

    /**
     * set the state of the shooter
     * @param state the new state to set
     */
    public void setState(ShooterState state) {
        m_state = state;
        setSpeed(m_speed);
    }

    public void setSpeed(double speed) {
        m_speed = speed;

        double actualSpeed = switch(m_state) {
            case kStopped -> 0;
            case kSpinning -> m_speed;
            case kShooting -> m_speed;
            case kAmp -> .08 * 1.2;
        };

        m_motor1.set(m_state == ShooterState.kAmp ? .4 * 1.2 : actualSpeed);
        m_motor2.set(actualSpeed);
    }

    /**
     * get the current state of the shooter
     * @return the state of the shooter
     */
    public ShooterState getState() {
        return m_state;
    }
}

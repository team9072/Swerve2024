package frc.robot.subsystems.attachment;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    public enum ShooterState {
        kStopped,
        kSpinning,
        kShooting
    }

    private CANSparkMax m_motor1;
    private CANSparkMax m_motor2;
    private ShooterState m_state = ShooterState.kStopped;

    public ShooterSubsystem() {
        m_motor1 = new CANSparkMax(ShooterConstants.kRightShooterMotorCANId, MotorType.kBrushless);
        m_motor2 = new CANSparkMax(ShooterConstants.kLeftShooterMotorCANId, MotorType.kBrushless);

        m_motor1.restoreFactoryDefaults();
        m_motor2.restoreFactoryDefaults();

        // set coast mode to prevent damage
        m_motor1.setIdleMode(IdleMode.kCoast);
        m_motor2.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void periodic() {
        // TODO flywheel velocity PID
    }

    /**
     * set the state of the shooter
     * @param state the new state to set
     */
    public void setState(ShooterState state) {
        m_state = state;

        double speed = switch(m_state) {
            case kStopped -> 0;
            case kSpinning -> ShooterConstants.kShootSpeed;
            case kShooting -> ShooterConstants.kShootSpeed;
        };

        m_motor1.set(speed * .5);
        m_motor2.set(speed);
    }

    /**
     * set the state of the shooter
     * @param state the new state to set
     * @return a command to set the state of the shooter
     */
    public Command getSetStateCommand(ShooterState state) {
        return this.runOnce(() -> setState(state));
    }

    /**
     * get the current state of the shooter
     * @return the state of the shooter
     */
    public ShooterState getState() {
        return m_state;
    }

    public boolean isShooterReady() {
        //TODO: check shooter rpm
        return true;
    }
}

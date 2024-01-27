package frc.robot.subsystems.intakers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UTBIntakerConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem.FeederState;
import frc.robot.subsystems.feeder.IFeederListener;

public class UTBIntakerSubsystem extends SubsystemBase implements IFeederListener {
    
    private CANSparkMax m_intakeMotor1;
    private CANSparkMax m_intakeMotor2;

    private FeederSubsystem m_feeder;

    public UTBIntakerSubsystem() {
        m_intakeMotor1 = new CANSparkMax(UTBIntakerConstants.kIntakeMotor1CANId, MotorType.kBrushless);
        m_intakeMotor2 = new CANSparkMax(UTBIntakerConstants.kIntakeMotor2CANId, MotorType.kBrushless);
    }

    @Override
    public void handleFeederChange(FeederState state, FeederSubsystem feeder) {
        if (state == FeederState.INTAKE) {
            m_intakeMotor1.set(UTBIntakerConstants.kIntakeMotorSpeed);
            m_intakeMotor2.set(UTBIntakerConstants.kIntakeMotorSpeed);
        } else {
            m_intakeMotor1.set(0);
            m_intakeMotor2.set(0);
        }
    }

    @Override
    public void onFeederRegister(FeederSubsystem feeder) {
        m_feeder = feeder;
    }

    /**
     * Check if theintaker is intaking
     * @return true if the intaker is intaking
     */
    public boolean isIntaking() {
        return m_feeder.getState() == FeederState.INTAKE;
    }

    /**
     * Check if the robot is in a state in which the intaker can start intaking
     * @return true if the intaker can be enabled
     */
    public boolean canStartIntaking() {
        return m_feeder.canStartIntaking();
    }

    /**
     * Try enabling the intake
     * It may or may not enable the intaker, so make sure it can
     * start intaking by calling {@link #canStartIntaking() canStartintaking}
     * @return true if it was able to intake, false otherwise
     */
    public boolean tryStartIntaking() {
        return m_feeder.tryStartIntaking();
    }

    /**
     * Stop the intake
     */
    public void stopIntaking() {
        m_feeder.tryStopFeeder();
    }

    /**
     * Get a command to try starting the intaker
     * It may or may not enable the intaker, so make sure it can
     * start intaking by calling {@link #canStartIntaking() canStartintaking}
     * @return A command to start intaking
     */
    public Command getStartIntakingCommand() {
        return runOnce(() -> tryStartIntaking());
    }

    /**
     * Get a command to stop the intaker
     * @return A command to stop intaking
     */
    public Command getStopIntakingCommand() {
        return runOnce(() -> stopIntaking());
    }
}

package frc.robot.subsystems.attachment;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
    public enum PivotPosition {
        kIntakePosition(PivotConstants.kIntakeMin, PivotConstants.kIntakeMax),
        kSpeakerPosition(PivotConstants.kSpeakerMin, PivotConstants.kSpeakerMax),
        kAmpPosition(PivotConstants.kAmpPos, PivotConstants.kAmpPos);

        public double lowLimit, highLimit;

        PivotPosition(double low, double high) {
            lowLimit = Math.max(PivotConstants.kGlobalMin, low);
            highLimit = Math.min(PivotConstants.kGlobalMax, high);
        }
    }

    private final CANSparkMax m_pivotMotor;
    private SparkPIDController m_pivotPID;
    private RelativeEncoder m_pivotEncoder;

    private PivotPosition m_position = PivotPosition.kIntakePosition;
    private double m_setpoint = 0;

    public PivotSubsystem() {
        m_pivotMotor = new CANSparkMax(PivotConstants.kLeftPivotMotorCANId, MotorType.kBrushless);

        m_pivotMotor.restoreFactoryDefaults();
        m_pivotMotor.setIdleMode(IdleMode.kBrake);

        m_pivotPID = m_pivotMotor.getPIDController();
        // set false to not pop off chain by going wrong way
        m_pivotPID.setPositionPIDWrappingEnabled(false);

        m_pivotEncoder = m_pivotMotor.getEncoder();

        // set pivot PID coefficients
        m_pivotPID.setP(PivotConstants.PivotPID.kP);
        m_pivotPID.setI(PivotConstants.PivotPID.kI);
        m_pivotPID.setD(PivotConstants.PivotPID.kD);
        m_pivotPID.setIZone(PivotConstants.PivotPID.kIz);
        m_pivotPID.setFF(PivotConstants.PivotPID.kFF);
        m_pivotPID.setOutputRange(PivotConstants.PivotPID.kMinOutput, PivotConstants.PivotPID.kMaxOutput);
    }

    @Override
    public void periodic() {
        m_pivotPID.setReference(m_setpoint, CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber("pivot setpoint", m_setpoint);
        SmartDashboard.putNumber("pivot position", m_pivotEncoder.getPosition());
    }

    /**
     * Set the general mode of the shooter.
     * This specifies the available ranges for the pivot.
     * @param pos The new general position for the shooter
     */
    public void setPosition(PivotPosition pos) {
        m_position = pos;
        setPrecisePosition(m_setpoint);
    }

    public PivotPosition getPosition() {
        return m_position;
    }

    public boolean isPivotReady() {
        return Math.abs(m_setpoint - m_pivotEncoder.getPosition()) > PivotConstants.kPositionDeadzone;
    }

    /**
     * Set the specific setpoint for the feeder
     * 
     * @param setpoint the new setpoint for the shooter
     */
    public void setPrecisePosition(double setpoint) {
        if (Double.isNaN(setpoint)) {
            System.out.println("Got NAN pivot setpoint");
            return;
        }

        m_setpoint = Math.max(m_position.lowLimit, Math.min(setpoint, m_position.highLimit));
    }
}

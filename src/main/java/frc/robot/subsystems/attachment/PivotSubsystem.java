package frc.robot.subsystems.attachment;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
    public enum PivotPosition {
        kIntakePosition(PivotConstants.kIntakePos),
        kCustomSpeakerPosition(PivotConstants.kSpeakerMin, PivotConstants.kSpeakerMax),
        kAmpPosition(PivotConstants.kAmpPos),
        kSubwooferPosition(PivotConstants.kSubwooferPos);

        public double lowLimit, highLimit;

        PivotPosition(double pos) {
            lowLimit = Math.max(PivotConstants.kGlobalMin, pos);
            highLimit = Math.min(PivotConstants.kGlobalMax, pos);
        }

        PivotPosition(double low, double high) {
            lowLimit = Math.max(PivotConstants.kGlobalMin, low);
            highLimit = Math.min(PivotConstants.kGlobalMax, high);
        }
    }

    private final CANSparkMax m_leftPivotMotor;    
    private final CANSparkMax m_rightPivotMotor;

    private final SparkPIDController m_pivotPID;
    private final AbsoluteEncoder m_pivotEncoder;

    private PivotPosition m_position = PivotPosition.kIntakePosition;
    private double m_setpoint = PivotConstants.kIntakePos;

    public PivotSubsystem() {
        m_leftPivotMotor = new CANSparkMax(PivotConstants.kLeftPivotMotorCANId, MotorType.kBrushless);
        m_rightPivotMotor = new CANSparkMax(PivotConstants.kRightPivotMotorCANId, MotorType.kBrushless);

        m_leftPivotMotor.restoreFactoryDefaults();
        m_leftPivotMotor.setIdleMode(IdleMode.kBrake);
        m_rightPivotMotor.restoreFactoryDefaults();
        m_rightPivotMotor.setIdleMode(IdleMode.kBrake);
        m_rightPivotMotor.follow(m_leftPivotMotor, true);

        m_pivotPID = m_leftPivotMotor.getPIDController();

        m_pivotEncoder = m_leftPivotMotor.getAbsoluteEncoder();
        m_pivotEncoder.setPositionConversionFactor(125);
        m_pivotEncoder.setVelocityConversionFactor(125);
        m_pivotPID.setFeedbackDevice(m_pivotEncoder);

        m_pivotPID.setPositionPIDWrappingMaxInput(125);
        m_pivotPID.setPositionPIDWrappingMinInput(0);
        m_pivotPID.setPositionPIDWrappingEnabled(true);

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
        if (m_setpoint < PivotConstants.kAmpPos && m_setpoint > 0) {
           m_pivotPID.setReference(m_setpoint, CANSparkMax.ControlType.kPosition);
        }

        SmartDashboard.putNumber("Pivot Setpoint", m_setpoint);
        SmartDashboard.putNumber("Pivot Position", m_pivotEncoder.getPosition());
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

    public double getSetpoint() {
        return m_setpoint;
    }

    public PivotPosition getPosition() {
        return m_position;
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

    public double getPrecisePosition() {
        return m_setpoint;
    }
}

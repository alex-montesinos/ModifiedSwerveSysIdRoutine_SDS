// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModuleConfig;

public class SwerveModule {
  private final String m_name;
  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;
  private final CANcoder m_encoder;
  private final double m_encoderOffset;

  private final StatusSignal<Angle> m_drivePosition;
  private final StatusSignal<AngularVelocity> m_driveVelocity;
  private final StatusSignal<Voltage> m_driveVoltage;
  private final StatusSignal<Angle> m_steerPosition;
  private final StatusSignal<AngularVelocity> m_steerVelocity;
  private final StatusSignal<Voltage> m_steerVoltage;
  private final StatusSignal<Angle> m_encoderAbsolute;

  private final VoltageOut m_driveVoltageRequest =
      new VoltageOut(0).withEnableFOC(SwerveConstants.kUseFOC);
  private final VoltageOut m_steerVoltageRequest =
      new VoltageOut(0).withEnableFOC(SwerveConstants.kUseFOC);
  private final PositionVoltage m_steerPositionRequest =
      new PositionVoltage(0).withSlot(0).withEnableFOC(SwerveConstants.kUseFOC);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  public SwerveModule(ModuleConfig config, CANBus bus) {
    m_name = config.name();
    m_driveMotor = new TalonFX(config.driveMotorId(), bus);
    m_steerMotor = new TalonFX(config.steerMotorId(), bus);
    m_encoder = new CANcoder(config.encoderId(), bus);
    m_encoderOffset = config.encoderOffsetRotations();

    // ---- CANcoder ----
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = config.encoderOffsetRotations();
    encoderConfig.MagnetSensor.SensorDirection =
        SwerveConstants.kSteerEncoderClockwisePositive
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // report -0.5..0.5 rot
    applyWithRetry("CANcoder", () -> m_encoder.getConfigurator().apply(encoderConfig));

    // ---- Drive motor ----
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted =
        config.driveInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveConfig.Feedback.SensorToMechanismRatio = SwerveConstants.kDriveGearRatio;
    driveConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.kDriveStatorCurrentLimitAmps;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    applyWithRetry("drive", () -> m_driveMotor.getConfigurator().apply(driveConfig));
    m_driveMotor.setPosition(0);

    // ---- Steer motor ----
    var steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted =
        config.steerInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    // RemoteCANcoder works without a Phoenix Pro license. (FusedCANcoder requires Pro.)
    steerConfig.Feedback.FeedbackRemoteSensorID = config.encoderId();
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    steerConfig.Feedback.RotorToSensorRatio = SwerveConstants.kSteerGearRatio;
    steerConfig.Feedback.SensorToMechanismRatio = 1.0;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.Slot0.kP = SwerveConstants.kSteerKP;
    steerConfig.Slot0.kD = SwerveConstants.kSteerKD;
    steerConfig.Slot0.kS = SwerveConstants.kSteerKS;
    steerConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.kSteerStatorCurrentLimitAmps;
    steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    applyWithRetry("steer", () -> m_steerMotor.getConfigurator().apply(steerConfig));

    // ---- Signals ----
    m_drivePosition = m_driveMotor.getPosition();
    m_driveVelocity = m_driveMotor.getVelocity();
    m_driveVoltage = m_driveMotor.getMotorVoltage();
    m_steerPosition = m_steerMotor.getPosition();
    m_steerVelocity = m_steerMotor.getVelocity();
    m_steerVoltage = m_steerMotor.getMotorVoltage();
    m_encoderAbsolute = m_encoder.getAbsolutePosition();

    // Faster updates give SysId cleaner data. 100 Hz for these few signals is fine on a CANivore
    // and acceptable on the rio bus for a characterization-only program.
    BaseStatusSignal.setUpdateFrequencyForAll(100, getSignals());
    m_driveMotor.optimizeBusUtilization();
    m_steerMotor.optimizeBusUtilization();
  }

  private void applyWithRetry(String what, java.util.function.Supplier<StatusCode> apply) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = apply.get();
      if (status.isOK()) {
        return;
      }
    }
    DriverStation.reportError(
        "Swerve " + m_name + " " + what + " config failed: " + status.toString(), false);
  }

  /** All status signals for this module, so the subsystem can refresh them together. */
  public BaseStatusSignal[] getSignals() {
    return new BaseStatusSignal[] {
      m_drivePosition,
      m_driveVelocity,
      m_driveVoltage,
      m_steerPosition,
      m_steerVelocity,
      m_steerVoltage,
      m_encoderAbsolute
    };
  }

  public String getName() {
    return m_name;
  }

  // ---------------- Outputs ----------------

  /** Open-loop drive voltage. Steer is left untouched. */
  public void setDriveVoltage(double volts) {
    m_driveMotor.setControl(m_driveVoltageRequest.withOutput(volts));
  }

  /** Open-loop steer voltage. Drive is left untouched. */
  public void setSteerVoltage(double volts) {
    m_steerMotor.setControl(m_steerVoltageRequest.withOutput(volts));
  }

  /** Closed-loop hold of the module angle. */
  public void setSteerAngle(Rotation2d angle) {
    m_steerMotor.setControl(m_steerPositionRequest.withPosition(angle.getRotations()));
  }

  public void stopDrive() {
    m_driveMotor.setControl(m_neutralRequest);
  }

  public void stopSteer() {
    m_steerMotor.setControl(m_neutralRequest);
  }

  /** Simple open-loop module control, used only for repositioning the robot between tests. */
  public void setOpenLoopState(SwerveModuleState desired) {
    Rotation2d current = getSteerAngle();
    desired.optimize(current);
    desired.cosineScale(current);
    double volts = desired.speedMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond * 12.0;
    setDriveVoltage(volts);
    if (Math.abs(desired.speedMetersPerSecond) > 0.01) {
      setSteerAngle(desired.angle);
    } else {
      stopSteer(); // don't jitter the modules when the sticks are centered
    }
  }

  public double getDrivePositionMeters() {
    return m_drivePosition.getValueAsDouble() * SwerveConstants.kWheelCircumferenceMeters;
  }

  public double getDriveVelocityMetersPerSecond() {
    return m_driveVelocity.getValueAsDouble() * SwerveConstants.kWheelCircumferenceMeters;
  }

  public double getDriveAppliedVolts() {
    return m_driveVoltage.getValueAsDouble();
  }

  public double getSteerPositionRotations() {
    return m_steerPosition.getValueAsDouble();
  }

  public double getSteerVelocityRotationsPerSecond() {
    return m_steerVelocity.getValueAsDouble();
  }

  public double getSteerAppliedVolts() {
    return m_steerVoltage.getValueAsDouble();
  }

  public Rotation2d getSteerAngle() {
    return Rotation2d.fromRotations(m_steerPosition.getValueAsDouble());
  }

  /** CANcoder absolute position with the current offset applied, -0.5..0.5 rotations. */
  public double getAbsoluteEncoderRotations() {
    return m_encoderAbsolute.getValueAsDouble();
  }

  /**
   * The offset to put in Constants so that the module's CURRENT physical angle reads as zero.
   * Point the wheel straight forward, then copy this value into the module's encoder offset.
   */
  public double getSuggestedEncoderOffset() {
    return MathUtil.inputModulus(m_encoderOffset - m_encoderAbsolute.getValueAsDouble(), -0.5, 0.5);
  }
}

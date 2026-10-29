// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SysIdConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Swerve drivetrain set up for SysId characterization. Robot-specific settings are all in {@link
 * frc.robot.Constants}; you shouldn't need to change this file.
 *
 * <p>Three routines are provided:
 *
 * <ul>
 *   <li><b>Translation</b>: all four wheels held pointing straight forward, the same voltage on all
 *       four drive motors. Gives drive kS / kV / kA in meters.
 *   <li><b>Steer</b>: the same voltage on all four steer motors, drive motors in neutral. Gives
 *       steer kS / kV / kA in module rotations.
 *   <li><b>Rotation</b> (optional): wheels pointed in a circle so the robot spins in place,
 *       measured with the Pigeon 2. Gives robot kS / kV / kA in radians and a starting kP for a
 *       heading controller.
 * </ul>
 */
public class Drive extends SubsystemBase {
  private final SwerveModule[] m_modules = new SwerveModule[SwerveConstants.kModules.length];
  private final Rotation2d[] m_rotationAngles = new Rotation2d[SwerveConstants.kModules.length];
  private final SwerveDriveKinematics m_kinematics;

  private final Pigeon2 m_pigeon;
  private final StatusSignal<Angle> m_yaw;
  private final StatusSignal<AngularVelocity> m_yawRate;

  private final BaseStatusSignal[] m_allSignals;

  // Mutable holders for unit-safe values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
  private final MutAngle m_angle = Rotations.mutable(0);
  private final MutAngularVelocity m_angularVelocity = RotationsPerSecond.mutable(0);

  private final SysIdRoutine m_translationRoutine;
  private final SysIdRoutine m_steerRoutine;
  private final SysIdRoutine m_rotationRoutine;

  /** Creates a new swerve Drive subsystem. */
  public Drive() {
    CANBus bus = new CANBus(SwerveConstants.kCANBusName);
    List<BaseStatusSignal> signals = new ArrayList<>();
    Translation2d[] locations = new Translation2d[m_modules.length];

    for (int i = 0; i < m_modules.length; i++) {
      var config = SwerveConstants.kModules[i];
      m_modules[i] = new SwerveModule(config, bus);
      signals.addAll(List.of(m_modules[i].getSignals()));
      locations[i] = new Translation2d(config.xMeters(), config.yMeters());
      // For the rotation test, point each wheel along the circle through the module, so that a
      // positive drive voltage spins the robot counter-clockwise.
      m_rotationAngles[i] = new Rotation2d(-config.yMeters(), config.xMeters());
    }
    m_kinematics = new SwerveDriveKinematics(locations);

    // The Pigeon's configuration (including any mount orientation set in Tuner X) is left as-is.
    m_pigeon = new Pigeon2(SwerveConstants.kPigeonId, bus);
    m_yaw = m_pigeon.getYaw();
    m_yawRate = m_pigeon.getAngularVelocityZWorld();
    BaseStatusSignal.setUpdateFrequencyForAll(100, m_yaw, m_yawRate);
    signals.add(m_yaw);
    signals.add(m_yawRate);

    m_allSignals = signals.toArray(new BaseStatusSignal[0]);

    // ---------------- Translation routine ----------------
    m_translationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(SysIdConstants.kTranslationRampVoltsPerSecond).per(Second),
                Volts.of(SysIdConstants.kTranslationStepVolts),
                Seconds.of(SysIdConstants.kTranslationTimeoutSeconds)),
            new SysIdRoutine.Mechanism(
                // Drive voltage to all four drive motors; steer motors hold the wheels straight.
                voltage -> {
                  double volts = voltage.in(Volts);
                  for (SwerveModule module : m_modules) {
                    module.setSteerAngle(Rotation2d.kZero);
                    module.setDriveVoltage(volts);
                  }
                },
                // Each module has its own encoder, so each is logged as its own "motor".
                log -> {
                  for (SwerveModule module : m_modules) {
                    log.motor("drive-" + module.getName())
                        .voltage(m_appliedVoltage.mut_replace(module.getDriveAppliedVolts(), Volts))
                        .linearPosition(
                            m_distance.mut_replace(module.getDrivePositionMeters(), Meters))
                        .linearVelocity(
                            m_velocity.mut_replace(
                                module.getDriveVelocityMetersPerSecond(), MetersPerSecond));
                  }
                },
                this,
                "swerve-translation"));

    // ---------------- Steer routine ----------------
    m_steerRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(SysIdConstants.kSteerRampVoltsPerSecond).per(Second),
                Volts.of(SysIdConstants.kSteerStepVolts),
                Seconds.of(SysIdConstants.kSteerTimeoutSeconds)),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  double volts = voltage.in(Volts);
                  for (SwerveModule module : m_modules) {
                    module.stopDrive();
                    module.setSteerVoltage(volts);
                  }
                },
                log -> {
                  for (SwerveModule module : m_modules) {
                    log.motor("steer-" + module.getName())
                        .voltage(m_appliedVoltage.mut_replace(module.getSteerAppliedVolts(), Volts))
                        .angularPosition(
                            m_angle.mut_replace(module.getSteerPositionRotations(), Rotations))
                        .angularVelocity(
                            m_angularVelocity.mut_replace(
                                module.getSteerVelocityRotationsPerSecond(), RotationsPerSecond));
                  }
                },
                this,
                "swerve-steer"));

    // ---------------- Rotation routine ----------------
    m_rotationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(SysIdConstants.kRotationRampVoltsPerSecond).per(Second),
                Volts.of(SysIdConstants.kRotationStepVolts),
                Seconds.of(SysIdConstants.kRotationTimeoutSeconds)),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  double volts = voltage.in(Volts);
                  for (int i = 0; i < m_modules.length; i++) {
                    m_modules[i].setSteerAngle(m_rotationAngles[i]);
                    m_modules[i].setDriveVoltage(volts);
                  }
                },
                log -> {
                  double avgVolts = 0;
                  for (SwerveModule module : m_modules) {
                    avgVolts += module.getDriveAppliedVolts() / m_modules.length;
                  }
                  log.motor("rotation")
                      .voltage(m_appliedVoltage.mut_replace(avgVolts, Volts))
                      .angularPosition(m_angle.mut_replace(getYawRadians(), Radians))
                      .angularVelocity(
                          m_angularVelocity.mut_replace(getYawRateRadPerSec(), RadiansPerSecond));
                },
                this,
                "swerve-rotation"));
  }

  @Override
  public void periodic() {
    // Grab fresh data from every device once per loop. Subsystem periodic() runs before commands,
    // so the SysId log callbacks always see this loop's data.
    BaseStatusSignal.refreshAll(m_allSignals);

    SmartDashboard.putNumber("Swerve/Heading (deg)", Math.toDegrees(getYawRadians()));
    SmartDashboard.putString("Swerve/Drive Ratio", SwerveConstants.kDriveRatio.name());
    for (SwerveModule module : m_modules) {
      String prefix = "Swerve/" + module.getName() + "/";
      SmartDashboard.putNumber(prefix + "Angle (deg)", module.getSteerAngle().getDegrees());
      SmartDashboard.putNumber(prefix + "Suggested Offset", module.getSuggestedEncoderOffset());
      SmartDashboard.putNumber(prefix + "Distance (m)", module.getDrivePositionMeters());
      SmartDashboard.putNumber(prefix + "Velocity (mps)", module.getDriveVelocityMetersPerSecond());
    }
  }

  private double getYawRadians() {
    return Math.toRadians(m_yaw.getValueAsDouble());
  }

  private double getYawRateRadPerSec() {
    return Math.toRadians(m_yawRate.getValueAsDouble());
  }

  // ---------------- Teleop ----------------

  /**
   * Gentle robot-relative open-loop drive, only meant for repositioning the robot between tests.
   *
   * @param xSpeed forward input, -1..1
   * @param ySpeed left input, -1..1
   * @param rot counter-clockwise rotation input, -1..1
   */
  public Command teleopDriveCommand(
      DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot) {
    return run(() -> {
          double deadband = SwerveConstants.kJoystickDeadband;
          double scale = SwerveConstants.kTeleopSpeedScale;
          var speeds =
              new ChassisSpeeds(
                  MathUtil.applyDeadband(xSpeed.getAsDouble(), deadband)
                      * SwerveConstants.kMaxSpeedMetersPerSecond
                      * scale,
                  MathUtil.applyDeadband(ySpeed.getAsDouble(), deadband)
                      * SwerveConstants.kMaxSpeedMetersPerSecond
                      * scale,
                  MathUtil.applyDeadband(rot.getAsDouble(), deadband)
                      * SwerveConstants.kMaxAngularSpeedRadPerSec
                      * scale);
          SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
          SwerveDriveKinematics.desaturateWheelSpeeds(
              states, SwerveConstants.kMaxSpeedMetersPerSecond);
          for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setOpenLoopState(states[i]);
          }
        })
        .withName("teleopDrive");
  }

  /** Points every wheel straight forward without driving. */
  public Command pointForwardCommand() {
    return run(() -> {
          for (SwerveModule module : m_modules) {
            module.stopDrive();
            module.setSteerAngle(Rotation2d.kZero);
          }
        })
        .withName("pointForward");
  }

  /** Points every wheel along the spin-in-place circle without driving. */
  private Command pointForRotationCommand() {
    return run(() -> {
          for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].stopDrive();
            m_modules[i].setSteerAngle(m_rotationAngles[i]);
          }
        })
        .withName("pointForRotation");
  }

  // ---------------- SysId commands ----------------

  /** Translation quasistatic test (wheels are pointed forward first). */
  public Command sysIdTranslationQuasistatic(SysIdRoutine.Direction direction) {
    return pointForwardCommand()
        .withTimeout(SysIdConstants.kAlignTimeSeconds)
        .andThen(m_translationRoutine.quasistatic(direction));
  }

  /** Translation dynamic test (wheels are pointed forward first). */
  public Command sysIdTranslationDynamic(SysIdRoutine.Direction direction) {
    return pointForwardCommand()
        .withTimeout(SysIdConstants.kAlignTimeSeconds)
        .andThen(m_translationRoutine.dynamic(direction));
  }

  /** Steer quasistatic test. */
  public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
    return m_steerRoutine.quasistatic(direction);
  }

  /** Steer dynamic test. */
  public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
    return m_steerRoutine.dynamic(direction);
  }

  /** Rotation quasistatic test (wheels are pointed around the circle first). */
  public Command sysIdRotationQuasistatic(SysIdRoutine.Direction direction) {
    return pointForRotationCommand()
        .withTimeout(SysIdConstants.kAlignTimeSeconds)
        .andThen(m_rotationRoutine.quasistatic(direction));
  }

  /** Rotation dynamic test (wheels are pointed around the circle first). */
  public Command sysIdRotationDynamic(SysIdRoutine.Direction direction) {
    return pointForRotationCommand()
        .withTimeout(SysIdConstants.kAlignTimeSeconds)
        .andThen(m_rotationRoutine.dynamic(direction));
  }
}

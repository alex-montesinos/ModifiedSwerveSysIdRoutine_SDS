// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  private Constants() {}

  public static final class SwerveConstants {
    private SwerveConstants() {}

    // Count the teeth on the small pinion gear on the drive Kraken's shaft.
    // 12 teeth = R1, 14 teeth = R2, 16 teeth = R3.
    //
    // >>> TODO: Change this ONE line: MK5iDriveRatio.R1, MK5iDriveRatio.R2, or MK5iDriveRatio.R3 <<<
    public static final MK5iDriveRatio kDriveRatio = MK5iDriveRatio.R2;

    /** MK5i drive gear ratio options. The ratio is calculated from the gear tooth counts. */
    public enum MK5iDriveRatio {
      R1(12),
      R2(14),
      R3(16);

      /** Motor rotations per wheel rotation. */
      public final double ratio;

      MK5iDriveRatio(int pinionTeeth) {
        this.ratio = (54.0 / pinionTeeth) * (25.0 / 32.0) * (30.0 / 15.0);
      }
    }

    public static final double kDriveGearRatio = kDriveRatio.ratio;
    public static final double kSteerGearRatio = 26.0; // Always 26:1 for the MK5i

    /**
     * TODO: Measure the wheel diameter. The MK5i wheel is 4" OD when new. Measure the
     * actual tread and put that number here.
     */
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);

    public static final double kWheelCircumferenceMeters = Math.PI * kWheelDiameterMeters;

    public static final String kCANBusName = "rio";

    /** TODO: Set the CAN ID of the Pigeon */
    public static final int kPigeonId = 13;

    public static final boolean kUseFOC = false;

    /** TODO: Measure the distance between the left and right wheel centers. */
    public static final double kTrackWidthMeters = Units.inchesToMeters(22.75);

    /** TODO: Measure the distance between the front and back wheel centers. */
    public static final double kWheelBaseMeters = Units.inchesToMeters(22.75);


    // TODO: Set the CAN IDs: read them from Tuner X.
    //
    // TODO: Set the encoder offsets (rotations, between -0.5 and 0.5):
    //  TunerConstants.java, you can copy <Module>EncoderOffset values directly.
    //  driveID steerID CANcoderID  encoderOffset   driveInvert     steerInvert
    public static final ModuleConfig kFrontLeft =
        new ModuleConfig("FL", 1, 2, 3, 0.0, false, false, +kWheelBaseMeters / 2, +kTrackWidthMeters / 2);
    public static final ModuleConfig kFrontRight =
        new ModuleConfig("FR", 4, 5, 6, 0.0, true, false, +kWheelBaseMeters / 2, -kTrackWidthMeters / 2);
    public static final ModuleConfig kBackLeft =
        new ModuleConfig("BL", 7, 8, 9, 0.0, false, false, -kWheelBaseMeters / 2, +kTrackWidthMeters / 2);
    public static final ModuleConfig kBackRight =
        new ModuleConfig("BR", 10, 11, 12, 0.0, true, false, -kWheelBaseMeters / 2, -kTrackWidthMeters / 2);

    /**
     * TODO: Set true if a module's angle on the dashboard goes DOWN when you turn it counter-clockwise by
     * hand (looking down at the robot from above). Applies to all four CANcoders.
     */
    public static final boolean kSteerEncoderClockwisePositive = false;

    // =============================================================================================
    // limits and gains used by this characterization program.
    // =============================================================================================

    // Keep the drive stator limit high enough that it doesn't cut off the dynamic test. A clipped
    // test gives a bad kA. Lower it only if the wheels slip during the dynamic test.
    public static final double kDriveStatorCurrentLimitAmps = 80.0;
    public static final double kSteerStatorCurrentLimitAmps = 60.0;

    // Steer gains used only to HOLD the wheels at an angle during the translation/rotation tests
    // and for the slow teleop drive. Units: volts per rotation of module error. SysId will give you
    // better ones.
    public static final double kSteerKP = 100.0;
    public static final double kSteerKD = 0.5;
    public static final double kSteerKS = 0.1;

    /** Kraken X60 free speed from the SDS table (RPM). */
    public static final double kKrakenFreeSpeedRpm = kUseFOC ? 5800.0 : 6000.0;

    /** Theoretical top speed for the selected ratio. Only used to scale joystick input. */
    public static final double kMaxSpeedMetersPerSecond =
        kKrakenFreeSpeedRpm / 60.0 / kDriveGearRatio * kWheelCircumferenceMeters;

    /** Fraction of top speed allowed while repositioning the robot. Keep it gentle. */
    public static final double kTeleopSpeedScale = 0.4;

    public static final double kMaxAngularSpeedRadPerSec = 1.5 * Math.PI;
    public static final double kJoystickDeadband = 0.08;

    public static final ModuleConfig[] kModules = {kFrontLeft, kFrontRight, kBackLeft, kBackRight};

    /** Hardware configuration for one swerve module. */
    public record ModuleConfig(
        String name,
        int driveMotorId,
        int steerMotorId,
        int encoderId,
        double encoderOffsetRotations,
        boolean driveInverted,
        boolean steerInverted,
        double xMeters,
        double yMeters) {}
  }

  public static final class SysIdConstants {
    private SysIdConstants() {}

    // Translation (drive motor) test.
    // The 1 V/s quasistatic ramp gets fast quickly. Release the button before you run out of room.
    // A test doesn't need to run until it times out.
    public static final double kTranslationRampVoltsPerSecond = 1.0;
    public static final double kTranslationStepVolts = 4.0;
    public static final double kTranslationTimeoutSeconds = 10.0;

    // Steer (azimuth) test. The modules spin in place, so a 7 V step is fine.
    public static final double kSteerRampVoltsPerSecond = 1.0;
    public static final double kSteerStepVolts = 7.0;
    public static final double kSteerTimeoutSeconds = 10.0;

    // Rotation (robot spins in place) test. Optional.
    public static final double kRotationRampVoltsPerSecond = 1.0;
    public static final double kRotationStepVolts = 4.0;
    public static final double kRotationTimeoutSeconds = 10.0;

    /** Time spent pointing the wheels before each translation/rotation test starts. */
    public static final double kAlignTimeSeconds = 0.5;
  }

  public static final class OIConstants {
    private OIConstants() {}

    public static final int kDriverControllerPort = 0;
  }
}

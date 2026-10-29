// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive;

/**
 * Declares the swerve subsystem and binds the SysId tests to the driver controller.
 *
 * Controls (hold the button for the whole test, release to stop early):
 * - Sticks: gentle robot-relative drive for repositioning between tests
 * - Right bumper + A / B / X / Y: TRANSLATION quasistatic fwd / quasistatic rev / dynamic fwd / dynamic rev
 * - Left bumper + A / B / X / Y: STEER quasistatic fwd / quasistatic rev / dynamic fwd / dynamic rev
 * - Back + A / B / X / Y: ROTATION quasistatic fwd / quasistatic rev / dynamic fwd / dynamic rev (optional)
 * - Start: point all wheels forward (handy for checking encoder offsets)
 */
public class SysIdRoutineBot {
  private final Drive m_drive = new Drive();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  /** Defines the button bindings. Should be called in the robot class constructor. */
  public void configureBindings() {
    // Robot-relative drive: left stick translates, right stick X rotates.
    m_drive.setDefaultCommand(
        m_drive.teleopDriveCommand(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    m_driverController.start().whileTrue(m_drive.pointForwardCommand());

    // Translation (drive motor) routine on the right bumper.
    m_driverController
        .a()
        .and(m_driverController.rightBumper())
        .whileTrue(m_drive.sysIdTranslationQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController
        .b()
        .and(m_driverController.rightBumper())
        .whileTrue(m_drive.sysIdTranslationQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController
        .x()
        .and(m_driverController.rightBumper())
        .whileTrue(m_drive.sysIdTranslationDynamic(SysIdRoutine.Direction.kForward));
    m_driverController
        .y()
        .and(m_driverController.rightBumper())
        .whileTrue(m_drive.sysIdTranslationDynamic(SysIdRoutine.Direction.kReverse));

    // Steer (azimuth) routine on the left bumper.
    m_driverController
        .a()
        .and(m_driverController.leftBumper())
        .whileTrue(m_drive.sysIdSteerQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController
        .b()
        .and(m_driverController.leftBumper())
        .whileTrue(m_drive.sysIdSteerQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController
        .x()
        .and(m_driverController.leftBumper())
        .whileTrue(m_drive.sysIdSteerDynamic(SysIdRoutine.Direction.kForward));
    m_driverController
        .y()
        .and(m_driverController.leftBumper())
        .whileTrue(m_drive.sysIdSteerDynamic(SysIdRoutine.Direction.kReverse));

    // Rotation (spin in place, measured by the Pigeon 2) routine on the Back button. Optional.
    m_driverController
        .a()
        .and(m_driverController.back())
        .whileTrue(m_drive.sysIdRotationQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController
        .b()
        .and(m_driverController.back())
        .whileTrue(m_drive.sysIdRotationQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController
        .x()
        .and(m_driverController.back())
        .whileTrue(m_drive.sysIdRotationDynamic(SysIdRoutine.Direction.kForward));
    m_driverController
        .y()
        .and(m_driverController.back())
        .whileTrue(m_drive.sysIdRotationDynamic(SysIdRoutine.Direction.kReverse));
  }

  /** Use this to define the command that runs during autonomous. */
  public Command getAutonomousCommand() {
    // Do nothing
    return Commands.none();
  }
}

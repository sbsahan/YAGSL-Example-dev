// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

  final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);

  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/talonsrx"));

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverPS4.getLeftY() * -1,
      () -> driverPS4.getLeftX() * -1)
      .withControllerRotationAxis(driverPS4::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverPS4::getRightX,
      driverPS4::getRightY)
      .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverPS4.getLeftY(),
      () -> -driverPS4.getLeftX())
      .withControllerRotationAxis(() -> driverPS4.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  
  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  private void configureBindings() {

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveFieldOriented = drivebase.driveCommand(
        () -> -driverPS4.getLeftX(), 
        () -> -driverPS4.getLeftY(), 
        () -> driverPS4.getRightX(), 
        true, 
        false);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
    } else {
      drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
    }

    if (Robot.isSimulation()) {
      driverPS4.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      driverPS4.circle().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverPS4.cross().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverPS4.triangle().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverPS4.square().whileTrue(drivebase.centerModulesCommand());
    } else {
      driverPS4.button(1).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.start().whileTrue(Commands.none());
      // driverXbox.back().whileTrue(Commands.none());
      // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock,
      // drivebase).repeatedly());
      // driverXbox.rightBumper().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}

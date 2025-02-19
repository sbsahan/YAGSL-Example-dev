// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

public class RobotContainer {

  final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);

  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/talonsrx"));

  Command driveFieldOriented = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverPS4.getLeftX() * -1, OperatorConstants.DEADBAND),
      () -> MathUtil.applyDeadband(driverPS4.getLeftY() * -1, OperatorConstants.DEADBAND),
      () -> MathUtil.applyDeadband(driverPS4.getRightX() * -1, OperatorConstants.DEADBAND),
        true, 
        false);
  
  Command driveRobotOriented = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverPS4.getLeftX() * -1, OperatorConstants.DEADBAND),
      () -> MathUtil.applyDeadband(driverPS4.getLeftY() * -1, OperatorConstants.DEADBAND),
      () -> MathUtil.applyDeadband(driverPS4.getRightX() * -1, OperatorConstants.DEADBAND),
        false, 
        false);
  
  /*changed commands, 
  removed sim & test for better readability, 
  if needed revert back to old vers. or copy from yagsl-example
  */

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    drivebase.setDefaultCommand(driveFieldOriented);
    driverPS4.button(1).onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverPS4.circle().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}

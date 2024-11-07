// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  CommandPS5Controller PS5Controller = new CommandPS5Controller(Constants.OperatorConstants.kDriverControllerPort);

  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  public RobotContainer() {
    configureBindings();

    Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(PS5Controller.getLeftY() * 0.8, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(PS5Controller.getLeftX() * 0.8, OperatorConstants.LEFT_X_DEADBAND),
        () -> PS5Controller.getRightX() * 0.6);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(PS5Controller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(PS5Controller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> PS5Controller.getRightX());

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);
  }

  private void configureBindings() {
    PS5Controller.options().onTrue((new InstantCommand(drivebase::zeroGyro)));
    PS5Controller.square().whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public void setDriveMode() {
    configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}

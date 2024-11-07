// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class AbsoluteFieldDrive extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX, vY, heading;

  public AbsoluteFieldDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
      DoubleSupplier heading) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get the desired chassis speeds based on a 2 joystick module.

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
        new Rotation2d(heading.getAsDouble() * Math.PI));

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
        Constants.Swerve.LOOP_TIME, Constants.Swerve.ROBOT_MASS, List.of(Constants.Swerve.CHASSIS),
        swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}

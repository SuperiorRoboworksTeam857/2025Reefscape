// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToAngleCommand extends Command {
  private final Swerve s_Swerve;
  private boolean complete = false;
  private double angle;
  private Timer timer = new Timer();
  private double timeout;

  public TurnToAngleCommand(Swerve subsystem, double degrees, double timeoutS) {
    s_Swerve = subsystem;
    angle = degrees;
    timeout = timeoutS;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    complete = false;
  }

  // @Override
  // public void execute() {
  //   double gyroAngle = s_Swerve.getYaw().getDegrees(); // do we need to negate this number?

  //   final double kP = 0.2;
  //   SmartDashboard.putNumber("TurnToAngle - gyroAngle", gyroAngle);
  //   SmartDashboard.putNumber("TurnToAngle - goal angle", angle);

  //   SwerveModuleState desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(angle * 0.5));
  //   desiredState =
  //       OnboardModuleState.optimize(desiredState, Rotation2d.fromDegrees(gyroAngle * 0.5));
  //   angle = desiredState.angle.getDegrees() * 2;

  //   SmartDashboard.putNumber("TurnToAngle - corrected goal angle", angle);

  //   double err = angle - gyroAngle;
  //   double speed =
  //       MathUtil.clamp(
  //           err * kP,
  //           -Constants.Swerve.maxAngularVelocity * 0.5,
  //           Constants.Swerve.maxAngularVelocity * 0.5);

  //   SmartDashboard.putNumber("TurnToAngle - speed", speed);

  //   if ((Math.abs(err) > 2 || Math.abs(s_Swerve.getYawRate()) > 1) && timer.get() < timeout) {
  //     s_Swerve.drive(new Translation2d(0, 0), speed, false, true);
  //   } else {
  //     complete = true;
  //   }
  // }

  // @Override
  // public void end(boolean inturrupted) {
  //   s_Swerve.drive(new Translation2d(0, 0), 0, false, true);
  //   timer.stop();
  // }

  // @Override
  // public boolean isFinished() {
  //   return complete;
  // }
}
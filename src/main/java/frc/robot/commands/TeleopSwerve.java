// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier limelight; // NEEDS TO BE CHECKED
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier slowSpeedSup;
  private BooleanSupplier highSpeedSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier limelight, // NEEDS TO BE CHECKED
      BooleanSupplier robotCentricSup,
      BooleanSupplier slowSpeedSup,
      BooleanSupplier highSpeedSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.limelight = limelight; // NEEDS TO BE CHECKED
    this.robotCentricSup = robotCentricSup;
    this.slowSpeedSup = slowSpeedSup;
    this.highSpeedSup = highSpeedSup;
  }

  @Override
  public void execute() {

    double speedMultiplier = Constants.Swerve.normalDriveSpeedMultiplier;
    if (highSpeedSup.getAsBoolean()) speedMultiplier = Constants.Swerve.fastDriveSpeedMultiplier;
    if (slowSpeedSup.getAsBoolean()) speedMultiplier = Constants.Swerve.slowDriveSpeedMultiplier;

    double rotationSpeedMultiplier = Math.min(speedMultiplier, Constants.Swerve.normalDriveSpeedMultiplier);

    /* Get Values, Deadband*/
    double translationVal =
        translationLimiter.calculate(
            speedMultiplier
                * MathUtil.applyDeadband(
                    translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            speedMultiplier
                * MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            rotationSpeedMultiplier
                * MathUtil.applyDeadband(
                    rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true);
  }
}
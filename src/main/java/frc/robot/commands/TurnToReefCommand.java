package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TurnToReefCommand extends Command {
    private final Swerve s_Swerve;
    private boolean complete = false;
    private double m_angle = 0;
    private Joystick m_stick;

    public TurnToReefCommand(Swerve subsystem, Joystick stick) {
        s_Swerve = subsystem;
        m_stick = stick;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        complete = false;

        double gyroAngle = s_Swerve.getYaw().getDegrees();
        m_angle = 60 * Math.round(gyroAngle / 60.0);

        SmartDashboard.putNumber("TurnToReef - goal angle", m_angle);
    }

    @Override
    public void execute() {
        double gyroAngle = s_Swerve.getYaw().getDegrees();

        final double kP = 0.2;
        SmartDashboard.putNumber("TurnToReef - gyroAngle", gyroAngle);

        SwerveModuleState desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(m_angle * 0.5));
        desiredState = OnboardModuleState.optimize(desiredState, Rotation2d.fromDegrees(gyroAngle * 0.5));
        m_angle = desiredState.angle.getDegrees() * 2;

        SmartDashboard.putNumber("TurnToReef - corrected goal angle", m_angle);

        double err = m_angle - gyroAngle;
        double speed = MathUtil.clamp(
                err * kP,
                -Constants.Swerve.maxAngularVelocity * 0.5,
                Constants.Swerve.maxAngularVelocity * 0.5);

        SmartDashboard.putNumber("TurnToReef - speed", speed);

        s_Swerve.drive(new Translation2d(-m_stick.getRawAxis(Joystick.AxisType.kY.value), 
                                         -m_stick.getRawAxis(Joystick.AxisType.kX.value)), speed, false, true);
    }

    @Override
    public boolean isFinished() {
        return complete;
    }
}

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class TurnToReefCommand extends Command {
    private final Swerve s_Swerve;
    private final Limelight s_Limelight;
    private boolean complete = false;
    private double angle = 0;
    private Timer timer = new Timer();
    private double timeout;
    private Joystick m_stick;
    
    // TODO
    // Hold down until it CAN stamp
    // Give red until it cannot see april tag
    // Switch angles 180deg
    // Ensure turning is field centric

    private static final double maxActivationDistance = 1.5;

    public TurnToReefCommand(Swerve subsystem, Limelight limeLight, Joystick stick, double timeoutS) {
        s_Swerve = subsystem;
        s_Limelight = limeLight;
        m_stick = stick;
        timeout = timeoutS;
        addRequirements(subsystem);
        addRequirements(limeLight);
    }

    @Override
    public void initialize() {
        // s_Limelight.setIsReefAprilTagValid(false);
        // timer.reset();
        complete = false;
        // int tag = s_Limelight.aprilTagID();
        // boolean exists = Constants.Swerve.reefAprilTagAngles.containsKey(tag);
        // double distanceAway = s_Limelight.distanceToAprilTagMeters();
        // if (exists && distanceAway < maxActivationDistance) {
        //     int getAngle = Constants.Swerve.reefAprilTagAngles.get(tag);
        //     angle = getAngle;
        //     s_Limelight.setIsReefAprilTagValid(true);
        // } else {
        //     complete = true;
        //     s_Limelight.setIsReefAprilTagValid(false);
        // }

        double gyroAngle = s_Swerve.getYaw().getDegrees();
        angle = 60 * Math.round(gyroAngle / 60.0);


        // SmartDashboard.putNumber("TurnToReef - tag id", tag);
        SmartDashboard.putNumber("TurnToReef - goal angle", angle);
    }

    @Override
    public void execute() {
        double gyroAngle = s_Swerve.getYaw().getDegrees(); // do we need to negate this number?

        final double kP = 0.2; // overall speed
        SmartDashboard.putNumber("TurnToReef - gyroAngle", gyroAngle);

        SwerveModuleState desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(angle * 0.5));
        desiredState = OnboardModuleState.optimize(desiredState, Rotation2d.fromDegrees(gyroAngle * 0.5));
        angle = desiredState.angle.getDegrees() * 2;

        SmartDashboard.putNumber("TurnToReef - corrected goal angle", angle);

        double err = angle - gyroAngle;
        double speed = MathUtil.clamp(
                err * kP,
                -Constants.Swerve.maxAngularVelocity * 0.5,
                Constants.Swerve.maxAngularVelocity * 0.5);

        SmartDashboard.putNumber("TurnToReef - speed", speed);

        s_Swerve.drive(new Translation2d(-m_stick.getRawAxis(Joystick.AxisType.kY.value), 
                                         -m_stick.getRawAxis(Joystick.AxisType.kX.value)), speed, false, true);

        // if ((Math.abs(err) > 2 || Math.abs(s_Swerve.getYawRate()) > 1) && timer.get() < timeout) {
        //     s_Swerve.drive(new Translation2d(0, 0), speed, false, true);
        // } else {
        //     //complete = true;
        // }
    }

    @Override
    public boolean isFinished() {
        return complete;
    }
}

// LEDs
// Default - Blue
// Blinking Red/Green - Having a Coral
// Feed out when reaches beam break

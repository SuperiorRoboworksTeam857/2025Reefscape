package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    private double angle;
    private Timer timer = new Timer();
    private double timeout;

    private Map<Integer, Integer> angles = new HashMap<Integer, Integer>() {
        {
            put(6, 120);
            put(7, 180);
            put(8, 240);
            put(9, 300);
            put(10, 0);
            put(11, 60);
            put(17, 60);
            put(18, 0);
            put(19, 300);
            put(20, 240);
            put(21, 180);
            put(22, 120);
        }
    };

    public TurnToReefCommand(Swerve subsystem, Limelight limeLight, double timeoutS) {
        s_Swerve = subsystem;
        s_Limelight = limeLight;
        timeout = timeoutS;
        addRequirements(subsystem);
        addRequirements(limeLight);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.reset();
        complete = false;
        int tag = s_Limelight.aprilTagID();
        boolean exists = angles.containsKey(tag);
        if (exists) {
            int getAngle = angles.get(tag);
            angle = getAngle;

            
        } else {
            complete = true;
        }

        SmartDashboard.putNumber("TurnToReef - tag id", tag);
        SmartDashboard.putNumber("TurnToReef - goal angle", angle);
    }

    @Override
    public void execute() {
        double gyroAngle = s_Swerve.getYaw().getDegrees(); // do we need to negate this number?

        final double kP = 0.2; // overall speed
        SmartDashboard.putNumber("TurnToReef - gyroAngle", gyroAngle);
        SmartDashboard.putNumber("TurnToReef - goal angle", angle);

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

        if ((Math.abs(err) > 2 || Math.abs(s_Swerve.getYawRate()) > 1) && timer.get() < timeout) {
            s_Swerve.drive(new Translation2d(0, 0), speed, false, true);
        } else {
            complete = true;
        }
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

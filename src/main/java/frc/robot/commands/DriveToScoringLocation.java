package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveToScoringLocation extends Command {
    private final Swerve s_Swerve;
    private boolean complete = false;
    private double m_angle = 0;
    private Pose2d m_targetPose;
    private Location m_poleLocation = Location.LEFT_L4;

    public enum Location {LEFT_L4, RIGHT_L4, LEFT_L2_L3, RIGHT_L2_L3};

    private enum FieldSector {UP_LEFT, LEFT, DOWN_LEFT, UP_RIGHT, RIGHT, DOWN_RIGHT};

    private static final Translation2d BlueReefCenter = new Translation2d(Units.inchesToMeters(176.745), Units.inchesToMeters(158.5));
    private static final Translation2d RedReefCenter = new Translation2d(Units.inchesToMeters(514.13), Units.inchesToMeters(158.5));

    private static final Translation2d LeftL4Alignment = new Translation2d(Units.inchesToMeters(16.75+8.2), Units.inchesToMeters(-7));
    private static final Translation2d RightL4Alignment = new Translation2d(Units.inchesToMeters(16.75+8.2), Units.inchesToMeters(5.5));

    private static final Translation2d LeftL2L3Alignment = new Translation2d(Units.inchesToMeters(16.75-2.5), Units.inchesToMeters(-7));
    private static final Translation2d RightL2L3Alignment = new Translation2d(Units.inchesToMeters(16.75-2.5), Units.inchesToMeters(5.5));
    

    AprilTagFieldLayout m_layout;

    public DriveToScoringLocation(Swerve subsystem, AprilTagFieldLayout layout, Location poleLocation) {
        s_Swerve = subsystem;
        m_layout = layout;
        m_poleLocation = poleLocation;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        complete = false;

        final FieldSector sector = currentFieldSector();
       
        final int tagID = tagIDForSector(sector);

        SmartDashboard.putNumber("DriveToScore - tag", tagID);

        final var tagPose3D = m_layout.getTagPose(tagID);

        // Verify tag is present
        if (!tagPose3D.isPresent()) {
            complete = true;
            return;
        }

        final Pose2d tagPose2D = tagPose3D.get().toPose2d();

        double angleOffset = 0;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red)) {
            angleOffset = 180;
        }
         
        m_angle = tagPose2D.getRotation().getDegrees() + angleOffset;

        SmartDashboard.putNumber("DriveToScore - goal angle", m_angle);

        Translation2d alignment = Translation2d.kZero;
        switch (m_poleLocation) {
            case LEFT_L4:
                alignment = LeftL4Alignment;
                break;
            case RIGHT_L4:
                alignment = RightL4Alignment;
                break;
            case LEFT_L2_L3:
                alignment = LeftL2L3Alignment;
                break;
            case RIGHT_L2_L3:
                alignment = RightL2L3Alignment;
                break;
        }
        m_targetPose = tagPose2D.plus(new Transform2d(alignment, Rotation2d.kZero));

        s_Swerve.field.getObject("tag").setPose(tagPose2D);
        s_Swerve.field.getObject("target").setPose(m_targetPose);

        SmartDashboard.putNumber("DriveToScore - target pose x", m_targetPose.getX());
        SmartDashboard.putNumber("DriveToScore - target pose y", m_targetPose.getY());
    }

    @Override
    public void execute() {
        double gyroAngle = s_Swerve.getYaw().getDegrees();

        final double kPTurn = 0.2;
        final double kPTranslate = 2.5;

        SmartDashboard.putNumber("DriveToScore - gyroAngle", gyroAngle);

        SwerveModuleState desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(m_angle * 0.5));
        desiredState = OnboardModuleState.optimize(desiredState, Rotation2d.fromDegrees(gyroAngle * 0.5));
        m_angle = desiredState.angle.getDegrees() * 2;

        SmartDashboard.putNumber("DriveToScore - corrected goal angle", m_angle);

        double turnErr = m_angle - gyroAngle;
        double turnSpeed = MathUtil.clamp(
                turnErr * kPTurn,
                -Constants.Swerve.maxAngularVelocity * 0.5,
                Constants.Swerve.maxAngularVelocity * 0.5);

        SmartDashboard.putNumber("DriveToScore - turn speed", turnSpeed);

        double xErr = m_targetPose.getX() - s_Swerve.getPose().getX();
        double xSpeed = MathUtil.clamp(
                xErr * kPTranslate,
                -Constants.Swerve.maxSpeed * 0.5,
                Constants.Swerve.maxSpeed * 0.5);
        double yErr = m_targetPose.getY() - s_Swerve.getPose().getY();
        double ySpeed = MathUtil.clamp(
                yErr * kPTranslate,
                -Constants.Swerve.maxSpeed * 0.5,
                Constants.Swerve.maxSpeed * 0.5);

        SmartDashboard.putNumber("DriveToScore - x speed", xSpeed);
        SmartDashboard.putNumber("DriveToScore - y speed", ySpeed);


        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red)) {
            xSpeed = -xSpeed;
            ySpeed = -ySpeed;
        }

        s_Swerve.drive(new Translation2d(xSpeed, ySpeed), turnSpeed, true, true);

    }

    @Override
    public boolean isFinished() {
        return complete;
    }

    private FieldSector currentFieldSector(){
        FieldSector sector = FieldSector.LEFT;

        Translation2d reefCenter = BlueReefCenter;

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red)) {
            reefCenter = RedReefCenter;
        }

        Translation2d robotPosition = s_Swerve.getPose().getTranslation();

        Translation2d directionToRobot = robotPosition.minus(reefCenter);

        double angleRadians = MathUtil.angleModulus(directionToRobot.getAngle().getRadians());

        double angleDegrees = Units.radiansToDegrees(angleRadians);

        if (angleDegrees >= -150 && angleDegrees <= -90) {
            sector = FieldSector.DOWN_LEFT;
        } else if (angleDegrees >= -90 && angleDegrees <= -30) {
            sector = FieldSector.DOWN_RIGHT;
        } else if (angleDegrees >= -30 && angleDegrees <= 30) {
            sector = FieldSector.RIGHT;
        } else if (angleDegrees >= 30 && angleDegrees <= 90) {
            sector = FieldSector.UP_RIGHT;
        } else if (angleDegrees >= 90 && angleDegrees <= 150) {
            sector = FieldSector.UP_LEFT;
        } 

        return sector;
    }
    
    int tagIDForSector(FieldSector sector){
        int tagID = -1;

        boolean isRedAlliance = false;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red)) {
            isRedAlliance = true;
        }
         
        switch (sector) {
            case UP_LEFT:
                tagID = isRedAlliance ? 9 : 19;
                break;
            case LEFT:
                tagID = isRedAlliance ? 10 : 18;
                break;
            case DOWN_LEFT:
                tagID = isRedAlliance ? 11 : 17;
                break;
            case DOWN_RIGHT:
                tagID = isRedAlliance ? 6 : 22;
                break;
            case RIGHT:
                tagID = isRedAlliance ? 7 : 21;
                break;
            case UP_RIGHT:
                tagID = isRedAlliance ? 8 : 20;
                break;
        }

        return tagID;
    }
}

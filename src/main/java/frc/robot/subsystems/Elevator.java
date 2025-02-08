package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

// Notes: Gear Ratio 20:1 and we are using NEOS

public class Elevator extends SubsystemBase {
    private static double deltaTime = 0.02;
    // Create a PID controller whose setpoint's change is subject to maximum
    // velocity and acceleration constraints.
    private final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(20, 80);
    private final ProfiledPIDController m_controller =
        new ProfiledPIDController(0.15, 0.0, 0.0, m_constraints, deltaTime);
  
    // or should we use SparkMaxPIDController?
  
    SparkMax motor = new SparkMax(ElevatorConstants.elevatorMotor, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
  
    // Range of motion of 0 inches at bottom to -24.5 inches at top
    public enum Positions {
      CORAL_STATION_L2,
      CORAL_STATION_L3,
      CORAL_STATION_L4,
      HUMANPLAYER_STATION,
      IDLE_MODE,
    }
  
    private double m_goalPosition = 0;
  
    public Elevator() {
      double sprocketDiameter = 22 * 0.25 / Math.PI; // 22 teeth at 0.25 inch pitch
      double gearRatio = 25; // 25:1
      double driveConversionPositionFactor = (sprocketDiameter * Math.PI) / gearRatio;
      double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
      config.encoder
      .positionConversionFactor(driveConversionPositionFactor)
      .velocityConversionFactor(driveConversionVelocityFactor);
      motor.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }
  
    @Override
    public void periodic() {
      m_controller.setGoal(m_goalPosition);
      motor.set(m_controller.calculate(motor.getEncoder().getPosition()));
  
      SmartDashboard.putNumber("elevator position", motor.getEncoder().getPosition());
      SmartDashboard.putNumber("elevator speed", motor.getEncoder().getVelocity());
    }
  
    public void goToPosition(Positions position) {
      switch (position) {
        case CORAL_STATION_L2:
          m_goalPosition = -30.41;
          break;
        case CORAL_STATION_L3:
          m_goalPosition = -46.26;
          break;
        case CORAL_STATION_L4:
          m_goalPosition = -71.87;
          break;
        case HUMANPLAYER_STATION:
          m_goalPosition = -37.75;
        case IDLE_MODE:
          m_goalPosition = -0;
          break;
      }
    }
  
    public boolean isElevatorAtGoal() {
      return Math.abs(motor.getEncoder().getPosition() - m_goalPosition) < 1.0;
    }
  
    public boolean isElevatorHigh() {
      return motor.getEncoder().getPosition() < -45;
    }
  
    public boolean isElevatorLow() {
      return motor.getEncoder().getPosition() > -1;
    }
  
    public void resetEncoders() {
      motor.getEncoder().setPosition(0);
    }
    public boolean isMoving() {
        return Math.abs(motor.getEncoder().getVelocity()) > 1.0;
    }
}
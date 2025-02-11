package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Servo;
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
    private final ProfiledPIDController m_controller_left =
        new ProfiledPIDController(0.15, 0.0, 0.0, m_constraints, deltaTime);
    private final ProfiledPIDController m_controller_right =
        new ProfiledPIDController(0.15, 0.0, 0.0, m_constraints, deltaTime);
  
    SparkMax motorLeft = new SparkMax(ElevatorConstants.elevatorLeftMotor, MotorType.kBrushless);
    SparkMax motorRight = new SparkMax(ElevatorConstants.elevatorRightMotor, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();

    // servos need to be pointed DOWN
    Servo servoLeft = new Servo(ElevatorConstants.elevatorLeftServoMotor);
    Servo servoRight = new Servo(ElevatorConstants.elevatorRightServoMotor);
  
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
      double gearRatio = 20; // 20:1
      double driveConversionPositionFactor = (sprocketDiameter * Math.PI) / gearRatio;
      double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;

      config.encoder
        .positionConversionFactor(driveConversionPositionFactor)
        .velocityConversionFactor(driveConversionVelocityFactor);

      motorLeft.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
      motorRight.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }
  
    @Override
    public void periodic() {

      servoLeft.set(ElevatorConstants.servoLeftPressed);
      servoRight.set(ElevatorConstants.servoRightPressed);

      m_controller_left.setGoal(ElevatorConstants.elevatorMotorInverse * -m_goalPosition);
      motorLeft.set(m_controller_left.calculate(motorLeft.getEncoder().getPosition()));
      m_controller_right.setGoal(ElevatorConstants.elevatorMotorInverse * m_goalPosition);
      motorRight.set(m_controller_right.calculate(motorRight.getEncoder().getPosition()));
  
      SmartDashboard.putNumber("elevator position left", motorLeft.getEncoder().getPosition());
      SmartDashboard.putNumber("elevator speed left", motorLeft.getEncoder().getVelocity());
      SmartDashboard.putNumber("elevator position right", motorRight.getEncoder().getPosition());
      SmartDashboard.putNumber("elevator speed right", motorRight.getEncoder().getVelocity());
      SmartDashboard.putNumber("elevator ideal position", m_goalPosition);
    }
  
    public void goToPosition(Positions position) {
      switch (position) {
        case HUMANPLAYER_STATION:
          m_goalPosition = 0;
        case IDLE_MODE:
          m_goalPosition = 0;
          break;
        case CORAL_STATION_L2:
          m_goalPosition = 10;
          break;
        case CORAL_STATION_L3:
          m_goalPosition = 29;
          break;
        case CORAL_STATION_L4:
          m_goalPosition = 20;
          break;
      }
    }
  
    // public boolean isElevatorAtGoal() {
    //   double leftGoal = m_goalPosition*ElevatorConstants.elevatorMotorInverse;
    //   double rightGoal = m_goalPosition*ElevatorConstants.elevatorMotorInverse*-1;
    //   return (Math.abs(motorLeft.getEncoder().getPosition() - leftGoal) < 1.0) && (Math.abs(motorRight.getEncoder().getPosition()- rightGoal) < 1.0);
    // }
  
    // public boolean isElevatorHigh() {
    //   return (Math.abs(motorLeft.getEncoder().getPosition()) > 45) || (Math.abs(motorRight.getEncoder().getPosition()) > 45);
    // }
  
    // public boolean isElevatorLow() {
    //   return (Math.abs(motorLeft.getEncoder().getPosition()) < 1) || (Math.abs(motorRight.getEncoder().getPosition()) < 1);
    // }
  
    public void resetEncoders() {
      motorLeft.getEncoder().setPosition(0);
      motorRight.getEncoder().setPosition(0);
    }
    // public boolean isMoving() {
    //     return (Math.abs(motorLeft.getEncoder().getVelocity()) > 1.0) || (Math.abs(motorRight.getEncoder().getVelocity()) > 1.0);
    // }
}
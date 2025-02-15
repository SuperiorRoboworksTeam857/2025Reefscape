package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Wrist extends SubsystemBase {

  SparkMax motor = new SparkMax(WristConstants.wristMotor, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  private final AbsoluteEncoder m_absoluteEncoder;

  private static final double wristL2L3 = 0.78;
  private static final double wristL4 = 0.17;
  private static final double intake = 0.74;
  
  public enum w_Positions {
    WRIST_L2_L3,
    WRIST_L4,
    INTAKE,
  }

  private static double deltaTime = 0.02;
  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(1, 40);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(5, 0.0, 0.0, m_constraints, deltaTime);

  private double m_goalAngle = wristL2L3;

  public Wrist() {
    m_absoluteEncoder = motor.getAbsoluteEncoder();

    double encoderPositionFactor = (2 * Math.PI); // radians
    double encoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    config.encoder
      .positionConversionFactor(encoderPositionFactor)
      .velocityConversionFactor(encoderVelocityFactor);
    motor.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    m_controller.setGoal(m_goalAngle);
    motor.set(m_controller.calculate(m_absoluteEncoder.getPosition()));

    SmartDashboard.putNumber("wrist position", m_absoluteEncoder.getPosition());
    SmartDashboard.putNumber("wrist speed", m_absoluteEncoder.getVelocity());
  }

  public void goToAngle(w_Positions position) {
    switch (position) {
      case WRIST_L2_L3:
        m_goalAngle = wristL2L3;
        break;
      case WRIST_L4:
        m_goalAngle = wristL4;
        break;
      case INTAKE:
        m_goalAngle = intake;
        break;
    }
  }

  public boolean isWristAtGoal() {
    return Math.abs(m_absoluteEncoder.getPosition() - m_goalAngle) < 0.03;
  }

  public void resetEncoders() {
    motor.getEncoder().setPosition(0);
  }
}
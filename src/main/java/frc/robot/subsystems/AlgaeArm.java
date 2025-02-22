package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.WristConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeArm extends SubsystemBase {

  SparkMax motor = new SparkMax(AlgaeArmConstants.armMotor, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();

  private static final double raisedPosition = 60;
  private static final double horizontalPosition = 42;
  private static final double loweredPosition = 0;
  
  public enum a_Positions {
    RAISED,
    HORIZONTAL,
    LOWERED,
  }

  private static double deltaTime = 0.02;
  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(1, 40);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(5, 0.0, 0.0, m_constraints, deltaTime);

  private double m_goalAngle = loweredPosition;

  public AlgaeArm() {
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
    //motor.set(m_controller.calculate(motor.getEncoder().getPosition()));

    SmartDashboard.putNumber("arm position", motor.getEncoder().getPosition());
  }

  public void goToAngle(a_Positions position) {
    switch (position) {
      case LOWERED:
        m_goalAngle = loweredPosition;
        break;
      case HORIZONTAL:
        m_goalAngle = horizontalPosition;
        break;
      case RAISED:
        m_goalAngle = raisedPosition;
        break;
    }
  }

  public void resetEncoders() {
    motor.getEncoder().setPosition(0);
  }
}
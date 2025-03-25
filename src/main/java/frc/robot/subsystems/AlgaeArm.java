package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeArmConstants;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeArm extends SubsystemBase {

  SparkMax motor = new SparkMax(AlgaeArmConstants.armMotor, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();

  private static final double raisedPosition = 2.7;
  private static final double horizontalPosition = 1.8;
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
      new TrapezoidProfile.Constraints(10, 10);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(0.15, 0.0, 0.0, m_constraints, deltaTime);

  private a_Positions goalPosition = a_Positions.LOWERED;

  public AlgaeArm() {
    double gearRatio = 25;
    double encoderPositionFactor = (2 * Math.PI) / gearRatio; // radians
    double encoderVelocityFactor = (2 * Math.PI) / 60.0 / gearRatio; // radians per second

    config.encoder
      .positionConversionFactor(encoderPositionFactor)
      .velocityConversionFactor(encoderVelocityFactor);
    motor.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    double goalAngle = loweredPosition;

    switch (goalPosition) {
      case LOWERED:
        goalAngle = loweredPosition;
        break;
      case HORIZONTAL:
        goalAngle = horizontalPosition;
        break;
      case RAISED:
        goalAngle = raisedPosition;
        break;
    }

    m_controller.setGoal(goalAngle);
    motor.set(m_controller.calculate(motor.getEncoder().getPosition()));

    SmartDashboard.putNumber("arm position", motor.getEncoder().getPosition());
    SmartDashboard.putNumber("arm goal position", goalAngle);
  }

  public void goToAngle(a_Positions position) {
    goalPosition = position;
  }

  public void incrementHeight() {
    if (goalPosition == a_Positions.LOWERED) {
      goalPosition = a_Positions.HORIZONTAL;
    } else if (goalPosition == a_Positions.HORIZONTAL) {
      goalPosition = a_Positions.RAISED;
    }
  }

  public void decrementHeight() {
    if (goalPosition == a_Positions.RAISED) {
      goalPosition = a_Positions.HORIZONTAL;
    } else if (goalPosition == a_Positions.HORIZONTAL) {
      goalPosition = a_Positions.LOWERED;
    }
  }

  public void resetEncoders() {
    motor.getEncoder().setPosition(0);
  }
}
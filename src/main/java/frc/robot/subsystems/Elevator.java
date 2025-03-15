
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

    Servo cageServo = new Servo(ElevatorConstants.cageGrabberServo);
  
    // Range of motion of 0 inches at bottom to -24.5 inches at top
    public enum Positions {
      CORAL_STATION_L2,
      CORAL_STATION_L3,
      CORAL_STATION_L4,
      HUMANPLAYER_STATION,
      IDLE_MODE,
      L1
    }
  
    private double m_goalPosition = 0;

    public enum OperatingMode{
      CLIMBING_ELEVATOR_UP,
      CLIMBING_ELEVATOR_DOWN,
      SCORING,
    }

    private OperatingMode m_OperatingMode = OperatingMode.SCORING;
  
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

      switch(m_OperatingMode) {
        case SCORING:
          servoLeft.set(ElevatorConstants.servoLeftPressed);
          servoRight.set(ElevatorConstants.servoRightPressed);

          m_controller_left.setGoal(-m_goalPosition);
          motorLeft.set(m_controller_left.calculate(motorLeft.getEncoder().getPosition()));
          m_controller_right.setGoal(m_goalPosition);
          motorRight.set(m_controller_right.calculate(motorRight.getEncoder().getPosition()));
          break;

        case CLIMBING_ELEVATOR_UP:
          servoLeft.set(ElevatorConstants.servoLeftPressed);
          servoRight.set(ElevatorConstants.servoRightPressed);
          break;
        
        case CLIMBING_ELEVATOR_DOWN:
          servoLeft.set(ElevatorConstants.servoLeftReleased);
          servoRight.set(ElevatorConstants.servoRightReleased);
          break;
      } 
      
      SmartDashboard.putNumber("elevator position left", motorLeft.getEncoder().getPosition());
      SmartDashboard.putNumber("elevator speed left", motorLeft.getEncoder().getVelocity());
      SmartDashboard.putNumber("elevator position right", motorRight.getEncoder().getPosition());
      SmartDashboard.putNumber("elevator speed right", motorRight.getEncoder().getVelocity());
      SmartDashboard.putNumber("elevator ideal position", m_goalPosition);
    }
  
    public void goToPosition(Positions position) {
      setOperatingMode(OperatingMode.SCORING);

      switch (position) {
        case HUMANPLAYER_STATION:
          m_goalPosition = 0;
        case IDLE_MODE:
          m_goalPosition = 0;
          break;
        case CORAL_STATION_L2:
          m_goalPosition = 16;
          break;
        case CORAL_STATION_L3:
          m_goalPosition = 30.5;
          break;
        case CORAL_STATION_L4:
          m_goalPosition = 29;
          break;
        case L1:
          m_goalPosition = 4.2;
          break;
      }
    }

    public void setOperatingMode(OperatingMode operatingMode){
      m_OperatingMode = operatingMode;
    }
  
    public boolean isElevatorAtGoal() {
      return Math.abs(motorRight.getEncoder().getPosition() - m_goalPosition) < 1.0;
    }
  
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

  public void runElevatorForClimbing(double speed)
  {
    //positive speed and position are up for right motor

    speed = MathUtil.applyDeadband(speed, Constants.Swerve.stickDeadband);

    boolean movingUp = (speed > 0);
    boolean movingDown = (speed < 0);

    if ((motorRight.getEncoder().getPosition() > 0 && movingDown) ||
       (motorRight.getEncoder().getPosition() < 7 && movingUp)) {
      if (movingUp) {
        setOperatingMode(OperatingMode.CLIMBING_ELEVATOR_UP);
      } else if (movingDown) {
        setOperatingMode(OperatingMode.CLIMBING_ELEVATOR_DOWN);
      }

      motorLeft.set(-speed);
      motorRight.set(speed);
    }
    else {
      setOperatingMode(OperatingMode.CLIMBING_ELEVATOR_DOWN);
      motorLeft.set(0);
      motorRight.set(0);
    }
  }


  public void openCageGrabber(){
    cageServo.set(ElevatorConstants.cageServoVertical);
  }

  public void closeCageGrabber(){
    cageServo.set(ElevatorConstants.cageServoHorizontal);
  }
    /*
     * Climbing
     * 
     * Stick up, release ratchet, don't set goal, move elevator up, bounds limit elevator
     * Stick down, engage ratchet, don't set goal, move elevator down, bounds limit elevator
     * 
     * other buttons, release ratchet, set goal
     * 
     */
}

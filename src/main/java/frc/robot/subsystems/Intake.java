package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
public class Intake extends SubsystemBase {
  
//TO DO - set intake constant to device id
    private SparkFlex flexMotor = new SparkFlex(IntakeConstants.intakeMotor, MotorType.kBrushless);

  public Intake() {}

  @Override
  public void periodic() {}

  public void runIntake(double speed) {
flexMotor.set(speed*IntakeConstants.intakeSpeedMultiplier);
  }

  public void intakeGamePiece() {
    runIntake(-1.0);
  }

  public void outtakeGamePiece() {
    runIntake(1.0);
  }

}

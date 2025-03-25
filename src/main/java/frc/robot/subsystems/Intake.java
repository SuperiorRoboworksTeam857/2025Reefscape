package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
public class Intake extends SubsystemBase {
  
  private SparkFlex flexMotor = new SparkFlex(IntakeConstants.intakeMotor, MotorType.kBrushless);

  private DigitalInput beamBreak = new DigitalInput(IntakeConstants.beamBreak);

  private final Wrist s_Wrist;


  public Intake(Wrist wrist) {
    this.s_Wrist = wrist;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Is Coral In Intake?", isCoralInIntake());
  }

  public boolean isCoralInIntake(){
    return !beamBreak.get();
  }

  public void runIntake(double speed) {
    flexMotor.set(speed*IntakeConstants.intakeSpeedMultiplier);
  }

  public void autoIntake(){
    if (isCoralInIntake()){
      stopIntake();
    } else {
      runIntake(-1.0);
    }
  }

  public void intakeGamePiece() {
    runIntake(-1.0);
  }

  public void intakeHoldGamePiece(){
    runIntake(0.5);
  }

  public void reverseIntake(){
    double intakeSpeed = s_Wrist.invertIntake() ? -5.0 : 5.0;
    runIntake(intakeSpeed);
  }

  public void outtakeGamePiece() {
    if (s_Wrist.isWristAtGoal()) {
      double intakeSpeed = s_Wrist.invertIntake() ? 1.0 : -1.0;
      runIntake(intakeSpeed);
    } else {
      stopIntake();
    }
  }

  public void stopIntake() {
    runIntake(0.0);
  }

}
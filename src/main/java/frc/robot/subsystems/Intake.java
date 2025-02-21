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

  public Intake() {}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Is Coral In Intake?", isCoralInIntake());
  }

  public boolean isCoralInIntake() {
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

  // TODO: make aware of L4 because outtaking is normally same as intaking except at L4 wrist position
  public void outtakeGamePiece() {
    runIntake(1.0);
  }

  public void stopIntake() {
    runIntake(0.0);
  }

}

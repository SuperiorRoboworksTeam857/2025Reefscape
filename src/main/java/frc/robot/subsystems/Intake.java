package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
public class Intake extends SubsystemBase {
  
  private SparkFlex flexMotor = new SparkFlex(IntakeConstants.intakeMotor, MotorType.kBrushless);

  private DigitalInput beamBreak = new DigitalInput(IntakeConstants.beamBreak);

  public Intake() {}

  private boolean intakeHasCoral = false;

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Is Coral In Intake?", intakeHasCoral);
    SmartDashboard.putBoolean("Is Coral Beam Broken?", hasBeamBreakTriggered());

    // Various operations
    // 1. Beam Break is Broken
    // 2. Intake reverses (and says it has coral)
    // 3. Intake has game piece, but not at beam break. Intake stops
    // 4. Button outtakes game piece and sets the variable to false

    if(hasBeamBreakTriggered()){
      if(intakeHasCoral){
        intakeHoldGamePiece();
      }
      intakeHasCoral = true;
    }else{
      if(!intakeHasCoral){
        intakeGamePiece();
      }else{
        stopIntake();
      }
    }

  }

  public boolean isCoralInIntake(){
    return intakeHasCoral;
  }

  public boolean hasBeamBreakTriggered() {
    return !beamBreak.get();
  }

  public void runIntake(double speed) {
    flexMotor.set(speed*IntakeConstants.intakeSpeedMultiplier);
  }

  public void autoIntake(){
    if (hasBeamBreakTriggered()){
      intakeHasCoral = true;
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
    runIntake(3.0);
  }

  // TODO: make aware of L4 because outtaking is normally same as intaking except at L4 wrist position
  public void outtakeGamePiece() {
    runIntake(-1.0);
    intakeHasCoral = false;
  }

  public void stopIntake() {
    runIntake(0.0);
  }

}
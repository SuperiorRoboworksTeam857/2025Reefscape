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

  private final Wrist s_Wrist;


  public Intake(Wrist wrist) {
    this.s_Wrist = wrist;
  }

 // private boolean intakeHasCoral = false;

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Is Coral In Intake?", isCoralInIntake());
    //SmartDashboard.putBoolean("Is Coral Beam Broken?", hasBeamBreakTriggered());

    // Various operations
    // 1. Beam Break is Broken
    // 2. Intake reverses (and says it has coral)
    // 3. Intake has game piece, but not at beam break. Intake stops
    // 4. Button outtakes game piece and sets the variable to false

    // if(hasBeamBreakTriggered()){
    //   if(intakeHasCoral){
    //     intakeHoldGamePiece();
    //   }
    //   intakeHasCoral = true;
    // }else{
    //   if(!intakeHasCoral){
    //     intakeGamePiece();
    //   }else{
    //     stopIntake();
    //   }
    // }

  }

  public boolean isCoralInIntake(){
    return !beamBreak.get();
  }

  public void runIntake(double speed) {
    flexMotor.set(speed*IntakeConstants.intakeSpeedMultiplier);
  }

  public void autoIntake(){
    if (isCoralInIntake()){
      //intakeHasCoral = true;
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
    double intakeSpeed = s_Wrist.invertIntake() ? 1.0 : -1.0;
    runIntake(intakeSpeed);
    //intakeHasCoral = false;
  }

  public void stopIntake() {
    runIntake(0.0);
  }

}
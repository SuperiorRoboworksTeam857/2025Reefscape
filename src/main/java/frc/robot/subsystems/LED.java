/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  // Ask Chris for the "PWM channel" number (it might not be 0)
  Spark ledStrip = new Spark(2);

  // LED Strip Lights:
  // Default Status: Solid Blue (0.83)
  // Coral in Intake: Blinking Yellow (-0.07) - Need to be green maybe - use
  // screwdriver with dial
  // Turn to Reef Mode: Solid Violet (0.91)
  // Not Within Tolerance: Blinking Red (-0.11)

  private final Intake s_Intake;
  private final Limelight s_Limelight;
  private final BooleanSupplier isRobotCentric;

  // Change these variables to reflect the values
  public static final double BLUE_LIGHTS = 0.83;
  public static final double STROBE_GREEN = 0.35;
  public static final double VIOLET_LIGHTS = 0.91;
  public static final double STROBE_RED = -0.11;

  // Any subsystems being passed in should have data inside of it
  // For example, s_Intake.isCoralInIntake().
  public LED(Intake intake, Limelight limelight, BooleanSupplier robotCentric) {
    // setup each of the subsystems necessary
    this.s_Intake = intake;
    this.isRobotCentric = robotCentric;
    this.s_Limelight = limelight;
  }

  @Override
  public void periodic() {
    double lightPattern;

    SmartDashboard.putBoolean("LED - isReefAprilTagValid", s_Limelight.getIsReefAprilTagValid());

    // You need to add additional if statements below for each LED color
    if (s_Intake.isCoralInIntake()) {
      SmartDashboard.putString("LED - color", "Strobe Green");
      lightPattern = STROBE_GREEN;
    } else if (isRobotCentric.getAsBoolean()) {

      if (s_Limelight.getIsReefAprilTagValid()) {
        SmartDashboard.putString("LED - color", "Solid Violet");
        lightPattern = VIOLET_LIGHTS;
      } else {
        SmartDashboard.putString("LED - color", "Strobe Red");
        lightPattern = STROBE_RED;
      }

    } else {
      SmartDashboard.putString("LED - color", "Solid Blue");
      lightPattern = BLUE_LIGHTS;
    }

    ledStrip.set(lightPattern);
  }

  public Spark getLEDController() {
    return ledStrip;
  }
}
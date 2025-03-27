// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveToScoringLocation;
import frc.robot.commands.LimelightRead;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurnToReefCommand;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeArm.a_Positions;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.w_Positions;
import frc.robot.subsystems.Elevator.Positions;
import frc.robot.subsystems.Limelight;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick gamepad = new Joystick(0);
  private final Joystick driverStick = new Joystick(1);
  private final Joystick buttonBox = new Joystick(2);

  /* Drive Controls */
  private final int translationAxis = Joystick.AxisType.kY.value;
  private final int strafeAxis = Joystick.AxisType.kX.value;
  private final int rotationAxis = Joystick.AxisType.kZ.value;

  /* Driver Buttons */
  private final JoystickButton robotCentric = new JoystickButton(driverStick, 4);
  private final JoystickButton zeroGyro = new JoystickButton(driverStick, 3);

  private final JoystickButton slowSpeed = new JoystickButton(driverStick, 2);
  private final JoystickButton highSpeed = new JoystickButton(driverStick,1);

  private final JoystickButton alignToReef = new JoystickButton(driverStick, 5);
  private final JoystickButton alignToLeftPoleL4 = new JoystickButton(driverStick, 6);
  private final JoystickButton alignToRightPoleL4 = new JoystickButton(driverStick, 7);
  private final JoystickButton alignToLeftPoleL2L3 = new JoystickButton(driverStick, 9);
  private final JoystickButton alignToRightPoleL2L3 = new JoystickButton(driverStick, 8);
  /* Gamepad Buttons */
  private final POVButton elevatorCoralStation = new POVButton(gamepad, 270);  // d-pad left
  private final POVButton elevatorL2 = new POVButton(gamepad, 180);  // d-pad down
  private final POVButton elevatorL3_L4 = new POVButton(gamepad, 90);  // d-pad right

  private final JoystickButton dumpClimb = new JoystickButton(gamepad, XboxController.Button.kY.value);
  private final POVButton dumpL4 = new POVButton(gamepad, 0);  // d-pad up

  private final Trigger raiseAlgaeArm = new Trigger(() -> Math.abs( gamepad.getRawAxis(XboxController.Axis.kLeftTrigger.value) ) > 0.1);
  private final JoystickButton lowerAlgaeArm = new JoystickButton(gamepad, XboxController.Button.kLeftBumper.value);
  
  private final Trigger scoreCoral = new Trigger(() -> Math.abs( gamepad.getRawAxis(XboxController.Axis.kRightTrigger.value) ) > 0.1);
  private final JoystickButton reverseIntake = new JoystickButton(gamepad, XboxController.Button.kRightBumper.value);

  private final JoystickButton closeCageGrabber = new JoystickButton(gamepad, XboxController.Button.kA.value);
  private final JoystickButton openCageGrabber = new JoystickButton(gamepad, XboxController.Button.kB.value);

  

  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public final Elevator s_Elevator = new Elevator();
  public final Wrist s_Wrist = new Wrist();
  public final Intake s_Intake = new Intake(s_Wrist);
  public final AlgaeArm s_Arm = new AlgaeArm();
  public final Limelight s_Limelight = new Limelight();
  public final LED s_LED;

  public final AprilTagFieldLayout layout;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);


    // Start camera streams for both webcams
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();

    s_Limelight.turnOnDriverCam();
    s_Limelight.enableLimelight(false);
    s_Limelight.setPipeline(Limelight.Pipeline.AprilTags);

    s_Elevator.closeCageGrabber();

    s_LED = new LED(s_Intake, s_Limelight, () -> alignToReef.getAsBoolean());

    // Configure the NamedCommands
    NamedCommands.registerCommand("scoreCoral",
      new ParallelRaceGroup(
        new RunCommand(() -> s_Intake.outtakeGamePiece(), s_Intake),
        new WaitCommand(1)
      ).asProxy()
    );

    NamedCommands.registerCommand("waitUntilCoralInIntake",
      new WaitUntilCommand(s_Intake::isCoralInIntake)
    );
    
    NamedCommands.registerCommand("lowerElevator",
      new SequentialCommandGroup(
        new InstantCommand(() -> s_Elevator.goToPosition(Positions.HUMANPLAYER_STATION), s_Elevator),
        new WaitUntilCommand(s_Elevator::isElevatorAtGoal)
      )
    );
    NamedCommands.registerCommand("elevatorL1",
      new SequentialCommandGroup(
        new InstantCommand(() -> s_Elevator.goToPosition(Positions.L1), s_Elevator),
        new InstantCommand(() -> s_Wrist.goToAngle(w_Positions.WRIST_L2_L3),s_Wrist),
        new WaitUntilCommand(s_Elevator::isElevatorAtGoal)
      )
    );
    NamedCommands.registerCommand("elevatorL2",
      new SequentialCommandGroup(
        new InstantCommand(() -> s_Elevator.goToPosition(Positions.CORAL_STATION_L2), s_Elevator),
        new InstantCommand(() -> s_Wrist.goToAngle(w_Positions.WRIST_L2_L3),s_Wrist),
        new WaitUntilCommand(s_Elevator::isElevatorAtGoal)
      )
    );
    NamedCommands.registerCommand("elevatorL3_L4",
      new SequentialCommandGroup(
        new InstantCommand(() -> s_Elevator.goToPosition(Positions.CORAL_STATION_L3), s_Elevator),
        new InstantCommand(() -> s_Wrist.goToAngle(w_Positions.WRIST_L2_L3),s_Wrist),
        new WaitUntilCommand(s_Elevator::isElevatorAtGoal)
      )
    );

    NamedCommands.registerCommand("wristIntake",
      new SequentialCommandGroup(
        new InstantCommand(() -> s_Wrist.goToAngle(w_Positions.INTAKE),s_Wrist),
        new WaitUntilCommand(s_Wrist::isWristAtGoal)
      )
    );
    NamedCommands.registerCommand("wristIntakeUntilClear",
      new SequentialCommandGroup(
        new InstantCommand(() -> s_Wrist.goToAngle(w_Positions.INTAKE),s_Wrist),
        new WaitUntilCommand(s_Wrist::isWristClearOfReef)
      )
    );

    NamedCommands.registerCommand("wristL4",
      new SequentialCommandGroup(
        new InstantCommand(() -> s_Wrist.goToAngle(w_Positions.WRIST_L4),s_Wrist),
        new WaitUntilCommand(s_Wrist::isWristAtGoal)
      )
    );
    NamedCommands.registerCommand("wristVertical",
      new SequentialCommandGroup(
        new InstantCommand(() -> s_Wrist.goToAngle(w_Positions.VERTICAL_FOR_AUTO),s_Wrist),
        new WaitUntilCommand(s_Wrist::isWristAtGoal)
      )
    );

    NamedCommands.registerCommand("lowerAlgaeArm",
      new SequentialCommandGroup(
        new InstantCommand(() -> s_Arm.goToAngle(a_Positions.LOWERED), s_Arm)
      )
    );

    NamedCommands.registerCommand("raiseAlgaeArm",
      new SequentialCommandGroup(
        new InstantCommand(() -> s_Arm.goToAngle(a_Positions.RAISED), s_Arm)
      )
    );

    NamedCommands.registerCommand("driveForwardRobotCentric",
      new RunCommand(() -> s_Swerve.drive(new Translation2d(0.2, 0).times(Constants.Swerve.maxSpeed), 0, false, true))
    );
    NamedCommands.registerCommand("driveRightRobotCentric",
      new RunCommand(() -> s_Swerve.drive(new Translation2d(0, -0.2).times(Constants.Swerve.maxSpeed), 0, false, true))
    );
    NamedCommands.registerCommand("stopDriving",
      new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 0, false, true))
    );

    NamedCommands.registerCommand("driveToL4Left",
      new DriveToScoringLocation(s_Swerve, layout, DriveToScoringLocation.Location.LEFT_L4)
    );
    NamedCommands.registerCommand("driveToL4Right",
      new DriveToScoringLocation(s_Swerve, layout, DriveToScoringLocation.Location.RIGHT_L4)
    );
    
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driverStick.getRawAxis(translationAxis),
            () -> -driverStick.getRawAxis(strafeAxis),
            () -> -driverStick.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean(),
            () -> slowSpeed.getAsBoolean(),
            () -> highSpeed.getAsBoolean(),
            () -> alignToReef.getAsBoolean()));

    s_Limelight.setDefaultCommand(new LimelightRead(s_Limelight));

    s_Intake.setDefaultCommand(new RunCommand(() -> s_Intake.autoIntake(), s_Intake));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    alignToReef.whileTrue(new TurnToReefCommand(s_Swerve, driverStick));

    alignToLeftPoleL4.whileTrue(new DriveToScoringLocation(s_Swerve, layout, DriveToScoringLocation.Location.LEFT_L4));
    alignToRightPoleL4.whileTrue(new DriveToScoringLocation(s_Swerve, layout, DriveToScoringLocation.Location.RIGHT_L4));
    alignToLeftPoleL2L3.whileTrue(new DriveToScoringLocation(s_Swerve, layout, DriveToScoringLocation.Location.LEFT_L2_L3));
    alignToRightPoleL2L3.whileTrue(new DriveToScoringLocation(s_Swerve, layout, DriveToScoringLocation.Location.RIGHT_L2_L3));

    // Elevator
    elevatorCoralStation.whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(
          () -> s_Elevator.goToPosition(Positions.HUMANPLAYER_STATION),s_Elevator
        ),
        new InstantCommand(
        () -> s_Wrist.goToAngle(w_Positions.INTAKE),s_Wrist
        )
      )
    );
    elevatorL2.whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(
          () -> s_Elevator.goToPosition(Positions.CORAL_STATION_L2),s_Elevator
        ),
        new InstantCommand(
        () -> s_Wrist.goToAngle(w_Positions.WRIST_L2_L3),s_Wrist
        )
      )
    );
    elevatorL3_L4.whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(
          () -> s_Elevator.goToPosition(Positions.CORAL_STATION_L3),s_Elevator
        ),
        new InstantCommand(
        () -> s_Wrist.goToAngle(w_Positions.WRIST_L2_L3),s_Wrist
        )
      )
    );

    // Wrist
    dumpClimb.whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(
        () -> s_Wrist.goToAngle(w_Positions.CLIMB_FINAL),s_Wrist
        )
      )
    );
    dumpL4.whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(
        () -> s_Wrist.goToAngle(w_Positions.WRIST_L4),s_Wrist
        )
      )
    );

    // Elevator
    new Trigger(() -> Math.abs( gamepad.getRawAxis(XboxController.Axis.kLeftY.value) ) > 0.1)
        .whileTrue(new RunCommand(() -> s_Elevator.runElevatorForClimbing(-gamepad.getRawAxis(XboxController.Axis.kLeftY.value)), s_Elevator))
        .onFalse(new InstantCommand(() -> s_Elevator.runElevatorForClimbing(0)));
    closeCageGrabber.whileTrue(new RunCommand(() -> s_Elevator.closeCageGrabber(), s_Elevator));
    openCageGrabber.whileTrue(new RunCommand(() -> s_Elevator.openCageGrabber(), s_Elevator));
   
    lowerAlgaeArm.onTrue(new InstantCommand(() -> s_Arm.decrementHeight(), s_Arm));
    raiseAlgaeArm.onTrue(new InstantCommand(() -> s_Arm.incrementHeight(), s_Arm));

    // Intake
    reverseIntake.whileTrue(new RunCommand(() -> s_Intake.reverseIntake(), s_Intake));
    scoreCoral.whileTrue(new RunCommand(() -> s_Intake.outtakeGamePiece(), s_Intake));
  }
    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if(buttonBox.getRawButton(6)) {
      if (buttonBox.getRawButton(3)){
        return new PathPlannerAuto("L1 Left");
      } else if (buttonBox.getRawButton(4)){
        return new PathPlannerAuto("L1 Center");
      } else if (buttonBox.getRawButton(5)){
        return new PathPlannerAuto("L1 Right");
      }
    } else if(buttonBox.getRawButton(7)) {
      if (buttonBox.getRawButton(3)){
        return new PathPlannerAuto("L4 Left");
      } else if (buttonBox.getRawButton(4)){
        //
      } else if (buttonBox.getRawButton(5)){
        return new PathPlannerAuto("L4 Right");
      }
    } else {
      if (buttonBox.getRawButton(3)){
        return new PathPlannerAuto("L2 Left");
      } else if (buttonBox.getRawButton(4)){
        return new PathPlannerAuto("L4 Center");
      } else if (buttonBox.getRawButton(5)){
        return new PathPlannerAuto("L2 Right");
      }
    }

    return new ParallelRaceGroup(
      new RunCommand(() -> s_Swerve.drive(new Translation2d(0.2, 0).times(Constants.Swerve.maxSpeed), 0, false, true), s_Swerve),
      new WaitCommand(1)
    );
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
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
import frc.robot.commands.LimelightRead;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurnToReefCommand;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.Positions;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.w_Positions;


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

  
  private final JoystickButton aligntoReef = new JoystickButton(driverStick, 5);

<<<<<<< HEAD
  /* Operator Buttons */
  // new Trigger(() -> gamepad.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.5).whileTrue(new RunCommand(() -> AlgaeArm.goToAngle(), AlgaeArm));
  private final JoystickButton algaeDown = new JoystickButton(gamepad,XboxController.Button.kLeftBumper.value);
  private final JoystickButton reverseIntake = new JoystickButton(gamepad,XboxController.Button.kRightBumper.value);

=======
  /* Gamepad Buttons */
  private final POVButton elevatorCoralStation = new POVButton(gamepad, 180);  // d-pad down
  private final POVButton elevatorL2 = new POVButton(gamepad, 90);  // d-pad right
  private final POVButton elevatorL3_L4 = new POVButton(gamepad, 270);  // d-pad left

  private final POVButton dumpL4 = new POVButton(gamepad, 0);  // d-pad up

  private final JoystickButton dumpClimb = new JoystickButton(gamepad, XboxController.Button.kY.value);

  private final JoystickButton closeCageGrabber = new JoystickButton(gamepad, XboxController.Button.kRightBumper.value);
  private final JoystickButton openCageGrabber = new JoystickButton(gamepad, XboxController.Button.kLeftBumper.value);

  private final JoystickButton lowerAlgaeArm = new JoystickButton(gamepad, XboxController.Button.kA.value);
  private final JoystickButton horizontalAlgaeArm = new JoystickButton(gamepad, XboxController.Button.kB.value);
  private final JoystickButton raiseAlgaeArm = new JoystickButton(gamepad, XboxController.Button.kY.value);

  private final JoystickButton reverseIntake = new JoystickButton(gamepad, XboxController.Button.kA.value);
  private final JoystickButton outtakeCoral = new JoystickButton(gamepad, XboxController.Button.kB.value);
>>>>>>> f2a53d0f93c5612644ffe56115a2964220fbb365

  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public final Elevator s_Elevator = new Elevator();
  public final Wrist s_Wrist = new Wrist();
  public final Intake s_Intake = new Intake(s_Wrist);
  public final AlgaeArm s_Arm = new AlgaeArm();
  public final Limelight s_Limelight = new Limelight();
  public final LED s_LED;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Start camera streams for both webcams
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();

    s_Limelight.turnOnDriverCam();
    s_Limelight.enableLimelight(false);
    s_Limelight.setPipeline(Limelight.Pipeline.AprilTags);

    s_Elevator.openCageGrabber();

    s_LED = new LED(s_Intake, s_Limelight, () -> aligntoReef.getAsBoolean());

    // Configure the NamedCommands
    NamedCommands.registerCommand("scoreCoral",
      new ParallelRaceGroup(
        new RunCommand(() -> s_Intake.outtakeGamePiece(), s_Intake),
        new WaitCommand(1)
      ).asProxy()
    );
    
    NamedCommands.registerCommand("lowerElevator",
      new SequentialCommandGroup(
        new InstantCommand(() -> s_Elevator.goToPosition(Positions.HUMANPLAYER_STATION), s_Elevator),
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
    NamedCommands.registerCommand("wristL4",
      new SequentialCommandGroup(
        new InstantCommand(() -> s_Wrist.goToAngle(w_Positions.WRIST_L4),s_Wrist),
        new WaitUntilCommand(s_Wrist::isWristAtGoal)
      )
    );


    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driverStick.getRawAxis(translationAxis),
            () -> -driverStick.getRawAxis(strafeAxis),
            () -> -driverStick.getRawAxis(rotationAxis),
            () -> false, // NEEDS TO BE CHECKED
            () -> robotCentric.getAsBoolean(),
            () -> slowSpeed.getAsBoolean(),
            () -> highSpeed.getAsBoolean(),
            () -> aligntoReef.getAsBoolean()));

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
    // TO DO:
    // Read through 2024 code on GitHub on Triggers and POV Buttons (POV Buttons = D-Pad Buttons)

    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    // tried to make it scan for tags
    // aligntoReef.whileTrue(
    //   new RepeatCommand(
    //     new TurnToReefCommand(s_Swerve, s_Limelight, driverStick, 5000)
    //   )
    // );
    aligntoReef.whileTrue(new TurnToReefCommand(s_Swerve, s_Limelight, driverStick, 5000));
    //new JoystickButton(driverStick, 4).whileTrue(new RunCommand(() -> s_Swerve.setX(), s_Swerve));

    // Limelight Controls
    // new JoystickButton(gamepad, XboxController.Button.kStart.value).whileTrue(
    //   new InstantCommand(
    //     () -> s_Limelight.setPipeline(Limelight.Pipeline.AprilTags)));

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
    dumpL4.whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(
        () -> s_Wrist.goToAngle(w_Positions.WRIST_L4),s_Wrist
        )
      )
    );
    dumpClimb.whileTrue(new InstantCommand(() -> s_Wrist.goToAngle(w_Positions.CLIMB_FINAL),s_Wrist));

    // Elevator
    new Trigger(() -> Math.abs( gamepad.getRawAxis(XboxController.Axis.kLeftY.value) ) > 0.1)
        .whileTrue(new RunCommand(() -> s_Elevator.runElevatorForClimbing(-gamepad.getRawAxis(XboxController.Axis.kLeftY.value)), s_Elevator))
        .onFalse(new InstantCommand(() -> s_Elevator.runElevatorForClimbing(0)));
    closeCageGrabber.whileTrue(new RunCommand(() -> s_Elevator.closeCageGrabber(), s_Elevator));
    openCageGrabber.whileTrue(new RunCommand(() -> s_Elevator.openCageGrabber(), s_Elevator));
   
    //Algae Arm
    // lowerAlgaeArm.whileTrue(new RunCommand(() -> s_Arm.goToAngle(a_Positions.LOWERED), s_Arm));
    // horizontalAlgaeArm.whileTrue(new RunCommand(() -> s_Arm.goToAngle(a_Positions.HORIZONTAL), s_Arm));
    // raiseAlgaeArm.whileTrue(new RunCommand(() -> s_Arm.goToAngle(a_Positions.RAISED), s_Arm));

    // Intake
    reverseIntake.whileTrue(new RunCommand(() -> s_Intake.reverseIntake(), s_Intake));
    outtakeCoral.whileTrue(new RunCommand(() -> s_Intake.outtakeGamePiece(), s_Intake));
  }
    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (buttonBox.getRawButton(3)){
      return new PathPlannerAuto("L2 Left");
    } else if (buttonBox.getRawButton(4)){
      return new PathPlannerAuto("L4 Center");
    } else if (buttonBox.getRawButton(5)){
      return new PathPlannerAuto("L2 Right");
    } else if (buttonBox.getRawButton(6)){
      return new PathPlannerAuto("test auto");
    }
    return new PathPlannerAuto("Leave Auto");
  }
}
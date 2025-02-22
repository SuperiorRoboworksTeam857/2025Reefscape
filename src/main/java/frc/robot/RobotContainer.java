// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.ArmAuton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.w_Positions;
import frc.robot.subsystems.Elevator.Positions;
import frc.robot.subsystems.Limelight;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public final Elevator s_Elevator = new Elevator();
  public final Wrist s_Wrist = new Wrist();
  public final Intake s_Intake = new Intake();
  public final AlgaeArm s_Arm = new AlgaeArm();
  public final Limelight s_limelight = new Limelight();


  /* Autonomous Chooser */
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Start camera streams for both webcams
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();

    s_limelight.turnOnDriverCam();
    s_limelight.enableLimelight(false);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Selector:", autoChooser);

    // Configure the NamedCommands
    NamedCommands.registerCommand("runIntakeIn", new InstantCommand(() -> s_Intake.intakeGamePiece()));

    NamedCommands.registerCommand("runIntakeOut", new InstantCommand(() -> s_Intake.outtakeGamePiece()));

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driverStick.getRawAxis(translationAxis),
            () -> -driverStick.getRawAxis(strafeAxis),
            () -> -driverStick.getRawAxis(rotationAxis),
            () -> false, // NEEDS TO BE CHECKED
            () -> robotCentric.getAsBoolean(),
            () -> slowSpeed.getAsBoolean(),
            () -> highSpeed.getAsBoolean()));

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

    new JoystickButton(driverStick, 4).whileTrue(new RunCommand(() -> s_Swerve.setX(), s_Swerve));

    // Limelight controls
    new JoystickButton(gamepad, XboxController.Button.kStart.value).whileTrue(
      new InstantCommand(
        () -> s_limelight.setPipeline(Limelight.Pipeline.AprilTags)));

    // Elevator
    new POVButton(gamepad, 180).whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(
          () -> s_Elevator.goToPosition(Positions.HUMANPLAYER_STATION),s_Elevator
        ),
        new InstantCommand(
        () -> s_Wrist.goToAngle(w_Positions.INTAKE),s_Wrist
        )
      )
    );
    new POVButton(gamepad, 90).whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(
          () -> s_Elevator.goToPosition(Positions.CORAL_STATION_L2),s_Elevator
        ),
        new InstantCommand(
        () -> s_Wrist.goToAngle(w_Positions.WRIST_L2_L3),s_Wrist
        )
      )
    );
    new POVButton(gamepad, 270).whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(
          () -> s_Elevator.goToPosition(Positions.CORAL_STATION_L3),s_Elevator
        ),
        new InstantCommand(
        () -> s_Wrist.goToAngle(w_Positions.WRIST_L2_L3),s_Wrist
        )
      )
    );

    // new POVButton(gamepad, 0).whileTrue(
    //   new SequentialCommandGroup(
    //     new InstantCommand(
    //       () -> s_Elevator.goToPosition(Positions.CORAL_STATION_L4),s_Elevator
    //     ),
    //     new InstantCommand(
    //     () -> s_Wrist.goToAngle(w_Positions.WRIST_L2_L3),s_Wrist // THIS SHOULD NOT BE L4
    //     )
    //   )
    // );

    new POVButton(gamepad, 0).whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(
        () -> s_Wrist.goToAngle(w_Positions.WRIST_L4),s_Wrist
        )
      )
    );

    new JoystickButton(gamepad, XboxController.Button.kY.value).whileTrue(new InstantCommand(() -> s_Wrist.goToAngle(w_Positions.CLIMB_FINAL),s_Wrist));




    new Trigger(() -> Math.abs( gamepad.getRawAxis(XboxController.Axis.kLeftY.value) ) > 0.1)
        .whileTrue(new RunCommand(() -> s_Elevator.runElevatorForClimbing(-gamepad.getRawAxis(XboxController.Axis.kLeftY.value)), s_Elevator))
        .onFalse(new InstantCommand(() -> s_Elevator.runElevatorForClimbing(0)));

    new JoystickButton(gamepad, XboxController.Button.kRightBumper.value).whileTrue(new RunCommand(() -> s_Elevator.openCageGrabber(), s_Elevator));
    new JoystickButton(gamepad, XboxController.Button.kLeftBumper.value).whileTrue(new RunCommand(() -> s_Elevator.closeCageGrabber(), s_Elevator));



    // Intake
    new JoystickButton(gamepad, XboxController.Button.kA.value).whileTrue(new RunCommand(() -> s_Intake.intakeGamePiece(), s_Intake));

    new JoystickButton(gamepad, XboxController.Button.kB.value).whileTrue(new RunCommand(() -> s_Intake.outtakeGamePiece(), s_Intake));

  }
    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // if (buttonBox.getRawButton(3) && buttonBox.getRawButton(4) && buttonBox.getRawButton(5) &&
    //     buttonBox.getRawButton(6) && buttonBox.getRawButton(7)) {
    //   return new PathPlannerAuto("4 note center at distance amp first");
    // } else if (buttonBox.getRawButton(3) && buttonBox.getRawButton(4) && buttonBox.getRawButton(5) &&
    //            buttonBox.getRawButton(6)) {
    //   return new PathPlannerAuto("4 note center at distance source first");
    // } else if (buttonBox.getRawButton(3)) {
    //   return new PathPlannerAuto("2 note center");
    // } else {
    //   if (buttonBox.getRawButton(4)) {
    //     // amp side
    //     if (buttonBox.getRawButton(5)) {
    //       return new PathPlannerAuto("2 note amp side");
    //     } else if (buttonBox.getRawButton(6)) {
    //       return new PathPlannerAuto("3 note center amp side");
    //     } else if (buttonBox.getRawButton(7)) {
    //       return new PathPlannerAuto("2 note amp side straight to centerline");
    //     }
    //   } else {
    //     // source side
    //     if (buttonBox.getRawButton(5)) {
    //       return new PathPlannerAuto("2 note source side");
    //     } else if (buttonBox.getRawButton(6)) {
    //       return new PathPlannerAuto("3 note center source side");
    //     } else if (buttonBox.getRawButton(7)) {
    //       return new PathPlannerAuto("2 note source side straight to centerline");
    //     }
    //   }
    // }

    // return new PathPlannerAuto("just shoot");

    // Use the autoChooser to choose an autonomous
    return autoChooser.getSelected();
  }
}
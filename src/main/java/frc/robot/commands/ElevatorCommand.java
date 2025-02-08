// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  private final Elevator elevator;
  private Supplier<Double> elevatorSpeed;
  
  /** Creates a new ElevatorCommand. */
  public ElevatorCommand(Elevator elevator, Supplier<Double> elevatorSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.elevatorSpeed = elevatorSpeed;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // elevator.extension(elevatorSpeed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // elevator.extension(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

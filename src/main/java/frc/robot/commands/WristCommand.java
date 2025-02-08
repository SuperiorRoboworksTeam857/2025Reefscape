// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristCommand extends Command {
  private final Wrist wrist;
  private boolean rotation;

  public WristCommand(Wrist wrist, boolean rotation) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.wrist = wrist;
      this.rotation = rotation;
      addRequirements(wrist);
  }

  @Override
  public void initialize() {
  }

  public void execute() {
      // if (!rotation) {
      //     wrist.rotation(0.4);
      // } else {
      //     wrist.rotation(-0.4);
      //     ;
      // }
  }

  @Override
  public boolean isFinished() {
      return false;
  }

  @Override
  public void end(boolean interrupted) {
      // wrist.stop();
  }
}

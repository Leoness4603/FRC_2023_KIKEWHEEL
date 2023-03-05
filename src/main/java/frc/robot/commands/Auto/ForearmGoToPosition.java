// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Forearm;

public class ForearmGoToPosition extends CommandBase {
  /** Creates a new ForearmGoToPosition. */
  Forearm forearm;
  double setpoint;
  public ForearmGoToPosition(Forearm forearm, double setpoint) {
    this.forearm = forearm;
    this.setpoint = setpoint;
    addRequirements(forearm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forearm.getForearmController().setTolerance(3);
    forearm.getForearmController().setSetpoint(setpoint);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    forearm.ForearmMovement(forearm.getForearmController().calculate(forearm.getForearmAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

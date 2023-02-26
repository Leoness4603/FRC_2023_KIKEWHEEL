// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Forearm;

public class MoveForearm extends CommandBase {
  /** Creates a new MoveForearm. */
  private Forearm forearmSubsystem;
  private double turn;

  public MoveForearm(Forearm forearmSubsystem) {
    this.forearmSubsystem = forearmSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(forearmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turn = forearmSubsystem.getJoystick().getRawAxis(1) * .45;
    forearmSubsystem.log();
    forearmSubsystem.ForearmMovement(turn);
    if(forearmSubsystem.getDistanceArm() >= 5){
         forearmSubsystem.ForearmMovement(turn -.1);
    } else{
      if(forearmSubsystem.getDistanceArm() <= -5){
        forearmSubsystem.ForearmMovement(turn +.1);
      }
    }
  }
//hola j
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

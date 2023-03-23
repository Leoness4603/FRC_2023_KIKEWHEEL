// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.SubsystemMaxOutput;

public class MoveArm extends CommandBase {
  private Arm armSubsystem;
  private double turn;
  private double maxOutput;

  /** Creates a new SpinIntakeWheels. */
  public MoveArm(Arm armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      turn = armSubsystem.getJoystick().getY();
      maxOutput = SubsystemMaxOutput.kArm_MaxOutput;
      if (armSubsystem.getJoystick().getY() > .08 || armSubsystem.getJoystick().getY() < .08) {
        armSubsystem.ArmMovement(-turn * maxOutput); 
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

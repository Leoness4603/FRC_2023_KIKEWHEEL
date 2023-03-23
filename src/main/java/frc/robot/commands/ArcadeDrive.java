// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  private final Chassis chassisSubsystem;
  private Joystick joy;

  public ArcadeDrive(Chassis chassisSubsystem, Joystick joy) {
    this.chassisSubsystem = chassisSubsystem;
    this.joy = joy;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (joy.getRawAxis(3) > 0) {
      chassisSubsystem.driveA(-joy.getRawAxis(1), joy.getRawAxis(4) * .45, 1);
    }else{
      chassisSubsystem.driveA(-joy.getRawAxis(1), joy.getRawAxis(4) * .45, .65);
    }*/
    chassisSubsystem.driveA(-joy.getRawAxis(1), joy.getRawAxis(4), 1) ;
}  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

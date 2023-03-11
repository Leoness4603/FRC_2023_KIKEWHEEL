// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Forearm;

public class AutonomusDrive extends CommandBase {
  /** Creates a new ForearmGoToPosition. */
 private final Chassis ChassisSubssystem;
  Chassis chassis;
  Timer timer;

  public AutonomusDrive(Chassis chassisSubsystem){
    this.ChassisSubssystem = chassisSubsystem;
    addRequirements(chassisSubsystem);
    } 
    // Use  addRequirements() here to declare subsystem dependencies.
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  public void driveAuto(){
    ChassisSubssystem.driveA(1, 0, 0.3);
    ChassisSubssystem.stop();
  }

  public void turnAuto(){
    ChassisSubssystem.driveA(0, 1, .6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if(timer.get() < 3){
    driveAuto();
  }else if (timer.get() < 6 && timer.get() >3){
    ChassisSubssystem.driveA(-1, 0, 0.3);
  }else if(timer.get() > 6){
    ChassisSubssystem.stop();
  }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSubssystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 15;
  }
}

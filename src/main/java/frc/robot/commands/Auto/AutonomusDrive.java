// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Claw;

public class AutonomusDrive extends CommandBase {
  /** Creates a new ForearmGoToPosition. */
  private final Chassis ChassisSubsystem;
  private final Arm armSubsystem;
  private final Claw clawSubsystem;
  Timer timer = new Timer();
  Boolean finish = false;

  public AutonomusDrive(Chassis chassisSubsystem, Arm armSubsytem, Claw clawSubsystem){
    this.ChassisSubsystem = chassisSubsystem;
    this.armSubsystem = armSubsytem;
    this.clawSubsystem = clawSubsystem;
    addRequirements(chassisSubsystem,  armSubsytem, clawSubsystem);
    } 
    // Use  addRequirements() here to declare subsystem dependencies.
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  public void driveAuto(double movement){
    ChassisSubsystem.driveA(movement, 0, 0.4);
  }

  public void turnAuto(double turn){
    ChassisSubsystem.driveA(0, turn, .5);
  }

  public void ArmMovementAuto(double ArmMovement) {
    armSubsystem.ArmMovement(ArmMovement);
  }

  public void clawOpenAuto() {
    clawSubsystem.open();
  }

  @Override
  public void execute() {
    while(timer.get() < 2){
      ArmMovementAuto(.6);
    }
    while(timer.get() > 2 && timer.get() < 4){
      driveAuto(1);
    }
    while (timer.get() > 4 && timer.get() < 5){
      clawOpenAuto();
    }
    while (timer.get() > 5 && timer.get() < 8.5){
      driveAuto(-1);
    }
    finish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSubsystem.stop();
    armSubsystem.stop();
    clawSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}

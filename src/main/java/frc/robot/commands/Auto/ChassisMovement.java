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

public class ChassisMovement extends CommandBase {
  /** Creates a new ForearmGoToPosition. */
  Chassis chassis;
  double time;
  Timer timer;
  int position;
  DriverStation driverStation;

  public int getLocation() {
    return position = driverStation.getLocation();
  }

  public ChassisMovement(Chassis chassis, double time) {
    this.chassis = chassis;
    addRequirements(chassis);
    } 
    // Use  addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    getLocation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*switch (position) {
      case 1:
        
        break;
      case 2:

        break;
      
      case 3:

        break;
      default:
        break;
    }*/
    if(position == 1){
      if (timer.get() < time) {
        chassis.driveA(.5, 0, 1);
    }else{
      if(position == 2){
        if (timer.get() < time) {
          chassis.driveA(.5, 0, 1);
      }
      }else{
        if(position == 3){
          if (timer.get() < time) {
            chassis.driveA(.5, 0, 1);
        }
        }
      }
    }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}

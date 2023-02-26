// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import frc.robot.Constants;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private Spark motor = new Spark(Constants.IDcan.Claw.kCLAW_Motor);
  private double Speed = .45;
  private Joystick firstTrigger;
  private Joystick secondTrigger;

  public Claw(Joystick firstTrigger, Joystick secondTrigger) {
    this.firstTrigger = firstTrigger;
    this.secondTrigger = secondTrigger;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public Joystick getJoystick(){
    return this.firstTrigger;
  }
  
  public Joystick getSecondJoystick() {
    return this.secondTrigger;
  }

  public void intake(boolean direction){
    if (direction){
      motor.set(-this.Speed);
    }else{
      motor.set(this.Speed);
    }
  }

  public void stop(){
    motor.set(0);
  }
}

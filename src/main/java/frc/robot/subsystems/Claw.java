// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import frc.robot.Constants.IoPWM;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private Spark clawMotor = new Spark(IoPWM.Claw.kCLAW_Motor);
  private double Speed = 1;
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

  public void open(){
    clawMotor.set(-Speed);
  }
  public void close() {
    clawMotor.set(Speed);
  } 

  public void stop(){
    clawMotor.stopMotor();
  }
}


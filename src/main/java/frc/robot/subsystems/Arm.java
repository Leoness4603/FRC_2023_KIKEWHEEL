// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends SubsystemBase {
  private Joystick joystick;
  CANSparkMax motor = new CANSparkMax(Constants.IDcan.Arm.kARM_Motor, MotorType.kBrushed);
  
  Gyro gyro;

  /** Creates a new IntakeWheels. */
  public Arm(Joystick joystick) {
    this.joystick = joystick;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Joystick getJoystick(){
    return joystick;
  }

  public void ArmMovement(double turn){
    this.motor.set(turn);
  }

  public void stop(){
    motor.stopMotor();
  }

}
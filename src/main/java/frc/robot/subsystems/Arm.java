// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends SubsystemBase {
  // declara argumentos del brazo
  private Joystick joystick;
  private CANSparkMax motor = new CANSparkMax(Constants.IDcan.Arm.kARM_Motor, MotorType.kBrushed);

  // crea clase joystick del brazo
  public Arm(Joystick joystick) {
    this.joystick = joystick;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // devolver joystick que usa el brazo 
  public Joystick getJoystick(){
    return joystick;
  }

  // metodo para mover el brazo
  public void ArmMovement(double turn){
    this.motor.set(turn);
  }

  // metodo log
  public void log () {
    SmartDashboard.putNumber("motor brazo temperatura :", motor.getMotorTemperature());
  }

  // metodo apagar del motor brazo
  public void stop(){
    motor.stopMotor();
  }

}
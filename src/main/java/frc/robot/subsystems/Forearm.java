// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DIO;
import frc.robot.Constants.ForeArm.PIDValues;

public class Forearm extends SubsystemBase {
  private Joystick joystick;
  private Spark motor = new Spark(Constants.IoPWM.Forearm.kForearm_Motor);

  Encoder encoderForeArm = new Encoder(DIO.kEncoderForeArm, DIO.kEncoderForeArm2);

  PIDController forearmController = new PIDController(PIDValues.kP, PIDValues.kI, PIDValues.kD);
  
  /** Creates a new IntakeWheels. */
  public Forearm(Joystick joystick) {
    this.joystick = joystick;

    encoderForeArm.setDistancePerPulse(0.6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Joystick getJoystick(){
    return this.joystick;
  }

  public void setSetpoint(double setpoint){
    forearmController.setSetpoint(setpoint);
  }

  public PIDController getForearmController(){
    return forearmController;
  }

  public double getForearmAngle(){
    return encoderForeArm.getDistance();
  }

  public void ForearmMovement(double turn){
    motor.set(turn);
  }

  public void log() {
    SmartDashboard.putNumber("angulo antebrazo :", getForearmAngle());
  }

  public void stop() {
    motor.stopMotor();
  }
}

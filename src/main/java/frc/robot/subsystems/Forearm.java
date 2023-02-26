// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DIO;

public class Forearm extends SubsystemBase {
  private Joystick joystick;
  private Spark motor = new Spark(Constants.IoPWM.Forearm.kForearm_Motor);
  private Spark motor2 = new Spark(Constants.IoPWM.Forearm.kForearm_Motor2);

  Encoder encoderForeArm = new Encoder(DIO.kEncoderForeArm, DIO.kEncoderForeArm2);
  
  /** Creates a new IntakeWheels. */
  public Forearm(Joystick joystick) {
    this.joystick = joystick;

    encoderForeArm.setDistancePerPulse(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Joystick getJoystick(){
    return this.joystick;
  }

  public double getDistanceArm(){
    return encoderForeArm.getDistance();
  }

  public void ForearmMovement(double turn){
    motor.set(turn);
    motor2.set(turn);
  }

  public void log() {
    SmartDashboard.putNumber("distancia del antebrazo :", getDistanceArm());
  }

  public void stop() {
    motor.set(0);
  }
}

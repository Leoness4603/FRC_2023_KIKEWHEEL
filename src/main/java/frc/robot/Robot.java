// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Claw;
import frc.robot.commands.Auto.AutonomusDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
  private Joystick joystick = new Joystick(Constants.Joystick.kJoystick_Port);
  private Joystick driveJoystick = new Joystick(Constants.Joystick.kSecond_Joystick_Port);
  private Joystick secondJoystick = new Joystick(Constants.Joystick.kThird_Joystick_Port);
  
  private Chassis chassis = new Chassis();
  private Arm arm = new Arm(joystick);
  private Claw claw = new Claw(joystick, secondJoystick);

  private ArcadeDrive arcadeDriveCommand = new ArcadeDrive(chassis, driveJoystick);
  private MoveArm moveArmCommand = new MoveArm(arm);
  private MoveClaw moveClawCommand = new MoveClaw(claw);
  private AutonomusDrive autonomusDriveCommand = new AutonomusDrive(chassis);

  private Timer autoTimer = new Timer();
 
  private UsbCamera camera1;
  private UsbCamera camera2;
  private VideoSink Server; 

  

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    chassis.setDefaultCommand(arcadeDriveCommand);
    claw.setDefaultCommand(moveClawCommand);
    arm.setDefaultCommand(moveArmCommand);
    
    camera1 = CameraServer.startAutomaticCapture();
    camera1.setResolution(10, 30);
    camera1.setFPS(60);
    Server = CameraServer.getServer();
    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
   
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoTimer.start();
    // schedule the autonomous command (example)
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if(autoTimer.get() < .6){
      claw.open();
    }else if (autoTimer.get() < 1.75) {
      chassis.driveA(-1, 0, 0.5);
    }else if (autoTimer.get() < 2 ){
      chassis.driveA(0, 1, 0.5);
    }else if(autoTimer.get() > 2){
      chassis.stop();
    }
    
  }


  @Override
  public void teleopInit() {
    // Esto hace certero que el autonomo pare de correr cuando 
    // teleop empiece a correr. si usted quiere ue autonomo continue
    // hasta que sea interrumpido por otro comando, quite
    // esta linea o comentelo afuera.
  }

  @Override
  public void teleopPeriodic() {
    // Drive con arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  
}

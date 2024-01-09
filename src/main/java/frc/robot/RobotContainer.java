// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveClaw;
import frc.robot.commands.Auto.AutonomusDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Claw;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private Joystick joystick = new Joystick(Constants.Joystick.kJoystick_Port);
  private Joystick driveJoystick = new Joystick(Constants.Joystick.kSecond_Joystick_Port);
  private Joystick secondJoystick = new Joystick(Constants.Joystick.kThird_Joystick_Port);
  
  private Chassis chassis = new Chassis();
  private Arm arm = new Arm(joystick);
  private Claw claw = new Claw(joystick, secondJoystick);

  private ArcadeDrive arcadeDriveCommand = new ArcadeDrive(chassis, driveJoystick);
  private MoveArm moveArmCommand = new MoveArm(arm);
  private MoveClaw moveClawCommand = new MoveClaw(claw);
  private AutonomusDrive autonomusDriveCommand = new AutonomusDrive(chassis, arm, claw);

  public RobotContainer() {
    // Configure the button bindings
    chassis.setDefaultCommand(arcadeDriveCommand);
    claw.setDefaultCommand(moveClawCommand);
    arm.setDefaultCommand(moveArmCommand);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonomusDriveCommand;
  }
}

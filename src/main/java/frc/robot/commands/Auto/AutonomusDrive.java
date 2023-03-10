package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;


public class AutonomusDrive extends CommandBase {
  /** Creates a new MoveArm. */
  private final Chassis chassisSubsystem;
  double movement;
  double rotation; 
  Encoder encoderLeft;
  Encoder encoderRight;
  

  public AutonomusDrive(Chassis chassisSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassisSubsystem = chassisSubsystem;
    addRequirements(chassisSubsystem);
  }
   
  public void chassisAuto() {
    if(encoderLeft.getDistance() < 3 && encoderRight.getDistance() < 3){
      chassisSubsystem.driveA(.5, 0, 1);
    }else{
      chassisSubsystem.stop();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     chassisAuto();
  }
    
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return true;
  }
}


package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisTurn extends CommandBase{
      /** Creates a new ForearmGoToPosition. */
  Chassis chassis;
  double setpoint;
  Timer timer;
  public ChassisTurn(Chassis chassis, double setpoint) {
    this.chassis = chassis;
    this.setpoint = setpoint;
    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    chassis.getChassisController().setTolerance(1);
    chassis.getChassisController().setSetpoint(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.driveA(0 ,chassis.getChassisController().calculate(chassis.getAngle()), 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.getChassisController().atSetpoint();
  }
}

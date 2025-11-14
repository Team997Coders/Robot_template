package frc.robot.commands;


import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
import swervelib.SwerveModule;

public class AutoDrive extends Command{

    private final Drivebase drivebase;
    private final double startPos;
    private final double error;
    PIDController pid = new PIDController(0, 0, 0);

  /** Creates a new Drive. */
  public AutoDrive(Drivebase drivebase, double startPos, double error) {
    this.drivebase = drivebase;
    this.startPos = startPos;
    this.error = error;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double startPos = drivebase.getPositions()[0].distanceMeters;
    PIDController pid = new PIDController(0.1, 0, 0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPos = drivebase.getPositions()[0].distanceMeters;
    double deltaPos = currentPos - startPos;
    double error = Constants.AutoDriveConstants.distance - deltaPos;
    SmartDashboard.putNumber("auto error", error);
    pid.setSetpoint(Constants.AutoDriveConstants.distance);
    drivebase.defaultDrive(pid.calculate(currentPos, Constants.AutoDriveConstants.distance), 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (error <= 0){
        return true;
    }
    return false;
  }
}
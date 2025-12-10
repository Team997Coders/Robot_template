// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class DriveAsist extends Command {

  private final Drivebase drivebase;
  private final Supplier<double[]> speedXY;
  private final int target;

  private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(60, 60);

  private final ProfiledPIDController thetaController = new ProfiledPIDController(8, 0, 0, THETA_CONSTRAINTS);

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  /** Creates a new Drive. */
  public DriveAsist(Drivebase drivebase, Supplier<double[]> speedXY, int target) {
    this.drivebase = drivebase;
    this.speedXY = speedXY;
    this.target = target;

    thetaController.setTolerance(Units.degreesToRadians(2));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset(drivebase.getFieldAngle()/2/Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xy = speedXY.get();
    Pose2d robotPose = drivebase.getPose();
    Pose2d goalPose = aprilTagFieldLayout.getTagPose(target).orElseThrow().toPose2d();

    thetaController.setGoal(Math.atan((goalPose.getY()-robotPose.getY())/(goalPose.getX()-robotPose.getX())));
    SmartDashboard.putNumber("X OFFset", goalPose.getX()-robotPose.getX());
    SmartDashboard.putNumber("Y OFFset", goalPose.getY()-robotPose.getY());
    SmartDashboard.putNumber("theta controller goal", Math.atan((goalPose.getY()-robotPose.getY())/(goalPose.getX()-robotPose.getX())));

    double thetaSpeed = thetaController.calculate(robotPose.getRotation().getRadians()+Math.PI);

    SmartDashboard.putNumber("theta Speed", thetaSpeed);

    drivebase.defaultDrive(-xy[1], -xy[0], thetaSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

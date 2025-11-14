package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;

public class goToVelocity extends Command {

    private Drive drivebase;
    private Pose2d goalPose;
    private List<Pose2d> poses;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 2);
    private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(60, 60);

    private final ProfiledPIDController xController = new ProfiledPIDController(5, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(5, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0, THETA_CONSTRAINTS);

    private static final TrapezoidProfile.Constraints VX_CONSTRAINTS = new TrapezoidProfile.Constraints(5, 2);
    private static final TrapezoidProfile.Constraints VY_CONSTRAINTS = new TrapezoidProfile.Constraints(5, 2);

    private final ProfiledPIDController vxController = new ProfiledPIDController(0.75, 0, 0, VX_CONSTRAINTS);
    private final ProfiledPIDController vyController = new ProfiledPIDController(0.75, 0, 0, VY_CONSTRAINTS);

    @SuppressWarnings("unused")
    private double xStart = 0;

    @SuppressWarnings("unused")
    private double yStart = 0;

    @SuppressWarnings("unused")
    private double thetaStart = 0;

    public goToVelocity(Drive drivebase, List<Pose2d> poses) {
        this.drivebase = drivebase;
        this.poses = poses;

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        thetaController.setTolerance(Units.degreesToRadians(2));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        vxController.setTolerance(0.02);
        vyController.setTolerance(0.02);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivebase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double bestDistance = 99999;

        Pose2d bestPose = new Pose2d(-99, -99, new Rotation2d(0));

        Pose2d robotPose = this.drivebase.getPose();

        List<Double> distances = new ArrayList<>();

        for (int i = 0; i < poses.size(); i++) {

            double distance = Math.sqrt(Math.pow(robotPose.getX() - poses.get(i).getX(), 2)
                    + Math.pow(robotPose.getY() - poses.get(i).getY(), 2));
            distances.add(distance);

            if (distance < bestDistance) {
                bestPose = poses.get(i);
                bestDistance = distance;
            }
        }

        this.goalPose = bestPose;

        ChassisSpeeds currentSpeeds = drivebase.getRobotRelativeSpeeds();
        vxController.reset(currentSpeeds.vxMetersPerSecond);
        vyController.reset(-currentSpeeds.vyMetersPerSecond);

        SmartDashboard.putNumber("inital vx Meters Per Second", -currentSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("inital vy Meters Per Second", -currentSpeeds.vyMetersPerSecond);

        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
        thetaController.reset(robotPose.getRotation().getRadians());

        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        thetaController.setGoal(goalPose.getRotation().getRadians());

        SmartDashboard.putNumber("goal pose x", goalPose.getX());
        SmartDashboard.putNumber("goal pose y", goalPose.getY());
        SmartDashboard.putNumber("goal pose r", goalPose.getRotation().getRadians());
    }

    public double xSpeed;
    public double ySpeed;
    public double thetaSpeed;

    public double vxSpeed;
    public double vySpeed;
    public double vthetaSpeed;
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d robotPose = drivebase.getPose();

        xSpeed = xController.calculate(robotPose.getX());
        ySpeed = yController.calculate(robotPose.getY());
        thetaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());

        double angle = goalPose.getRotation().getRadians();

        vxSpeed = vxController.calculate(xSpeed); // * Math.cos(angle) + ySpeed * Math.sin(angle));
        vySpeed = vxController.calculate(ySpeed); // * Math.cos(-angle) + xSpeed * Math.sin(-angle));

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rSpeed", thetaSpeed);

        SmartDashboard.putNumber("vxSpeed", vxSpeed);
        SmartDashboard.putNumber("vySpeed", vySpeed);

        if (xController.atGoal()) {
            vxSpeed = 0;
        }
        if (yController.atGoal()) {
            vySpeed = 0;
        }
        if (thetaController.atGoal()) {
            vthetaSpeed = 0;
        }

        if (vxSpeed == 0 && vySpeed == 0 && vthetaSpeed == 0) {
            this.cancel();
        }

        drivebase.runVelocity(new ChassisSpeeds(
                -(vxSpeed * Math.cos(angle) + vySpeed * Math.sin(angle)),
                -(vySpeed * Math.cos(-angle) + vxSpeed * Math.sin(-angle)),
                thetaSpeed));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean finished = false;
        if (xController.atGoal() && yController.atGoal() && thetaController.atGoal()) {
            finished = true;
        } else {
            finished = false;
        }

        return finished;
    }
}

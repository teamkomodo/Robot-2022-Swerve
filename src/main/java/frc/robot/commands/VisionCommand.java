package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class VisionCommand extends CommandBase {
    PhotonCamera camera;

    private final double ROTATION_SPEED = -3;

    private DrivetrainSubsystem drivetrainSubsystem;

    public VisionCommand(DrivetrainSubsystem ds) {
        this.drivetrainSubsystem = ds;
    }

    @Override
    public void initialize() {
        camera = new PhotonCamera("FrontCamera");
        camera.setPipelineIndex(0);
        camera.setDriverMode(true);
        camera.setDriverMode(false);
    }

    public double cap(double c, double v) {
        if (Math.abs(v) < c) {
            return v;
        }
        return v > 0 ? c : -c;
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            System.out.println("TARGET DETECTIONS:");
            for (PhotonTrackedTarget target : targets) {
                if (target.getFiducialId() == 4) {
                    double yaw = target.getYaw();
                    System.out.println("Yaw: " + yaw);
                    if (Math.abs(yaw) < 5) {
                        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
                        return;
                    }
                    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, cap(1, (yaw / 25.0) * ROTATION_SPEED)));
                }
            }
        } else {
            drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

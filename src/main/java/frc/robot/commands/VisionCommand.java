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

    private final double ROTATION_SPEED = 0.1;

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

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        System.out.println("Driver mode: " + camera.getDriverMode());
        if (result.hasTargets()) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            System.out.println("TARGET DETECTIONS:");
            for (PhotonTrackedTarget target : targets) {
                // System.out.println(" ID: " + target.getFiducialId());
                // List<TargetCorner> corners = target.getCorners();
                // System.out.println(" X: " + corners.get(0).x);
                // System.out.println(" Y: " + corners.get(0).y);
                if (target.getFiducialId() == 4) {
                    List<TargetCorner> corners = target.getCorners();
                    double x = 0;
                    for (TargetCorner c : corners) {
                        x += c.x;
                    }
                    x /= corners.size(); // Average x-coords
                    System.out.println("Average x: " + x);
                    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, x > 320 ? ROTATION_SPEED : -ROTATION_SPEED));
                }
            }
        } else {
            drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionCommand extends CommandBase {
    PhotonCamera camera = new PhotonCamera("photonvision");
    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (result.hasTargets()) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            System.out.println("TARGET DETECTIONS:");
            for (PhotonTrackedTarget target : targets) {
                System.out.println("  ID: " + target.getFiducialId());
                List<TargetCorner> corners = target.getCorners();
                System.out.println("  X: " + corners.get(0).x);
                System.out.println("  Y: " + corners.get(0).y);
            }
        } else {
            System.out.println("No targets");
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

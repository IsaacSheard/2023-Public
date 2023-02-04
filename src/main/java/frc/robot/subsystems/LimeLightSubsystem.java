package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class LimeLightSubsystem extends SubsystemBase {
    public LimeLightSubsystem() {

    }

    public Pose3d fieldPosition() {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
   
      NetworkTableEntry botpose = table.getEntry("botpose");

      double[] position = botpose.getDoubleArray(new double[] { });
      
      if (position.length == 6) {
        Translation3d tran3d = new Translation3d(position[0], position[1],(position[2]));
        Rotation3d r3d = new Rotation3d(position[3], position[4], position[5]);
        return new Pose3d(tran3d, r3d);
      }
    
       return null; 
    }

    public long aprilTagId() {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      NetworkTableEntry entry = table.getEntry("tid");
      return entry.getInteger(-1);
  }
}
  
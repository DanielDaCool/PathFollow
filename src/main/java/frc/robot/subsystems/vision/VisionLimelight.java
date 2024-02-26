package frc.robot.subsystems.vision;

//#region imports
import java.util.Arrays;
import java.util.Comparator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.vision.utils.LimelightVisionUtils;
import frc.robot.subsystems.vision.utils.VisionData;
import frc.robot.subsystems.vision.utils.UpdatedPoseEstimatorClasses.SwerveDrivePoseEstimator;

import static frc.robot.subsystems.vision.VisionConstants.*;
//#endregion

public class VisionLimelight extends SubsystemBase {
    // #region declaring fields
    Field2d noFilterVisionField;
    Field2d visionFieldavg5;
    // #endregion

    // #region declaring poseEstimator chassis and buffers
    SwerveDrivePoseEstimator poseEstimator;
    Chassis chassis;

    VisionData[] buf5Avg;
    int lastData5;
    double lastUpdateTime5;
    double[] timestamp = {0,0};

    // #endregion

    // #region C'tor
    public VisionLimelight(Chassis chassis, SwerveDrivePoseEstimator estimator) {

        this.chassis = chassis;
        this.poseEstimator = estimator;
        this.lastData5 = -1;
        this.buf5Avg = new VisionData[5];

        for (int i = 0; i < buf5Avg.length; i++) {
            this.buf5Avg[i] = new VisionData(null, 0, poseEstimator);
        }
        this.noFilterVisionField = new Field2d();
        this.visionFieldavg5 = new Field2d();
        SmartDashboard.putData("no Filter vision field", noFilterVisionField);
        SmartDashboard.putData("vision Avg 5 field", visionFieldavg5);
    }

    // #endregion

    // #region Vision Main Methods

    // calls the limelights to get updates and put the data in the buffer
    private void getNewDataFromLimelights() {
        // determines camera
        if (chassis.getVelocity().getNorm() <= maxValidVelcity) {
            for(int i = 0; i < LimelightVisionUtils.LimeLightTables.length; i++) {
                var data = LimelightVisionUtils.getVisionPose(i);
                if(data != null && data.getSecond() > timestamp[i]) {
                    Pose2d pose = data.getFirst();
                    timestamp[i] = data.getSecond();
                    lastData5 = next5();
                    buf5Avg[lastData5] = new VisionData(pose, timestamp[i], poseEstimator);
                    noFilterVisionField.setRobotPose(pose);
                }
            }
        }
    }

    // takes the visions snapshots from the buffer and medians or avg it and add
    // vision mesurements to pose estimator
    public void updateRobotPose5() {
        double time = getTime();
        if (validBuf5(time)) {
            Pair<Pose2d, Double> vData5AvgPair = avg(buf5Avg);
            VisionData vDataAvg5 = new VisionData(vData5AvgPair.getFirst(), vData5AvgPair.getSecond(), poseEstimator);
            SmartDashboard.putBoolean("updates", vDataAvg5.getPose() != null && vDataAvg5 != null);
            if (vDataAvg5.getPose() != null && vDataAvg5 != null) {

                poseEstimator.addVisionMeasurement(new Pose2d(vDataAvg5.getPose().getTranslation(),
                        chassis.getAngle()), vDataAvg5.getTimeStamp());
                visionFieldavg5.setRobotPose(vDataAvg5.getPose());

                lastUpdateTime5 = time;

                for (VisionData vd : buf5Avg) {
                    vd.clear();
                }

            }
        }
    }

    // #endregion

    @Override
    public void periodic() {
        super.periodic();

        try {
            getNewDataFromLimelights();
            updateRobotPose5();
        } catch (Exception e) {

        }
    }

    // #region util methods

    public double getTime() {
        return Timer.getFPGATimestamp();
    }

    public Pair<Pose2d, Double> avg(VisionData[] visionDataArr) {

        double averageX = 0.0;
        double averageY = 0.0;
        double averageRotation = 0.0;
        double avarageTimeStamp = 0.0;

        for (VisionData vData : visionDataArr) {
            if (vData != null && vData.getPose() != null) {
                Pose2d pose = vData.getPose();
                averageX += pose.getTranslation().getX();
                averageY += pose.getTranslation().getY();
                averageRotation += pose.getRotation().getRadians();
                avarageTimeStamp += vData.getTimeStamp();
            }
        }

        averageX /= visionDataArr.length;
        averageY /= visionDataArr.length;
        averageRotation /= visionDataArr.length;
        avarageTimeStamp /= visionDataArr.length;
        // Create a new Pose2d with the calculated averages
        return new Pair<Pose2d, Double>((new Pose2d(averageX, averageY, new Rotation2d(averageRotation))),
                avarageTimeStamp);
    }

    Comparator<VisionData> comperator = new Comparator<VisionData>() {
        @Override
        public int compare(VisionData data0, VisionData data1) {
            return Double.compare(data0.getDiffrence(), data1.getDiffrence());
        }
    };

    public VisionData median(VisionData[] visionDataArr) {
        Arrays.sort(visionDataArr, comperator);
        return visionDataArr[visionDataArr.length / 2];
    }
    // #endregion

    // #region BUF 5 methods
    int next5() {
        return (lastData5 + 1) % buf5Avg.length;
    }

    private boolean validBuf5(double time) {
        double minTime = time - 2;

        for (VisionData vData : buf5Avg) {
            if (vData.getTimeStamp() < minTime) {
                return false;
            }
        }
        return true;
    }

    public double lastUpdateTimeStamp() {
        return lastUpdateTime5;
    }

    public boolean validVisionPosition5() {
        return getTime() - lastUpdateTimeStamp() < 1;
    }
    // #endregion

}

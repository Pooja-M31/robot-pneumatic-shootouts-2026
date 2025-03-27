/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.tinylog.Logger;

public class CheckCameraErrorCommand extends Command {
    private int iterationNum = 0;
    private final int averageWindow;
    private final Pose3d[] climberSwervePoses;
    private final Pose3d[] elevatorSwervePoses;
    private Vision vision = Vision.getInstance();

    /**
        Create a command that checks the difference between the two camera positions. This command will
        average the camera positions over a number of iterations and then check the difference.
        @param averageWindow number of iterations to average camera positions over.
     */
    public CheckCameraErrorCommand(int averageWindow) {
        this.averageWindow = averageWindow;

        climberSwervePoses = new Pose3d[averageWindow];
        elevatorSwervePoses = new Pose3d[averageWindow];
    }

    @Override
    public void initialize() {
        Logger.info("Starting camera error checking.");
        vision = Vision.getInstance(); // If this isn't done here, vision == null.
    }

    @Override
    public void execute() {
        // Record estimated pose at each iteration
        if (vision.climberSwerve.getRobotPose() == null || vision.elevatorSwerve.getRobotPose() == null) {
            Logger.error("Did not find camera position readings, waiting for valid readings");
            return;
        }
        climberSwervePoses[iterationNum] = vision.climberSwerve.getRobotPose();
        elevatorSwervePoses[iterationNum] = vision.elevatorSwerve.getRobotPose();
        iterationNum++;
    }

    @Override
    public boolean isFinished() {
        return iterationNum >= averageWindow - 1;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.info("Calculating camera error...");

        // Initialize and zero the position sums
        Translation3d translationSum = new Translation3d();
        Rotation3d rotationSum = new Rotation3d();

        // Average Climber Swerve poses that were gathered during the window
        for (int i = 0; i < iterationNum; i++) {
            Pose3d pose = climberSwervePoses[i];
            if (pose == null) {
                Logger.error("Invalid position found in camera error checking at iteration " + i);
                continue;
            }
            translationSum = translationSum.plus(pose.getTranslation());
            rotationSum = rotationSum.plus(pose.getRotation());
        }
        Pose3d climberSwerveAverage = new Pose3d(translationSum.div(iterationNum), rotationSum.div(iterationNum));

        // Average Elevator Swerve poses that were gathered during the window
        translationSum = new Translation3d();
        rotationSum = new Rotation3d();
        for (int i = 0; i < iterationNum; i++) {
            Pose3d pose = elevatorSwervePoses[i];
            if (pose == null) {
                Logger.error("Invalid position found in camera error checking at iteration " + i);
                continue;
            }
            translationSum = translationSum.plus(pose.getTranslation());
            rotationSum = rotationSum.plus(pose.getRotation());
        }
        Pose3d elevatorSwerveAverage = new Pose3d(translationSum.div(iterationNum), rotationSum.div(iterationNum));

        // Calculate error as the difference between the two
        Transform3d error = climberSwerveAverage.minus(elevatorSwerveAverage);

        // Report error to SmartDashboard
        SmartDashboard.putNumberArray("Camera Error Checking (Meters and Radians)/Camera Position Error", new double[] {
            error.getTranslation().getX(),
            error.getTranslation().getY(),
            error.getTranslation().getZ(),
            error.getRotation().getX(),
            error.getRotation().getY(),
            error.getRotation().getZ(),
        });
        Logger.info("Camera error checked.");
    }
}

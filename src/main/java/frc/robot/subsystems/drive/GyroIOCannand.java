// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIOCannand implements GyroIO {
    private final Canandgyro gyro = new Canandgyro(30);
    private Queue<Double> yawPositionQueue;
    private Queue<Double> yawTimestampQueue;

    public GyroIOCannand() {
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(() -> gyro.getYaw() * 360);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(-gyro.getYaw() * 360);
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-gyro.getAngularVelocityYaw() * 360);

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(-value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}

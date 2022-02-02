/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.subsystems.drive

import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import org.ghrobotics.lib.commands.FalconSubsystem

/**
 * An interface to implement to follow trajectories.
 */
abstract class TrajectoryTrackerDriveBase : FalconSubsystem() {

    abstract val controller: RamseteController
    abstract var robotPosition: Pose2d
    abstract val kinematics: DifferentialDriveKinematics

    abstract fun setOutputSI(
        leftVelocity: Double,
        rightVelocity: Double,
        leftAcceleration: Double,
        rightAcceleration: Double
    )
}

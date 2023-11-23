/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.subsystems.drive.swerve

import com.pathplanner.lib.commands.FollowPathCommand
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.subsystems.SensorlessCompatibleSubsystem
import org.ghrobotics.lib.subsystems.drive.FalconDriveHelper
import org.ghrobotics.lib.utils.Source

abstract class FalconSwerveDrivetrain : TrajectoryTrackerSwerveDriveBase(), SensorlessCompatibleSubsystem {
    /**
     * Helper for different drive styles.
     */
    protected val driveHelper = FalconDriveHelper()

    abstract val wheelbase: Double

    abstract val trackWidth: Double

    abstract val maxSpeed: SIUnit<Velocity<Meter>>

    abstract val motorOutputLimiter: Source<Double>


    val field = Field2d()
    private val fieldTab = Shuffleboard.getTab("Field")

    abstract fun resetPosition(pose: Pose2d, positions: Array<SwerveModulePosition>)

    fun navigateToPose(pose: Pose2d) = PathfindThenFollowPathHolonomic(
        PathPlannerPath(PathPlannerPath.bezierFromPoses(pose), pathConstraints, GoalEndState(0.0, pose.rotation)),
        pathConstraints,
        swerveDriveInputs::robotPose,
        swerveDriveInputs::chassisSpeeds,
        ::setOutputSI,
        pathFollowingConfig,
        this
    )

    fun follow(path: PathPlannerPath) = FollowPathCommand(
        path,
        swerveDriveInputs::robotPose,
        swerveDriveInputs::chassisSpeeds,
        ::setOutputSI,
        controller,
        ReplanningConfig(),
        this
    )

    fun resetPosition(newPose: Pose2d) {
        resetPosition(newPose, swerveDriveIO.positions)
    }

    fun setTrajectory(traj: Trajectory) {
        field.getObject("traj").setTrajectory(traj)
    }

    /**
     * Represents periodic data
     */
    override fun lateInit() {
        resetPosition(Pose2d(), swerveDriveIO.positions)
        fieldTab.add("Field", field).withSize(8, 4)
    }

    override fun setNeutral() {
        swerveDriveIO.setNeutral()
    }

    override fun setOutputSI(
        states: Array<SwerveModuleState>,
    ) {
        swerveDriveIO.setModuleStates(states)
    }

    override fun setOutputSI(speeds: ChassisSpeeds) {
        kinematics.toSwerveModuleStates(speeds).let {
            SwerveDriveKinematics.desaturateWheelSpeeds(it, maxSpeed.value)
            swerveDriveIO.setModuleStates(it)
        }
    }

    fun swerveDrive(forwardInput: Double, strafeInput: Double, rotationInput: Double, fieldRelative: Boolean = true) {
        val outputLimiter = motorOutputLimiter()
        val speeds = driveHelper.swerveDrive(
            this,
            forwardInput * outputLimiter,
            strafeInput * outputLimiter,
            rotationInput * outputLimiter * .75,
            fieldRelative,
        )
        val states = kinematics.toSwerveModuleStates(speeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed.value)
        swerveDriveIO.setModuleStates(states)
    }

    val List<AbstractFalconSwerveModule<*, *>>.positions: List<SwerveModulePosition>
        get() = List(4) {
            SwerveModulePosition(
                this[it].drivePosition.value,
                Rotation2d(this[it].encoder.absolutePosition.value),
            )
        }
}

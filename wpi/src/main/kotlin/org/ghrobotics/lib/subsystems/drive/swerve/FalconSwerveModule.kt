/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.subsystems.drive.swerve

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.motors.AbstractFalconAbsoluteEncoder
import org.ghrobotics.lib.motors.AbstractFalconMotor
import kotlin.math.PI
import kotlin.math.abs

abstract class FalconSwerveModule<D : AbstractFalconMotor<Meter>, T : AbstractFalconMotor<Radian>>(
    private val swerveModuleConstants: SwerveModuleConstants,
) : Sendable {
    abstract val driveMotor: D
    abstract val azimuthMotor: T

    abstract val encoder: AbstractFalconAbsoluteEncoder<Radian>

    private val feedForward = SimpleMotorFeedforward(
        swerveModuleConstants.kDriveKs, swerveModuleConstants.kDriveKv, swerveModuleConstants.kDriveKa
    )

    private var resetIteration: Int = 500
    private var referenceAngle: Double = 0.0

    val name = swerveModuleConstants.kName
    private val maxVoltage = swerveModuleConstants.kDriveMaxVoltage


    fun setState(state: SwerveModuleState, openLoop: Boolean = false) {
        val state = SwerveModuleState.optimize(state, Rotation2d(encoder.position.value))
        val setAngle = state.angle.radians % (2 * Math.PI)
        setSpeed(state.speedMetersPerSecond, openLoop)
        setAngle(setAngle)
    }

    private fun stateAngle(): Double {
        var motorAngle = azimuthMotor.encoder.position.value
        motorAngle %= 2.0 * PI
        if (motorAngle < 0.0) motorAngle += 2.0 * PI
        return motorAngle
    }

    // Keep an eye on this function
    fun swervePosition(): SwerveModulePosition = SwerveModulePosition(
        drivePosition.value,
        Rotation2d(stateAngle()),
    )

    fun setNeutral() {
        driveMotor.setNeutral()
    }

    fun setAngle(angle: Double) {
        var currentAngleRadians = azimuthMotor.encoder.position.value

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        if (abs(azimuthMotor.encoder.velocity.value) < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                resetIteration = 0
                val absoluteAngle: SIUnit<Radian> = encoder.absolutePosition
                azimuthMotor.encoder.resetPosition(absoluteAngle)
                currentAngleRadians = absoluteAngle.value
            }
        } else {
            resetIteration++
        }

        var currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI)
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
        var adjustedReferenceAngleRadians: Double = angle + currentAngleRadians - currentAngleRadiansMod
        if (angle - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI
        } else if (angle - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI
        }

        referenceAngle = angle

        azimuthMotor.setPosition(adjustedReferenceAngleRadians.radians)
    }

    fun setSpeed(speed: Double, openLoop: Boolean) {
        if(openLoop) {
            val voltage = (speed / swerveModuleConstants.kDriveMaxSpeed) * maxVoltage
            driveMotor.setVoltage(voltage.volts)
        } else {
            driveMotor.setVelocity(SIUnit(speed), feedForward.calculate(speed).volts)
        }

    }

    val voltageOutput: SIUnit<Volt> get() = driveMotor.voltageOutput
    val drawnCurrent: SIUnit<Ampere> get() = driveMotor.drawnCurrent
    val drivePosition: SIUnit<Meter> get() = driveMotor.encoder.position
    val driveVelocity: SIUnit<Velocity<Meter>> get() = driveMotor.encoder.velocity
    val anglePosition: SIUnit<Radian> get() = encoder.position

    companion object {
        private const val ENCODER_RESET_ITERATIONS = 500
        private val ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5)
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder!!.run {
            addDoubleProperty("Absolute Position", {
                encoder.absolutePosition.inDegrees()
            }, {})
            addDoubleProperty("Drive Voltage", {
                driveMotor.voltageOutput.value
            }, {})
            addDoubleProperty("State Angle", { stateAngle() }, {})
        }
    }
}

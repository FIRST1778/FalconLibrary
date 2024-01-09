/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.motors.rev

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkPIDController
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Acceleration
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.inAmps
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.motors.AbstractFalconMotor
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.subsystems.drive.swerve.SwerveModuleConstants
import kotlin.math.PI
import kotlin.properties.Delegates

/**
 * Creates a Spark MAX motor controller. The alternate encoder CPR is defaulted
 * to the CPR of the REV Through Bore Encoder.
 *
 * @param canSpark The underlying motor controller.
 * @param model The native unit model.
 * @param useAlternateEncoder Whether to use the alternate encoder or not.
 * @param alternateEncoderCPR The CPR of the alternate encoder.
 */
class FalconSpark<K : SIKey>(
    val canSpark: CANSparkBase,
    private val model: NativeUnitModel<K>,
    useAlternateEncoder: Boolean = false,
    alternateEncoderCPR: Int = 8192,
) : AbstractFalconMotor<K>() {

    /**
     * Creates a Spark MAX motor controller. The alternate encoder CPR is defaulted
     * to the CPR of the REV Through Bore Encoder.
     *
     * @param id The ID of the motor controller.
     * @param model The native unit model.
     * @param useAlternateEncoder Whether to use the alternate encoder or not.
     * @param alternateEncoderCPR The CPR of the alternate encoder.
     */
    constructor(
        id: Int,
        type: CANSparkLowLevel.MotorType,
        model: NativeUnitModel<K>,
        useAlternateEncoder: Boolean = false,
        alternateEncoderCPR: Int = 8196,
        motor: MotorType,
    ) : this(
        when (motor) {
            MotorType.Max -> CANSparkMax(id, type)
            MotorType.Flex -> CANSparkFlex(id, type)
        },
        model,
        useAlternateEncoder,
        alternateEncoderCPR,
    )

    enum class MotorType {
        Max,
        Flex,
    }

    /**
     * The PID controller for the Spark
     */
    @Suppress("MemberVisibilityCanBePrivate")
    val controller: SparkPIDController = canSpark.pidController

    /**
     * The encoder for the Spark
     */
    override val encoder = FalconREVEncoder(
        if (useAlternateEncoder) {

            when (canSpark) {
                is CANSparkMax -> {
                    canSpark.getAlternateEncoder(alternateEncoderCPR)
                }

                is CANSparkFlex -> {
                    canSpark.getExternalEncoder(alternateEncoderCPR)
                }
                else -> {
                    throw Error("FalconSpark does not support custom Sparks")
                }
            }
        } else canSpark.encoder,
        model,
    )

    /**
     * Constructor that sets the feedback device and enables voltage compensation.
     */
    init {
//         controller.setFeedbackDevice(encoder.canEncoder)
        CANSensorShim.configCANEncoderonCanPIDController(controller, encoder.canEncoder)
        canSpark.enableVoltageCompensation(12.0)
    }

    /**
     * Returns the voltage across the motor windings.
     */
    override val voltageOutput: SIUnit<Volt>
        get() = (canSpark.appliedOutput * canSpark.busVoltage).volts

    /**
     * Returns the current drawn by the motor.
     */
    override val drawnCurrent: SIUnit<Ampere>
        get() = canSpark.outputCurrent.amps

    /**
     * Whether the output of the motor is inverted or not. This has
     * no effect on slave motors.
     */
    override var outputInverted: Boolean by Delegates.observable(false) { _, _, newValue ->
        canSpark.inverted = newValue
    }

    /**
     * Configures brake mode for the motor controller.
     */
    override var brakeMode: Boolean by Delegates.observable(false) { _, _, newValue ->
        canSpark.idleMode = if (newValue) CANSparkBase.IdleMode.kBrake else CANSparkBase.IdleMode.kCoast
    }

    /**
     * Configures voltage compensation for the motor controller.
     */
    override var voltageCompSaturation: SIUnit<Volt> by Delegates.observable(12.0.volts) { _, _, newValue ->
        canSpark.enableVoltageCompensation(newValue.value)
    }

    /**
     * Configures the motion profile cruise velocity for Smart Motion.
     */
    override var motionProfileCruiseVelocity: SIUnit<Velocity<K>> by Delegates.observable(SIUnit(0.0)) { _, _, newValue ->
        controller.setSmartMotionMaxVelocity(model.toNativeUnitVelocity(newValue).value * 60.0, 0)
    }

    /**
     * Configures the max acceleration for the motion profile generated by Smart Motion.
     */
    override var motionProfileAcceleration: SIUnit<Acceleration<K>> by Delegates.observable(SIUnit(0.0)) { _, _, newValue ->
        controller.setSmartMotionMaxAccel(model.toNativeUnitAcceleration(newValue).value * 60.0, 0)
    }

    /**
     * Configures the forward soft limit and enables it.
     */
    override var softLimitForward: SIUnit<K> by Delegates.observable(SIUnit(0.0)) { _, _, newValue ->
        canSpark.setSoftLimit(
            CANSparkBase.SoftLimitDirection.kForward,
            model.toNativeUnitPosition(newValue).value.toFloat(),
        )
        canSpark.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true)
    }

    /**
     * Configures a smart current limit for the motor.
     */
    var smartCurrentLimit: SIUnit<Ampere> by Delegates.observable(SIUnit(60.0)) { _, _, newValue ->
        canSpark.setSmartCurrentLimit(newValue.inAmps().toInt())
    }

    /**
     * Configures the reverse soft limit and enables it.
     */
    override var softLimitReverse: SIUnit<K> by Delegates.observable(SIUnit(0.0)) { _, _, newValue ->
        canSpark.setSoftLimit(
            CANSparkBase.SoftLimitDirection.kReverse,
            model.toNativeUnitPosition(newValue).value.toFloat(),
        )
        canSpark.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true)
    }

    /**
     * Sets a certain voltage across the motor windings.
     *
     * @param voltage The voltage to set.
     * @param arbitraryFeedForward The arbitrary feedforward to add to the motor output.
     */
    override fun setVoltage(voltage: SIUnit<Volt>, arbitraryFeedForward: SIUnit<Volt>) {
        controller.setReference(voltage.value, CANSparkBase.ControlType.kVoltage, 0, arbitraryFeedForward.value)
    }

    /**
     * Commands a certain duty cycle to the motor.
     *
     * @param dutyCycle The duty cycle to command.
     * @param arbitraryFeedForward The arbitrary feedforward to add to the motor output.
     */
    override fun setDutyCycle(dutyCycle: Double, arbitraryFeedForward: SIUnit<Volt>) {
        controller.setReference(dutyCycle, CANSparkBase.ControlType.kDutyCycle, 0, arbitraryFeedForward.value)
    }

    /**
     * Sets the velocity setpoint of the motor controller.
     *
     * @param velocity The velocity setpoint.
     * @param arbitraryFeedForward The arbitrary feedforward to add to the motor output.
     */
    override fun setVelocity(velocity: SIUnit<Velocity<K>>, arbitraryFeedForward: SIUnit<Volt>) {
        controller.setReference(
            model.toNativeUnitVelocity(velocity).value * 60,
            CANSparkBase.ControlType.kVelocity,
            0,
            arbitraryFeedForward.value,
        )
    }

    /**
     * Sets the position setpoint of the motor controller. This uses a motion profile
     * if motion profiling is configured.
     *
     * @param position The position setpoint.
     * @param arbitraryFeedForward The arbitrary feedforward to add to the motor output.
     */
    override fun setPosition(position: SIUnit<K>, arbitraryFeedForward: SIUnit<Volt>) {
        controller.setReference(
            model.toNativeUnitPosition(position).value,
            if (useMotionProfileForPosition) CANSparkBase.ControlType.kSmartMotion else CANSparkBase.ControlType.kPosition,
            0,
            arbitraryFeedForward.value,
        )
    }

    /**
     * Gives the motor neutral output.
     */
    override fun setNeutral() = setDutyCycle(0.0)

    /**
     * Follows the output of another motor controller.
     *
     * @param motor The other motor controller.
     */
    override fun follow(motor: FalconMotor<*>): Boolean = if (motor is FalconSpark<*>) {
        canSpark.follow(motor.canSpark)
        true
    } else {
        super.follow(motor)
    }

    companion object {
        fun fromSwerveConstantsDrive(swerveModuleConstants: SwerveModuleConstants): FalconSpark<Meter> =
            with(swerveModuleConstants) {
                falconMAX(
                    kDriveId,
                    CANSparkLowLevel.MotorType.kBrushless,
                    kDriveNativeUnitModel,
                ) {
                    outputInverted = kInvertDrive
                    brakeMode = kDriveBrakeMode
                    voltageCompSaturation = 12.volts
                    smartCurrentLimit = 40.amps
                    controller.run {
                        p = swerveModuleConstants.kDriveKp
                    }
                    canSpark.run {
                        setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100)
                        setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20)
                        setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 20)
                    }
                }
            }

        fun fromSwerveConstantsAzimuth(swerveModuleConstants: SwerveModuleConstants) = with(swerveModuleConstants) {
            falconMAX(
                kAzimuthId,
                CANSparkLowLevel.MotorType.kBrushless,
                kAzimuthNativeUnitModel,
            ) {
                outputInverted = kInvertAzimuth
                brakeMode = kAzimuthBrakeMode
                canSpark.run {
                    setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100)
                    setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20)
                    setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 20)
                }
                voltageCompSaturation = 12.volts
                smartCurrentLimit = kAzimuthCurrentLimit.amps
                controller.run {
                    ff = kAzimuthKf
                    p = kAzimuthKp
                    i = kAzimuthKi
                    d = kAzimuthKd
                    iZone = kAzimuthIZone
                    positionPIDWrappingEnabled = true
                    positionPIDWrappingMaxInput = 2 * PI
                    positionPIDWrappingMinInput = 0.0
                    setFeedbackDevice(canSpark.encoder)
                }
            }
        }
    }
}

fun <K : SIKey> falconMAX(
    canSparkMax: CANSparkMax,
    model: NativeUnitModel<K>,
    useAlternateEncoder: Boolean = false,
    alternateEncoderCPR: Int = 8192,
    block: FalconSpark<K>.() -> Unit,
) = FalconSpark(canSparkMax, model, useAlternateEncoder, alternateEncoderCPR).also(block)

fun <K : SIKey> falconFlex(
    canSparkMax: CANSparkFlex,
    model: NativeUnitModel<K>,
    useAlternateEncoder: Boolean = false,
    alternateEncoderCPR: Int = 8192,
    block: FalconSpark<K>.() -> Unit,
) = FalconSpark(canSparkMax, model, useAlternateEncoder, alternateEncoderCPR).also(block)

fun <K : SIKey> falconMAX(
    id: Int,
    type: CANSparkLowLevel.MotorType,
    model: NativeUnitModel<K>,
    useAlternateEncoder: Boolean = false,
    alternateEncoderCPR: Int = 8192,
    block: FalconSpark<K>.() -> Unit,
) = FalconSpark(id, type, model, useAlternateEncoder, alternateEncoderCPR, FalconSpark.MotorType.Max).also(block)

fun <K : SIKey> falconFlex(
    id: Int,
    type: CANSparkLowLevel.MotorType,
    model: NativeUnitModel<K>,
    useAlternateEncoder: Boolean = false,
    alternateEncoderCPR: Int = 8192,
    block: FalconSpark<K>.() -> Unit,
) = FalconSpark(id, type, model, useAlternateEncoder, alternateEncoderCPR, FalconSpark.MotorType.Flex).also(block)

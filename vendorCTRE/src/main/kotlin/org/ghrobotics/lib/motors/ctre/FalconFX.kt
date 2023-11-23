/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.motors.ctre

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.configs.TalonFXConfigurator
import com.ctre.phoenix6.configs.VoltageConfigs
import com.ctre.phoenix6.controls.CoastOut
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.StaticBrake
import com.ctre.phoenix6.controls.StrictFollower
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Acceleration
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.AbstractFalconMotor
import org.ghrobotics.lib.motors.FalconMotor
import kotlin.properties.Delegates

/**
 * Wrapper around the TalonFX motor controller.
 *
 * @param talonFX The underlying TalonFX motor controller.
 * @param model The native unit model.
 */
@Suppress("Unused")
class FalconFX<K : SIKey>(
    @Suppress("MemberVisibilityCanBePrivate") val talonFX: TalonFX,
    model: NativeUnitModel<K>,
) : AbstractFalconMotor<K>() {

    /**
     * Alternate constructor where users can supply ID and native unit model.
     *
     * @param id The ID of the motor controller.
     * @param model The native unit model.
     */
    constructor(id: Int, model: NativeUnitModel<K>) : this(TalonFX(id), model)

    val configurator: TalonFXConfigurator get() = talonFX.configurator

    fun updateConfigs(block: TalonFXConfigurator.() -> Unit) = configurator.apply(block)

    /**
     * Configures the feedback device for the motor controller.
     */
    var feedbackDevice by Delegates.observable(FeedbackConfigs()) { _, _, newValue ->
        talonFX.configurator.apply(newValue)
    }

    var motorInverted by Delegates.observable(false) { _, _, newValue ->
        talonFX.inverted = newValue
    }

    override var brakeMode by Delegates.observable(false) { _, _, newValue ->
        talonFX.setNeutralMode(if (newValue) NeutralModeValue.Brake else NeutralModeValue.Coast)
    }
    override var voltageCompSaturation: SIUnit<Volt> by Delegates.observable(12.volts) { _, _, newValue ->
        talonFX.configurator.apply(
            VoltageConfigs().withPeakForwardVoltage(newValue.value).withPeakReverseVoltage(-newValue.value),
        )
    }

    @Deprecated("Use UpdateConfigs instead with MotionMagicConfigs")
    override var motionProfileCruiseVelocity: SIUnit<Velocity<K>>
        get() = TODO("Deprecated")
        set(value) = TODO("Deprecated")

    @Deprecated("Use UpdateConfigs instead with SoftwareLimitSwitchConfigs")
    override var motionProfileAcceleration: SIUnit<Acceleration<K>>
        get() = TODO("Deprecated")
        set(value) = TODO("Deprecated")

    @Deprecated("Use UpdateConfigs instead with SoftwareLimitSwitchConfigs")
    override var softLimitForward: SIUnit<K>
        get() = TODO("Deprecated")
        set(value) = TODO("Deprecated")

    @Deprecated("Use UpdateConfigs instead with MotionMagicConfigs")
    override var softLimitReverse: SIUnit<K>
        get() = TODO("Deprecated")
        set(value) = TODO("Deprecated")

    /**
     * Returns the current drawn by the motor.
     */
    override val drawnCurrent: SIUnit<Ampere>
        get() = talonFX.supplyCurrent.value.amps
    override var outputInverted: Boolean
        get() = TODO("Not yet implemented")
        set(value) {}

    /**
     * The previous demand.
     */
    private var lastRequest: ControlRequest = if (brakeMode) StaticBrake() else CoastOut()

    /**
     * The encoder (if any) that is connected to the motor controller.
     */
    override val encoder = FalconCTREEncoder(talonFX, model)

    /**
     * Returns the voltage across the motor windings.
     */
    override val voltageOutput: SIUnit<Volt>
        get() = talonFX.motorVoltage.value.volts

    /**
     * Constructor that enables voltage compensation on the motor controllers by default.
     */
    init {
        talonFX.configurator.apply {
            apply(VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0))
            apply(FeedbackConfigs().withRotorToSensorRatio(model.fromNativeUnitPosition(1.0.nativeUnits).value))
        }
    }

    /**
     * Sets a certain voltage across the motor windings.
     *
     * @param voltage The voltage to set.
     * @param arbitraryFeedForward The arbitrary feedforward to add to the motor output.
     */
    override fun setVoltage(voltage: SIUnit<Volt>, arbitraryFeedForward: SIUnit<Volt>) = sendRequest(
        VoltageOut(voltage.value + arbitraryFeedForward.value),
    )

    /**
     * Commands a certain duty cycle to the motor.
     *
     * @param dutyCycle The duty cycle to command.
     * @param arbitraryFeedForward The arbitrary feedforward to add to the motor output.
     */
    override fun setDutyCycle(dutyCycle: Double, arbitraryFeedForward: SIUnit<Volt>) = sendRequest(
        DutyCycleOut(dutyCycle + arbitraryFeedForward.value / voltageCompSaturation.value),
    )

    /**
     * Sets the velocity setpoint of the motor controller.
     *
     * @param velocity The velocity setpoint.
     * @param arbitraryFeedForward The arbitrary feedforward to add to the motor output.
     */
    override fun setVelocity(velocity: SIUnit<Velocity<K>>, arbitraryFeedForward: SIUnit<Volt>) = sendRequest(
        VelocityVoltage(velocity.value).withFeedForward(arbitraryFeedForward.value).withEnableFOC(false),
    )

    /**
     * Sets the position setpoint of the motor controller. This uses a motion profile
     * if motion profiling is configured.
     *
     * @param position The position setpoint.
     * @param arbitraryFeedForward The arbitrary feedforward to add to the motor output.
     */
    override fun setPosition(position: SIUnit<K>, arbitraryFeedForward: SIUnit<Volt>) = sendRequest(
        PositionVoltage(position.value).withFeedForward(arbitraryFeedForward.value).withEnableFOC(false),
    )

    /**
     * Gives the motor neutral output.
     */
    override fun setNeutral() = sendRequest(
        if (brakeMode) StaticBrake() else CoastOut(),
    )

    /**
     * Sends the demand to the motor controller.
     *
     * @param request The demand to send.
     */
    private fun sendRequest(request: ControlRequest) {
        if (request != lastRequest) {
            talonFX.setControl(request)
            lastRequest = request
        }
    }

    /**
     * Follows the output of another motor controller.
     *
     * @param motor The other motor controller.
     */
    override fun follow(motor: FalconMotor<*>): Boolean = if (motor is FalconFX<*>) {
        talonFX.setControl(StrictFollower(motor.talonFX.deviceID))
        true
    } else {
        super.follow(motor)
    }

    fun follow(motor: FalconMotor<*>, invert: Boolean): Boolean = if (motor is FalconFX<*>) {
        talonFX.setControl(Follower(motor.talonFX.deviceID, invert))
        true
    } else {
        super.follow(motor)
    }
}

fun <K : SIKey> falconFX(
    talonFX: TalonFX,
    model: NativeUnitModel<K>,
    block: TalonFXConfiguration.() -> Unit,
) = FalconFX(talonFX, model).also {
    it.configurator.apply(TalonFXConfiguration().apply(block))
}

fun <K : SIKey> falconFX(
    id: Int,
    model: NativeUnitModel<K>,
    block: TalonFXConfiguration.() -> Unit,
) = FalconFX(id, model).also {
    it.configurator.apply(TalonFXConfiguration().apply(block))
}

fun <K : SIKey> falconFX(
    talonFX: TalonFX,
    model: NativeUnitModel<K>,
    config: TalonFXConfiguration,
    block: FalconFX<K>.() -> Unit,
) = FalconFX(talonFX, model).also {
    it.configurator.apply(config)
}.also(block)

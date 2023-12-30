/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.motors.ctre

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.hardware.core.CoreTalonFX
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.AbstractFalconEncoder
import kotlin.math.abs
import kotlin.properties.Delegates

/**
 * Represents the encoder connected to a CTRE motor controller.
 *
 * @param motorController The motor controller.
 * @param model The native unit model.
 */
class FalconCTREEncoder<K : SIKey>(
    private val motorController: CoreTalonFX,
    model: NativeUnitModel<K>,
) : AbstractFalconEncoder<K>(model) {
    /**
     * Returns the raw velocity from the encoder.
     */
    override val rawVelocity: SIUnit<NativeUnitVelocity> get() = model.toNativeUnitVelocity(SIUnit(motorController.velocity.value))

    /**
     * Returns the raw position from the encoder.
     */
    override val rawPosition: SIUnit<NativeUnit> get() = model.toNativeUnitPosition(SIUnit(motorController.position.value))

    override val velocity: SIUnit<Velocity<K>>
        get() = SIUnit(motorController.velocity.value)

    override val position: SIUnit<K>
        get() = SIUnit(motorController.position.value)

    /**
     * Sets the encoder phase for the encoder.
     */
    var encoderPhase by Delegates.observable<Boolean>(false) { _, _, newValue ->
        motorController.configurator.apply(
            FeedbackConfigs().apply {
                SensorToMechanismRatio = abs(SensorToMechanismRatio) * if (newValue) -1 else 1
            },
        )
    }

    init {
        motorController.configurator.apply(
            FeedbackConfigs().apply {
                SensorToMechanismRatio = model.fromNativeUnitPosition(1.0.nativeUnits).value
            },
        )
    }

    /**
     * Resets the encoder position to a certain value.
     *
     * @param newPosition The position to reset to.
     */
    override fun resetPositionRaw(newPosition: SIUnit<NativeUnit>) {
        motorController.setPosition(model.fromNativeUnitPosition(newPosition).value)
    }
}

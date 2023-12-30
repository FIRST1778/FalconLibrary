/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.motors.rev

import com.revrobotics.RelativeEncoder
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.AbstractFalconEncoder

/**
 * Represents an encoder connected to a Spark MAX.
 *
 * @param canEncoder The underlying encoder.
 * @param model The native unit model.
 */
class FalconMAXEncoder<K : SIKey>(
    val canEncoder: RelativeEncoder,
    model: NativeUnitModel<K>,
) : AbstractFalconEncoder<K>(model) {

    init {
        canEncoder.positionConversionFactor = model.fromNativeUnitPosition(1.0.nativeUnits).value
        canEncoder.velocityConversionFactor = model.fromNativeUnitVelocity(SIUnit(1.0 / 60.0)).value
    }

    /**
     * Returns the raw velocity from the encoder.
     */
    override val rawVelocity: SIUnit<NativeUnitVelocity> get() = model.toNativeUnitVelocity(SIUnit(canEncoder.velocity))

    /**
     * Returns the raw position from the encoder.
     */
    override val rawPosition: SIUnit<NativeUnit> get() = model.toNativeUnitPosition(SIUnit(canEncoder.position))

    /**
     * Returns the position of the encoder.
     */
    override val position: SIUnit<K>
        get() = SIUnit(canEncoder.position)

    /**
     * Returns the velocity of the encoder.
     */
    override val velocity: SIUnit<Velocity<K>>
        get() = SIUnit(canEncoder.velocity)

    /**
     * Resets the encoder position to a certain value.
     *
     * @param newPosition The position to reset to.
     */
    override fun resetPositionRaw(newPosition: SIUnit<NativeUnit>) {
        canEncoder.position = model.fromNativeUnitPosition(newPosition).value
    }

    override fun resetPosition(newPosition: SIUnit<K>) {
        canEncoder.position = newPosition.value
    }
}

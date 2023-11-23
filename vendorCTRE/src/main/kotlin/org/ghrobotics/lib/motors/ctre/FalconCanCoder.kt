/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.motors.ctre

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.sim.CANcoderSimState
import edu.wpi.first.util.sendable.SendableBuilder
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnitsPer100ms
import org.ghrobotics.lib.motors.AbstractFalconAbsoluteEncoder
import kotlin.math.PI

class FalconCanCoder<K : SIKey>(
    canId: Int,
    model: NativeUnitModel<K>,
    offsetAngle: Double = 0.0,
) : AbstractFalconAbsoluteEncoder<K>(model) {

    private val canCoder = CANcoder(canId).apply {
        configurator.apply(
            CANcoderConfiguration().withMagnetSensor(
                MagnetSensorConfigs()
                    .withMagnetOffset(Math.toDegrees(offsetAngle)),
            ),
        )
    }

    val simCollection: CANcoderSimState get() = canCoder.simState

    override val rawPosition: SIUnit<NativeUnit> = canCoder.position.value.nativeUnits
    override val rawVelocity: SIUnit<NativeUnitVelocity> = canCoder.velocity.value.nativeUnitsPer100ms
    override val absolutePosition: SIUnit<Radian> get() = (canCoder.absolutePosition.value * 2 * PI).radians

    override fun resetPositionRaw(newPosition: SIUnit<NativeUnit>) {
        canCoder.setPosition(newPosition.value)
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder?.addDoubleProperty(
            "Absolute Position",
            {
                absolutePosition.inDegrees()
            },
            { },

        )
    }
}

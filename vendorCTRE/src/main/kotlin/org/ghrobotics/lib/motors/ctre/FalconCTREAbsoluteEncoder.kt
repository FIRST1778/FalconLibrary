/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.motors.ctre

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import edu.wpi.first.util.sendable.SendableBuilder
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.AbstractFalconAbsoluteEncoder

class FalconCTREAbsoluteEncoder(
    private val canTalonFX: TalonFX,
    sensorType: FeedbackSensorSourceValue,
    model: NativeUnitModel<Radian>,
    var offset: SIUnit<Radian> = 0.0.radians,
) : AbstractFalconAbsoluteEncoder<Radian>(model) {

    init {
        canTalonFX.configurator.apply(
            FeedbackConfigs().withFeedbackSensorSource(
                sensorType,
            ).withSensorToMechanismRatio(
                model.fromNativeUnitPosition(1.nativeUnits).value,
            ),
        )
    }

    constructor(id: Int, model: NativeUnitModel<Radian>, sensorType: FeedbackSensorSourceValue, offset: SIUnit<Radian> = 0.0.radians) : this(
        TalonFX(id),
        sensorType,
        model,
        offset,
    )

    override val absolutePosition: SIUnit<Radian>
        get() = position + offset
    override val rawPosition: SIUnit<NativeUnit>
        get() = model.toNativeUnitPosition(SIUnit(canTalonFX.position.value))
    override val rawVelocity: SIUnit<NativeUnitVelocity>
        get() = model.toNativeUnitVelocity(SIUnit(canTalonFX.velocity.value))

    override val velocity: SIUnit<Velocity<Radian>>
        get() = SIUnit(canTalonFX.velocity.value)

    override val position: SIUnit<Radian>
        get() = SIUnit(canTalonFX.position.value)

    override fun resetPositionRaw(newPosition: SIUnit<NativeUnit>) {
        offset = model.fromNativeUnitPosition(newPosition)
    }

    override fun resetPosition(newPosition: SIUnit<Radian>) {
        offset = newPosition
    }

    var inverted: Boolean
        get() = canTalonFX.inverted
        set(v) {
            canTalonFX.inverted = v
        }

    override fun initSendable(builder: SendableBuilder?) {
        builder?.addDoubleProperty("Encoder Value", {
            absolutePosition.value
        }, {})
    }
}

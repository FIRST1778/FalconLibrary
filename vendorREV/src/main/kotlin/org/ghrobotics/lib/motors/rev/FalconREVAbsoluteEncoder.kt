/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.motors.rev

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkAbsoluteEncoder
import com.revrobotics.SparkAbsoluteEncoder.Type
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotBase
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnitsPer100ms
import org.ghrobotics.lib.motors.AbstractFalconAbsoluteEncoder

class FalconREVAbsoluteEncoder(
    canSparkMax: CANSparkBase,
    type: SparkAbsoluteEncoder.Type,
    model: NativeUnitModel<Radian>,
) : AbstractFalconAbsoluteEncoder<Radian>(model) {

    @Deprecated("Please use the other constructors provided, pass the SparkObject you are referencing the encoder from")
    constructor(
        id: Int,
        motorType: CANSparkLowLevel.MotorType,
        type: SparkAbsoluteEncoder.Type,
        model: NativeUnitModel<Radian>,
    ) : this(
        CANSparkMax(id, motorType),
        type,
        model,
    )

    constructor(
        falconSpark: FalconSpark<SIKey>,
        type: Type,
        model: NativeUnitModel<Radian>,
    ) : this(
        falconSpark.canSpark,
        type,
        model,
    )

    private val canAbsoluteEncoder = canSparkMax.getAbsoluteEncoder(type)
    var simulatedPosition = 0.0.radians

    /**
     * Same as [position]
     */
    override val absolutePosition: SIUnit<Radian>
        get() {
            if (RobotBase.isSimulation()) {
                return simulatedPosition
            }
            return position
        }
    override val rawPosition: SIUnit<NativeUnit>
        get() = canAbsoluteEncoder.position.nativeUnits
    override val rawVelocity: SIUnit<NativeUnitVelocity>
        get() = canAbsoluteEncoder.velocity.nativeUnitsPer100ms

    override fun resetPositionRaw(newPosition: SIUnit<NativeUnit>) {
        throw Error("Unable to set position of Absolute Encoder")
    }

    fun setZeroOffset(offset: Double) {
        canAbsoluteEncoder.zeroOffset = offset
    }

    override fun initSendable(builder: SendableBuilder?) {
        TODO("Not yet implemented")
    }
}

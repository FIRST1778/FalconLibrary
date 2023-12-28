/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib

import com.revrobotics.CANSparkMaxLowLevel
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.motors.AbstractFalconAbsoluteEncoder
import org.ghrobotics.lib.motors.ctre.FalconCanCoder
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.motors.rev.falconMAX
import org.ghrobotics.lib.subsystems.drive.swerve.FalconSwerveModule
import org.ghrobotics.lib.subsystems.drive.swerve.SwerveModuleConstants

class FalconNeoSwerveModule(private val swerveModuleConstants: SwerveModuleConstants) :
    FalconSwerveModule<FalconMAX<Meter>, FalconMAX<Radian>>(swerveModuleConstants) {

    override var encoder: AbstractFalconAbsoluteEncoder<Radian> = FalconCanCoder(
        swerveModuleConstants.kCanCoderId,
        swerveModuleConstants.kCanCoderNativeUnitModel,
        swerveModuleConstants.kAzimuthEncoderHomeOffset,
    )

    override var driveMotor = with(swerveModuleConstants) {
        falconMAX(
            kDriveId,
            CANSparkMaxLowLevel.MotorType.kBrushless,
            kDriveNativeUnitModel,
        ) {
            outputInverted = kInvertDrive
            brakeMode = kDriveBrakeMode
            voltageCompSaturation = 12.volts
            smartCurrentLimit = 40.amps
            controller.run {
                p = swerveModuleConstants.kDriveKp
            }
            canSparkMax.run {
                setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100)
                setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20)
                setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20)
            }
        }
    }
    override var azimuthMotor = with(swerveModuleConstants) {
        falconMAX(
            kAzimuthId,
            CANSparkMaxLowLevel.MotorType.kBrushless,
            kAzimuthNativeUnitModel,
        ) {
            outputInverted = kInvertAzimuth
            brakeMode = kAzimuthBrakeMode
            canSparkMax.run {
                setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100)
                setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20)
                setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20)
            }
            voltageCompSaturation = 12.volts
            smartCurrentLimit = 20.amps
            controller.run {
                ff = kAzimuthKf
                p = kAzimuthKp
                i = kAzimuthKi
                d = kAzimuthKd
                iZone = kAzimuthIZone
                setFeedbackDevice(canSparkMax.encoder)
            }
        }
    }
}

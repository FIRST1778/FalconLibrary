/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.motors.ctre

import org.ghrobotics.lib.mathematics.units.SIKey

/**
 * Wrapper around the TalonSRX motor controller.
 *
 * @param talonSRX The underlying TalonSRX motor controller.
 * @param model The native unit model.
 */

@Deprecated(
    "Use FalconFX instead.",
    ReplaceWith(
        "FalconFX",
        "org.ghrobotics.lib.motors.ctre.FalconFX",
    ),
)
class FalconSRX<K : SIKey>

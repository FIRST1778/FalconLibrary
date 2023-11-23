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
 * Represents the abstract class for all CTRE motor controllers.
 *
 * @param motorController The underlying motor controller.
 * @param model The native unit model.
 */
@Deprecated("CTRE only supports TalonFXs as of Phoenix v6")
abstract class FalconCTRE<K : SIKey>

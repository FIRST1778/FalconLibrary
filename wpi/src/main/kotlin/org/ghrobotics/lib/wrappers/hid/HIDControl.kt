/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.lib.wrappers.hid

import kotlin.math.absoluteValue

class HIDButton(
    private val source: HIDSource,
    private val threshold: Double,
    private val whileOff: List<HIDControlListener>,
    private val whileOn: List<HIDControlListener>,
    private val changeOn: List<HIDControlListener>,
    private val changeOff: List<HIDControlListener>,
) : HIDControl {

    companion object {
        const val DEFAULT_THRESHOLD = 0.5
    }

    private var lastValue = source().absoluteValue >= threshold

    override fun update() {
        val newValue = source().absoluteValue >= threshold
        when {
            // Value has changed
            lastValue != newValue -> when {
                newValue -> changeOn
                else -> changeOff
            }
            // Value stayed the same
            else -> when {
                newValue -> whileOn
                else -> whileOff
            }
        }.forEach { it() }
        lastValue = newValue
    }
}

typealias HIDControlListener = () -> Unit

interface HIDControl {
    fun update()
}

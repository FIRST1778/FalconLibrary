package org.ghrobotics.lib.mathematics.units.derivedunits

import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.expressions.SIExp3
import org.ghrobotics.lib.mathematics.units.expressions.SIExp4
import org.ghrobotics.lib.mathematics.units.fractions.SIFrac33
import org.ghrobotics.lib.mathematics.units.fractions.SIFrac34
import org.ghrobotics.lib.mathematics.units.fractions.SIFrac35

val Number.volt: Volt
    get() = SIFrac34(
        SIExp3(this.gram, 1.meter, 1.meter),
        SIExp4(1.second, 1.second, 1.second, 1.amp)
    )

typealias Volt = SIFrac34<Mass, Length, Length,
        Time, Time, Time, ElectricCurrent>

typealias Watt = SIFrac33<Mass, Length, Length,
        Time, Time, Time>

typealias Ohm = SIFrac35<Mass, Length, Length,
        Time, Time, Time, ElectricCurrent, ElectricCurrent>
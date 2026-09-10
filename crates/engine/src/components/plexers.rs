//! Multiplexer/Demultiplexer (`std/plexers/Multiplexer.java`/
//! `Demultiplexer.java`): route data through a shared select/enable
//! decision. Both share the exact same select-decode and enable-status
//! classification (`decode_select`/`enable_status` below), verified
//! line-by-line against both Java sources' `propagate` methods — Logisim
//! itself doesn't factor them out (each file repeats the logic), but
//! there's no reason to duplicate it here too.

use super::{bit_at, u32_to_signal, Gate};
use plugin_abi::{Bit, Signal};

enum Select {
    Defined(usize),
    Error,
    Undefined,
}

/// Mirrors `Value.isFullyDefined`/`isErrorValue`'s precedence at the
/// aggregate (multi-bit) level: a select signal with *any* `Error` bit is
/// `Error` outright (even if other bits are fine or `Unknown`); only
/// otherwise does any `Unknown` bit make it `Undefined`; only with
/// neither is it a clean index.
fn decode_select(signal: &Signal, select_bits: u8) -> Select {
    let mut value = 0usize;
    let mut has_error = false;
    let mut has_unknown = false;
    for i in 0..select_bits as usize {
        match bit_at(signal, i) {
            Bit::One => value |= 1 << i,
            Bit::Zero => {}
            Bit::Error => has_error = true,
            Bit::Unknown => has_unknown = true,
        }
    }
    if has_error {
        Select::Error
    } else if has_unknown {
        Select::Undefined
    } else {
        Select::Defined(value)
    }
}

enum EnableStatus {
    Disabled,
    ConflictError,
    Active,
}

/// `en != Zero` (i.e. `Unknown` counts as active) mirrors the same
/// "undriven control input defaults to on" convention already seen on
/// `Register`'s `EN` (`state.getPort(EN) != Value.FALSE`) — not a
/// coincidence, both come from the same Logisim idiom (`Value.java` never
/// treats "not driven" as "off"). The Java source additionally guards the
/// `Error` branch with `isPortConnected` before treating it as a real
/// conflict; in our engine an unconnected pin already resolves to
/// `Unknown` (never `Error`) via `sim.rs::gather_inputs`'s "zero real
/// drivers -> Unknown" fold, so `Error` here can only mean a genuine
/// multi-driver conflict, which is only possible when connected — that
/// extra check would be redundant.
fn enable_status(en: Bit) -> EnableStatus {
    match en {
        Bit::Zero => EnableStatus::Disabled,
        Bit::Error => EnableStatus::ConflictError,
        Bit::One | Bit::Unknown => EnableStatus::Active,
    }
}

impl Gate {
    pub(super) fn input_count_plexers(&self) -> usize {
        match self {
            // data lines, then select, then enable?
            Gate::Mux { select_bits, has_enable, .. } => (1usize << select_bits) + 1 + (*has_enable as usize),
            // select, then enable?, then the one data line.
            Gate::Demux { has_enable, .. } => 2 + (*has_enable as usize),
            // select, then enable? — no data line at all, unlike Demux.
            Gate::Decoder { has_enable, .. } => 1 + (*has_enable as usize),
            // n data lines, then enable_in — always present, unlike the
            // others' optional `enable` (`PriorityEncoder` has no
            // no-enable-pin mode in Java).
            Gate::PriorityEncoder { select_bits, .. } => (1usize << select_bits) + 1,
            _ => unreachable!("dispatch bug: not a plexer"),
        }
    }

    /// Mux: data lines (0..n) are `bits` wide, select (index n) is
    /// `select_bits` wide, the optional enable (index n+1) is 1 bit.
    /// Demux: select (index 0) is `select_bits` wide, the optional enable
    /// (index 1) is 1 bit, the one data line (last index) is `bits` wide.
    /// Decoder: select (index 0) is `select_bits` wide, the optional enable
    /// (index 1) is 1 bit — same as Demux minus the data line.
    pub(super) fn input_width_plexers(&self, pin: usize) -> u8 {
        match self {
            Gate::Mux { bits, select_bits, .. } => {
                let n = 1usize << select_bits;
                if pin < n {
                    *bits
                } else if pin == n {
                    *select_bits
                } else {
                    1
                }
            }
            Gate::Demux { bits, select_bits, has_enable, .. } => {
                if pin == 0 {
                    *select_bits
                } else if *has_enable && pin == 1 {
                    1
                } else {
                    *bits
                }
            }
            Gate::Decoder { select_bits, .. } => {
                if pin == 0 {
                    *select_bits
                } else {
                    1
                }
            }
            // Every data line and `enable_in` are 1 bit each — `PriorityEncoder`
            // has no wide input pin at all (unlike Mux/Demux/Decoder's select).
            Gate::PriorityEncoder { .. } => 1,
            _ => unreachable!("dispatch bug: not a plexer"),
        }
    }

    /// Every output pin (Mux's one, Demux's `2^select_bits`, each `bits`
    /// wide) — except `Decoder`, whose outputs are fixed at 1 bit each
    /// regardless of any `bits` attribute (it doesn't have one: `Decoder`
    /// only ever routes a constant `One`, verified in `Decoder.java`'s
    /// `propagate`, which hardcodes `BitWidth data = BitWidth.ONE`), and
    /// `PriorityEncoder`, whose `out` (pin 0) is `select_bits`-wide but
    /// `enable_out`/`group_signal` (pins 1, 2) are each 1 bit.
    pub(super) fn output_width_plexers(&self, pin: usize) -> u8 {
        match self {
            Gate::Mux { bits, .. } | Gate::Demux { bits, .. } => *bits,
            Gate::Decoder { .. } => 1,
            Gate::PriorityEncoder { select_bits, .. } => {
                if pin == 0 {
                    *select_bits
                } else {
                    1
                }
            }
            _ => unreachable!("dispatch bug: not a plexer"),
        }
    }

    pub(super) fn eval_plexers(&mut self, inputs: &[Signal]) -> Vec<Signal> {
        match self {
            Gate::Mux { bits, select_bits, has_enable, disabled_zero } => {
                let n = 1usize << *select_bits;
                let en = if *has_enable { bit_at(&inputs[n + 1], 0) } else { Bit::One };
                let out: Signal = match enable_status(en) {
                    EnableStatus::Disabled => vec![if *disabled_zero { Bit::Zero } else { Bit::Unknown }; *bits as usize],
                    EnableStatus::ConflictError => vec![Bit::Error; *bits as usize],
                    EnableStatus::Active => match decode_select(&inputs[n], *select_bits) {
                        Select::Defined(idx) => (0..*bits as usize).map(|i| bit_at(&inputs[idx], i)).collect(),
                        Select::Error => vec![Bit::Error; *bits as usize],
                        Select::Undefined => vec![Bit::Unknown; *bits as usize],
                    },
                };
                vec![out]
            }
            Gate::Demux { bits, select_bits, has_enable, disabled_zero, tristate } => {
                let en = if *has_enable { bit_at(&inputs[1], 0) } else { Bit::One };
                let data_idx = 1 + (*has_enable as usize);
                let n = 1usize << *select_bits;

                // `selected`: which output index (if any) gets `active`;
                // every other output gets `idle`. Mirrors
                // `Demultiplexer.propagate`'s `outIndex`/`others` pair —
                // `outIndex` stays "none" whenever the whole array should
                // read uniformly (disabled/error/undefined-select).
                let (selected, active, idle): (Option<usize>, Signal, Signal) = match enable_status(en) {
                    EnableStatus::Disabled => {
                        let v = if *disabled_zero { Bit::Zero } else { Bit::Unknown };
                        (None, Vec::new(), vec![v; *bits as usize])
                    }
                    EnableStatus::ConflictError => (None, Vec::new(), vec![Bit::Error; *bits as usize]),
                    EnableStatus::Active => match decode_select(&inputs[0], *select_bits) {
                        Select::Defined(idx) => {
                            let picked = (0..*bits as usize).map(|i| bit_at(&inputs[data_idx], i)).collect();
                            let default_bit = if *tristate { Bit::Unknown } else { Bit::Zero };
                            (Some(idx), picked, vec![default_bit; *bits as usize])
                        }
                        Select::Error => (None, Vec::new(), vec![Bit::Error; *bits as usize]),
                        Select::Undefined => (None, Vec::new(), vec![Bit::Unknown; *bits as usize]),
                    },
                };

                (0..n).map(|i| if Some(i) == selected { active.clone() } else { idle.clone() }).collect()
            }
            Gate::Decoder { select_bits, has_enable, disabled_zero, tristate } => {
                let n = 1usize << *select_bits;
                let en = if *has_enable { bit_at(&inputs[1], 0) } else { Bit::One };

                // Same decision tree as `Demux` above, minus a data line to
                // route — the selected output is just a fixed `One`.
                let (selected, idle): (Option<usize>, Bit) = match enable_status(en) {
                    EnableStatus::Disabled => (None, if *disabled_zero { Bit::Zero } else { Bit::Unknown }),
                    EnableStatus::ConflictError => (None, Bit::Error),
                    EnableStatus::Active => match decode_select(&inputs[0], *select_bits) {
                        Select::Defined(idx) => (Some(idx), if *tristate { Bit::Unknown } else { Bit::Zero }),
                        Select::Error => (None, Bit::Error),
                        Select::Undefined => (None, Bit::Unknown),
                    },
                };

                (0..n).map(|i| vec![if Some(i) == selected { Bit::One } else { idle }]).collect()
            }
            Gate::PriorityEncoder { select_bits, disabled_zero } => {
                let n = 1usize << *select_bits;
                // Deliberately not `enable_status`: `PriorityEncoder.
                // propagate` uses a plain `!= Value.FALSE` test, so an
                // `Error` enable counts as active here (unlike Mux/Demux/
                // Decoder, which special-case it into `Error` outright).
                let enabled = bit_at(&inputs[n], 0) != Bit::Zero;
                // Highest index wins; each line is a plain `== One` check
                // (`Error`/`Unknown` there just means "not asserted", no
                // per-bit error propagation — verified against `propagate`).
                let found = if enabled { (0..n).rev().find(|&i| bit_at(&inputs[i], 0) == Bit::One) } else { None };

                match found {
                    Some(idx) => vec![u32_to_signal(idx as u32, *select_bits), vec![Bit::Zero], vec![Bit::One]],
                    None => {
                        // Enabled-but-nothing-found always floats
                        // (`Value.createUnknown`, ignoring `disabled_zero` —
                        // that option only governs the *disabled* case).
                        let out_bit = if !enabled && *disabled_zero { Bit::Zero } else { Bit::Unknown };
                        let enable_out = if enabled { Bit::One } else { Bit::Zero };
                        vec![vec![out_bit; *select_bits as usize], vec![enable_out], vec![Bit::Zero]]
                    }
                }
            }
            _ => unreachable!("dispatch bug: not a plexer"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use plugin_abi::Component;

    fn mux(select_bits: u8) -> Gate {
        Gate::Mux { bits: 1, select_bits, has_enable: true, disabled_zero: false }
    }

    fn demux(select_bits: u8) -> Gate {
        Gate::Demux { bits: 1, select_bits, has_enable: true, disabled_zero: false, tristate: false }
    }

    #[test]
    fn mux_selects_the_indexed_data_line() {
        let mut m = mux(1); // 2 data lines
        // inputs: [d0, d1, select, enable]
        let out = m.eval(&[vec![Bit::Zero], vec![Bit::One], vec![Bit::Zero], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::Zero]], "select=0 -> d0");

        let out = m.eval(&[vec![Bit::Zero], vec![Bit::One], vec![Bit::One], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::One]], "select=1 -> d1");
    }

    #[test]
    fn mux_disabled_reads_floating_by_default_not_zero() {
        let mut m = mux(1);
        let out = m.eval(&[vec![Bit::One], vec![Bit::One], vec![Bit::Zero], vec![Bit::Zero]]); // en=0
        assert_eq!(out, vec![vec![Bit::Unknown]], "default `disabled` option is floating (Unknown), not Zero");
    }

    #[test]
    fn mux_disabled_zero_option_reads_zero() {
        let mut m = Gate::Mux { bits: 1, select_bits: 1, has_enable: true, disabled_zero: true };
        let out = m.eval(&[vec![Bit::One], vec![Bit::One], vec![Bit::Zero], vec![Bit::Zero]]); // en=0
        assert_eq!(out, vec![vec![Bit::Zero]]);
    }

    #[test]
    fn mux_undriven_enable_still_selects_matching_value_ne_false() {
        let mut m = mux(1);
        let out = m.eval(&[vec![Bit::Zero], vec![Bit::One], vec![Bit::One], vec![Bit::Unknown]]); // en=Unknown
        assert_eq!(out, vec![vec![Bit::One]], "Unknown enable still selects, same as Register's EN");
    }

    #[test]
    fn mux_conflicting_enable_drivers_yield_error_output() {
        let mut m = mux(1);
        let out = m.eval(&[vec![Bit::Zero], vec![Bit::One], vec![Bit::One], vec![Bit::Error]]);
        assert_eq!(out, vec![vec![Bit::Error]]);
    }

    #[test]
    fn mux_partially_undefined_select_yields_unknown_not_a_guess() {
        let mut m = mux(2); // 4 data lines, 2-bit select
        let out = m.eval(&[
            vec![Bit::Zero],
            vec![Bit::One],
            vec![Bit::Zero],
            vec![Bit::One],
            vec![Bit::Zero, Bit::Unknown], // select: bit0=0, bit1=Unknown
            vec![Bit::One],
        ]);
        assert_eq!(out, vec![vec![Bit::Unknown]]);
    }

    #[test]
    fn mux_select_with_an_error_bit_yields_error_not_unknown() {
        let mut m = mux(1);
        let out = m.eval(&[vec![Bit::Zero], vec![Bit::One], vec![Bit::Error], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::Error]], "an Error bit in select takes precedence over merely-Unknown");
    }

    #[test]
    fn mux_without_enable_pin_is_always_active() {
        let mut m = Gate::Mux { bits: 1, select_bits: 1, has_enable: false, disabled_zero: false };
        // inputs: [d0, d1, select] — no enable pin at all.
        let out = m.eval(&[vec![Bit::Zero], vec![Bit::One], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::One]]);
    }

    #[test]
    fn demux_routes_data_to_the_selected_output_others_default_to_zero() {
        let mut d = demux(1); // 2 outputs
        // inputs: [select, enable, data]
        let out = d.eval(&[vec![Bit::One], vec![Bit::One], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::Zero], vec![Bit::One]], "select=1 -> output[1] gets data, output[0] idles at Zero");
    }

    #[test]
    fn demux_tristate_option_idles_at_unknown_instead_of_zero() {
        let mut d = Gate::Demux { bits: 1, select_bits: 1, has_enable: true, disabled_zero: false, tristate: true };
        let out = d.eval(&[vec![Bit::One], vec![Bit::One], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::Unknown], vec![Bit::One]]);
    }

    #[test]
    fn demux_disabled_sets_every_output_uniformly() {
        let mut d = demux(1);
        let out = d.eval(&[vec![Bit::One], vec![Bit::Zero], vec![Bit::One]]); // en=0
        assert_eq!(out, vec![vec![Bit::Unknown], vec![Bit::Unknown]], "disabled: no output is `selected`, all read the disabled value");
    }

    #[test]
    fn demux_select_error_makes_every_output_error() {
        let mut d = demux(1);
        let out = d.eval(&[vec![Bit::Error], vec![Bit::One], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::Error], vec![Bit::Error]]);
    }

    fn decoder(select_bits: u8) -> Gate {
        Gate::Decoder { select_bits, has_enable: true, disabled_zero: false, tristate: false }
    }

    #[test]
    fn decoder_asserts_the_selected_output_others_default_to_zero() {
        let mut d = decoder(1); // 2 outputs
        // inputs: [select, enable]
        let out = d.eval(&[vec![Bit::One], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::Zero], vec![Bit::One]], "select=1 -> output[1]=One, output[0] idles at Zero");
    }

    #[test]
    fn decoder_tristate_option_idles_at_unknown_instead_of_zero() {
        let mut d = Gate::Decoder { select_bits: 1, has_enable: true, disabled_zero: false, tristate: true };
        let out = d.eval(&[vec![Bit::One], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::Unknown], vec![Bit::One]]);
    }

    #[test]
    fn decoder_disabled_sets_every_output_uniformly() {
        let mut d = decoder(1);
        let out = d.eval(&[vec![Bit::One], vec![Bit::Zero]]); // en=0
        assert_eq!(out, vec![vec![Bit::Unknown], vec![Bit::Unknown]]);
    }

    #[test]
    fn decoder_select_error_makes_every_output_error() {
        let mut d = decoder(1);
        let out = d.eval(&[vec![Bit::Error], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::Error], vec![Bit::Error]]);
    }

    #[test]
    fn decoder_without_enable_pin_is_always_active() {
        let mut d = Gate::Decoder { select_bits: 1, has_enable: false, disabled_zero: false, tristate: false };
        // inputs: [select] — no enable pin at all.
        let out = d.eval(&[vec![Bit::Zero]]);
        assert_eq!(out, vec![vec![Bit::One], vec![Bit::Zero]]);
    }

    fn priority_encoder(select_bits: u8) -> Gate {
        Gate::PriorityEncoder { select_bits, disabled_zero: false }
    }

    #[test]
    fn priority_encoder_picks_the_highest_asserted_index() {
        let mut p = priority_encoder(2); // 4 data lines
        // inputs: [i0, i1, i2, i3, enable_in]; i1 and i2 both asserted -> i2 wins.
        let out = p.eval(&[vec![Bit::Zero], vec![Bit::One], vec![Bit::One], vec![Bit::Zero], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::Zero, Bit::One], vec![Bit::Zero], vec![Bit::One]], "out=2 (binary 10), enable_out=0, group_signal=1");
    }

    #[test]
    fn priority_encoder_nothing_asserted_floats_and_passes_enable_downstream() {
        let mut p = priority_encoder(1); // 2 data lines
        let out = p.eval(&[vec![Bit::Zero], vec![Bit::Zero], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::Unknown], vec![Bit::One], vec![Bit::Zero]]);
    }

    #[test]
    fn priority_encoder_disabled_reads_disabled_zero_option_not_the_enabled_default() {
        let mut p = Gate::PriorityEncoder { select_bits: 1, disabled_zero: true };
        let out = p.eval(&[vec![Bit::One], vec![Bit::One], vec![Bit::Zero]]); // en=0, both inputs asserted
        assert_eq!(out, vec![vec![Bit::Zero], vec![Bit::Zero], vec![Bit::Zero]], "disabled -> disabled_zero wins over any asserted input");
    }

    /// The key divergence from `Mux`/`Demux`/`Decoder`'s `enable_status`:
    /// there, an `Error` enable is its own conflict branch. Here it's a
    /// plain `!= Zero` test, so `Error` counts as *active* — verified
    /// against `PriorityEncoder.propagate`'s `enabled = en != Value.FALSE`.
    #[test]
    fn priority_encoder_error_enable_counts_as_active_not_a_conflict() {
        let mut p = priority_encoder(1);
        // inputs: [i0, i1, enable_in]; only i1 asserted, enable_in = Error.
        let out = p.eval(&[vec![Bit::Zero], vec![Bit::One], vec![Bit::Error]]);
        assert_eq!(out, vec![vec![Bit::One], vec![Bit::Zero], vec![Bit::One]], "i1 wins, same as any other active enable");
    }

    /// Unlike `Mux`/`Demux`'s select decode, an individual data line's
    /// `Error`/`Unknown` never propagates — it's simply "not asserted",
    /// the same as `Zero`.
    #[test]
    fn priority_encoder_undefined_data_line_reads_as_not_asserted_not_error() {
        let mut p = priority_encoder(1);
        let out = p.eval(&[vec![Bit::Error], vec![Bit::Unknown], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::Unknown], vec![Bit::One], vec![Bit::Zero]], "neither line reads as asserted -> nothing found");
    }
}

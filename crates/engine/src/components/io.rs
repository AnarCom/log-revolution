//! `std/io`'s `DotMatrix`/`Tty`/`Keyboard` — external-I/O-flavored leaf
//! components, grouped together the same way `logisim-port` groups them
//! (distinct from `Random`, `std/memory`, ported alongside `Register`/
//! `Rom`/`Ram` in `memory.rs` despite sharing this module's "external I/O"
//! flavor).

use super::{bit_at, bit_to_byte, byte_to_bit, Gate};
use plugin_abi::{ActionError, Bit, ReadoutError, Signal, Value};

/// `DotMatrix.ATTR_INPUT_TYPE`'s three option strings (`"column"`/`"row"`/
/// `"select"`) — see `Gate::DotMatrix`'s doc comment for what each wiring
/// convention means.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DotMatrixInput {
    Column,
    Row,
    Select,
}

/// `DotMatrix.State.setColumn`, ported bit-for-bit: `signal`'s bit 0 (LSB)
/// lands on the *bottom* row, bit `rows-1` (MSB) on the *top* row — Java's
/// own `gridloc = (rows-1)*cols+col; stride=-cols` walked forward through
/// `vals[0..]`, verified by substitution.
fn set_column(grid: &mut Signal, rows: u8, cols: u8, col: usize, signal: &Signal) {
    let (rows, cols) = (rows as usize, cols as usize);
    for i in 0..rows {
        let row = rows - 1 - i;
        grid[row * cols + col] = bit_at(signal, i);
    }
}

/// `DotMatrix.State.setRow`, ported bit-for-bit: `signal`'s bit 0 (LSB)
/// lands on the *rightmost* column, bit `cols-1` (MSB) on the *leftmost* —
/// note this is the mirror-image convention of `set_select`'s col-data
/// below, which is LSB-leftmost; that asymmetry is really in the Java
/// source (`setRow`'s `gridloc=(index+1)*cols-1; stride=-1` vs `setSelect`'s
/// ascending `gridloc`), not an inconsistency introduced here.
fn set_row(grid: &mut Signal, cols: u8, row: usize, signal: &Signal) {
    let cols = cols as usize;
    for i in 0..cols {
        let col = cols - 1 - i;
        grid[row * cols + col] = bit_at(signal, i);
    }
}

/// `DotMatrix.State.setSelect`, ported bit-for-bit. `row_select`'s bit `i`
/// picks grid row `rows-1-i` (bit 0/LSB = bottom row, same top/bottom
/// convention as `set_column`); that row becomes `col_data` mapped
/// bit-for-position (bit 0/LSB = *leftmost* column this time — see
/// `set_row`'s doc for the asymmetry) when the select bit is exactly
/// `One`, all-`Zero` when exactly `Zero`, or all-`Error` for anything else
/// (`Unknown`/`Error`) — Java's `if (wholeRow != FALSE) wholeRow = ERROR`
/// applied per-cell across the whole row, not just the offending bit.
///
/// Always assumes 2 ports (`cols`-wide `col_data`, `rows`-wide
/// `row_select`) regardless of `rows`/`cols` — real Logisim instead
/// collapses to a single port when `rows <= 1` or `cols <= 1`
/// (`DotMatrix.updatePorts`), a case its own `propagate` (unconditional
/// `getPort(1)`) would throw on if ever actually hit; not replicated here
/// since a 1xN/Nx1 `Select`-mode matrix is a degenerate edge case, not a
/// behavior worth reproducing a crash for.
fn set_select(grid: &mut Signal, rows: u8, cols: u8, row_select: &Signal, col_data: &Signal) {
    let (rows, cols) = (rows as usize, cols as usize);
    for i in 0..rows {
        let row = rows - 1 - i;
        let start = row * cols;
        let fill = match bit_at(row_select, i) {
            Bit::One => {
                for col in 0..cols {
                    grid[start + col] = bit_at(col_data, col);
                }
                continue;
            }
            Bit::Zero => Bit::Zero,
            Bit::Unknown | Bit::Error => Bit::Error,
        };
        for col in 0..cols {
            grid[start + col] = fill;
        }
    }
}

/// One character's port-width (7-bit) encoding, LSB-first — matches
/// `Value.createKnown(BitWidth.create(7), c & 0x7F)`.
fn char_bits(c: char) -> Signal {
    let code = c as u32 & 0x7F;
    (0..7).map(|i| if (code >> i) & 1 != 0 { Bit::One } else { Bit::Zero }).collect()
}

/// The inverse of `char_bits` — `'?'` convention matches `Tty.propagate`'s
/// own `in.isFullyDefined() ? (char) in.toIntValue() : '?'`, applied by the
/// caller when a bit isn't cleanly `Zero`/`One`.
fn bits_to_char(bits: &Signal) -> char {
    let mut code = 0u32;
    for i in 0..7 {
        if bit_at(bits, i) == Bit::One {
            code |= 1 << i;
        }
    }
    code as u8 as char
}

/// `TtyState.add`, ported: control-L clears everything, backspace erases
/// the last character of the in-progress row, newline/CR commits it,
/// anything else (non-control) appends, auto-committing first if the row
/// is already full. Takes `cols`/`rows` each call rather than storing them
/// on a struct — `Gate::Tty`'s own fields already carry them, this is just
/// the free function `eval_io` calls.
fn tty_add(cols: u8, rows: u8, row_data: &mut [Vec<char>], last_row: &mut Vec<char>, row: &mut usize, c: char) {
    match c {
        '\u{0c}' => {
            *row = 0;
            last_row.clear();
            for r in row_data.iter_mut() {
                r.clear();
            }
        }
        '\u{8}' => {
            last_row.pop();
        }
        '\n' | '\r' => tty_commit(rows, row_data, last_row, row),
        c if !c.is_control() => {
            if last_row.len() as u8 == cols {
                tty_commit(rows, row_data, last_row, row);
            }
            last_row.push(c);
        }
        _ => {}
    }
}

/// `TtyState.commit`: pushes `last_row` into `row_data`, scrolling the
/// oldest committed row out once the `rows-1`-deep history is full (a
/// `Tty` shows `rows` rows total: `rows-1` committed ones plus the
/// in-progress row itself, exactly `TtyState`'s own `rowData = new
/// String[rows-1]`). `rows == 1` (no committed history at all) is a
/// degenerate case real Logisim's own `commit()` would throw on
/// (`System.arraycopy(..., -1)`) — not replicated; here it just discards
/// the committed line, same "don't reproduce an upstream crash" call as
/// `set_select`'s single-row/col `Select` case.
fn tty_commit(rows: u8, row_data: &mut [Vec<char>], last_row: &mut Vec<char>, row: &mut usize) {
    let committed = rows as usize - 1;
    if *row >= committed {
        if committed > 0 {
            row_data.rotate_left(1);
            row_data[committed - 1] = std::mem::take(last_row);
        } else {
            last_row.clear();
        }
    } else {
        row_data[*row] = std::mem::take(last_row);
        *row += 1;
    }
}

fn keyboard_dequeue(buffer: &mut Vec<char>, cursor: &mut usize) -> char {
    if buffer.is_empty() {
        return '\0';
    }
    let c = buffer.remove(0);
    if *cursor > 0 {
        *cursor -= 1;
    }
    c
}

/// `KeyboardData.insert`: fails silently (no-op) once `buffer.len()`
/// reaches `capacity` — matches `if (len >= buf.length) return false;`.
fn keyboard_insert(buffer: &mut Vec<char>, cursor: &mut usize, capacity: u16, c: char) {
    if buffer.len() >= capacity as usize {
        return;
    }
    buffer.insert(*cursor, c);
    *cursor += 1;
}

/// `KeyboardData.delete`: removes the character *after* the cursor (like a
/// terminal's Delete key, not Backspace) — a no-op at end-of-buffer.
fn keyboard_delete(buffer: &mut Vec<char>, cursor: usize) {
    if cursor < buffer.len() {
        buffer.remove(cursor);
    }
}

/// `KeyboardData.moveCursorBy`: clamped to `0..=buffer.len()`, a no-op if
/// the move would leave that range.
fn keyboard_move_cursor(buffer: &[char], cursor: &mut usize, delta: i32) {
    let new_pos = *cursor as i32 + delta;
    if new_pos >= 0 && new_pos as usize <= buffer.len() {
        *cursor = new_pos as usize;
    }
}

fn keyboard_set_cursor(buffer: &[char], cursor: &mut usize, value: usize) {
    *cursor = value.min(buffer.len());
}

fn serialize_chars(out: &mut Vec<u8>, chars: &[char]) {
    out.extend((chars.len() as u32).to_le_bytes());
    for &c in chars {
        out.extend((c as u32).to_le_bytes());
    }
}

fn read_u32(bytes: &[u8], offset: &mut usize) -> u32 {
    let v = bytes.get(*offset..*offset + 4).and_then(|b| b.try_into().ok()).map(u32::from_le_bytes).unwrap_or(0);
    *offset += 4;
    v
}

fn deserialize_chars(bytes: &[u8], offset: &mut usize) -> Vec<char> {
    let len = read_u32(bytes, offset) as usize;
    (0..len).map(|_| char::from_u32(read_u32(bytes, offset)).unwrap_or('\0')).collect()
}

fn serialize_char_rows(out: &mut Vec<u8>, rows: &[Vec<char>]) {
    out.extend((rows.len() as u32).to_le_bytes());
    for row in rows {
        serialize_chars(out, row);
    }
}

fn deserialize_char_rows(bytes: &[u8], offset: &mut usize) -> Vec<Vec<char>> {
    let len = read_u32(bytes, offset) as usize;
    (0..len).map(|_| deserialize_chars(bytes, offset)).collect()
}

impl Gate {
    pub(super) fn init_io(&mut self) {
        match self {
            Gate::DotMatrix { rows, cols, grid, .. } => *grid = vec![Bit::Unknown; *rows as usize * *cols as usize],
            // `TtyState`'s own constructor initializes `lastClock` to
            // `Value.UNKNOWN`, not `Value.FALSE` the way `ClockState`
            // (`Register`/`Clock`/`Random`) does — a real, if minor,
            // difference between the two lineages, kept rather than
            // normalized away.
            Gate::Tty { row_data, last_row, row, last_clock, .. } => {
                for r in row_data.iter_mut() {
                    r.clear();
                }
                last_row.clear();
                *row = 0;
                *last_clock = Bit::Unknown;
            }
            // Same `Value.UNKNOWN` convention as `Tty` above —
            // `KeyboardData`'s own constructor, not `ClockState`'s.
            Gate::Keyboard { buffer, cursor, last_clock, .. } => {
                buffer.clear();
                *cursor = 0;
                *last_clock = Bit::Unknown;
            }
            _ => unreachable!("dispatch bug: not an io gate"),
        }
    }

    pub(super) fn input_count_io(&self) -> usize {
        match self {
            Gate::DotMatrix { cols, input: DotMatrixInput::Column, .. } => *cols as usize,
            Gate::DotMatrix { rows, input: DotMatrixInput::Row, .. } => *rows as usize,
            Gate::DotMatrix { input: DotMatrixInput::Select, .. } => 2,
            Gate::Tty { .. } => 4,      // clear, ck, we, in
            Gate::Keyboard { .. } => 3, // clear, ck, re
            _ => unreachable!("dispatch bug: not an io gate with inputs"),
        }
    }

    /// `DotMatrix`'s per-mode pin width (see `Gate::DotMatrix`'s doc);
    /// `Tty`'s fixed `clear`/`ck`/`we`/`in` shape (only `in` is 7 bits);
    /// `Keyboard`'s three 1-bit control pins.
    pub(super) fn input_width_io(&self, pin: usize) -> u8 {
        match self {
            Gate::DotMatrix { rows, input: DotMatrixInput::Column, .. } => *rows,
            Gate::DotMatrix { cols, input: DotMatrixInput::Row, .. } => *cols,
            Gate::DotMatrix { rows, cols, input: DotMatrixInput::Select, .. } => {
                if pin == 0 {
                    *cols
                } else {
                    *rows
                }
            }
            Gate::Tty { .. } => {
                if pin == 3 {
                    7
                } else {
                    1
                }
            }
            Gate::Keyboard { .. } => 1,
            _ => unreachable!("dispatch bug: not an io gate with inputs"),
        }
    }

    /// Only `Keyboard` has output pins among this module's gates (`avl`
    /// 1-bit, `out` 7-bit) — `DotMatrix`/`Tty` are pure sinks, never
    /// dispatched here (`Gate::output_width`'s own `Keyboard`-only arm).
    pub(super) fn output_width_io(&self, pin: usize) -> u8 {
        match self {
            Gate::Keyboard { .. } => {
                if pin == 0 {
                    1
                } else {
                    7
                }
            }
            _ => unreachable!("dispatch bug: not an io gate with outputs"),
        }
    }

    pub(super) fn eval_io(&mut self, inputs: &[Signal]) -> Vec<Signal> {
        match self {
            Gate::DotMatrix { rows, cols, input, grid } => {
                match input {
                    DotMatrixInput::Column => {
                        for (col, signal) in inputs.iter().enumerate() {
                            set_column(grid, *rows, *cols, col, signal);
                        }
                    }
                    DotMatrixInput::Row => {
                        for (row, signal) in inputs.iter().enumerate() {
                            set_row(grid, *cols, row, signal);
                        }
                    }
                    DotMatrixInput::Select => set_select(grid, *rows, *cols, &inputs[1], &inputs[0]),
                }
                Vec::new()
            }
            Gate::Tty { cols, rows, trigger, last_clock, row_data, last_row, row } => {
                let clear = bit_at(&inputs[0], 0);
                let ck = bit_at(&inputs[1], 0);
                let we = bit_at(&inputs[2], 0);
                let in_signal = &inputs[3];

                let triggered = trigger.fired(*last_clock, ck);
                *last_clock = ck;

                if clear == Bit::One {
                    for r in row_data.iter_mut() {
                        r.clear();
                    }
                    last_row.clear();
                    *row = 0;
                } else if we != Bit::Zero && triggered {
                    let fully_defined = (0..7).all(|i| matches!(bit_at(in_signal, i), Bit::Zero | Bit::One));
                    let c = if fully_defined { bits_to_char(in_signal) } else { '?' };
                    tty_add(*cols, *rows, row_data, last_row, row, c);
                }
                Vec::new()
            }
            Gate::Keyboard { trigger, last_clock, buffer, cursor, .. } => {
                let clear = bit_at(&inputs[0], 0);
                let ck = bit_at(&inputs[1], 0);
                let re = bit_at(&inputs[2], 0);

                let triggered = trigger.fired(*last_clock, ck);
                *last_clock = ck;

                if clear == Bit::One {
                    buffer.clear();
                    *cursor = 0;
                } else if re != Bit::Zero && triggered {
                    keyboard_dequeue(buffer, cursor);
                }

                let c = buffer.first().copied().unwrap_or('\0');
                let avl = if c != '\0' { Bit::One } else { Bit::Zero };
                vec![vec![avl], char_bits(c)]
            }
            _ => unreachable!("dispatch bug: not an io gate"),
        }
    }

    pub(super) fn serialize_io(&self) -> Vec<u8> {
        match self {
            Gate::DotMatrix { grid, .. } => grid.iter().copied().map(bit_to_byte).collect(),
            Gate::Tty { row_data, last_row, row, last_clock, .. } => {
                let mut out = vec![bit_to_byte(*last_clock)];
                out.extend((*row as u32).to_le_bytes());
                serialize_char_rows(&mut out, row_data);
                serialize_chars(&mut out, last_row);
                out
            }
            Gate::Keyboard { buffer, cursor, last_clock, .. } => {
                let mut out = vec![bit_to_byte(*last_clock)];
                out.extend((*cursor as u32).to_le_bytes());
                serialize_chars(&mut out, buffer);
                out
            }
            _ => unreachable!("dispatch bug: not an io gate"),
        }
    }

    pub(super) fn deserialize_io(&mut self, state: &[u8]) {
        match self {
            Gate::DotMatrix { grid, .. } => {
                for (i, cell) in grid.iter_mut().enumerate() {
                    *cell = state.get(i).copied().map(byte_to_bit).unwrap_or(Bit::Unknown);
                }
            }
            Gate::Tty { row_data, last_row, row, last_clock, .. } => {
                *last_clock = state.first().copied().map(byte_to_bit).unwrap_or(Bit::Unknown);
                let mut offset = 1usize;
                *row = read_u32(state, &mut offset) as usize;
                *row_data = deserialize_char_rows(state, &mut offset);
                *last_row = deserialize_chars(state, &mut offset);
            }
            Gate::Keyboard { buffer, cursor, last_clock, .. } => {
                *last_clock = state.first().copied().map(byte_to_bit).unwrap_or(Bit::Unknown);
                let mut offset = 1usize;
                *cursor = read_u32(state, &mut offset) as usize;
                *buffer = deserialize_chars(state, &mut offset);
            }
            _ => unreachable!("dispatch bug: not an io gate"),
        }
    }

    pub(super) fn actions_io(&self) -> Vec<&'static str> {
        match self {
            // `Keyboard.Poker`'s `keyPressed`/`keyTyped`, ported as named
            // actions (see `Gate::Keyboard`'s doc comment for why `invoke`
            // is the right stand-in for "the canvas editor has focus").
            Gate::Keyboard { .. } => vec!["key", "delete", "left", "right", "home", "end"],
            _ => Vec::new(),
        }
    }

    pub(super) fn invoke_io(&mut self, name: &str, arg: Option<Value>) -> Result<(), ActionError> {
        match self {
            Gate::Keyboard { buffer, cursor, capacity, .. } => match name {
                "key" => match arg {
                    Some(Value::Int(code)) => match char::from_u32(code as u32) {
                        Some(c) => {
                            keyboard_insert(buffer, cursor, *capacity, c);
                            Ok(())
                        }
                        None => Err(ActionError::InvalidArg { action: "key".to_string(), reason: format!("not a valid char code: {code}") }),
                    },
                    other => Err(ActionError::InvalidArg { action: "key".to_string(), reason: format!("expected Int, got {other:?}") }),
                },
                "delete" => {
                    keyboard_delete(buffer, *cursor);
                    Ok(())
                }
                "left" => {
                    keyboard_move_cursor(buffer, cursor, -1);
                    Ok(())
                }
                "right" => {
                    keyboard_move_cursor(buffer, cursor, 1);
                    Ok(())
                }
                "home" => {
                    keyboard_set_cursor(buffer, cursor, 0);
                    Ok(())
                }
                "end" => {
                    keyboard_set_cursor(buffer, cursor, usize::MAX);
                    Ok(())
                }
                other => Err(ActionError::UnknownAction(other.to_string())),
            },
            _ => unreachable!("dispatch bug: not an io gate with actions"),
        }
    }

    pub(super) fn readouts_io(&self) -> Vec<&'static str> {
        match self {
            Gate::DotMatrix { .. } => vec!["get"],
            Gate::Tty { .. } => vec!["cursor_row", "cursor_col"],
            Gate::Keyboard { .. } => vec!["get", "avail", "cursor", "len"],
            _ => unreachable!("dispatch bug: not an io gate"),
        }
    }

    /// `DotMatrix`: `"get"` -> the whole grid, row-major (see `Gate::
    /// DotMatrix`'s doc). `Tty`: `"cursor_row"`/`"cursor_col"`, plus a
    /// dynamically-named `"row<N>"` per row (committed or in-progress,
    /// `TtyState.getRowString`'s own three-way split) packed as 7-bit-per-
    /// char `Bits` — not enumerable via `readouts_io` (which only returns
    /// `&'static str`s), but that list is informational only (`ctest::run`
    /// dispatches `read` directly by name, never validates against it
    /// first). `Keyboard`: `"get"` (front-of-queue char, 7-bit, matches the
    /// `out` pin), `"avail"` (matches `avl`), `"cursor"`/`"len"`.
    pub(super) fn read_io(&self, name: &str) -> Result<Value, ReadoutError> {
        match self {
            Gate::DotMatrix { grid, .. } => {
                if name == "get" {
                    Ok(Value::Bits(grid.clone()))
                } else {
                    Err(ReadoutError::UnknownReadout(name.to_string()))
                }
            }
            Gate::Tty { row, last_row, row_data, .. } => match name {
                "cursor_row" => Ok(Value::Int(*row as i64)),
                "cursor_col" => Ok(Value::Int(last_row.len() as i64)),
                _ => match name.strip_prefix("row").and_then(|n| n.parse::<usize>().ok()) {
                    Some(index) => {
                        let chars: &[char] = match index.cmp(row) {
                            std::cmp::Ordering::Less => &row_data[index],
                            std::cmp::Ordering::Equal => last_row,
                            std::cmp::Ordering::Greater => &[],
                        };
                        let bits: Signal = chars.iter().flat_map(|&c| char_bits(c)).collect();
                        Ok(Value::Bits(bits))
                    }
                    None => Err(ReadoutError::UnknownReadout(name.to_string())),
                },
            },
            Gate::Keyboard { buffer, cursor, .. } => {
                let c = buffer.first().copied().unwrap_or('\0');
                match name {
                    "get" => Ok(Value::Bits(char_bits(c))),
                    "avail" => Ok(Value::Bool(c != '\0')),
                    "cursor" => Ok(Value::Int(*cursor as i64)),
                    "len" => Ok(Value::Int(buffer.len() as i64)),
                    _ => Err(ReadoutError::UnknownReadout(name.to_string())),
                }
            }
            _ => unreachable!("dispatch bug: not an io gate"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::Trigger;
    use plugin_abi::Component;

    fn zeros(n: u8) -> Signal {
        vec![Bit::Zero; n as usize]
    }

    fn dot_matrix(rows: u8, cols: u8, input: DotMatrixInput) -> Gate {
        Gate::DotMatrix { rows, cols, input, grid: vec![Bit::Unknown; rows as usize * cols as usize] }
    }

    fn tty(cols: u8, rows: u8) -> Gate {
        Gate::Tty { cols, rows, trigger: Trigger::Rising, last_clock: Bit::Unknown, row_data: vec![Vec::new(); rows as usize - 1], last_row: Vec::new(), row: 0 }
    }

    fn keyboard(capacity: u16) -> Gate {
        Gate::Keyboard { capacity, trigger: Trigger::Rising, last_clock: Bit::Unknown, buffer: Vec::new(), cursor: 0 }
    }

    fn get_bits(g: &Gate, name: &str) -> Vec<Bit> {
        match g.read(name).unwrap() {
            Value::Bits(bits) => bits,
            other => panic!("unexpected readout {other:?}"),
        }
    }

    /// `0b100` (bit 0/LSB = `Zero`, bit 2/MSB = `One`) into column 0 of a
    /// 3x1 column-mode matrix must light the *bottom* row (LSB) `Zero` and
    /// the *top* row (MSB) `One` — `set_column`'s documented convention,
    /// asymmetric input so the mapping direction is actually pinned down.
    #[test]
    fn dot_matrix_column_mode_lsb_is_the_bottom_row() {
        let mut g = dot_matrix(3, 1, DotMatrixInput::Column);
        g.eval(&[vec![Bit::Zero, Bit::Zero, Bit::One]]);
        assert_eq!(get_bits(&g, "get"), vec![Bit::One, Bit::Zero, Bit::Zero], "row 0 (top) = MSB = One, row 2 (bottom) = LSB = Zero");
    }

    /// `0b100` (bit 0/LSB = `Zero`, bit 2/MSB = `One`) into row 0 of a 1x3
    /// row-mode matrix must light the *leftmost* column (MSB) `One` and
    /// the *rightmost* column (LSB) `Zero` — `set_row`'s documented
    /// (opposite-of-column) convention.
    #[test]
    fn dot_matrix_row_mode_lsb_is_the_rightmost_column() {
        let mut g = dot_matrix(1, 3, DotMatrixInput::Row);
        g.eval(&[vec![Bit::Zero, Bit::Zero, Bit::One]]);
        assert_eq!(get_bits(&g, "get"), vec![Bit::One, Bit::Zero, Bit::Zero], "col 0 (left) = MSB = One, col 2 (right) = LSB = Zero");
    }

    /// Select mode, 2 rows x 3 cols: selecting row 0 (MSB of the 2-bit
    /// row-select) with col data `0b101` must light the *top* row,
    /// leftmost column = LSB of the col data (opposite convention from
    /// `set_row`, see `set_select`'s doc) — the other (unselected, select
    /// bit `Zero`) row reads all-`Zero`.
    #[test]
    fn dot_matrix_select_mode_row0_is_msb_of_row_select_col_data_lsb_is_leftmost() {
        let mut g = dot_matrix(2, 3, DotMatrixInput::Select);
        // col_data = 0b101 (3 bits), row_select = 0b10 (2 bits: bit1=row0 selected)
        g.eval(&[vec![Bit::One, Bit::Zero, Bit::One], vec![Bit::Zero, Bit::One]]);
        assert_eq!(get_bits(&g, "get"), vec![Bit::One, Bit::Zero, Bit::One, Bit::Zero, Bit::Zero, Bit::Zero]);
    }

    /// An `Unknown`/`Error` row-select bit paints that whole row `Error`,
    /// not per-cell — `DotMatrix.State.setSelect`'s `if (wholeRow !=
    /// FALSE) wholeRow = ERROR` applied uniformly.
    #[test]
    fn dot_matrix_select_mode_undefined_row_select_bit_errors_the_whole_row() {
        let mut g = dot_matrix(1, 2, DotMatrixInput::Select);
        g.eval(&[vec![Bit::One, Bit::Zero], vec![Bit::Unknown]]);
        assert_eq!(get_bits(&g, "get"), vec![Bit::Error, Bit::Error]);
    }

    /// `Tty` clocked-in characters commit on `\n`, visible via the
    /// dynamically-named `rowN` readouts; a 3-row `Tty` has 2 committed
    /// slots plus the in-progress row.
    #[test]
    fn tty_commits_a_row_on_newline_and_starts_the_next() {
        let mut g = tty(8, 3);
        for c in "hi\n".chars() {
            g.eval(&[zeros(1), zeros(1), vec![Bit::One], char_bits(c)]);
            g.eval(&[zeros(1), vec![Bit::One], vec![Bit::One], char_bits(c)]); // rising edge
            g.eval(&[zeros(1), zeros(1), vec![Bit::One], char_bits(c)]); // falling edge, back to idle
        }
        assert_eq!(g.read("row0").unwrap(), Value::Bits(chars_bits("hi")));
        assert_eq!(g.read("cursor_row").unwrap(), Value::Int(1));
        assert_eq!(g.read("cursor_col").unwrap(), Value::Int(0));
    }

    fn chars_bits(s: &str) -> Signal {
        s.chars().flat_map(char_bits).collect()
    }

    /// `we == Zero` must not accept a character even across a clock edge —
    /// `TtyState`'s own `else if (enable != FALSE)` guard.
    #[test]
    fn tty_ignores_clock_edges_while_write_enable_is_low() {
        let mut g = tty(8, 3);
        g.eval(&[zeros(1), zeros(1), zeros(1), char_bits('x')]);
        g.eval(&[zeros(1), vec![Bit::One], zeros(1), char_bits('x')]);
        assert_eq!(g.read("cursor_col").unwrap(), Value::Int(0));
    }

    /// `clear == One` wins outright and resets the cursor, regardless of
    /// the clock — `TtyState`'s `if (clear == TRUE) state.clear();` with
    /// no `else` fallthrough to the enable/clock check.
    #[test]
    fn tty_clear_resets_everything() {
        let mut g = tty(8, 3);
        g.eval(&[zeros(1), zeros(1), vec![Bit::One], char_bits('x')]);
        g.eval(&[zeros(1), vec![Bit::One], vec![Bit::One], char_bits('x')]);
        assert_eq!(g.read("cursor_col").unwrap(), Value::Int(1));
        g.eval(&[vec![Bit::One], vec![Bit::One], vec![Bit::One], char_bits('x')]);
        assert_eq!(g.read("cursor_col").unwrap(), Value::Int(0));
    }

    /// `Keyboard`'s external input is `invoke`, not a port — `"key"`
    /// inserts, `AVL`/`OUT` report the front of the queue, and a rising
    /// `ck` edge (while `re != Zero`) dequeues it.
    #[test]
    fn keyboard_key_then_dequeue_on_clock_edge() {
        let mut g = keyboard(8);
        g.invoke("key", Some(Value::Int('A' as i64))).unwrap();
        g.invoke("key", Some(Value::Int('B' as i64))).unwrap();

        let out = g.eval(&[zeros(1), zeros(1), zeros(1)]);
        assert_eq!(out[0], vec![Bit::One], "avl: a character is ready");
        assert_eq!(out[1], char_bits('A'));

        let out = g.eval(&[zeros(1), vec![Bit::One], vec![Bit::One]]); // rising edge, re enabled
        assert_eq!(out[1], char_bits('B'), "dequeued 'A', 'B' now at the front");
    }

    /// Inserting past `capacity` is a silent no-op — `KeyboardData.insert`'s
    /// own `if (len >= buf.length) return false;`.
    #[test]
    fn keyboard_insert_past_capacity_is_a_no_op() {
        let mut g = keyboard(1);
        g.invoke("key", Some(Value::Int('A' as i64))).unwrap();
        g.invoke("key", Some(Value::Int('B' as i64))).unwrap();
        assert_eq!(g.read("len").unwrap(), Value::Int(1));
        assert_eq!(g.read("get").unwrap(), Value::Bits(char_bits('A')));
    }

    /// An empty queue reports `'\0'`/`avail == false` — `KeyboardData.
    /// getChar`'s own `'\0'` sentinel for "nothing there".
    #[test]
    fn keyboard_empty_queue_reports_not_available() {
        let g = keyboard(8);
        assert_eq!(g.read("avail").unwrap(), Value::Bool(false));
        assert_eq!(g.read("get").unwrap(), Value::Bits(char_bits('\0')));
    }
}

# Motorola 6802 Emulator on ESP32

**Copyright (c) 2026 Kamel Khelif — MIT License (open source)**

A complete emulator of the Motorola 6802 8-bit microprocessor (1977) running on an
ESP32 microcontroller via Arduino IDE. No program is built-in — you load your own
machine code from the serial terminal.

---

## What's new in v7

| # | Change |
|---|--------|
| ★ | **NVS Flash slots** — `save`, `load`, `slots`, `del` — 8 persistent program slots |
| ★ | **S19 Motorola SREC** — `xs` export and `xsr` import |
| ★ | **Disassembler** — full 6802 instruction set (no more `???` for valid opcodes) |
| 🔧 | **Program B2 (HELLO)** — corrected: ends with `BRA $0112` (20 FE) not WAI |

---

## License

```
MIT License
Copyright (c) 2026 Kamel Khelif

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the "Software"), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify,
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be included in all copies
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```

---

## Project files

```
esp32_6802/
├── esp32_6802.ino   ← Main Arduino sketch  (open this first)
├── cpu6802.h        ← CPU structure, constants, public API
└── cpu6802.cpp      ← Emulator core — all instructions, interrupts, I/O
```

---

## Requirements

| Item | Detail | Mandatory |
|------|--------|-----------|
| ESP32 dev board | Any model with USB | Yes |
| Arduino IDE | 1.8.x or 2.x | Yes |
| ESP32 Arduino core | Via Board Manager + **Preferences library included** | Yes |
| Serial terminal | PuTTY — 115200 baud, **Local echo: Force OFF** | Yes |
| USB cable | To flash the ESP32 | Yes |

> The `Preferences` library (used for NVS slots) is bundled with the ESP32 Arduino
> core — no extra install needed.

---

## Installation

1. Place all three files in a folder named `esp32_6802/`
2. Open `esp32_6802.ino` in Arduino IDE
3. **Tools → Board → ESP32 Dev Module**
4. Select the correct COM port
5. Click Upload
6. Open PuTTY: 115200 baud, **Local echo = Force OFF**, **Local line editing = Force OFF**
7. Press RESET on the ESP32

**You should see:**
```
========================================
  Motorola 6802 Emulator on ESP32  v7
  Copyright (c) 2026 Kamel Khelif
  MIT License — open source
========================================
Memory is empty. Commands:
  w ADDR     write program
  load N     restore from Flash slot
  xsr        import S19 file
  g ADDR     run
  q          full help

>
```

> ⚠️ **PuTTY — Local echo must be "Force OFF".**
> If it is ON every character appears twice and commands will not work.

---

## Monitor command reference

### Execution

| Command | Description |
|---------|-------------|
| `r` | Reset CPU |
| `g ADDR` | Go — set PC = ADDR and run |
| `g` | Go — run from last saved address (after `load` or `xsr`) |
| `p` | Pause |
| `s` / `s N` | Step 1 or N instructions |
| `d` | Toggle debug trace |
| `i` | Trigger IRQ |
| `n` | Trigger NMI |

### Inspect

| Command | Description |
|---------|-------------|
| `x` | Show all registers |
| `v [ADDR [N]]` | Hex + disassembly view |
| `m ADDR` | Hex memory dump (64 bytes) |

### Write

| Command | Description |
|---------|-------------|
| `w ADDR` | Continuous write mode (`.` to exit) |
| `w ADDR VAL` | Write a single byte |

### Save / Load — NVS Flash ★ new in v7

| Command | Description |
|---------|-------------|
| `slots` | List all 8 Flash slots |
| `save N START END [NAME]` | Save bytes $START–$END to slot N |
| `load N` | Restore slot N to RAM, set go_addr |
| `del N` | Erase slot N |

### S19 Motorola SREC ★ new in v7

| Command | Description |
|---------|-------------|
| `xs ADDR LEN [NAME]` | Export S19 to terminal |
| `xsr` | Import S19 — paste records, type `.` to end |

---

## Quick start — display 'A'

```
> w 0100
$0100: 8E  $0101: 00  $0102: FF    LDS #$00FF
$0103: 86  $0104: 41               LDAA #$41 ('A')
$0105: B7  $0106: 80  $0107: 00    STAA $8000 (UART)
$0108: 20  $0109: FE               BRA $0108  (halt)
.
> v 0100
> g 0100
A
```

---

## Program B2 — HELLO (corrected in v7)

### What was wrong

The previous version ended with `WAI` (`3E`). On the ESP32, WAI does not truly
freeze the CPU — the loop restarted and the terminal was flooded with garbage.

**Fix:** Replace WAI with `BRA $0112` (`20 FE`). The CPU loops on itself without
sending anything to the UART.

### Corrected listing

```
$0100    8E 00 FF    LDS  #$00FF     Init stack pointer
$0103    CE 02 00    LDX  #$0200     X -> string at $0200
$0106    A6 00       LDAA 0,X        Load char into A
$0108    81 00       CMPA #0         Null terminator?
$010A    27 04       BEQ  $0110      Yes -> exit loop (offset=+4)
$010C    B7 80 00    STAA $8000      No  -> send to UART
$010F    08          INX             X++
$0110    20 F4       BRA  $0106      Loop back (0xF4=-12)
$0112    20 FE       BRA  $0112      Halt — loop on itself
```

Offset verification:
```
BEQ at $010A, target $0110:  $0110 - ($010A+2) = $0110 - $010C = +4 = 0x04  v
BRA at $0110, target $0106:  $0106 - ($0110+2) = $0106 - $0112 = -12 = 0xF4 v
BRA at $0112, target $0112:  $0112 - ($0112+2) = -2 = 0xFE                  v
```

> ⚠️ BEQ offset is `0x04` not `0x03` — it jumps over INX to reach BRA $0112.

### w commands

```
> w 0100
$0100: 8E  $0101: 00  $0102: FF
$0103: CE  $0104: 02  $0105: 00
$0106: A6  $0107: 00
$0108: 81  $0109: 00
$010A: 27  $010B: 04
$010C: B7  $010D: 80  $010E: 00
$010F: 08
$0110: 20  $0111: F4
$0112: 20  $0113: FE
.

> w 0200
$0200: 48  $0201: 45  $0202: 4C  $0203: 4C  $0204: 4F
$0205: 0D  $0206: 0A  $0207: 00
.

> v 0100
> g 0100
HELLO
```

---

## NVS Flash slots ★ new in v7

The ESP32 NVS stores up to 8 program slots that survive power cuts and resets.

### Storage format

Each slot N uses 4 NVS keys in namespace `6802`:

| Key | Type | Content |
|-----|------|---------|
| `sN_nm` | String | Program name (max 31 chars) |
| `sN_sa` | UInt16 | Start address |
| `sN_ea` | UInt16 | End address (inclusive) |
| `sN_dt` | Bytes | Raw memory bytes |

### Example session

```
> save 0 0100 0207 HELLO
[SAVE] Slot 0 saved: "HELLO"  $0100-$0207  (264 bytes)

> slots
╔═══╦══════════════════╦═══════════╦═════════╗
║ # ║ Name             ║ Range     ║ Bytes   ║
╠═══╬══════════════════╬═══════════╬═════════╣
║ 0 ║ HELLO            ║ $0100-$0207 ║   264  ║
║ 1 ║ (empty)          ║ ---       ║ ---     ║
...

; Next session:
> load 0
[LOAD] Slot 0 restored  go_addr=$0100
> g
HELLO
```

> Include interrupt vectors in your save range if needed.
> Example: `save 2 0100 FFFF IRQ_PROG` saves code and all vectors.

---

## S19 Motorola SREC ★ new in v7

S19 (SREC) is the standard format produced by Motorola 6802 assemblers.

### Record layout

```
S1  13  0100  8E 00 FF ...data...  XX
│   │   │     └── data bytes       │
│   │   └─ 16-bit address          │
│   └─ byte count = 2+len+1        │
└─ type                  checksum ─┘

Checksum = ~(bytecount + addr_hi + addr_lo + sum_of_data) & 0xFF
```

| Type | Purpose |
|------|---------|
| `S0` | Header — program name |
| `S1` | Data — bytes at 16-bit address |
| `S9` | End — execution start address |

### Export

```
> xs 0100 14 HELLO
; --- Motorola S19 Export ---
S0070000 48454C4C4F 00 B0
S1130100 8E00FFCE0200A60081002704B780000820F420FE XX
S9030100 FB
; --- End of S19 ---
```

### Import

```
> xsr
[XSR] S19 import mode. Paste records. Type '.' to end.
S1130100 8E00FF...
[S19] $0100 +16 bytes OK
S9030100FB
[S19] Import complete. Type 'g' to run.
> g
HELLO
```

If a checksum error occurs, re-paste that line.

---

## Memory map

| Range | Description |
|-------|-------------|
| `$0000–$007F` | Internal RAM (128 bytes, zero page) |
| `$0080–$7FFF` | Free RAM |
| `$8000` | UART DATA — write=TX, read=RX |
| `$8001` | UART STATUS — bit0=RX ready, bit1=TX ready |
| `$8010` | GPIO_DATA — user peripheral |
| `$8011` | GPIO_DIR — user peripheral direction |
| `$F000–$FFFF` | High memory (RAM, writable) |
| `$FFF8–$FFF9` | IRQ vector |
| `$FFFA–$FFFB` | SWI vector |
| `$FFFC–$FFFD` | NMI vector |
| `$FFFE–$FFFF` | RESET vector |

---

## Registers and flags

```
A, B : 8-bit accumulators
X    : 16-bit index register
SP   : 16-bit stack pointer
PC   : 16-bit program counter
CC   : H I N Z V C
       │ │ │ │ │ └─ C  Carry
       │ │ │ │ └─── V  Overflow
       │ │ │ └───── Z  Zero
       │ │ └─────── N  Negative
       │ └───────── I  IRQ Mask (1=disabled)
       └─────────── H  Half-carry (BCD)
```

---

## Interrupt handling

```
; Set IRQ vector to $0200
> w FFF8 02
> w FFF9 00

; Handler must end with RTI (3B), never RTS (39)
$0200: 0E   CLI
...
$02xx: 3B   RTI
```

---

## Branch offset formula

```
offset = target − (branch_addr + 2)    range: -128 to +127
Beyond range: use JMP addr (7E hh ll)
```

---

## Adding your own peripherals

Edit `user_io_read()` / `user_io_write()` in `esp32_6802.ino`:

```cpp
void user_io_write(uint16_t addr, uint8_t val) {
    if (addr == 0x8000) { Serial.write(val); return; }
    if (addr == 0x8010) { digitalWrite(MY_PIN, val ? HIGH : LOW); }
}
```

---

## Common pitfalls

| Symptom | Cause | Fix |
|---------|-------|-----|
| CPU jumps to random address | Wrong branch offset | Recalculate: `target − (branch+2)` |
| HELLO loops forever | Wrong BEQ offset | BEQ offset must be `0x04`, verify with `v 0100` |
| Garbage on terminal | WAI at end instead of `BRA $x` | Replace `3E` with `20 FE` |
| IRQ ignored | CLI not called | Add `0E` after `LDS #$00FF` |
| Crash after JSR | Missing RTS | Last byte must be `39` |
| Crash after interrupt | RTS used instead of RTI | Replace `39` with `3B` |
| S19 checksum error | Corrupt paste | Re-paste the failing line |
| `??? $A6` in disasm | Old firmware v6 | Re-upload — fixed in v7 |
| `w` doesn't respond | Local echo ON | Force OFF in PuTTY |

---

*Motorola 6802 — 8-bit, 64 KB, 72 instructions, 1977*
*Copyright (c) 2026 Kamel Khelif — MIT License*

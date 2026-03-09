// ============================================================
//  MIT License
//
//  Copyright (c) 2026 Kamel Khelif
//
//  Permission is hereby granted, free of charge, to any person
//  obtaining a copy of this software and associated documentation
//  files (the "Software"), to deal in the Software without
//  restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or
//  sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following
//  conditions:
//
//  The above copyright notice and this permission notice shall be
//  included in all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
//  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
//  OTHER DEALINGS IN THE SOFTWARE.
// ============================================================
//
//  MOTOROLA 6802 EMULATOR FOR ESP32  —  esp32_6802.ino
//  Main Arduino sketch  (v7 — NVS slots + S19 import/export)
//
//  NEW in v7:
//    save SLOT START END [NAME]  — save RAM range to NVS slot (0-7)
//    load SLOT                   — restore NVS slot to RAM
//    slots                       — list all saved slots
//    del  SLOT                   — erase a slot
//    xs   ADDR LEN               — export S19 to terminal
//    xsr                         — receive S19 from terminal (paste mode)
//
//  S19 format (Motorola SREC):
//    S0  — header record (program name)
//    S1  — data record with 16-bit address
//    S9  — end-of-file record with start address
//
//  NVS slot format (Preferences / NVS):
//    Key "sN_nm"  String  — program name  (N = slot 0-7)
//    Key "sN_sa"  UInt16  — save start address
//    Key "sN_ea"  UInt16  — save end address
//    Key "sN_dt"  Bytes   — raw memory bytes
//
//  Memory map:
//    $0000-$007F   Internal RAM (128 bytes, zero page)
//    $0080-$7FFF   Free RAM
//    $8000         UART DATA  (write = TX,  read = RX)
//    $8001         UART STATUS (bit0=RX ready, bit1=TX ready)
//    $8010         GPIO_DATA  (user peripheral — wire as needed)
//    $8011         GPIO_DIR   (user peripheral — direction)
//    $F000-$FFFF   High memory (RAM, writable)
//      $FFF8-$FFF9   IRQ  vector
//      $FFFA-$FFFB   SWI  vector
//      $FFFC-$FFFD   NMI  vector
//      $FFFE-$FFFF   RESET vector
//
// ============================================================

#include "cpu6802.h"
#include <Preferences.h>

CPU6802    cpu;
Preferences prefs;                // NVS storage

// Physical push-button pins for IRQ / NMI — set to -1 to disable.
// WARNING: GPIO0 = ESP32 BOOT button, always LOW at startup — never use it here.
#define PIN_NMI   -1
#define PIN_IRQ   -1

#define NVS_NS    "6802"          // NVS namespace
#define MAX_SLOTS 8               // number of save slots

// ============================================================
//  USER PERIPHERALS  (memory-mapped I/O callbacks)
// ============================================================

uint8_t user_io_read(uint16_t addr) {
    if (addr == 0x8000) {
        if (Serial.available()) return (uint8_t)Serial.read();
        return 0x00;
    }
    if (addr == 0x8001) {
        uint8_t st = 0x02;
        if (Serial.available()) st |= 0x01;
        return st;
    }
    return 0xFF;
}

void user_io_write(uint16_t addr, uint8_t val) {
    if (addr == 0x8000) { Serial.write(val); return; }
}

// ============================================================
//  SERIAL MONITOR  —  state variables
// ============================================================

static bool     running     = false;
static uint16_t go_addr     = 0x0000;
static bool     go_addr_set = false;

static bool     write_mode  = false;
static uint16_t write_addr  = 0x0000;

// S19 receive mode
static bool     s19_mode    = false;
static uint16_t s19_start   = 0xFFFF; // lowest address seen
static uint16_t s19_end     = 0x0000; // highest address seen

// ============================================================
//  DISPLAY HELPERS
// ============================================================

void printAddr(uint16_t addr) {
    if (addr < 0x1000) Serial.print('0');
    if (addr < 0x0100) Serial.print('0');
    if (addr < 0x0010) Serial.print('0');
    Serial.print(addr, HEX);
}

void printHex2(uint8_t v) {
    if (v < 0x10) Serial.print('0');
    Serial.print(v, HEX);
}

// ============================================================
//  OPCODE SIZE TABLE  (bytes per instruction, used by cmd_verify)
// ============================================================
static const uint8_t opcode_size[256] = {
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  // $0x
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  // $1x
  2,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,  // $2x
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  // $3x
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  // $4x
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  // $5x
  2,1,1,2,2,1,2,2,2,2,2,1,2,2,2,2,  // $6x
  3,1,1,3,3,1,3,3,3,3,3,1,3,3,3,3,  // $7x
  2,2,2,3,2,2,2,1,2,2,2,2,3,2,3,1,  // $8x
  2,2,2,3,2,2,2,2,2,2,2,2,3,2,3,2,  // $9x
  2,2,2,3,2,2,2,2,2,2,2,2,2,2,1,2,  // $Ax
  3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,  // $Bx
  2,2,2,3,2,2,2,1,2,2,2,2,3,1,3,1,  // $Cx
  2,2,2,3,2,2,2,2,2,2,2,2,2,2,1,2,  // $Dx
  2,2,2,3,2,2,2,2,2,2,2,2,2,2,1,2,  // $Ex
  3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,  // $Fx
};

// ============================================================
//  COMMAND v — Hex + disassembly side-by-side view
// ============================================================
void cmd_verify(uint16_t start_addr, int nb_instr) {
    Serial.println();
    Serial.println(F(" Adr    | Octets      | Mnemonique          | Cy"));
    Serial.println(F("--------+-------------+---------------------+----"));
    uint16_t addr = start_addr;
    for (int i = 0; i < nb_instr; i++) {
        Serial.print(addr == cpu.PC ? '>' : ' ');
        Serial.print(F("$")); printAddr(addr); Serial.print(F(" | "));
        uint8_t op   = cpu6802_read(&cpu, addr);
        uint8_t size = opcode_size[op];
        for (int b = 0; b < 3; b++) {
            if (b < size) { printHex2(cpu6802_read(&cpu, addr + b)); Serial.print(' '); }
            else            Serial.print(F("   "));
        }
        Serial.print(F("| "));
        char buf[32];
        cpu6802_disasm(&cpu, addr, buf, sizeof(buf));
        Serial.print(buf);
        int pad = 20 - strlen(buf);
        for (int p = 0; p < pad; p++) Serial.print(' ');
        Serial.print(F("| "));
        Serial.println(size + 1);
        addr += size;
    }
    Serial.println(F("--------+-------------+---------------------+----"));
}

// ============================================================
//  S19 — CHECKSUM HELPER
//  Checksum = one's complement of the sum of all bytes
//  in the record (byte count + address + data).
// ============================================================
static uint8_t s19_checksum(uint8_t bytecount, uint16_t addr,
                             const uint8_t *data, uint8_t datalen) {
    uint8_t sum = bytecount;
    sum += (addr >> 8) & 0xFF;
    sum += addr & 0xFF;
    for (uint8_t i = 0; i < datalen; i++) sum += data[i];
    return (~sum) & 0xFF;
}

// ============================================================
//  S19 EXPORT  —  command: xs ADDR LEN [NAME]
//
//  Outputs a valid Motorola SREC file to the terminal.
//  S0  : header with program name
//  S1  : data records, 16 bytes each
//  S9  : end record with start address
//
//  Example output:
//    S0060000484F4F00B4        (header "HOO")
//    S1130100 8E00FFCE020...XX (16 data bytes at $0100)
//    S9030100FB                (start address $0100)
// ============================================================
void cmd_s19_export(uint16_t start, uint16_t length, const char *name) {
    Serial.println();
    Serial.println(F("; --- Motorola S19 Export ---"));
    Serial.println(F("; Copyright (c) 2026 Kamel Khelif"));

    // ── S0 header record ──────────────────────────────────────
    // S0 + byte_count + 0000 + name_bytes + checksum
    uint8_t nlen = strlen(name);
    if (nlen > 16) nlen = 16;
    // byte_count = 2 (addr) + nlen + 1 (checksum)
    uint8_t bc0 = 2 + nlen + 1;
    uint8_t sum = bc0; // addr=0000 contributes 0
    for (uint8_t i = 0; i < nlen; i++) sum += (uint8_t)name[i];
    uint8_t ck0 = (~sum) & 0xFF;

    Serial.print(F("S0"));
    printHex2(bc0);
    Serial.print(F("0000"));
    for (uint8_t i = 0; i < nlen; i++) printHex2((uint8_t)name[i]);
    printHex2(ck0);
    Serial.println();

    // ── S1 data records (16 bytes per record) ─────────────────
    uint16_t addr = start;
    uint16_t remaining = length;
    while (remaining > 0) {
        uint8_t chunk = (remaining > 16) ? 16 : (uint8_t)remaining;
        // byte_count = 2 (addr) + chunk (data) + 1 (checksum)
        uint8_t bc = 2 + chunk + 1;
        uint8_t data[16];
        for (uint8_t i = 0; i < chunk; i++)
            data[i] = cpu6802_read(&cpu, addr + i);
        uint8_t ck = s19_checksum(bc, addr, data, chunk);

        Serial.print(F("S1"));
        printHex2(bc);
        printHex2((addr >> 8) & 0xFF);
        printHex2(addr & 0xFF);
        for (uint8_t i = 0; i < chunk; i++) printHex2(data[i]);
        printHex2(ck);
        Serial.println();

        addr      += chunk;
        remaining -= chunk;
    }

    // ── S9 end record (start address = first address of export) ─
    uint8_t bc9 = 3; // 2 addr + 1 checksum
    uint8_t ck9 = s19_checksum(bc9, start, NULL, 0);
    Serial.print(F("S9"));
    printHex2(bc9);
    printHex2((start >> 8) & 0xFF);
    printHex2(start & 0xFF);
    printHex2(ck9);
    Serial.println();
    Serial.println(F("; --- End of S19 ---"));
}

// ============================================================
//  S19 IMPORT  —  parse one S19 line received from terminal
//
//  Returns true if S9 (end record) was received.
//  Updates s19_start / s19_end for slot auto-save.
//
//  Supported record types:
//    S0  header  — ignored (just noted)
//    S1  data    — 16-bit address, written to emulator RAM
//    S9  end     — sets go_addr to start address
// ============================================================
static uint8_t hexbyte(const char *s, int pos) {
    char hi = s[pos];
    char lo = s[pos+1];
    uint8_t h = (hi >= 'A') ? (hi - 'A' + 10) : (hi >= 'a') ? (hi-'a'+10) : (hi-'0');
    uint8_t l = (lo >= 'A') ? (lo - 'A' + 10) : (lo >= 'a') ? (lo-'a'+10) : (lo-'0');
    return (h << 4) | l;
}

bool s19_parse_line(const String &line) {
    if (line.length() < 4) return false;

    // ── Normalise: strip all trailing whitespace / CR / LF ──────
    String L = line;
    L.trim();
    // Extra safety: remove any embedded spaces (copy-paste artefacts)
    L.replace(" ", "");
    L.toUpperCase();
    const char *s = L.c_str();
    uint16_t slen = (uint16_t)L.length();

    if (s[0] != 'S') return false;
    char type = s[1];

    // ── S0 header ────────────────────────────────────────────────
    if (type == '0') {
        Serial.println(F("[S19] Header record received"));
        return false;
    }

    // ── S1 data record ───────────────────────────────────────────
    // Layout: S1 | BC(2) | ADDR(4) | DATA(2*n) | CK(2)
    //  slen = 2 + 2 + 4 + 2*n + 2 = 10 + 2*n
    //  n  = (slen - 10) / 2
    if (type == '1') {
        if (slen < 12) { Serial.println(F("[S19] ERROR: S1 record too short")); return false; }

        uint8_t  bc      = hexbyte(s, 2);
        uint16_t addr    = ((uint16_t)hexbyte(s,4) << 8) | hexbyte(s,6);

        // Derive datalen from the actual string length — robust against
        // any mismatch between bc and the number of bytes in the record.
        // bc formula:  bc = 2(addr) + datalen + 1(ck)  → datalen = bc - 3
        // String formula: datalen = (slen - 10) / 2
        // Use string length as ground truth; warn if they disagree.
        uint8_t datalen_bc  = (bc >= 3) ? (bc - 3) : 0;
        uint8_t datalen_str = (uint8_t)((slen - 10) / 2);

        if (datalen_bc != datalen_str) {
            Serial.print(F("[S19] WARN: bc says "));
            Serial.print(datalen_bc);
            Serial.print(F(" data bytes but record has "));
            Serial.print(datalen_str);
            Serial.println(F(" — using string length"));
        }
        uint8_t datalen = datalen_str;   // trust the actual characters received

        // Checksum is ALWAYS the last 2 characters of the record
        uint8_t ck_recv = hexbyte(s, slen - 2);

        // Recompute checksum: sum(bc + addr_hi + addr_lo + data...)
        uint8_t sum = bc + hexbyte(s,4) + hexbyte(s,6);
        for (uint8_t i = 0; i < datalen; i++) sum += hexbyte(s, 8 + i*2);
        uint8_t ck_calc = (~sum) & 0xFF;

        if (ck_calc != ck_recv) {
            Serial.print(F("[S19] ERROR: checksum mismatch at $"));
            printAddr(addr);
            Serial.print(F("  expected $")); printHex2(ck_calc);
            Serial.print(F("  got $"));      printHex2(ck_recv);
            Serial.print(F("  (record len=")); Serial.print(slen);
            Serial.print(F(", bc=0x")); Serial.print(bc, HEX);
            Serial.println(F(")"));
            Serial.println(F("[S19] Hint: check for spaces or extra chars in pasted record"));
            return false;
        }

        // Write data bytes to emulator RAM
        for (uint8_t i = 0; i < datalen; i++) {
            uint8_t val = hexbyte(s, 8 + i*2);
            cpu6802_write(&cpu, addr + i, val);
        }

        // Track address range
        if (addr < s19_start) s19_start = addr;
        uint16_t end_addr = addr + datalen - 1;
        if (end_addr > s19_end) s19_end = end_addr;

        Serial.print(F("[S19] $"));
        printAddr(addr);
        Serial.print(F(" +"));
        Serial.print(datalen);
        Serial.println(F(" bytes OK"));
        return false;
    }

    // ── S9 end record ────────────────────────────────────────────
    if (type == '9') {
        if (slen < 10) { Serial.println(F("[S19] ERROR: S9 record too short")); return false; }
        uint16_t addr = ((uint16_t)hexbyte(s,4) << 8) | hexbyte(s,6);
        go_addr     = addr;
        go_addr_set = true;
        Serial.print(F("[S19] End record — start address: $"));
        printAddr(addr);
        Serial.println();
        Serial.println(F("[S19] Import complete. Type 'g' to run."));
        return true;
    }

    // S2/S3/S5/S7/S8 — not needed for 6802
    return false;
}

// ============================================================
//  NVS SLOT SAVE  —  command: save SLOT START END [NAME]
//
//  Saves CPU RAM bytes [START..END] into NVS flash slot N.
//  The slot survives power cycles and resets.
//
//  NVS keys used (N = slot number 0-7):
//    "sN_nm"  String  — program name
//    "sN_sa"  UInt16  — start address
//    "sN_ea"  UInt16  — end address  (inclusive)
//    "sN_dt"  Bytes   — raw data
// ============================================================
void cmd_save(uint8_t slot, uint16_t start, uint16_t end_addr, const char *name) {
    if (slot >= MAX_SLOTS) {
        Serial.println(F("[SAVE] ERROR: slot must be 0-7"));
        return;
    }
    if (end_addr < start) {
        Serial.println(F("[SAVE] ERROR: end < start"));
        return;
    }

    uint16_t len = end_addr - start + 1;

    // Build NVS keys
    char key_nm[8], key_sa[8], key_ea[8], key_dt[8];
    snprintf(key_nm, 8, "s%d_nm", slot);
    snprintf(key_sa, 8, "s%d_sa", slot);
    snprintf(key_ea, 8, "s%d_ea", slot);
    snprintf(key_dt, 8, "s%d_dt", slot);

    // Collect bytes from emulator RAM
    uint8_t *buf = (uint8_t *)malloc(len);
    if (!buf) { Serial.println(F("[SAVE] ERROR: out of heap")); return; }
    for (uint16_t i = 0; i < len; i++)
        buf[i] = cpu6802_read(&cpu, start + i);

    prefs.begin(NVS_NS, false);
    prefs.putString(key_nm, name);
    prefs.putUShort(key_sa, start);
    prefs.putUShort(key_ea, end_addr);
    prefs.putBytes(key_dt, buf, len);
    prefs.end();
    free(buf);

    Serial.print(F("\n[SAVE] Slot "));
    Serial.print(slot);
    Serial.print(F(" saved: \""));
    Serial.print(name);
    Serial.print(F("\"  $"));
    printAddr(start);
    Serial.print(F("-$"));
    printAddr(end_addr);
    Serial.print(F("  ("));
    Serial.print(len);
    Serial.println(F(" bytes)"));
}

// ============================================================
//  NVS SLOT LOAD  —  command: load SLOT
//
//  Restores a previously saved slot back into emulator RAM.
//  Also sets go_addr to the saved start address.
// ============================================================
void cmd_load(uint8_t slot) {
    if (slot >= MAX_SLOTS) {
        Serial.println(F("[LOAD] ERROR: slot must be 0-7"));
        return;
    }

    char key_nm[8], key_sa[8], key_ea[8], key_dt[8];
    snprintf(key_nm, 8, "s%d_nm", slot);
    snprintf(key_sa, 8, "s%d_sa", slot);
    snprintf(key_ea, 8, "s%d_ea", slot);
    snprintf(key_dt, 8, "s%d_dt", slot);

    prefs.begin(NVS_NS, true); // read-only
    if (!prefs.isKey(key_dt)) {
        prefs.end();
        Serial.print(F("\n[LOAD] ERROR: slot "));
        Serial.print(slot);
        Serial.println(F(" is empty"));
        return;
    }

    char     name[32];
    uint16_t start    = prefs.getUShort(key_sa, 0x0100);
    uint16_t end_addr = prefs.getUShort(key_ea, 0x01FF);
    prefs.getString(key_nm, name, sizeof(name));
    uint16_t len      = end_addr - start + 1;

    uint8_t *buf = (uint8_t *)malloc(len);
    if (!buf) { prefs.end(); Serial.println(F("[LOAD] ERROR: out of heap")); return; }

    prefs.getBytes(key_dt, buf, len);
    prefs.end();

    for (uint16_t i = 0; i < len; i++)
        cpu6802_write(&cpu, start + i, buf[i]);
    free(buf);

    go_addr     = start;
    go_addr_set = true;

    Serial.print(F("\n[LOAD] Slot "));
    Serial.print(slot);
    Serial.print(F(" restored: \""));
    Serial.print(name);
    Serial.print(F("\"  $"));
    printAddr(start);
    Serial.print(F("-$"));
    printAddr(end_addr);
    Serial.print(F("  ("));
    Serial.print(len);
    Serial.println(F(" bytes)  go_addr=$"));
    printAddr(start);
    Serial.println(F("  — type 'g' to run"));
}

// ============================================================
//  NVS SLOTS LIST  —  command: slots
// ============================================================
void cmd_slots() {
    Serial.println();
    Serial.println(F("╔═══╦══════════════════╦═══════════╦═════════╗"));
    Serial.println(F("║ # ║ Name             ║ Range     ║ Bytes   ║"));
    Serial.println(F("╠═══╬══════════════════╬═══════════╬═════════╣"));
    prefs.begin(NVS_NS, true);
    for (uint8_t s = 0; s < MAX_SLOTS; s++) {
        char key_dt[8], key_nm[8], key_sa[8], key_ea[8];
        snprintf(key_dt, 8, "s%d_dt", s);
        snprintf(key_nm, 8, "s%d_nm", s);
        snprintf(key_sa, 8, "s%d_sa", s);
        snprintf(key_ea, 8, "s%d_ea", s);

        Serial.print(F("║ "));
        Serial.print(s);
        Serial.print(F(" ║ "));

        if (prefs.isKey(key_dt)) {
            char name[32]; prefs.getString(key_nm, name, sizeof(name));
            uint16_t sa = prefs.getUShort(key_sa, 0);
            uint16_t ea = prefs.getUShort(key_ea, 0);
            uint16_t ln = ea - sa + 1;
            // pad name to 16 chars
            char padded[17]; snprintf(padded, 17, "%-16s", name);
            Serial.print(padded);
            Serial.print(F(" ║ $"));
            printAddr(sa); Serial.print(F("-$")); printAddr(ea);
            Serial.print(F(" ║ "));
            char lnbuf[6]; snprintf(lnbuf,6,"%5u",ln);
            Serial.print(lnbuf);
            Serial.println(F("   ║"));
        } else {
            Serial.println(F("(empty)          ║ ---       ║ ---     ║"));
        }
    }
    prefs.end();
    Serial.println(F("╚═══╩══════════════════╩═══════════╩═════════╝"));
}

// ============================================================
//  NVS SLOT DELETE  —  command: del SLOT
// ============================================================
void cmd_del(uint8_t slot) {
    if (slot >= MAX_SLOTS) {
        Serial.println(F("[DEL] ERROR: slot must be 0-7"));
        return;
    }
    char key_nm[8], key_sa[8], key_ea[8], key_dt[8];
    snprintf(key_nm, 8, "s%d_nm", slot);
    snprintf(key_sa, 8, "s%d_sa", slot);
    snprintf(key_ea, 8, "s%d_ea", slot);
    snprintf(key_dt, 8, "s%d_dt", slot);

    prefs.begin(NVS_NS, false);
    prefs.remove(key_nm);
    prefs.remove(key_sa);
    prefs.remove(key_ea);
    prefs.remove(key_dt);
    prefs.end();

    Serial.print(F("\n[DEL] Slot "));
    Serial.print(slot);
    Serial.println(F(" erased"));
}

// ============================================================
//  HELP TEXT
// ============================================================
void print_help() {
    Serial.println();
    Serial.println(F("========== 6802 MONITOR v7 =========="));
    Serial.println(F("--- Execution ---"));
    Serial.println(F("  r            Reset CPU"));
    Serial.println(F("  g ADDR       Go: run from ADDR"));
    Serial.println(F("  g            Go: run from last go_addr"));
    Serial.println(F("  p            Pause"));
    Serial.println(F("  s  / s N     Step 1 or N instructions"));
    Serial.println(F("  d            Toggle debug trace"));
    Serial.println(F("--- Inspect ---"));
    Serial.println(F("  x            Show registers"));
    Serial.println(F("  v [ADDR [N]] Hex+disasm view"));
    Serial.println(F("  m ADDR       Memory dump (64 bytes)"));
    Serial.println(F("--- Write ---"));
    Serial.println(F("  w ADDR       Continuous write mode"));
    Serial.println(F("  w ADDR VAL   Write single byte"));
    Serial.println(F("--- Interrupts ---"));
    Serial.println(F("  i            Trigger IRQ"));
    Serial.println(F("  n            Trigger NMI"));
    Serial.println(F("--- Save/Load (NVS Flash) ---"));
    Serial.println(F("  slots                  List all slots"));
    Serial.println(F("  save N S E [NAME]      Save $S-$E to slot N"));
    Serial.println(F("  load N                 Load slot N"));
    Serial.println(F("  del  N                 Erase slot N"));
    Serial.println(F("--- S19 Motorola SREC ---"));
    Serial.println(F("  xs ADDR LEN [NAME]     Export S19 to terminal"));
    Serial.println(F("  xsr                    Import S19 (paste records)"));
    Serial.println(F("                         Type '.' to end import"));
    Serial.println(F("====================================="));
    Serial.print(F("> "));
}

// ============================================================
//  COMMAND PROCESSING
// ============================================================
void process_monitor_command(const String &cmd) {

    // --- Empty line ---
    if (cmd.length() == 0) {
        if (write_mode) {
            Serial.print(F("$")); printAddr(write_addr); Serial.print(F(": "));
        } else if (s19_mode) {
            // ignore blank lines in S19 mode
        } else {
            Serial.print(F("> "));
        }
        return;
    }

    // ── S19 receive mode ──────────────────────────────────────
    if (s19_mode) {
        if (cmd.charAt(0) == '.') {
            s19_mode = false;
            Serial.println(F("\n[S19] Import ended."));
            Serial.print(F("> "));
            return;
        }
        // Show received record info in debug mode
        if (cpu.debug_mode) {
            Serial.print(F("[S19 DBG] recv len="));
            Serial.print(cmd.length());
            Serial.print(F(" : "));
            Serial.println(cmd);
        }
        // Check for S0/S1/S9 records (case-insensitive)
        char c0 = cmd.charAt(0);
        if (c0 == 'S' || c0 == 's') {
            bool done = s19_parse_line(cmd);
            if (done) {
                s19_mode = false;
                Serial.print(F("> "));
            }
        }
        return;
    }

    // ── Continuous write mode ─────────────────────────────────
    if (write_mode) {
        if (cmd.charAt(0) == '.') {
            write_mode = false;
            Serial.println(F("\n[WRITE MODE END]"));
        } else {
            uint8_t val = (uint8_t)strtol(cmd.c_str(), NULL, 16);
            cpu6802_write(&cpu, write_addr, val);
            Serial.print(F("  [$")); printAddr(write_addr);
            Serial.print(F("] = $")); Serial.println(val, HEX);
            write_addr++;
            Serial.print(F("$")); printAddr(write_addr); Serial.print(F(": "));
            return;
        }
        Serial.print(F("> "));
        return;
    }

    // ── Normal commands ───────────────────────────────────────
    char c = cmd.charAt(0);

    // ---- r : Reset CPU ----
    if (c == 'r') {
        cpu6802_reset(&cpu);
        running = false; go_addr_set = false;
        Serial.print(F("\n[RESET] PC=$")); printAddr(cpu.PC); Serial.println();

    // ---- g [ADDR] : Go ----
    } else if (c == 'g') {
        if (cmd.length() > 2) {
            go_addr     = (uint16_t)strtol(cmd.substring(2).c_str(), NULL, 16);
            go_addr_set = true;
        }
        if (!go_addr_set) {
            Serial.println(F("\n[ERROR] Address required: g ADDR"));
        } else {
            cpu.PC = go_addr; running = true;
            Serial.print(F("\n[GO] PC=$")); printAddr(cpu.PC);
            Serial.println(F("  type 'p' to pause."));
        }

    // ---- p : Pause ----
    } else if (c == 'p') {
        running = false;
        Serial.println(F("\n[PAUSE]"));
        cpu6802_print_state(&cpu);

    // ---- s [N] : Step ----
    } else if (c == 's' && (cmd.length() == 1 || cmd.charAt(1) == ' ')) {
        uint32_t n = 1;
        if (cmd.length() > 2) { n = (uint32_t)cmd.substring(2).toInt(); if (n<1||n>100000) n=1; }
        bool old_dbg = cpu.debug_mode; cpu.debug_mode = true; Serial.println();
        for (uint32_t i = 0; i < n; i++) cpu6802_step(&cpu);
        cpu.debug_mode = old_dbg; cpu6802_print_state(&cpu);

    // ---- d : Debug trace ----
    } else if (c == 'd') {
        cpu.debug_mode = !cpu.debug_mode;
        Serial.print(F("\n[DEBUG] ")); Serial.println(cpu.debug_mode ? F("ON") : F("OFF"));

    // ---- xsr : import S19  (MUST be before 'x' single-char handler) ----
    } else if (cmd.startsWith("xsr")) {
        s19_mode  = true;
        s19_start = 0xFFFF;
        s19_end   = 0x0000;
        Serial.println(F("\n[XSR] S19 import mode."));
        Serial.println(F("Paste S19 records line by line."));
        Serial.println(F("Type '.' to exit import mode."));
        return;

    // ---- xs ADDR LEN [NAME] : export S19  (MUST be before 'x' single-char handler) ----
    } else if (cmd.startsWith("xs") && (cmd.length() == 2 || cmd.charAt(2) == ' ')) {
        {
        String rest = cmd.substring(3); rest.trim();
        int i1 = rest.indexOf(' ');
        if (i1 < 0) { Serial.println(F("\n[XS] Usage: xs ADDR LEN [name]")); goto done; }
        uint16_t addr = (uint16_t)strtol(rest.substring(0,i1).c_str(), NULL, 16);
        rest = rest.substring(i1+1); rest.trim();
        int i2 = rest.indexOf(' ');
        uint16_t xslen;
        char xsname[32] = "program";
        if (i2 > 0) {
            xslen = (uint16_t)strtol(rest.substring(0,i2).c_str(), NULL, 16);
            rest.substring(i2+1).toCharArray(xsname, sizeof(xsname));
        } else {
            xslen = (uint16_t)strtol(rest.c_str(), NULL, 16);
        }
        if (xslen == 0) { Serial.println(F("\n[XS] ERROR: length = 0")); goto done; }
        cmd_s19_export(addr, xslen, xsname);
        }

    // ---- x : eXamine registers ----
    } else if (c == 'x') {
        Serial.println(); cpu6802_print_state(&cpu);

    // ---- v [ADDR [N]] : disasm view ----
    } else if (c == 'v') {
        uint16_t addr = cpu.PC; int nb = 16;
        if (cmd.length() > 2) {
            int sp2 = cmd.indexOf(' ', 2);
            if (sp2 > 0) {
                addr = (uint16_t)strtol(cmd.substring(2, sp2).c_str(), NULL, 16);
                nb   = cmd.substring(sp2+1).toInt();
                if (nb<1||nb>64) nb=16;
            } else {
                addr = (uint16_t)strtol(cmd.substring(2).c_str(), NULL, 16);
            }
        }
        cmd_verify(addr, nb);

    // ---- m ADDR : memory dump ----
    } else if (c == 'm') {
        uint16_t addr = 0;
        if (cmd.length() > 2) addr = (uint16_t)strtol(cmd.substring(2).c_str(), NULL, 16);
        Serial.println(); cpu6802_dump_memory(&cpu, addr, 64);

    // ---- w ADDR [VAL] : write memory ----
    } else if (c == 'w') {
        int sp = cmd.indexOf(' ', 2);
        if (sp > 0 && cmd.length() > (unsigned)(sp+1)) {
            uint16_t addr = (uint16_t)strtol(cmd.substring(2,sp).c_str(), NULL, 16);
            uint8_t  val  = (uint8_t) strtol(cmd.substring(sp+1).c_str(), NULL, 16);
            cpu6802_write(&cpu, addr, val);
            Serial.print(F("\n[$")); printAddr(addr); Serial.print(F("] = $")); Serial.println(val,HEX);
        } else if (cmd.length() > 2) {
            write_addr = (uint16_t)strtol(cmd.substring(2).c_str(), NULL, 16);
            write_mode = true;
            Serial.println();
            Serial.println(F("[WRITE MODE] Enter one hex byte per line."));
            Serial.println(F("Type '.' to exit."));
            Serial.print(F("$")); printAddr(write_addr); Serial.print(F(": "));
            return;
        } else {
            Serial.println(F("\nUsage: w 0100  or  w 0100 8E"));
        }

    // ---- i / n : IRQ / NMI ----
    } else if (c == 'i') {
        cpu6802_irq(&cpu); Serial.println(F("\n[IRQ]"));
    } else if (c == 'n') {
        cpu6802_nmi(&cpu); Serial.println(F("\n[NMI]"));

    // ---- q : help ----
    } else if (c == 'q') {
        print_help(); return;

    // ============================================================
    //  SAVE / LOAD / SLOTS / DEL  — NVS Flash storage
    // ============================================================

    // ---- slots : list all slots ----
    } else if (cmd.startsWith("slots")) {
        cmd_slots(); return;

    // ---- save N START END [NAME] ----
    } else if (cmd.startsWith("save")) {
        // Tokenise: save N SSSS EEEE [NAME...]
        String rest = cmd.substring(5);
        rest.trim();
        int i1 = rest.indexOf(' ');
        if (i1 < 0) { Serial.println(F("\n[SAVE] Usage: save N START END [name]")); goto done; }
        uint8_t  slot  = (uint8_t)rest.substring(0, i1).toInt();
        rest = rest.substring(i1+1); rest.trim();
        int i2 = rest.indexOf(' ');
        if (i2 < 0) { Serial.println(F("\n[SAVE] Usage: save N START END [name]")); goto done; }
        uint16_t start = (uint16_t)strtol(rest.substring(0,i2).c_str(), NULL, 16);
        rest = rest.substring(i2+1); rest.trim();
        int i3 = rest.indexOf(' ');
        uint16_t endad;
        char name[32] = "program";
        if (i3 > 0) {
            endad = (uint16_t)strtol(rest.substring(0,i3).c_str(), NULL, 16);
            rest.substring(i3+1).toCharArray(name, sizeof(name));
        } else {
            endad = (uint16_t)strtol(rest.c_str(), NULL, 16);
        }
        cmd_save(slot, start, endad, name);

    // ---- load N ----
    } else if (cmd.startsWith("load")) {
        String rest = cmd.substring(5); rest.trim();
        if (rest.length() == 0) { Serial.println(F("\n[LOAD] Usage: load N")); goto done; }
        cmd_load((uint8_t)rest.toInt());

    // ---- del N ----
    } else if (cmd.startsWith("del")) {
        String rest = cmd.substring(4); rest.trim();
        if (rest.length() == 0) { Serial.println(F("\n[DEL] Usage: del N")); goto done; }
        cmd_del((uint8_t)rest.toInt());

    } else {
        Serial.print(F("\n[?] Unknown command: "));
        Serial.println(cmd);
    }

done:
    Serial.print(F("> "));
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    while (Serial.available()) Serial.read();

    if (PIN_NMI >= 0) pinMode(PIN_NMI, INPUT_PULLUP);
    if (PIN_IRQ >= 0) pinMode(PIN_IRQ, INPUT_PULLUP);

    cpu6802_init(&cpu);
    cpu.io_read_cb  = user_io_read;
    cpu.io_write_cb = user_io_write;

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("  Motorola 6802 Emulator on ESP32  v7"));
    Serial.println(F("  Copyright (c) 2026 Kamel Khelif"));
    Serial.println(F("  MIT License — open source"));
    Serial.println(F("========================================"));
    Serial.println(F("Memory is empty. Commands:"));
    Serial.println(F("  w ADDR     write program"));
    Serial.println(F("  load N     restore from Flash slot"));
    Serial.println(F("  xsr        import S19 file"));
    Serial.println(F("  g ADDR     run"));
    Serial.println(F("  q          full help"));
    Serial.println();
    Serial.print(F("> "));
}

// ============================================================
//  MAIN LOOP
// ============================================================
static String   serial_buf      = "";
static bool     cr_received     = false;
static uint32_t last_gpio_check = 0;

#define STEPS_PER_LOOP  200

void loop() {

    // ---- 1. Serial monitor (non-blocking) ----
    while (Serial.available()) {
        char ch = (char)Serial.read();
        if (ch == '\r') {
            process_monitor_command(serial_buf);
            serial_buf = ""; cr_received = true;
        } else if (ch == '\n') {
            if (cr_received) { cr_received = false; }
            else { process_monitor_command(serial_buf); serial_buf = ""; }
        } else {
            cr_received = false;
            Serial.print(ch);
            if (serial_buf.length() < 80) serial_buf += ch;  // increased for long cmds
        }
    }

    // ---- 2. CPU execution ----
    if (running && !cpu.halted) {
        for (int i = 0; i < STEPS_PER_LOOP; i++) {
            cpu6802_step(&cpu);
            if (cpu.halted) {
                running = false;
                Serial.println(F("\n[HALT] CPU halted."));
                cpu6802_print_state(&cpu);
                Serial.print(F("> "));
                break;
            }
        }
    }

    // ---- 3. Physical interrupt buttons (if wired) ----
    if (millis() - last_gpio_check > 50) {
        last_gpio_check = millis();
        if (PIN_NMI >= 0 && digitalRead(PIN_NMI) == LOW) {
            cpu6802_nmi(&cpu);
            Serial.println(F("\n[NMI via GPIO button]"));
            Serial.print(F("> ")); delay(200);
        }
        if (PIN_IRQ >= 0 && digitalRead(PIN_IRQ) == LOW) {
            cpu6802_irq(&cpu);
            Serial.println(F("\n[IRQ via GPIO button]"));
            Serial.print(F("> ")); delay(200);
        }
    }

    yield();
}

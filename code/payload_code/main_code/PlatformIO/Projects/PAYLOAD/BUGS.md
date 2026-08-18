Real bugs
1. setCRC(true) silently fails — CRC is only on by accident
LoraTx.cpp:106

RadioLib's signature is setCRC(uint8_t len, ...), not a bool. true → len = 1. In the LoRa branch (SX128x.cpp:1074-1080) only len == 0 (off) and len == 2 (on) are accepted; anything else returns RADIOLIB_ERR_INVALID_CRC_CONFIGURATION without touching the packet params. The return value is discarded.

CRC is currently on only because SX128x::begin() defaults crcLoRa = RADIOLIB_SX128X_LORA_CRC_ON. Per datasheet Table 14-53, LORA_CRC_ENABLE = 0x20 is packetParam4. Fix: setCRC(2) and check the return.

Your ground station has the identical bug (ground_station/src/main_scanner.cpp:73) — which is why they still agree.

2. setWhitening(true) is a no-op error — LoRa has no whitening
LoraTx.cpp:107

Datasheet Table 11-60: LoRa/Ranging SetPacketParams has exactly five params — PreambleLength, HeaderType, PayloadLength, CRC, InvertIQ. There is no whitening field. Whitening only exists for GFSK (Table 14-10), BLE (14-23) and FLRC (14-41).

RadioLib matches the datasheet: setWhitening() returns RADIOLIB_ERR_WRONG_MODEM immediately for LoRa (SX128x.cpp:1084-1088). The line does nothing. Delete it, and drop "whitening" from the ground-station matching NOTE at Config.hpp:118 — it's not a parameter that can mismatch.

3. Buffer is written before its length is checked
MavlinkFrames.cpp:16-18 → App.cpp:265,416

buildXxx() calls mavlink_msg_to_send_buffer(out, &msg) into uint8_t buf[LORA_FRAME_MAX] (64 B) with no bound; sendFrame_() checks len > LORA_FRAME_MAX only after the write has already happened. MAVLINK_MAX_PACKET_LEN is 280.

Harmless today (largest frame is SCALED_IMU at 36 B), but it's a stack smash waiting for whoever adds a message. STATUSTEXT alone would be 63 bytes — one byte of headroom. Either size the buffer MAVLINK_MAX_PACKET_LEN or add a static_assert.

4. cw on doesn't stay on
App.cpp:482, LoraTx.cpp:196

transmitDirect() issues SetTxContinuousWave (0xD1) and the datasheet (§11.6.9) says the device stays in CW "until the host sends a mode configuration command." But tick() keeps calling lora_.service() and runTxCycle_(), and the next startTransmit() issues SetPacketParams/SetTx — killing CW within one loop iteration. The diagnostic in §6B of lora_rf_debug.md ("cw on + power meter") won't measure what you think unless you gate the TX cycle while CW is active.

Minor: stopCw() doesn't check ready_ while startCw() does.

Datasheet / RF concerns
5. 2400.0 MHz sits exactly on the band edge — half the signal is out of band
Config.hpp:102 (and both GS configs)

Table 3-3: synthesizer range is 2400–2500 MHz. You're at the exact minimum. With BW 812.5 kHz, the occupied spectrum is roughly 2399.6 – 2400.4 MHz — the lower half falls below the specified synth range and below the 2.4 GHz ISM allocation. Nothing guarantees PLL/filter performance there, and it's a regulatory problem for a flight article.

Move to ~2402–2480 MHz. Costs nothing, and it moves you off the 2400 MHz edge where Wi-Fi ch.1 spillover lives too.

6. Spreading factor should be set last, not before coding rate
LoraTx.cpp:100-104

Datasheet §14.4.1, immediately after the SetModulationParams step:

After SetModulationParams command: … If the Spreading Factor is SF9, SF10, SF11 or SF12, then the command WriteRegister(0x925, 0x32) must be used. In all cases 0x1 must be written to the Frequency Error Compensation mode register 0x093C.

RadioLib writes 0x925/0x93C only inside setSpreadingFactor() (SX128x.cpp:630-650). Your order is BW → SF → CR, and setCodingRate() issues another SetModulationParams afterwards (SX128x.cpp:684) with no re-write of 0x925.

Whether SetModulationParams clears that register isn't documented, so this may be benign — but the datasheet's instruction is unambiguously "after SetModulationParams." Reorder to setBandwidth → setCodingRate → setSpreadingFactor so the register write is genuinely last. Free insurance. (RadioLib's own begin() has the same ordering, so the GS inherits it too.)

7. The planned "switch to BW 203 kHz for +12 dB" has a frequency-tolerance trap
lora_rf_debug.md §7

Table 6-3, Total Permissible Reference Drift:

LoRa BW	SF5–SF11	SF12
800 kHz	±80 ppm	±50 ppm
200 kHz	±21 ppm	±21 ppm
At BW 812.5 you have ±80 ppm to play with — a plain crystal on each end is nowhere near the limit. At BW 203.125 the budget collapses to ±21 ppm total for the link, i.e. ~±10 ppm per end. A standard ±10 ppm XTAL at 25 °C will drift well past that over a rocket's −40…+85 °C range, and §6.2.7.2 adds a dynamic-drift limit of BW/(3·2^SF) ≈ 32 Hz at SF11/BW203 during the packet.

So the "biggest free win" isn't free: verify both crystals' spec + temperature curve, or fit a TCXO (§15.3 gives the AC-coupled 0.8 V clipped-sine circuit). Otherwise you trade 12 dB of sensitivity for a link that drops out when the payload gets cold.

8. Pad "~1 µA" sleep claim is optimistic
LoraTx.hpp:56-59

sleep() correctly does SetSaveContext (0xD5) then SetSleep with data-RAM + buffer retention — matches §14.7.3 exactly, and 0.4 µA typ from Table 3-3. Good.

But during the multi-hour pad wait: PIN_LORA_PWR stays HIGH, the DAC stays at full-scale PA bias (3300 mV, set in Sensors::begin()), and CXT/CrX stay latched in the TX state (LoraTx.cpp:85-86). The 1 µA is the SX1280 die only — the external PA and switch rail are still up. If pad endurance actually matters, the DAC and PIN_LORA_PWR need to go down too.

Also note §15.6 (Table 15-2) wants NSS=1, SCK=0, NRESET=1 for the spec sleep current; nothing in sleep() asserts that, it just relies on the SPI idle state.

Comments that no longer match the code
LORA_BW_KHZ = 812.5 but three separate comments still describe 203.125:

Config.hpp:103-108 — "203.125 buys ~+6-12 dB sensitivity vs 812.5" (fine as advice, but reads as if it's the setting)
Config.hpp:119 — "Flight profile = SF11, CR 4/8, BW 203.125 kHz" ← flatly wrong, and this is the line someone will use to configure the ground station
Config.hpp:130-137 — "At SF11 / BW 203.125 each packet is ~0.8-1.1 s on air … a full ~8-packet cycle is ~7 s. 10 s keeps duty ~70%"
Running the §7.4.4.1 time-on-air formula for your actual config (SF11, BW 812.5, CR 4/8, explicit header, CRC on, 12-symbol preamble, 36-byte frame):

Nsymbol = 12 + 4.25 + 8 + ceil((8·36 + 16 − 44 + 8 + 20)/(4·9))·8 = 88.25
Ts = 2^11/812.5 kHz = 2.52 ms → ToA ≈ 222 ms, not 0.8–1.1 s
So an 8-packet cycle is ~1.8 s, not ~7 s, and duty cycle is ~18%, not ~70%. You have far more headroom than the comment claims — worth knowing before anyone "optimizes" TX_CYCLE_INTERVAL_MS.

Two more:

Config.hpp:112-117: LORA_TX_PWR = 10, but the comment argues for +13 ("+13 keeps headroom if the knee drifts"). Value and rationale disagree. Also "(max +13)" is the register max (P = −18 + power, power ≤ 31); §4.1 and Table 3-8 put actual max output at +12.5 dBm.
lora_rf_debug.md:186: "Switch state applied in production: LoraTx::begin() (CXT=LOW, CrX=HIGH)" — the code is now CXT=HIGH/CrX=LOW and §4 of the same document says so. Stale line in the "Relevant files" section.
Good news on the switch, though: your CXT=HIGH/CrX=LOW agrees with the ground station's own FEM convention (fem_control.cpp: TX ⇒ CTX=1, CRX=0). That's independent confirmation the 2026-07-06 flip was correct.

Ground-station side (errata that bite the RX, not the TX)
Your payload is TX-only, so most of §16 is the GS's problem — but the link is only as good as both ends:

§16.2 — with the explicit header enabled (your config), a header error produces neither RxDone nor RxTimeout; the receiver just sits there. The GS must map HeaderError (IRQ bit 5) to a DIO and re-arm RX. Your main_scanner.cpp prints header errors, but the production Ratatoskr-GS-adapter-firmware path doesn't handle this.
§16.1 — continuous Rx mode (SetRx(0xFFFF), which startReceive() uses by default) can lock BUSY high permanently under heavy co-channel traffic. At 2400.0 MHz you are sitting under Wi-Fi channel 1. Single-mode RX with host re-arm is the datasheet's stated workaround.
§14.7.1 — after waking from retention sleep in LoRa packet type, {0x00, 0xAD, 0x08, 0x9A} must be written to address 0x0EF or RSSI reads are wrong. RadioLib does not do this (no reference to 0x0EF anywhere in the driver). Only matters if the GS sleeps; irrelevant for the TX-only payload.
§16.5 — no RxDone for 1-byte payload / SF9-10 / BW800-1600 / CR4-5 / explicit / CRC off. Doesn't apply to you (CRC on, SF11, min frame 21 B), but keep it in mind if anyone shrinks the beacon.
Things I checked that are correct
Worth stating so you don't re-litigate them:

IRQ polling for TxDone is sound. getIrqStatus() returns (data[0]<<8)|data[1] = the 16-bit register per Table 11-76, and TxDone is bit 0 (Table 11-73). Your 0x0001 mask is right. RadioLib's TX staging also enables TX_DONE|RX_TX_TIMEOUT and clears flags before keying, so the poll can't see a stale bit.
RADIOLIB_NC for DIO1 is the right call given GPIO11 is the switch line, and launchMode() still busy-waits on the real BUSY pin (GPIO13) for PA ramp-up.
Frequency register math: 2400 MHz → 2400·2^18/52 = 12098953 = 0xB89D89, exactly the datasheet's §11.7.3 worked example.
SF11 + BW 812.5 + CR 4/8 are all individually legal (Tables 14-47/48/49). The SF11/SF12 ranging restriction (§14.5.1) doesn't apply — you're not ranging.
Sync word 0x12 matches on both ends (RADIO_SYNC_WORD = 18), and RadioLib's setSyncWord(0x12, 0x44) writes 0x14/0x24 into 0x944/0x945 — preserving the reset-value low nibbles as §14.4.1 requires.
Preamble = 12 symbols on both ends (RadioLib default), satisfying §7.4.1's "configure identically."
DAC/PA bias is set before the radio comes up — sensors_.begin() at App.cpp:72 precedes lora_.begin() at line 75, and Sensors.cpp uses the corrected GENERAL_CONFIG register 0xD1.
Retry ring buffer, attempt accounting (1 initial + retries to LORA_MAX_ATTEMPTS), and the 3 s TX timeout (vs 222 ms real ToA) are all fine.
If you only fix three things: setCRC(2) with a checked return, move off 2400.0 MHz, and correct the LORA_BW_KHZ comment block before someone configures a ground station from it.
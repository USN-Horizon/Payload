#include "New/LoraTx.hpp"

#if USE_LORA

// =============================================================================
//  LoraTx.cpp
//  RadioLib SX1280 wrapper plus a ring buffer of frames waiting for a retry.
//
//  TX completion: poll the SX1280 IRQ status register for the TxDone bit.
//  Neither DIO1 (CXT / GPIO11) nor the BUSY pin are reliable indicators on
//  this board — DIO1 never asserts HIGH on TxDone, and BUSY can already
//  read LOW by the time we sample it after startTransmit(). Polling the
//  IRQ register over SPI works regardless of how those pins are wired.
// =============================================================================

constexpr uint32_t LORA_TX_TIMEOUT_MS = 1000;  // upper bound on TX-on-air time.
constexpr uint16_t SX128X_IRQ_TX_DONE = 0x0001;

LoraTx::LoraTx()
  : mod_(new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY,
                    SPI, SPISettings(8000000, MSBFIRST, SPI_MODE0))),
    radio_(mod_) {}

// Input  : data, len - bytes to transmit; len <= LORA_FRAME_MAX.
// What   : starts a non-blocking transmit, then polls the SX1280 IRQ status
//          register for the TxDone bit (bit 0). Clears the IRQ and brings
//          the chip back to standby once TxDone is observed.
// Output : RADIOLIB_ERR_NONE on success, the startTransmit error otherwise,
//          or RADIOLIB_ERR_TX_TIMEOUT if TxDone never sets in time.
static int16_t txBlockOnIrq(SX1280& radio, const uint8_t* data, uint8_t len) {
  // startTransmit() internally clears stale IRQ flags before keying TX.
  uint32_t tStart = millis();
  int16_t s = radio.startTransmit(const_cast<uint8_t*>(data), len);
  if (s != RADIOLIB_ERR_NONE) return s;

  while ((radio.getIrqStatus() & SX128X_IRQ_TX_DONE) == 0) {
    if (millis() - tStart > LORA_TX_TIMEOUT_MS) {
#if USE_SERIAL
      Serial.printf("[LoRa] TX timeout after %lu ms, IRQ=0x%04X\n",
                    static_cast<unsigned long>(millis() - tStart),
                    static_cast<unsigned>(radio.getIrqStatus()));
#endif
      radio.finishTransmit();
      return RADIOLIB_ERR_TX_TIMEOUT;
    }
    delay(1);
  }

  uint32_t dur = millis() - tStart;
#if USE_SERIAL && PRINT_TX
  Serial.printf("[LoRa] TX %u bytes done in %lu ms\n",
                static_cast<unsigned>(len), static_cast<unsigned long>(dur));
#else
  (void)dur;
#endif

  // finishTransmit() clears the IRQ flags and returns the chip to standby.
  radio.finishTransmit();
  return RADIOLIB_ERR_NONE;
}

// =============================================================================
//  begin
// =============================================================================
bool LoraTx::begin() {
  // Bring the LoRa module's power rail up before any SPI traffic.
  pinMode(PIN_LORA_PWR, OUTPUT);
  digitalWrite(PIN_LORA_PWR, HIGH);
  delay(10);

#if USE_SERIAL
  Serial.println("[LoRa] begin");
#endif

  int s = radio_.begin();
  if (s != RADIOLIB_ERR_NONE) {
#if USE_SERIAL
    Serial.printf("[LoRa] begin() failed: %d\n", s);
#endif
    return false;
  }

  if (radio_.setFrequency(LORA_FREQ_MHZ)        != RADIOLIB_ERR_NONE) return false;
  if (radio_.setBandwidth(LORA_BW_KHZ)          != RADIOLIB_ERR_NONE) return false;
  if (radio_.setSpreadingFactor(LORA_SF)        != RADIOLIB_ERR_NONE) return false;
  if (radio_.setCodingRate(LORA_CR)             != RADIOLIB_ERR_NONE) return false;
  if (radio_.setOutputPower(LORA_TX_PWR)        != RADIOLIB_ERR_NONE) return false;

  radio_.setCRC(true);
  radio_.setWhitening(true);

  ready_ = true;
#if USE_SERIAL
  Serial.println("[LoRa] ready");
#endif
  return true;
}

// =============================================================================
//  send  -  one immediate attempt; on failure, push to retry queue.
// =============================================================================
bool LoraTx::send(const uint8_t* data, uint8_t len) {
  if (!ready_ || data == nullptr || len == 0 || len > LORA_FRAME_MAX) return false;

  int s = txBlockOnIrq(radio_, data, len);
  if (s == RADIOLIB_ERR_NONE) {
    sent_++;
    return true;
  }

#if USE_SERIAL
  Serial.printf("[LoRa] transmit() failed: %d (len=%u)\n", s, static_cast<unsigned>(len));
#endif
  enqueue_(data, len, /*prevAttempts=*/1);
  retries_++;
  return false;
}

// =============================================================================
//  service  -  retry up to maxAttemptsThisTick queued frames once each.
// =============================================================================
size_t LoraTx::service(uint8_t maxAttemptsThisTick) {
  if (!ready_) return 0;

  size_t did = 0;
  size_t toCheck = count_;
  if (toCheck > maxAttemptsThisTick) toCheck = maxAttemptsThisTick;

  for (size_t i = 0; i < toCheck; ++i) {
    PendingFrame f;
    if (!popOldest_(f)) break;

    int s = txBlockOnIrq(radio_, f.data, f.len);
    if (s == RADIOLIB_ERR_NONE) {
      sent_++;
      did++;
    } else {
#if USE_SERIAL
      Serial.printf("[LoRa] retry transmit() failed: %d (attempt=%u, len=%u)\n",
                    s, static_cast<unsigned>(f.attempts + 1),
                    static_cast<unsigned>(f.len));
#endif
      f.attempts++;
      retries_++;
      if (f.attempts >= LORA_MAX_ATTEMPTS) {
        dropped_++;
#if USE_SERIAL
        Serial.printf("[LoRa] dropped frame after %u attempts\n",
                      static_cast<unsigned>(f.attempts));
#endif
      } else {
        // Re-queue with the updated attempt count; tail is now where it goes.
        enqueue_(f.data, f.len, f.attempts);
      }
      // Small delay between back-to-back transmissions on the same tick.
      delay(LORA_BACKOFF_MS);
    }
  }

  return did;
}

// =============================================================================
//  startCw / stopCw  -  diagnostic continuous-wave transmission.
// =============================================================================
bool LoraTx::startCw() {
  if (!ready_) return false;
  int16_t s = radio_.transmitDirect();
#if USE_SERIAL
  Serial.printf("[LoRa] CW start -> %d (0=OK)\n", s);
#endif
  return s == RADIOLIB_ERR_NONE;
}

bool LoraTx::stopCw() {
  int16_t s = radio_.standby();
#if USE_SERIAL
  Serial.printf("[LoRa] CW stop  -> %d (0=OK)\n", s);
#endif
  return s == RADIOLIB_ERR_NONE;
}

// =============================================================================
//  enqueue_  -  push onto the ring buffer (overwrite oldest if full).
// =============================================================================
void LoraTx::enqueue_(const uint8_t* data, uint8_t len, uint8_t prevAttempts) {
  if (count_ == LORA_RETRY_QUEUE) {
    // Overwrite the oldest entry to make room.
    tail_ = (tail_ + 1) % LORA_RETRY_QUEUE;
    count_--;
    dropped_++;
#if USE_SERIAL
    Serial.println("[LoRa] retry queue full, dropping oldest");
#endif
  }

  PendingFrame& slot = queue_[head_];
  memcpy(slot.data, data, len);
  slot.len      = len;
  slot.attempts = prevAttempts;

  head_  = (head_ + 1) % LORA_RETRY_QUEUE;
  count_++;
}

// =============================================================================
//  popOldest_  -  remove oldest queued entry into out.
// =============================================================================
bool LoraTx::popOldest_(PendingFrame& out) {
  if (count_ == 0) return false;
  out = queue_[tail_];
  tail_ = (tail_ + 1) % LORA_RETRY_QUEUE;
  count_--;
  return true;
}

#endif  // USE_LORA

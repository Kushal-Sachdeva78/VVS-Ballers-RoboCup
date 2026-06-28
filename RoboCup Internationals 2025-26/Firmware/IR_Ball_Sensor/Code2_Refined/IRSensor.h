#ifndef IRSENSOR_H
#define IRSENSOR_H
#include <Arduino.h>
#include "config.h"

// ============================================================================
//  IRSensor  (Code #2 - refined)
//  Full-ring, intensity-weighted IR ball tracker for the Teensy 4.1 board.
//  Algorithm lineage: Aegis_2025 "defense_IRvector w/" (vector-sum idea),
//  re-architected - see IRSensor.cpp and the README for the rationale.
// ============================================================================
class IRSensor
{
public:
  void  begin();
  void  update();                                 // non-blocking; ~tens of us

  // ---- clean output API (for a future merged main loop) -------------------
  bool     detected()      const { return _detected; }
  float    angle()         const { return _angle; }     // deg, robot frame, 0 = front
  float    distance()      const { return _distance; }  // cm (calibrated LUT)

  // ---- introspection (used by debug / calibration) ------------------------
  uint16_t raw(uint8_t ch)       const { return _raw[ch]; }
  uint16_t intensity(uint8_t ch) const { return _intensity[ch]; }
  uint8_t  peakChannel()         const { return _peakCh; }
  uint16_t peakIntensity()       const { return _peak; }
  float    updateRateHz()        const { return _rateHz; }

  void  captureBaseline(uint16_t samples = 64);   // fill IR_baseline[] live

private:
  uint16_t _raw[NUM_SENSORS]       = {0};
  uint16_t _intensity[NUM_SENSORS] = {0};
  uint8_t  _peakCh   = 0;
  uint16_t _peak     = 0;
  bool     _detected = false;
  float    _angle    = 0.0f;
  float    _distance = 999.0f;

  float    _sSin = 0.0f, _sCos = 0.0f;            // EWMA-smoothed vector components
  bool     _haveVec  = false;
  float    _rateHz   = 0.0f;
  uint32_t _lastMicros = 0;

  void  readRing();
  float angleOfChannel(uint8_t ch) const;
  float distanceFromIntensity(uint16_t peakIntensity) const;
};

#endif // IRSENSOR_H

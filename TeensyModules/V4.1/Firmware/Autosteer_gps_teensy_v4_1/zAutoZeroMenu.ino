// =============================================================
// SERIAL MENU - Wasless WAS auto-zero tuning (Keya encoder)
// =============================================================
// Usage:
//   - Open the serial monitor (115200 baud)
//   - Type 'z' then ENTER to show the menu
//   - Type the parameter number, ENTER, then the value, ENTER
//   - Values are saved to EEPROM (see EEPROM_ADDR_AZ_PARAMS)
// Same settings are also exposed on the web config page (zWebConfig.ino), which is the
// easier way to tune them in the field - this menu predates that page and still works.
// =============================================================

#define EEPROM_ADDR_AZ_PARAMS  120

// AutoZeroParams struct is declared in Autosteer.ino (compiled before this file)

// Global instance - defaults
AutoZeroParams azParams = {
  .speedMin    = 2.5f,
  .yawRateMax  = 0.3f,
  .gpsHdgMax   = 0.3f,
  .timeSlowMs  = 500,
  .timeFastMs  = 200,
  .speedSlow   = 3.0f,
  .speedFast   = 12.0f,
  .useBno      = 1,
  .useGps      = 1,
  .beta        = 0.3f,
  .ident       = 0xA202
};

// -----------------------------------------------------------------
// Call from autosteerSetup() after the other EEPROM.get() calls
// -----------------------------------------------------------------
void azMenuSetup()
{
  AutoZeroParams saved;
  EEPROM.get(EEPROM_ADDR_AZ_PARAMS, saved);
  if (saved.ident == 0xA202) {
    azParams = saved;
    Serial.println("[AZ-MENU] Parameters loaded from EEPROM.");
  } else {
    EEPROM.put(EEPROM_ADDR_AZ_PARAMS, azParams);
    Serial.println("[AZ-MENU] First use - defaults saved.");
  }
}

// -----------------------------------------------------------------
// Menu display
// -----------------------------------------------------------------
void azMenuPrint()
{
  Serial.println();
  Serial.println("======= AUTO-ZERO WAS MENU =======");
  Serial.print("1. Minimum speed        : "); Serial.print(azParams.speedMin,    1); Serial.println(" km/h");
  Serial.print("2. Max yaw rate (BNO)   : "); Serial.print(azParams.yawRateMax,  2); Serial.println(" deg/s  (lower=stricter)");
  Serial.print("3. Max GPS heading delta: "); Serial.print(azParams.gpsHdgMax,   2); Serial.println(" deg    (lower=stricter)");
  Serial.print("4. Low-speed duration   : "); Serial.print(azParams.timeSlowMs      ); Serial.println(" ms");
  Serial.print("5. High-speed duration  : "); Serial.print(azParams.timeFastMs      ); Serial.println(" ms");
  Serial.print("6. Low-speed threshold  : "); Serial.print(azParams.speedSlow,   1); Serial.println(" km/h");
  Serial.print("7. High-speed threshold : "); Serial.print(azParams.speedFast,   1); Serial.println(" km/h");
  Serial.print("8. BNO source           : "); Serial.println(azParams.useBno ? "ACTIVE" : "INACTIVE");
  Serial.print("9. GPS source           : "); Serial.println(azParams.useGps ? "ACTIVE" : "INACTIVE");
  Serial.print("10. Beta correction     : "); Serial.print(azParams.beta,        3); Serial.println("  (0.01=slow .. 0.2=fast)");
  Serial.println("11. Reset to defaults");
  Serial.println("12. Quit");
  Serial.println("===================================");
  Serial.println("Type number + ENTER:");
}

// -----------------------------------------------------------------
// Menu loop - call from autosteerLoop()
// Returns true while the menu is active (blocks the rest of that loop iteration)
// -----------------------------------------------------------------
static bool    azMenuActive = false;
static uint8_t azMenuStep   = 0;  // 0 = waiting for choice, 1 = waiting for value
static uint8_t azMenuChoice = 0;

bool azMenuLoop()
{
  if (!azMenuActive) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      // EMA BNO filter commands (EY / ER / EP / ES) - see zHandlers.ino
      if (handleEmaSerialCommand(input)) return false;

      if (input == "z" || input == "Z") {
        azMenuActive = true;
        azMenuStep   = 0;
        azMenuPrint();
      }
    }
    return false;
  }

  if (!Serial.available()) return true;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return true;

  if (azMenuStep == 0)
  {
    azMenuChoice = input.toInt();

    if (azMenuChoice == 11) {
      azParams = { 1.0f, 0.8f, 1.0f, 500, 200, 3.0f, 12.0f, 1, 1, 0.05f, 0xA202 };
      EEPROM.put(EEPROM_ADDR_AZ_PARAMS, azParams);
      Serial.println("[AZ-MENU] Defaults restored and saved.");
      azMenuPrint();
      return true;
    }

    if (azMenuChoice == 12) {
      Serial.println("[AZ-MENU] Menu closed. Type 'z' to reopen.");
      azMenuActive = false;
      azMenuStep   = 0;
      return false;
    }

    if (azMenuChoice >= 1 && azMenuChoice <= 10) {
      if (azMenuChoice == 8 || azMenuChoice == 9) {
        Serial.print("New value (0=inactive, 1=active) for parameter ");
      } else {
        Serial.print("New value for parameter ");
      }
      Serial.print(azMenuChoice);
      Serial.println(" :");
      azMenuStep = 1;
    } else {
      Serial.println("Invalid choice.");
      azMenuPrint();
    }
  }
  else if (azMenuStep == 1)
  {
    float val = input.toFloat();

    switch (azMenuChoice) {
      case 1: azParams.speedMin    = val; break;
      case 2: azParams.yawRateMax  = val; break;
      case 3: azParams.gpsHdgMax   = val; break;
      case 4: azParams.timeSlowMs  = (uint32_t)val; break;
      case 5: azParams.timeFastMs  = (uint32_t)val; break;
      case 6: azParams.speedSlow   = val; break;
      case 7: azParams.speedFast   = val; break;
      case 8: azParams.useBno      = (val >= 1.0f) ? 1 : 0; break;
      case 9: azParams.useGps      = (val >= 1.0f) ? 1 : 0; break;
      case 10:
        if (val >= 0.001f && val <= 1.0f) azParams.beta = val;
        else Serial.println("Beta out of range (0.001 - 1.0), ignored.");
        break;
    }

    EEPROM.put(EEPROM_ADDR_AZ_PARAMS, azParams);
    Serial.println("[AZ-MENU] Saved.");
    azMenuStep = 0;
    azMenuPrint();
  }

  return true;
}

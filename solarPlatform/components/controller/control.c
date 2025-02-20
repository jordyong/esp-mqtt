#include "jsonParser.h"
#include "mqttManager.h"

#include "models.h"

// Should be called by uart after sucessfull GPS reading
void SendGPSReadings(gpsInfo gi) {
  // Create Json message
  // Send to mqtt
}

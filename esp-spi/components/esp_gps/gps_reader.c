#include <math.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>

#include "freertos/FreeRTOS.h"

#include "driver/uart.h"
#include "uartManager.h"

#include "gps_reader.h"

struct gps {
  float nmeaLat;
  char ns;
  float nmeaLong;
  char ew;
  float utc;
  char gllstats;

  float deciLat;
  float deciLong;
} GPS;

float GPS_nmea_to_dec(float deg_coord, char nsew) {
  int degree = (int)(deg_coord / 100.0);  // Extract degrees correctly
  float minutes = fmod(deg_coord, 100.0); // Use fmod for precise minutes
  float dec_deg = minutes / 60.0;
  float decimal = degree + dec_deg;
  if (nsew == 'S' || nsew == 'W') { // return negative
    decimal *= -1.0;
  }
  return decimal;
}

void readGPS() {
  // read from UART buffer
  uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
  bzero(dtmp, RD_BUF_SIZE);
  char line[256];     // Buffer to store a single line
  int line_index = 0; // Track position in line buffer
  while (1) {
    // Read available data from UART
    int len =
        uart_read_bytes(UART_NUM, dtmp, BUF_SIZE - 1, pdMS_TO_TICKS(1000));

    if (len > 0) {
      dtmp[len] = '\0'; // Null-terminate the received data
                        // printf("Raw Received: %s\n", data);

      // Process each character received
      for (int i = 0; i < len; i++) {
        if (dtmp[i] == '\n') {        // End of a line
          line[line_index] = '\0';    // Null-terminate line
          printf("Line: %s\n", line); // Print full line

          // Check if the line contains "$GNGLL"
          if (strstr(line, "$GNGLL") != NULL) {
            printf("%s\n", line);
            if (sscanf(line, "$GNGLL,%f,%c,%f,%c,%f,%c", &GPS.nmeaLat, &GPS.ns,
                       &GPS.nmeaLong, &GPS.ew, &GPS.utc, &GPS.gllstats) >= 1) {
              GPS.deciLat = GPS_nmea_to_dec(GPS.nmeaLat, GPS.ns);
              GPS.deciLong = GPS_nmea_to_dec(GPS.nmeaLong, GPS.ew);
              printf("DeciLat = %f \n", GPS.deciLat);
              printf("DeciLong = %f \n", GPS.deciLong);
            }
          }
          // Reset line buffer for next line
          line_index = 0;
        } else if (line_index < sizeof(line) - 1) {
          line[line_index++] = dtmp[i]; // Add character to line buffer
        }
      }
    }
  }

  // clean up
  free(dtmp);
  dtmp = NULL;
}

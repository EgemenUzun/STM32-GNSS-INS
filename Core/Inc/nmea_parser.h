#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <stdint.h>
#include <stdbool.h>

// Maximum length of an NMEA sentence
#define NMEA_MAX_LEN 100

// Structure for NMEA GPRMC data
typedef struct {
    double time_utc;          // HHMMSS.SS (UTC Time)
    char status;              // A=Active (Data Valid), V=Void (Data Invalid)
    double latitude;          // Latitude (Decimal Degrees)
    char lat_hemi;            // N/S (North/South Hemisphere)
    double longitude;         // Longitude (Decimal Degrees)
    char lon_hemi;            // E/W (East/West Hemisphere)
    double speed_knots;       // Speed Over Ground (Knots)
    double track_angle;       // Track Angle (Degrees)
    uint32_t date_utc;        // DDMMYY (UTC Date)
    // Other fields (magnetic variation, mode indicator, etc.) are omitted for simplicity
} NMEA_GPRMC_t;

// Structure for NMEA GPGGA data
typedef struct {
    double time_utc;          // HHMMSS.SS (UTC Time)
    double latitude;          // Latitude (Decimal Degrees)
    char lat_hemi;            // N/S (North/South Hemisphere)
    double longitude;         // Longitude (Decimal Degrees)
    char lon_hemi;            // E/W (East/West Hemisphere)
    int fix_quality;          // GPS Fix Quality (0=No fix, 1=GPS, 2=DGPS, etc.)
    int num_satellites;       // Number of Satellites in use
    double hdop;              // Horizontal Dilution of Precision
    double altitude_msl;      // Altitude above Mean Sea Level (Meters)
    char alt_unit;            // Altitude Unit (M)
    double geoid_sep;         // Geoid Separation (Meters)
    char geo_sep_unit;        // Geoid Separation Unit (M)
    // Other fields (DGPS Age, DGPS Station ID etc.) are omitted for simplicity
} NMEA_GGA_t;

/**
 * @brief Checks the checksum of an NMEA sentence.
 * @param sentence The NMEA sentence to check (e.g., "$GPRMC,..." or "GPS: $GPRMC,...").
 * @return true if the checksum is correct, false otherwise.
 */
bool nmea_check_checksum(const char *sentence);

/**
 * @brief Parses a GPRMC NMEA sentence.
 * @param sentence The GPRMC sentence string.
 * @param data Pointer to the GprmcData structure to store the parsed data.
 * @return true on success, false on failure (including checksum error).
 */
bool parse_gprmc(const char *sentence, NMEA_GPRMC_t *data);

/**
 * @brief Parses a GPGGA NMEA sentence.
 * @param sentence The GPGGA sentence string.
 * @param data Pointer to the GpggaData structure to store the parsed data.
 * @return true on success, false on failure (including checksum error).
 */
bool parse_gpgga(const char *sentence, NMEA_GGA_t *data);

#endif // NMEA_PARSER_H
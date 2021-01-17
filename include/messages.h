// This file contains definitions of messages that are passed between pipeline nodes.
#pragma once
#include "primitives/timestamp.h"
#include "primitives/vector.h"
#include <stdint.h>
#include <string>

// Tunable constants
constexpr int max_num_inputs = 8;            // Number of concurrent sensors supported.
constexpr int max_bytes_in_data_frame = 64;  // Current DataFrame length is 33. This param should be larger.
constexpr int max_bytes_in_data_chunk = 64;

// Not tunable: constant for Lighthouse system.
constexpr int num_base_stations = 2;
constexpr int num_cycle_phases = 4;

// Pulses are generated by InputNodes and processed by PulseProcessor
struct Pulse {
    uint32_t input_idx;
    Timestamp start_time;
    TimeDelta pulse_len;
};

enum class FixLevel {
    kNoSignals      =    0,  // No signals visible at all.
    kCycleSyncing   =  100,  // Base station sync pulses are visible and we're syncing to them.
    kCycleSynced    =  200,  // We're synced to the base station sync pulses.
    kPartialVis     =  500,  // Some sensors/base stations don't have visibility and angles are stale. Position is invalid.
    kStaleFix       =  800,  // Position fix is valid, but uses angles from previous 1-2 cycles.
    kFullFix        = 1000,  // Position fix is valid and fresh.
};

struct SensorAngles {
    float angles[num_cycle_phases]; // Angles of base stations to sensor, -1/3 Pi to 1/3 Pi
    uint32_t updated_cycles[num_cycle_phases]; // Cycle id when this angle was last updated.
};

// SensorAnglesFrame is produced by PulseProcessor every 4 cycles and consumed by GeometryBuilders. It contains
// a snapshot of angles visible by sensors.
struct SensorAnglesFrame {
    Timestamp time;
    FixLevel fix_level;  // Up to kCycleSynced
    uint32_t cycle_idx;  // Increasing number of cycles since last fix.
    int32_t phase_id;    // 0..3
    Vector<SensorAngles, max_num_inputs> sensors;
};

// One data bit extracted from a long pulse from one base station. Produced by PulseProcessor and consumed by DataFrameDecoder.
struct DataFrameBit {
    Timestamp time;
    uint32_t base_station_idx;
    uint32_t cycle_idx;
    bool bit;
};

// Decoded data frame. Produced by DataFrameDecoder. 'bytes' array can be casted to DecodedDataFrame to get meaningful values.
struct DataFrame {
    Timestamp time;
    uint32_t base_station_idx;
    Vector<uint8_t, max_bytes_in_data_frame> bytes;
};

// Position of an object. Calculated by GeometryBuilders and consumed by FormatterNodes.
struct ObjectPosition {
    Timestamp time;
    uint32_t object_idx; // Index of the object.
    FixLevel fix_level;
    float pos[3];     // 3d object position
    float pos_delta;  // Distance between base station rays. Can be used as a measure of position uncertainty.
    float q[4];       // Rotation quaternion (unit if no rotation information available)
};

// DataChunk is used to send raw data to outputs.
struct DataChunk {
    Timestamp time;
    Vector<uint8_t, max_bytes_in_data_chunk> data;  // Data of this chunk.
    uint32_t stream_idx;  // Used to distinguish between different streams going to the same output. Useful for polling mode.
    bool last_chunk;  // True if this is the last chunk in a "packet". Useful for polling mode.
};

struct DebugString
{
    std::string text;
    float data[10];
};


enum class OutputCommandType {
    kMakeExclusive,  // Make given stream_idx exclusive and don't accept data chunks from other streams.
    kMakeNonExclusive,  // Remove exclusivity.
};

// OutputCommand is used to control OutputNodes, see OutputCommandType enum for types of commands.
struct OutputCommand {
    OutputCommandType type;
    uint32_t stream_idx;
};

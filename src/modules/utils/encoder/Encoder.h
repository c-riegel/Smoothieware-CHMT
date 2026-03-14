#pragma once

#include "Module.h"
#include <stdint.h>

#define MAX_ENCODER_SEGMENTS 128
#define MIN_ENCODER_DELTA 10       // minimum encoder counts per axis to arm encoder target

class StreamOutput;
class Gcode;

struct EncoderSegment {
    int32_t x_target;
    int32_t y_target;
    float feed_rate;        // mm/min — overall segment feed rate (from G-code F parameter)
    float acceleration;     // mm/s² — from M204 S parameter
    int64_t x_steps_per_tick; // precomputed 2.62 fixed-point stepping rate for X axis
    int64_t y_steps_per_tick; // precomputed 2.62 fixed-point stepping rate for Y axis
    uint32_t timeout_us;    // precomputed timeout for remaining distance from this segment
    uint32_t armed_at;      // us_ticker when this segment was actually armed (filled at runtime)
    uint32_t completed_at;  // us_ticker when this segment completed
    int32_t x_enc_at_arm;   // encoder X count when armed
    int32_t y_enc_at_arm;   // encoder Y count when armed
    bool has_x;
    bool has_y;
};

class Encoder : public Module {
    public:
        Encoder();

        void on_module_loaded();
        void on_gcode_received(void *argument);
        void on_idle(void *argument);
        void on_halt(void *argument);

        int32_t get_x_count();
        int32_t get_y_count();
        void set_x_count(int32_t count);
        void set_y_count(int32_t count);

        // Called from OC ISRs — must be fast, no allocation
        void on_x_target_reached();
        void on_y_target_reached();

    private:
        void init_encoders();
        void init_output_compare();
        void report_encoder_position(Gcode *gcode);
        void report_stepper_position(Gcode *gcode);
        void auto_calibrate(Gcode *gcode, float distance);
        void arm_x_target(int32_t target);
        void arm_y_target(int32_t target);
        void disarm_x();
        void disarm_y();
        void arm_segment(int index);
        void try_advance_segment();
        void precompute_segment_timeouts();
        uint32_t compute_single_timeout_x();
        uint32_t compute_single_timeout_y();

        float x_counts_per_mm;
        float y_counts_per_mm;
        float x_encoder_offset;
        float y_encoder_offset;
        // Timeout state — arm_time and timeout_us are written together by arm_segment()
        // and read together by on_idle(). Keeping them paired avoids cross-segment races.
        volatile uint32_t x_arm_time_us;
        volatile uint32_t x_timeout_us;
        volatile uint32_t y_arm_time_us;
        volatile uint32_t y_timeout_us;
        volatile bool x_move_armed;
        volatile bool y_move_armed;

        // Non-segment single-move state (arm_x_target / arm_y_target)
        float x_move_distance_mm;
        float y_move_distance_mm;
        float x_feed_rate_mmpm;
        float y_feed_rate_mmpm;
        bool encoder_enabled;

        // Segment buffering (M920)
        EncoderSegment segments[MAX_ENCODER_SEGMENTS];
        volatile int current_segment;
        int segment_count;           // total G1 commands expected (from M920 S<N>)
        int segments_received;       // total G1 commands received so far
        int encoder_segment_count;   // how many had X/Y (encoder-buffered)
        int encoder_segments_received; // how many X/Y segments buffered so far
        volatile bool segment_mode;
        bool buffering;
        volatile bool x_segment_done;
        volatile bool y_segment_done;
        volatile bool segments_complete;   // set by ISR when all segments done
        volatile uint32_t segments_done_at; // us_ticker when last segment completed
        volatile int last_reported_segment; // on_idle prints up to this point

        // Buffering state for capturing M204/F values across G-code lines
        float pending_feed_rate;     // last F value seen during buffering (mm/min)
        float pending_acceleration;  // last M204 S value seen during buffering (mm/s²)

        // Debug counters (written from ISR, read/cleared from on_idle)
        volatile uint32_t dbg_x_oc_count;      // OC ISR fired for X
        volatile uint32_t dbg_y_oc_count;      // OC ISR fired for Y
};

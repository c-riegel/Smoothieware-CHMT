#pragma once

#include "Module.h"
#include <stdint.h>

#define MAX_ENCODER_SEGMENTS 128

class StreamOutput;
class Gcode;

struct EncoderSegment {
    int32_t x_target;
    int32_t y_target;
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

        float x_counts_per_mm;
        float y_counts_per_mm;
        float x_encoder_offset;
        float y_encoder_offset;
        float x_move_distance_mm;
        float y_move_distance_mm;
        uint32_t x_arm_time_us;
        uint32_t y_arm_time_us;
        volatile bool x_move_armed;
        volatile bool y_move_armed;
        bool encoder_enabled;

        // Segment buffering (M920)
        EncoderSegment segments[MAX_ENCODER_SEGMENTS];
        volatile int current_segment;
        int segment_count;
        int segments_received;
        volatile bool segment_mode;
        bool buffering;
        volatile bool x_segment_done;
        volatile bool y_segment_done;
};

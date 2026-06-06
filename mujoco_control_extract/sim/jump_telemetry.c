#include "jump_telemetry.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

enum
{
    TELEMETRY_MOTOR_COUNT = 6,
};

typedef struct
{
    double time_s;
    double base_z_m;
    double base_rise_m;
    double wheel_clearance_m;
    double command_torque_nm[TELEMETRY_MOTOR_COUNT];
    double applied_torque_nm[TELEMETRY_MOTOR_COUNT];
    int jump_active;
    int jump_phase;
    int airborne;
} JumpTelemetrySample;

struct JumpTelemetry
{
    char *output_prefix;
    JumpTelemetrySample *samples;
    size_t sample_count;
    size_t sample_capacity;

    int jump_count;
    int last_jump_active;
    int last_airborne;
    int takeoff_valid;
    double takeoff_base_z_m;
    double first_jump_time_s;
    double last_jump_end_time_s;
    double post_jump_end_time_s;

    double max_base_rise_m;
    double max_base_z_m;
    double max_wheel_clearance_m;
    double peak_command_abs_nm[TELEMETRY_MOTOR_COUNT];
    double peak_command_signed_nm[TELEMETRY_MOTOR_COUNT];
    double peak_command_time_s[TELEMETRY_MOTOR_COUNT];
    double peak_applied_abs_nm[TELEMETRY_MOTOR_COUNT];
    double peak_applied_signed_nm[TELEMETRY_MOTOR_COUNT];
    double peak_applied_time_s[TELEMETRY_MOTOR_COUNT];
};

static const char *const motor_names[TELEMETRY_MOTOR_COUNT] = {
    "left_front",
    "left_rear",
    "right_rear",
    "right_front",
    "left_wheel",
    "right_wheel",
};

static char *telemetry_strdup(const char *text)
{
    const size_t length = strlen(text);
    char *copy = (char *)malloc(length + 1);
    if (copy != 0)
    {
        memcpy(copy, text, length + 1);
    }
    return copy;
}

static int make_parent_directories(const char *path)
{
    char *copy = telemetry_strdup(path);
    if (copy == 0)
    {
        return 0;
    }

    for (char *cursor = copy + 1; *cursor != '\0'; ++cursor)
    {
        if (*cursor != '/')
        {
            continue;
        }

        *cursor = '\0';
        if (mkdir(copy, 0775) != 0 && errno != EEXIST)
        {
            free(copy);
            return 0;
        }
        *cursor = '/';
    }

    free(copy);
    return 1;
}

static char *make_output_path(const char *prefix, const char *suffix)
{
    const size_t length = strlen(prefix) + strlen(suffix) + 1;
    char *path = (char *)malloc(length);
    if (path != 0)
    {
        snprintf(path, length, "%s%s", prefix, suffix);
    }
    return path;
}

static int reserve_sample(JumpTelemetry *telemetry)
{
    if (telemetry->sample_count < telemetry->sample_capacity)
    {
        return 1;
    }

    const size_t next_capacity = telemetry->sample_capacity == 0
                                     ? 4096
                                     : telemetry->sample_capacity * 2;
    JumpTelemetrySample *next =
        (JumpTelemetrySample *)realloc(telemetry->samples,
                                       next_capacity * sizeof(*next));
    if (next == 0)
    {
        return 0;
    }

    telemetry->samples = next;
    telemetry->sample_capacity = next_capacity;
    return 1;
}

JumpTelemetry *JumpTelemetry_Create(const char *output_prefix)
{
    if (output_prefix == 0 || output_prefix[0] == '\0')
    {
        return 0;
    }

    JumpTelemetry *telemetry =
        (JumpTelemetry *)calloc(1, sizeof(*telemetry));
    if (telemetry == 0)
    {
        return 0;
    }

    telemetry->output_prefix = telemetry_strdup(output_prefix);
    if (telemetry->output_prefix == 0)
    {
        free(telemetry);
        return 0;
    }

    telemetry->first_jump_time_s = NAN;
    telemetry->last_jump_end_time_s = NAN;
    telemetry->post_jump_end_time_s = -INFINITY;
    telemetry->max_base_rise_m = -INFINITY;
    telemetry->max_base_z_m = -INFINITY;
    telemetry->max_wheel_clearance_m = -INFINITY;
    return telemetry;
}

void JumpTelemetry_Record(JumpTelemetry *telemetry,
                          double time_s,
                          double base_z_m,
                          double wheel_clearance_m,
                          const double command_torque_nm[6],
                          const double applied_torque_nm[6],
                          int jump_active,
                          int jump_phase,
                          int airborne)
{
    if (telemetry == 0 ||
        command_torque_nm == 0 ||
        applied_torque_nm == 0 ||
        !reserve_sample(telemetry))
    {
        return;
    }

    jump_active = jump_active != 0;
    airborne = airborne != 0;

    if (jump_active && !telemetry->last_jump_active)
    {
        ++telemetry->jump_count;
        if (isnan(telemetry->first_jump_time_s))
        {
            telemetry->first_jump_time_s = time_s;
        }
        telemetry->takeoff_valid = 0;
        telemetry->post_jump_end_time_s = INFINITY;
    }
    else if (!jump_active && telemetry->last_jump_active)
    {
        telemetry->last_jump_end_time_s = time_s;
        telemetry->post_jump_end_time_s = time_s + 0.5;
    }

    if (telemetry->jump_count > 0 &&
        airborne &&
        !telemetry->last_airborne &&
        !telemetry->takeoff_valid)
    {
        telemetry->takeoff_base_z_m = base_z_m;
        telemetry->takeoff_valid = 1;
    }

    const int event_window =
        telemetry->jump_count > 0 &&
        (jump_active || time_s <= telemetry->post_jump_end_time_s);
    const double base_rise_m = telemetry->takeoff_valid
                                   ? base_z_m - telemetry->takeoff_base_z_m
                                   : NAN;

    JumpTelemetrySample *sample =
        &telemetry->samples[telemetry->sample_count++];
    sample->time_s = time_s;
    sample->base_z_m = base_z_m;
    sample->base_rise_m = base_rise_m;
    sample->wheel_clearance_m = wheel_clearance_m;
    sample->jump_active = jump_active;
    sample->jump_phase = jump_phase;
    sample->airborne = airborne;
    memcpy(sample->command_torque_nm,
           command_torque_nm,
           sizeof(sample->command_torque_nm));
    memcpy(sample->applied_torque_nm,
           applied_torque_nm,
           sizeof(sample->applied_torque_nm));

    if (event_window)
    {
        if (airborne && telemetry->takeoff_valid &&
            base_rise_m > telemetry->max_base_rise_m)
        {
            telemetry->max_base_rise_m = base_rise_m;
            telemetry->max_base_z_m = base_z_m;
        }
        if (wheel_clearance_m > telemetry->max_wheel_clearance_m)
        {
            telemetry->max_wheel_clearance_m = wheel_clearance_m;
        }

        for (int motor = 0; motor < TELEMETRY_MOTOR_COUNT; ++motor)
        {
            const double command_abs = fabs(command_torque_nm[motor]);
            const double applied_abs = fabs(applied_torque_nm[motor]);
            if (command_abs > telemetry->peak_command_abs_nm[motor])
            {
                telemetry->peak_command_abs_nm[motor] = command_abs;
                telemetry->peak_command_signed_nm[motor] =
                    command_torque_nm[motor];
                telemetry->peak_command_time_s[motor] = time_s;
            }
            if (applied_abs > telemetry->peak_applied_abs_nm[motor])
            {
                telemetry->peak_applied_abs_nm[motor] = applied_abs;
                telemetry->peak_applied_signed_nm[motor] =
                    applied_torque_nm[motor];
                telemetry->peak_applied_time_s[motor] = time_s;
            }
        }
    }

    telemetry->last_jump_active = jump_active;
    telemetry->last_airborne = airborne;
}

static int write_csv(const JumpTelemetry *telemetry, const char *path)
{
    FILE *file = fopen(path, "w");
    if (file == 0)
    {
        return 0;
    }

    fprintf(file,
            "time_s,jump_active,jump_phase,airborne,"
            "base_z_m,base_rise_from_takeoff_m,wheel_clearance_m");
    for (int motor = 0; motor < TELEMETRY_MOTOR_COUNT; ++motor)
    {
        fprintf(file, ",cmd_%s_nm", motor_names[motor]);
    }
    for (int motor = 0; motor < TELEMETRY_MOTOR_COUNT; ++motor)
    {
        fprintf(file, ",applied_%s_nm", motor_names[motor]);
    }
    fputc('\n', file);

    for (size_t index = 0; index < telemetry->sample_count; ++index)
    {
        const JumpTelemetrySample *sample = &telemetry->samples[index];
        fprintf(file,
                "%.6f,%d,%d,%d,%.9f,",
                sample->time_s,
                sample->jump_active,
                sample->jump_phase,
                sample->airborne,
                sample->base_z_m);
        if (isnan(sample->base_rise_m))
        {
            fprintf(file, "nan");
        }
        else
        {
            fprintf(file, "%.9f", sample->base_rise_m);
        }
        fprintf(file, ",%.9f", sample->wheel_clearance_m);
        for (int motor = 0; motor < TELEMETRY_MOTOR_COUNT; ++motor)
        {
            fprintf(file, ",%.9f", sample->command_torque_nm[motor]);
        }
        for (int motor = 0; motor < TELEMETRY_MOTOR_COUNT; ++motor)
        {
            fprintf(file, ",%.9f", sample->applied_torque_nm[motor]);
        }
        fputc('\n', file);
    }

    return fclose(file) == 0;
}

static double map_x(double time_s,
                    double time_min_s,
                    double time_max_s,
                    double x,
                    double width)
{
    return x + width * (time_s - time_min_s) /
                   fmax(1.0e-9, time_max_s - time_min_s);
}

static double map_y(double value,
                    double value_min,
                    double value_max,
                    double y,
                    double height)
{
    return y + height -
           height * (value - value_min) /
               fmax(1.0e-9, value_max - value_min);
}

static void svg_panel(FILE *file,
                      double x,
                      double y,
                      double width,
                      double height,
                      double time_min_s,
                      double time_max_s,
                      double value_min,
                      double value_max,
                      const char *title,
                      const char *unit)
{
    fprintf(file,
            "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" "
            "fill=\"#ffffff\" stroke=\"#cbd5e1\"/>\n",
            x,
            y,
            width,
            height);
    fprintf(file,
            "<text x=\"%.1f\" y=\"%.1f\" font-size=\"18\" "
            "font-weight=\"600\" fill=\"#111827\">%s</text>\n",
            x,
            y - 14.0,
            title);

    for (int grid = 0; grid <= 5; ++grid)
    {
        const double fraction = (double)grid / 5.0;
        const double grid_y = y + height * fraction;
        const double value = value_max - (value_max - value_min) * fraction;
        fprintf(file,
                "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" "
                "stroke=\"#e5e7eb\"/>\n",
                x,
                grid_y,
                x + width,
                grid_y);
        fprintf(file,
                "<text x=\"%.1f\" y=\"%.1f\" text-anchor=\"end\" "
                "font-size=\"12\" fill=\"#4b5563\">%.3f</text>\n",
                x - 8.0,
                grid_y + 4.0,
                value);
    }

    for (int grid = 0; grid <= 6; ++grid)
    {
        const double fraction = (double)grid / 6.0;
        const double grid_x = x + width * fraction;
        const double time_s =
            time_min_s + (time_max_s - time_min_s) * fraction;
        fprintf(file,
                "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" "
                "stroke=\"#f1f5f9\"/>\n",
                grid_x,
                y,
                grid_x,
                y + height);
        fprintf(file,
                "<text x=\"%.1f\" y=\"%.1f\" text-anchor=\"middle\" "
                "font-size=\"12\" fill=\"#4b5563\">%.2f</text>\n",
                grid_x,
                y + height + 20.0,
                time_s);
    }

    fprintf(file,
            "<text x=\"%.1f\" y=\"%.1f\" text-anchor=\"middle\" "
            "font-size=\"12\" fill=\"#4b5563\">time (s)</text>\n",
            x + width * 0.5,
            y + height + 39.0);
    fprintf(file,
            "<text x=\"%.1f\" y=\"%.1f\" text-anchor=\"middle\" "
            "font-size=\"12\" fill=\"#4b5563\" "
            "transform=\"rotate(-90 %.1f %.1f)\">%s</text>\n",
            x - 56.0,
            y + height * 0.5,
            x - 56.0,
            y + height * 0.5,
            unit);
}

static double height_series_value(const JumpTelemetrySample *sample,
                                  int series)
{
    if (series == 0)
    {
        return sample->base_z_m;
    }
    if (series == 1)
    {
        return sample->base_rise_m;
    }
    return sample->wheel_clearance_m;
}

static void svg_height_series(FILE *file,
                              const JumpTelemetry *telemetry,
                              int series,
                              const char *color,
                              double time_min_s,
                              double time_max_s,
                              double value_min,
                              double value_max,
                              double x,
                              double y,
                              double width,
                              double height)
{
    int line_open = 0;
    for (size_t index = 0; index < telemetry->sample_count; ++index)
    {
        const JumpTelemetrySample *sample = &telemetry->samples[index];
        const double value = height_series_value(sample, series);
        const int visible =
            sample->time_s >= time_min_s &&
            sample->time_s <= time_max_s &&
            isfinite(value);
        if (!visible)
        {
            if (line_open)
            {
                fprintf(file, "\"/>\n");
                line_open = 0;
            }
            continue;
        }
        if (!line_open)
        {
            fprintf(file,
                    "<polyline fill=\"none\" stroke=\"%s\" "
                    "stroke-width=\"2\" points=\"",
                    color);
            line_open = 1;
        }
        fprintf(file,
                "%.2f,%.2f ",
                map_x(sample->time_s,
                      time_min_s,
                      time_max_s,
                      x,
                      width),
                map_y(value,
                      value_min,
                      value_max,
                      y,
                      height));
    }
    if (line_open)
    {
        fprintf(file, "\"/>\n");
    }
}

static void svg_torque_series(FILE *file,
                              const JumpTelemetry *telemetry,
                              int motor,
                              const char *color,
                              double time_min_s,
                              double time_max_s,
                              double torque_limit,
                              double x,
                              double y,
                              double width,
                              double height)
{
    fprintf(file,
            "<polyline fill=\"none\" stroke=\"%s\" "
            "stroke-width=\"1.6\" points=\"",
            color);
    for (size_t index = 0; index < telemetry->sample_count; ++index)
    {
        const JumpTelemetrySample *sample = &telemetry->samples[index];
        if (sample->time_s < time_min_s || sample->time_s > time_max_s)
        {
            continue;
        }
        fprintf(file,
                "%.2f,%.2f ",
                map_x(sample->time_s,
                      time_min_s,
                      time_max_s,
                      x,
                      width),
                map_y(sample->applied_torque_nm[motor],
                      -torque_limit,
                      torque_limit,
                      y,
                      height));
    }
    fprintf(file, "\"/>\n");
}

static void svg_legend_item(FILE *file,
                            double x,
                            double y,
                            const char *color,
                            const char *label)
{
    fprintf(file,
            "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" "
            "stroke=\"%s\" stroke-width=\"3\"/>\n",
            x,
            y,
            x + 25.0,
            y,
            color);
    fprintf(file,
            "<text x=\"%.1f\" y=\"%.1f\" font-size=\"12\" "
            "fill=\"#374151\">%s</text>\n",
            x + 32.0,
            y + 4.0,
            label);
}

static int write_svg(const JumpTelemetry *telemetry, const char *path)
{
    if (telemetry->sample_count == 0)
    {
        return 0;
    }

    FILE *file = fopen(path, "w");
    if (file == 0)
    {
        return 0;
    }

    const JumpTelemetrySample *first = &telemetry->samples[0];
    const JumpTelemetrySample *last =
        &telemetry->samples[telemetry->sample_count - 1];
    double time_min_s = first->time_s;
    double time_max_s = last->time_s;
    if (telemetry->jump_count > 0)
    {
        time_min_s = fmax(time_min_s, telemetry->first_jump_time_s - 0.25);
        if (isfinite(telemetry->last_jump_end_time_s))
        {
            time_max_s =
                fmin(time_max_s, telemetry->last_jump_end_time_s + 0.75);
        }
    }
    if (time_max_s <= time_min_s)
    {
        time_max_s = time_min_s + 1.0;
    }

    double height_min = INFINITY;
    double height_max = -INFINITY;
    double torque_limit = 5.0;
    for (size_t index = 0; index < telemetry->sample_count; ++index)
    {
        const JumpTelemetrySample *sample = &telemetry->samples[index];
        if (sample->time_s < time_min_s || sample->time_s > time_max_s)
        {
            continue;
        }
        for (int series = 0; series < 3; ++series)
        {
            const double value = height_series_value(sample, series);
            if (isfinite(value))
            {
                height_min = fmin(height_min, value);
                height_max = fmax(height_max, value);
            }
        }
        for (int motor = 0; motor < TELEMETRY_MOTOR_COUNT; ++motor)
        {
            torque_limit =
                fmax(torque_limit, fabs(sample->applied_torque_nm[motor]));
        }
    }

    if (!isfinite(height_min) || !isfinite(height_max))
    {
        height_min = 0.0;
        height_max = 1.0;
    }
    const double height_margin =
        fmax(0.01, 0.08 * (height_max - height_min));
    height_min -= height_margin;
    height_max += height_margin;
    torque_limit *= 1.08;

    const double panel_x = 100.0;
    const double panel_width = 1260.0;
    const double panel_height = 310.0;
    const double height_y = 150.0;
    const double torque_y = 610.0;
    static const char *const height_colors[3] = {
        "#111827",
        "#16a34a",
        "#2563eb",
    };
    static const char *const torque_colors[TELEMETRY_MOTOR_COUNT] = {
        "#dc2626",
        "#f97316",
        "#7c3aed",
        "#db2777",
        "#0891b2",
        "#16a34a",
    };

    fprintf(file,
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            "<svg xmlns=\"http://www.w3.org/2000/svg\" "
            "width=\"1440\" height=\"1040\" viewBox=\"0 0 1440 1040\">\n"
            "<rect width=\"1440\" height=\"1040\" fill=\"#f8fafc\"/>\n"
            "<text x=\"100\" y=\"48\" font-size=\"28\" "
            "font-weight=\"700\" fill=\"#111827\">MuJoCo Jump Telemetry</text>\n");
    fprintf(file,
            "<text x=\"100\" y=\"78\" font-size=\"14\" fill=\"#475569\">"
            "jumps=%d | max base rise=%.3f m | max wheel clearance=%.3f m"
            "</text>\n",
            telemetry->jump_count,
            isfinite(telemetry->max_base_rise_m)
                ? telemetry->max_base_rise_m
                : 0.0,
            isfinite(telemetry->max_wheel_clearance_m)
                ? telemetry->max_wheel_clearance_m
                : 0.0);

    svg_panel(file,
              panel_x,
              height_y,
              panel_width,
              panel_height,
              time_min_s,
              time_max_s,
              height_min,
              height_max,
              "Jump height",
              "height (m)");
    for (int series = 0; series < 3; ++series)
    {
        svg_height_series(file,
                          telemetry,
                          series,
                          height_colors[series],
                          time_min_s,
                          time_max_s,
                          height_min,
                          height_max,
                          panel_x,
                          height_y,
                          panel_width,
                          panel_height);
    }
    svg_legend_item(file, 105.0, 117.0, height_colors[0], "base z");
    svg_legend_item(file, 230.0, 117.0, height_colors[1], "base rise from takeoff");
    svg_legend_item(file, 455.0, 117.0, height_colors[2], "wheel clearance");

    svg_panel(file,
              panel_x,
              torque_y,
              panel_width,
              panel_height,
              time_min_s,
              time_max_s,
              -torque_limit,
              torque_limit,
              "Applied motor torque",
              "torque (Nm)");
    for (int motor = 0; motor < TELEMETRY_MOTOR_COUNT; ++motor)
    {
        svg_torque_series(file,
                          telemetry,
                          motor,
                          torque_colors[motor],
                          time_min_s,
                          time_max_s,
                          torque_limit,
                          panel_x,
                          torque_y,
                          panel_width,
                          panel_height);
        svg_legend_item(file,
                        105.0 + 205.0 * (motor % 6),
                        577.0,
                        torque_colors[motor],
                        motor_names[motor]);
    }

    fprintf(file,
            "<text x=\"100\" y=\"1000\" font-size=\"12\" fill=\"#64748b\">"
            "Applied torque is read from MuJoCo actuator_force after mj_step. "
            "Command torque is available in the CSV file."
            "</text>\n"
            "</svg>\n");

    return fclose(file) == 0;
}

int JumpTelemetry_Write(JumpTelemetry *telemetry)
{
    if (telemetry == 0 || telemetry->sample_count == 0)
    {
        return 0;
    }
    if (!make_parent_directories(telemetry->output_prefix))
    {
        fprintf(stderr,
                "Failed to create telemetry output directory for: %s\n",
                telemetry->output_prefix);
        return 0;
    }

    char *csv_path = make_output_path(telemetry->output_prefix, ".csv");
    char *svg_path = make_output_path(telemetry->output_prefix, ".svg");
    if (csv_path == 0 || svg_path == 0)
    {
        free(csv_path);
        free(svg_path);
        return 0;
    }

    const int csv_ok = write_csv(telemetry, csv_path);
    const int svg_ok = write_svg(telemetry, svg_path);
    if (csv_ok)
    {
        printf("Jump telemetry CSV: %s\n", csv_path);
    }
    else
    {
        fprintf(stderr, "Failed to write jump telemetry CSV: %s\n", csv_path);
    }
    if (svg_ok)
    {
        printf("Jump telemetry SVG: %s\n", svg_path);
    }
    else
    {
        fprintf(stderr, "Failed to write jump telemetry SVG: %s\n", svg_path);
    }

    printf("Jump telemetry summary: jumps=%d base_rise=%.3f m apex_z=%.3f m wheel_clearance=%.3f m\n",
           telemetry->jump_count,
           isfinite(telemetry->max_base_rise_m)
               ? telemetry->max_base_rise_m
               : 0.0,
           isfinite(telemetry->max_base_z_m)
               ? telemetry->max_base_z_m
               : 0.0,
           isfinite(telemetry->max_wheel_clearance_m)
               ? telemetry->max_wheel_clearance_m
               : 0.0);
    for (int motor = 0; motor < TELEMETRY_MOTOR_COUNT; ++motor)
    {
        printf("  %-12s applied_peak=% .3f Nm @ %.3f s | command_peak=% .3f Nm @ %.3f s\n",
               motor_names[motor],
               telemetry->peak_applied_signed_nm[motor],
               telemetry->peak_applied_time_s[motor],
               telemetry->peak_command_signed_nm[motor],
               telemetry->peak_command_time_s[motor]);
    }

    free(csv_path);
    free(svg_path);
    return csv_ok && svg_ok;
}

void JumpTelemetry_Destroy(JumpTelemetry *telemetry)
{
    if (telemetry == 0)
    {
        return;
    }
    free(telemetry->samples);
    free(telemetry->output_prefix);
    free(telemetry);
}

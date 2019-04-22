#include "FiveAxisStrategy.h"
#include "Kernel.h"
#include "Config.h"
#include "Robot.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "Conveyor.h"
#include "ZProbe.h"
#include "Plane3D.h"
#include "nuts_bolts.h"

#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#define probe_point_1_checksum CHECKSUM("point1")
#define probe_point_2_checksum CHECKSUM("point2")
#define probe_point_3_checksum CHECKSUM("point3")
#define probe_point_4_checksum CHECKSUM("point4")
#define probe_point_5_checksum CHECKSUM("point5")
#define probe_point_6_checksum CHECKSUM("point6")
#define probe_point_7_checksum CHECKSUM("point7")
#define probe_point_8_checksum CHECKSUM("point8")
#define probe_point_9_checksum CHECKSUM("point9")
#define probe_point_10_checksum CHECKSUM("point10")
#define probe_device_big_part_length CHECKSUM("big_part_length")
#define probe_device_small_part_length CHECKSUM("small_part_length")
#define home_checksum CHECKSUM("home_first")

FiveAxisStrategy::FiveAxisStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
    for (size_t i = 0; i < 10; i++)
    {
        probe_points[i] = std::make_tuple(NAN, NAN, NAN);
    }
}

FiveAxisStrategy::~FiveAxisStrategy() {}

FiveAxisStrategy::handleConfig()
{
    std::string p1 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_1_checksum)->by_default("")->as_string();
    std::string p2 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_2_checksum)->by_default("")->as_string();
    std::string p3 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_3_checksum)->by_default("")->as_string();
    std::string p4 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_4_checksum)->by_default("")->as_string();
    std::string p5 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_5_checksum)->by_default("")->as_string();
    std::string p6 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_6_checksum)->by_default("")->as_string();
    std::string p7 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_7_checksum)->by_default("")->as_string();
    std::string p8 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_8_checksum)->by_default("")->as_string();
    std::string p9 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_9_checksum)->by_default("")->as_string();
    std::string p10 = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_point_10_checksum)->by_default("")->as_string();

    if (!p1.empty())
        probe_points[0] = parseXYZ(p1.c_str());
    if (!p2.empty())
        probe_points[1] = parseXYZ(p2.c_str());
    if (!p3.empty())
        probe_points[2] = parseXYZ(p3.c_str());
    if (!p4.empty())
        probe_points[3] = parseXYZ(p4.c_str());
    if (!p5.empty())
        probe_points[4] = parseXYZ(p5.c_str());
    if (!p6.empty())
        probe_points[5] = parseXYZ(p6.c_str());
    if (!p7.empty())
        probe_points[6] = parseXYZ(p7.c_str());
    if (!p8.empty())
        probe_points[7] = parseXYZ(p8.c_str());
    if (!p9.empty())
        probe_points[8] = parseXYZ(p9.c_str());
    if (!p10.empty())
        probe_points[9] = parseXYZ(p10.c_str());

    this->big_part_length = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_device_big_part_length)->by_default(10F)->as_number();
    this->small_part_length = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, probe_device_small_part_length)->by_default(10F)->as_number();
    this->home = THEKERNEL->config->value(leveling_strategy_checksum, five_axis_strategy_checksum, home_checksum)->by_default(true)->as_bool();

    for (int i = 0; i < 10; i++)
    {
        actual_probe_points[i] = std::make_tuple(0, 0, 0);
    }

    return true;
}

bool FiveAxisStrategy::handleGcode(Gcode *gcode)
{
    if (gcode->has_m)
    {
        if (gcode->m == 557)
        { // M557 - set probe points eg M557 P0 X30 Y40.5  where P is 0,1,2
            int idx = 0;
            float x = NAN, y = NAN, z = NAN;
            if (gcode->has_letter('P'))
                idx = gcode->get_value('P');
            if (gcode->has_letter('X'))
                x = gcode->get_value('X');
            if (gcode->has_letter('Y'))
                y = gcode->get_value('Y');
            if (gcode->has_letter('Z'))
                z = gcode->get_value('Z');
            if (idx >= 0 && idx <= 9)
            {
                probe_points[idx] = std::make_tuple(x, y, z);
            }
            else
            {
                gcode->stream->printf("only 9 probe points allowed P0-P9\n");
            }
            return true;
        }
        else if (gcode->m == 1005)
        {
            uint8_t stepNum = 0;
            if (gcode->has_letter('S'))
                stepNum = gcode->get_uint('S');
            gotoStep(stepNum, gcode->stream);
            return true;
        }
    }
    return false;
}

void FiveAxisStrategy::gotoStep(uint8_t step, StreamOutput *stream)
{
    if (step == 0)
    {
        THEKERNEL->conveyor->wait_for_idle();
        setAdjustFunction(false);

        if (this->home)
        {
            zprobe->home();
        }

        //Move to the first point
        std::tie(x, y, z) = probe_points[0];
        zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
        stream->printf("Moving to probe point 1: x%1.3f y%1.3f z%1.3f\n", x, y, z);
    }
    else if (step == 1)
    {
        //Save first point
        float position[3];
        THEROBOT->get_axis_position(position);
        actual_probe_points[0] = std::make_tuple(position[0], position[1], position[2]);
        stream->printf("Probe point 1 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);
        //Move to the second point
        float x, y, z;
        std::tie(x, y, z) = probe_points[1];
        zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
        stream->printf("Moving to probe point 2: x%1.3f y%1.3f z%1.3f\n", x, y, z);
    }
    else if (step == 2)
    {
        setAAxisZero(stream);
    }
    else if (step == 3)
    {
        setCAxisRegardingXY(stream);
    }
    else if (step == 4)
    {
        preLinearCorrection(stream);
    }
}

void FiveAxisStrategy::setAAxisZero(StreamOutput *stream)
{
    float a_offset = 0;
    //Save second point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[1] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 2 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    float z1, z2;
    z1 = std::get<2>(actual_probe_points[0]);
    z2 = std::get<2>(actual_probe_points[1]);
    a_offset = asinf((z2 - z1) / (2 * (this->big_part_length + this.small_part_length)));

    char *cmd = new char[128];
    if (!isnan(a_offset))
    {
        size_t n = strlen(cmd);
        snprintf(&cmd[n], CMDLEN - n, "M206 A%1.3f", a_offset);
        stream->printf("A axis offset is:%1.3f", a_offset);
        Gcode offset(cmd, &(StreamOuput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &offset);
    }
    delete[] cmd;

    zprobe->coordinated_move(NAN, NAN, 20, zprobe->getFastFeedrate(), true);
    Gcode rotateA("G0 A0", &(StreamOuput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &a_offset);
}

void FiveAxisStrategy::setCAxisRegardingXY(StreamOutput *stream)
{
    float c_offset = 0;
    float y1, y2, x1, x2, z1, z2;
    std::tie(x1, y1, z1) = actual_probe_points[0];
    std::tie(x2, y2, z2) = actual_probe_points[1];
    c_offset = atan2f((y2 - y1) / (x2 - x1));

    char *cmd = new char[128];
    if (!isnan(c_offset))
    {
        size_t n = strlen(cmd);
        snprintf(&cmd[n], CMDLEN - n, "G0 C%1.3f", c_offset);
        stream->printf("C axis offset is:%1.3f", c_offset);
        Gcode offset(cmd, &(StreamOuput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &offset);
    }
    delete[] cmd;

    //Move to the third point
    float x, y, z;
    std::tie(x, y, z) = probe_points[2];
    zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
    stream->printf("Moving to probe point 3: x%1.3f y%1.3f z%1.3f\n", x, y, z);
}

void FiveAxisStrategy::preLinearCorrection(StreamOutput *stream)
{
    //Save third point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[2] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 3 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    //Move to the fourth point
    float x, y, z;
    std::tie(x, y, z) = probe_points[3];
    zprobe->coordinated_move(x, y, z, zprobe->getFastFeedrate());
    stream->printf("Moving to probe point 4: x%1.3f y%1.3f z%1.3f\n", x, y, z);
}

void FiveAxisStrategy::setBAxisCorrection(StreamOutput *stream)
{
    //Save fourth point
    float position[3];
    THEROBOT->get_axis_position(position);
    actual_probe_points[3] = std::make_tuple(position[0], position[1], position[2]);
    stream->printf("Probe point 4 at: x%1.3f y%1.3f z%1.3f\n", position[0], position[1], position[2]);

    float b_correction = 0;
    float z3, z4;
    z3 = std::get<2>(actual_probe_points[2]);
    z4 = std::get<2>(actual_probe_points[3]);
    b_correction = asinf((z4 - z3) / ((this->big_part_length + this.small_part_length)));
    stream->printf("B axis angle: b%1.3f", b_correction);

    float x0, z0, x3;
    x3 = std : get<0>(actual_probe_points[2]);
    x0 = x3 + (this->big_part_length + this.small_part_length) * sinf(b_correction);
    z0 = z3 - (this->big_part_length + this.small_part_length) * sinf(b_correction);
    stream->printf("Real A axis rotation point: x%1.3f y%1.3f z%1.3f", x0, position[1], z0);

    auto l1 = [=](float x, float z) {
        return sqrtf((x - x0) * (x - x0) + (z - z0) * (z - z0));
    };
    auto p = [=](float x, float z) {
        return 4 * ((x + x0) * (x + x0) + (z + z0) * (z + z0));
    };
    auto q = [=](float x, float z) {};
    float dddd = p();
}

void FiveAxisStrategy::home()
{
    Gcode gc("G28", &(StreamOuput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
}

void FiveAxisStrategy::setAdjustFunction(bool on)
{
    if (on)
    {
        // set compensation function
    }
    else
    {
        THEROBOT->compensationTransform = nullptr;
    }
}

std::tuple<float, float, float> FiveAxisStrategy::parseXYZ(const char *str)
{
    float x = 0, y = 0, z = 0;
    char *p;
    x = strtof(str, &p);
    if (p + 1 < str + strlen(str))
    {
        y = strtof(p + 1, &p);
        if (p + 1 < str + strlen(str))
        {
            z = strtof(p + 1, nullptr);
        }
    }
    return std::make_tuple(x, y, z);
}
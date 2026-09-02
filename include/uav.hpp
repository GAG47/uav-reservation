#pragma once

#include "types.hpp"

struct UAV {
    int id{};
    double arrival_time{};
    Direction source{Direction::North};
    Direction destination{Direction::South};
    Movement movement{Movement::Straight};
    double speed{1.0};

    double scheduled_entry_time{-1.0};
    double exit_time{-1.0};
    double delay{-1.0};
    int selected_path_id{-1};
};

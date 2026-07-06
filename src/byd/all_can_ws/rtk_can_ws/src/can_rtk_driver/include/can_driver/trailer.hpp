#define AGV_CONFIG_H

// 添加以下包含
#include <nlohmann/json.hpp>  // 添加这一行

using json = nlohmann::json;

// 默认每节挂车的构造相同，长度单位均为mm，l1长度必读大于a
const json trailer_config = R"(
    {
        "trailer_num" : 1,
        "l" : 370,
        "a" : 335,
        "b" : 254,
        "l1" : 1009,
        "max_trailer_angle" : 60
    }
    )"_json;

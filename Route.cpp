#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <map>
#include <cmath>
#include <random>
#include <algorithm>
#include "json.hpp"
#include <climits>
#include <queue>
#include <memory>
#include <unordered_set>
#include <functional>
#include <set>

int MAX_PER_LAYER = 100;          // 每层最大元件数
int CIRCLE = 1;                   // 循环次数
double INIT_TEMP = 100000.0;      // 初始温度 (退火)
const double COOLING_RATE = 0.98; // 冷却速率 (退火)
int SA_STEPS = 1000;              // 退火步骤 (退火)
const double MIN_TEMP = 1e-5;     // 最小温度 (退火)
int MAX_LAYER = 3;                // 最大层数
double MIN_MOS_NUM = 20;          // 最小MOS数量
double SIZE_WEIGHT = 1000000;     // 面积成本权重
double IN_MATTER = 1.5;           // input的重要性
double OUT_MATTER = 0.1;          // output的重要性
int MAX_METAL_LAYER = 10;         // 最大金属层数
int VIA_COST = 100;               // 过孔代价
int LAYER_COST = 10000;           // 层数代价

using json = nlohmann::json;
using namespace std;
void print_help();
struct MosNode;
struct SubModuleNode;

// 布线相关结构
struct PathNode {
    int x, y;           // 坐标
    int layer;          // 所在层
};

struct Point {
    int x, y;           // 坐标
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
};

struct Pin {
    Point pos;
    int layer;
};

struct Segment {
    Point start, end;
    int layer;
};

struct Net {
    string name;
    vector<shared_ptr<Pin>> pins;
    vector<Segment> segments;       // 路径点和层
    vector<Point> vias;             // 过孔
};

// 金属层管理
struct MetalLayer {
    int layer_id;
    bool is_horizontal;     // 是否是水平方向
    vector<vector<bool>> used;
};

class RoutingGrid {
private:

public:
    vector<MetalLayer> metal_layers;
    vector<vector<bool>> via_space;
    int width, height;
    RoutingGrid() : width(0), height(0) {}
    RoutingGrid(int w, int h, int num_layers) : width(min(w,int(0.91*w+1))), height(min(h,int(0.92*h+1))) {
        metal_layers.resize(num_layers);
        for (int i = 0; i < num_layers; ++i) {
            metal_layers[i].layer_id = i;
            metal_layers[i].is_horizontal = (i % 2 == 0);
            metal_layers[i].used.resize(height, vector<bool>(width, false));
        }
        via_space.resize(height, vector<bool>(width, false));
    }

    bool isPositionFree(int layer, Point p) const {
        return !metal_layers[layer].used[p.y][p.x];
    }

    bool isViaFree(Point p) const {
        return via_space[p.y][p.x] == false;
    }

    void setUsed(int layer, Point p, bool status) {
        metal_layers[layer].used[p.y][p.x] = status;
    }

    void setViaOccupied(Point p, bool b) {
        via_space[p.y][p.x] = b;
    }
};


// 用于布局的元件
struct Component {
    string type; // "mosfet"s, "subModules"s, "ports"s
    string name;
    int x = 0;
    int y = 0;
    int layer = 0;
    int width = 1;
    int height = 1;
    shared_ptr<SubModuleNode> pSubModuleNode;
    shared_ptr<MosNode> pMosNode;
    vector<string> in;
    vector<string> out;
    tuple<int, int, int, int> bbox() const {
        return make_tuple(x, y, x + width, y + height);
    }
    bool overlaps(shared_ptr<Component> other) const {
        if (layer != other->layer) return false;
        auto [left1, bottom1, right1, top1] = bbox();
        auto [left2, bottom2, right2, top2] = other->bbox();
        return (left1 < right2) && (right1 > left2) &&
            (bottom1 < top2) && (top1 > bottom2);
    }
};

struct SubModuleNode
{
    unordered_map<string, vector<string>> net_in_map;
    unordered_map<string, vector<string>> net_out_map;
    string name;
    string module_name;
    vector<string> inputPorts;
    vector<string> outputPorts;
    vector<string> wirePorts;
    vector<string> mosfets;
    vector<shared_ptr<Component>> components;
    RoutingGrid routing_grid;
    SubModuleNode() : routing_grid() {}
    unordered_map<string, shared_ptr<Component>> comp_map;
    unordered_map<string, vector<shared_ptr<Component>>> in_map;
    unordered_map<string, vector<shared_ptr<Component>>> out_map;
    unordered_map<string, shared_ptr<Component>> subModuleMap;
    vector<shared_ptr<Net>> nets;
    bool isvcc = false;
    bool isgnd = false;
};
struct MosNode
{
    string drain;
    string source;
    string gate;
};
unordered_map<string, pair<int, int>> component_sizes = {
    {"input", {2, 2}},
    {"output", {2, 2}},
    {"power", {2, 2}},
    {"wire", {0, 0}},
    {"nmos", {6, 4}},
    {"pmos", {6, 4}},
};
unordered_map<string, shared_ptr<SubModuleNode>> Layouted_map;
// 全局缓存，避免重复构建相同模块
unordered_map<string, shared_ptr<SubModuleNode>> module_cache;

void markNetOnGrid(Net& net, RoutingGrid& grid) {
    for (const auto& seg : net.segments) {
        if (seg.start.x == seg.end.x) { // 垂直线
            int y_min = min(seg.start.y, seg.end.y);
            int y_max = max(seg.start.y, seg.end.y);
            for (int y = y_min; y <= y_max; y++) {
                grid.metal_layers[seg.layer].used[y][seg.start.x] = true;
            }
        }
        else { // 水平线
            int x_min = min(seg.start.x, seg.end.x);
            int x_max = max(seg.start.x, seg.end.x);
            for (int x = x_min; x <= x_max; x++) {
                grid.metal_layers[seg.layer].used[seg.start.y][x] = true;
            }
        }
    }
    for (const auto& via : net.vias) {
        grid.via_space[via.y][via.x] = true; // 标记过孔已被占用
    }
}

void unmarkNetOnGrid(Net& net, RoutingGrid& grid) {
    for (const auto& seg : net.segments) {
        if (seg.start.x == seg.end.x) { // 垂直线
            int y_min = min(seg.start.y, seg.end.y);
            int y_max = max(seg.start.y, seg.end.y);
            for (int y = y_min; y <= y_max; y++) {
                grid.metal_layers[seg.layer].used[y][seg.start.x] = false;
            }
        }
        else { // 水平线
            int x_min = min(seg.start.x, seg.end.x);
            int x_max = max(seg.start.x, seg.end.x);
            for (int x = x_min; x <= x_max; x++) {
                grid.metal_layers[seg.layer].used[seg.start.y][x] = false;
            }
        }
    }
    for (const auto& via : net.vias) {
        grid.via_space[via.y][via.x] = false;
    }
}



//计算两个元件之间的距离（a的output端口的位置到b的input端口的位置）
double distance(const Component& a, const Component& b) {
    /*
    double ax = a.x, ay = a.y;
    if (a.type == "pmos" || a.type == "nmos") {
        ax += a.width;
        ay += a.height / 2.0;
    }
    else if (a.type == "input" || a.type == "output" || a.type == "power" || a.type == "wire") {
        ax += a.width;
        ay += a.height / 2.0;
    }
    else {
        ax += a.width;
    }
    double bx = b.x, by = b.y;
    if (b.type == "nmos" || b.type == "pmos") {
        if (b.pMosNode->source == a.name) {
            by += b.height / 2.0;
        }
        else if (b.pMosNode->drain == a.name) {
            by += b.height;
            bx += b.width / 2.0;
        }
    }
    else if (a.type == "input" || a.type == "output" || a.type == "power" || a.type == "wire") {
        ay += a.height / 2.0;
    }
    return sqrt(pow(ax - bx, 2) + pow(ay - by, 2)); // 使用欧几里得距离
    */
    if (a.type == "wire" || b.type == "wire") {
        cout << "试图计算" + a.type + "类型的" + a.name + "到" + b.type + "类型的" + b.name + "之间的距离" << endl;
        return -1;
    }
    else return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// 计算退火中的元件线长成本
double calculate_component_cost(double p, shared_ptr<Component> comp,
    const unordered_map<string, vector<shared_ptr<Component>>>& in_map,
    const unordered_map<string, vector<shared_ptr<Component>>>& out_map) {
    double cost = 0.0;
    if (in_map.count(comp->name)) for (const auto& net : in_map.at(comp->name)) {
        if (net->type == "input" || net->type == "power") cost += IN_MATTER * distance(*net, *comp);
        else if (in_map.count(net->name)) for (const auto& one : in_map.at(net->name)) {
            cost += distance(*one, *comp);
        }
        else {
            cout << net->name << endl;
        }
    }
    if (out_map.count(comp->name)) for (const auto& net : out_map.at(comp->name)) {
        if (net->type == "output" || net->type == "power") cost += OUT_MATTER * distance(*net, *comp);
        else if (out_map.count(net->name)) for (const auto& tar : out_map.at(net->name)) {
            cost += distance(*comp, *tar);
        }
        else {
            cout << net->name << endl;
        }
    }
    return cost;
}

// 计算模块面积成本（模块宽乘高）
double calculate_size_cost(vector<shared_ptr<Component>>& components) {
    int min_x = 100000, max_x = -100000, min_y = 100000, max_y = -100000;
    for (const auto& comp : components) {
        if (comp->type == "input" || comp->type == "output"
            || comp->type == "power" || comp->type == "wire") continue;
        if (comp->x < min_x) min_x = comp->x;
        if (comp->x + comp->width > max_x) max_x = comp->x + comp->width;
        if (comp->y < min_y) min_y = comp->y;
        if (comp->y + comp->height > max_y) max_y = comp->y + comp->height;
    }
    int width = max_x - min_x;
    int height = max_y - min_y;
    return width * height;
}

void simulated_annealing(vector<shared_ptr<Component>>& components,
    const unordered_map<string, vector<shared_ptr<Component>>>& in_map,
    const unordered_map<string, vector<shared_ptr<Component>>>& out_map,
    int width_bound,
    int height_bound
) {
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<int> comp_dist(0, components.size() - 1);
    uniform_int_distribution<int> move_dist(0, 4);
    uniform_int_distribution<int> layer_dist(-1, 1);
    uniform_real_distribution<double> prob_dist(0.0, 1.0);

    // 计算元件平均边长
    int tatolsi = 0;
    for (auto one : components) {
        tatolsi += one->height * one->width;
    }
    int aversi = sqrt(tatolsi / (components.size()));

    // 计算初始最大步长
    int step_max0 = aversi * (1 + log(components.size()));
    if (step_max0 < component_sizes["nmos"].first) {
        cout << "好小的初始步长，是不是哪里错了" << endl;
        step_max0 = component_sizes["nmos"].first;
    }

    // 模拟退火
    double temp = INIT_TEMP;
    int ecount = 0;
    while (temp > MIN_TEMP) {
        // 计算当前最大步长
        double progress = (temp / INIT_TEMP);
        progress = max(0.0, min(1.0, progress));
        int current_max_step = progress * progress * step_max0;
        if (current_max_step < step_max0 / 4) current_max_step = step_max0 / 4;

        // 创建位置分布
        uniform_int_distribution<int> pos_dist(-current_max_step, current_max_step);

        for (int step = 0; step < SA_STEPS; ++step) {
            double action = prob_dist(gen);

            // 50%概率移动元件，50%概率交换元件
            if (action < 0.5) {
                // 移动元件
                int idx = comp_dist(gen);
                shared_ptr<Component> comp = components[idx];

                // 跳过输入端口和电源和线
                if (comp->type == "input" || comp->type == "power"
                    || comp->type == "output" || comp->type == "wire") continue;

                // 保存原位置
                int old_x = comp->x;
                int old_y = comp->y;
                int old_layer = comp->layer;

                // 生成随机偏移
                int dx = pos_dist(gen);
                int dy = pos_dist(gen);

                // 生成新位置
                int new_x = old_x + dx;
                int new_y = old_y + dy;
                int new_layer = old_layer;
                new_x = max(0, min(width_bound - comp->width, new_x));
                new_y = max(0, min(height_bound - comp->height, new_y));

                // 20%概率换层
                //if (prob_dist(gen) < 0.2) {
                //    new_layer = old_layer + layer_dist(gen);
                //    new_layer = max(0, min(MAX_LAYER - 1, new_layer));
                //}

                // 临时更新位置
                comp->x = new_x;
                comp->y = new_y;
                comp->layer = new_layer;

                // 检查是否与其他元件重叠
                bool overlap = false;
                for (auto& other : components) {
                    if (comp == other || other->type == "output" || other->type == "wire") continue;
                    if (comp->overlaps(other)) {
                        overlap = true;
                        break;
                    }
                }

                if (overlap) {
                    // 恢复原位置并跳过
                    comp->x = old_x;
                    comp->y = old_y;
                    comp->layer = old_layer;
                    continue;
                }

                // 计算成本变化
                comp->x = old_x;
                comp->y = old_y;
                comp->layer = old_layer;
                double old_size_cost = calculate_size_cost(components);
                double old_cost = calculate_component_cost(progress, comp, in_map, out_map);
                comp->x = new_x;
                comp->y = new_y;
                comp->layer = new_layer;
                double new_size_cost = calculate_size_cost(components);
                double new_cost = calculate_component_cost(progress, comp, in_map, out_map);
                double line_delta = new_cost - old_cost;
                double size_delta = new_size_cost - old_size_cost;
                double dp = (1 - progress) < 0.001 ? 1000 : 1 / (1 - progress);
                dp = (dp - 1) < 0.01 ? 0.01 : dp - 1;
                double delta = line_delta + SIZE_WEIGHT * dp * size_delta;
                // Metropolis准则
                if (delta < 0 || prob_dist(gen) < exp(-delta / temp)) {
                    // 接受移动
                }
                else {
                    // 拒绝移动，恢复原位置
                    comp->x = old_x;
                    comp->y = old_y;
                    comp->layer = old_layer;
                }
            }
            else {
                // 交换两个元件位置
                int idx1 = comp_dist(gen);
                int idx2 = comp_dist(gen);
                if (idx1 == idx2) continue;

                shared_ptr<Component> comp1 = components[idx1];
                shared_ptr<Component> comp2 = components[idx2];

                // 跳过端口和电源
                if (comp1->type == "input" || comp1->type == "power" || comp1->type == "output" || comp1->type == "wire"
                    || comp2->type == "input" || comp2->type == "power" || comp2->type == "output" || comp2->type == "wire") continue;

                // 保存原位置
                int old_x1 = comp1->x, old_y1 = comp1->y, old_layer1 = comp1->layer;
                int old_x2 = comp2->x, old_y2 = comp2->y, old_layer2 = comp2->layer;

                // 交换位置
                comp1->x = old_x2;
                comp1->y = old_y2;
                comp1->layer = old_layer2;
                comp2->x = old_x1;
                comp2->y = old_y1;
                comp2->layer = old_layer1;

                // 检查是否与其他元件重叠
                bool overlap = false;
                for (auto& other : components) {
                    if (other->type == "output") continue;
                    if (comp1 != other && comp1->overlaps(other)) {
                        overlap = true;
                        break;
                    }
                }
                if (!overlap) {
                    for (auto& other : components) {
                        if (other->type == "output") continue;
                        if (comp2 != other && comp2->overlaps(other)) {
                            overlap = true;
                            break;
                        }
                    }
                }

                if (overlap) {
                    // 恢复原位置并跳过
                    comp1->x = old_x1;
                    comp1->y = old_y1;
                    comp1->layer = old_layer1;
                    comp2->x = old_x2;
                    comp2->y = old_y2;
                    comp2->layer = old_layer2;
                    continue;
                }

                // 计算成本变化
                comp1->x = old_x1;
                comp1->y = old_y1;
                comp1->layer = old_layer1;
                comp2->x = old_x2;
                comp2->y = old_y2;
                comp2->layer = old_layer2;
                double old_cost = calculate_component_cost(progress, comp1, in_map, out_map) +
                    calculate_component_cost(progress, comp2, in_map, out_map);
                comp1->x = old_x2;
                comp1->y = old_y2;
                comp1->layer = old_layer2;
                comp2->x = old_x1;
                comp2->y = old_y1;
                comp2->layer = old_layer1;
                double new_cost = calculate_component_cost(progress, comp1, in_map, out_map) +
                    calculate_component_cost(progress, comp2, in_map, out_map);
                double delta = new_cost - old_cost;

                // Metropolis准则
                if (delta < 0 || prob_dist(gen) < exp(-delta / temp)) {
                    // 接受交换
                }
                else {
                    // 拒绝交换，恢复原位置
                    comp1->x = old_x1;
                    comp1->y = old_y1;
                    comp1->layer = old_layer1;
                    comp2->x = old_x2;
                    comp2->y = old_y2;
                    comp2->layer = old_layer2;
                }
            }
        }
        temp *= COOLING_RATE;
        int progress_percent = static_cast<int>(100 * ( log(temp/INIT_TEMP) / log(MIN_TEMP / INIT_TEMP)));
        progress_percent = max(0, min(100, progress_percent));
        if (progress_percent != ecount){
            ecount = progress_percent;
            cout << "\r[";
            int bar_length = 50;
            int filled_length = (ecount * bar_length) / 100;
            for (int i = 0; i < bar_length; ++i) {
                cout << (i < filled_length ? '=' : ' ');
            }
            cout << "] " << ecount << "%";
            cout.flush();
        }
        
    }
    cout << "\n";
}

// 应用力导向与模拟退火布局
void mixed_layout(vector<shared_ptr<Component>>& components,
    const unordered_map<string, vector<shared_ptr<Component>>>& in_map,
    const unordered_map<string, vector<shared_ptr<Component>>>& out_map,
    int width_bound, int height_bound) {
    int time = 0;
    while (time < CIRCLE) {
        simulated_annealing(components, in_map, out_map, width_bound, height_bound);
        // 计算尺寸
        int min_x = 1000000, max_x = -1000000;
        int min_y = 1000000, max_y = -1000000;
        int count = 0;
        for (const auto& comp : components) {
            if (comp->type == "input" || comp->type == "power" ||
                comp->type == "output" || comp->type == "wire") {
                continue; // 跳过特殊元件
            }
            min_x = min(min_x, comp->x);
            max_x = max(max_x, comp->x + comp->width);
            min_y = min(min_y, comp->y);
            max_y = max(max_y, comp->y + comp->height);
            count++;
        }

        // 调整特殊元件位置
        if (count > 0) {
            int input_y = min_y;
            int output_y = min_y;
            if (0) {
                for (auto& comp : components) {
                    if (comp->type == "input" || comp->type == "power") {
                        comp->x = max(min_x - comp->width, comp->x);
                        comp->y = input_y;
                        input_y += comp->height; // 垂直排列
                    }
                    else if (comp->type == "output") {
                        comp->x = min(max_x, comp->x);
                        comp->y = output_y;
                        output_y += comp->height; // 垂直排列
                    }
                }
            }
            else {
                for (auto& comp : components) {
                    if (comp->type == "input" || comp->type == "power") {
                        comp->x = min_x - comp->width;
                        comp->y = input_y;
                        input_y += comp->height; // 垂直排列
                    }
                    else if (comp->type == "output") {
                        comp->x = max_x;
                        comp->y = output_y;
                        output_y += comp->height; // 垂直排列
                    }
                }
            }
        }
        // 将所有元件平移min_x,min_y
        for (auto& comp : components) {
            comp->x -= min_x;
            comp->y -= min_y;
        }
        time++;
    }
}

json subModuleToLayoutJson(const SubModuleNode& module, const int offset_x, const int offset_y) {
    json j;
    json subModules = json::object();
    for (const auto& comp : module.components) {
        if ((comp->type != "input" && comp->type != "output" && comp->type != "power"
            && comp->type != "wire" && comp->type != "nmos" && comp->type != "pmos") && comp->pSubModuleNode) {
            subModules[comp->name] = subModuleToLayoutJson(*comp->pSubModuleNode, offset_x + comp->x, offset_y + comp->y);
        }
    }
    j["type"] = module.module_name;
    j["name"] = module.name;
    j["layout"] = {
        {"height", component_sizes[module.module_name].second},
        {"layer", 0},
        {"width", component_sizes[module.module_name].first},
        {"x", offset_x},
        {"y", offset_y}
    };
    json ports = json::object();
    for (const auto& comp : module.components) {
        if (comp->type == "input" || comp->type == "output" || comp->type == "power" || comp->type == "wire") {
            json portJson;
            portJson["type"] = comp->type;
            if (!comp->in.empty()) portJson["in"] = comp->in;
            if (!comp->out.empty()) portJson["out"] = comp->out;
            portJson["layout"] = {
                {"height", comp->height},
                {"layer", comp->layer},
                {"width", comp->width},
                {"x", offset_x + comp->x},
                {"y", offset_y + comp->y}
            };
            ports[comp->name] = portJson;
        }
    }
    j["ports"] = ports;
    json mosfets = json::object();
    for (const auto& comp : module.components) {
        if ((comp->type == "nmos" || comp->type == "pmos") && comp->pMosNode) {
            json mosfetJson;
            mosfetJson["type"] = comp->type;
            mosfetJson["drain"] = comp->pMosNode->drain;
            mosfetJson["source"] = comp->pMosNode->source;
            mosfetJson["gate"] = comp->pMosNode->gate;
            mosfetJson["layout"] = {
                {"height", comp->height},
                {"layer", comp->layer},
                {"width", comp->width},
                {"x", offset_x + comp->x},
                {"y", offset_y + comp->y}
            };
            mosfets[comp->name] = mosfetJson;
        }
    }
    j["mosfets"] = mosfets;
    j["subModules"] = subModules;
    j["inputPorts"] = module.inputPorts;
    j["outputPorts"] = module.outputPorts;
    j["isvcc"] = module.isvcc;
    j["isgnd"] = module.isgnd;
    return j;
}

json subModuleToRouteJson(const SubModuleNode& rootModule, int x, int y) {

    json routeJson;
    queue<shared_ptr<const SubModuleNode>> q;
    q.push(make_shared<const SubModuleNode>(rootModule));
    routeJson["name"] = rootModule.name;
    routeJson["module_name"] = rootModule.module_name;
    while (!q.empty()) {
        auto module = q.front();
        q.pop();

        // 记录当前模块的nets
        for (auto& net : module->nets) {
            json netJson;
            netJson["name"] = net->name;

            netJson["pins"] = json::array();
            // 转换引脚
            for (auto& pin : net->pins) {
                json pinJson;
                pinJson["x"] = pin->pos.x + x;
                pinJson["y"] = pin->pos.y + y;
                pinJson["layer"] = pin->layer;
                netJson["pins"].push_back(pinJson);
            }

            // 转换线段
            for (auto& seg : net->segments) {
                json segJson;
                segJson["start"] = { {"x", x + seg.start.x}, {"y", y + seg.start.y} };
                segJson["end"] = { {"x", x + seg.end.x}, {"y", y + seg.end.y} };
                segJson["layer"] = seg.layer;
                netJson["segments"].push_back(segJson);
            }

            // 转换过孔
            for (auto& via : net->vias) {
                json viaJson;
                viaJson["x"] = x + via.x;
                viaJson["y"] = y + via.y;
                netJson["vias"].push_back(viaJson);
            }

            routeJson["nets"].push_back(netJson);
        }

        // 处理子模块
        routeJson["subModules"] = json::object();
        for (auto& comp : module->components) {
            if (comp->pSubModuleNode) {
                comp->pSubModuleNode;
                json subJson = subModuleToRouteJson(*comp->pSubModuleNode, x + comp->x, y + comp->y);
                routeJson["subModules"][comp->name] = subJson;
            }
        }
    }
    return routeJson;
}

void outputRouteToJson(const SubModuleNode& rootModule, const string& filename) {
    json RtJson;
    RtJson[rootModule.name] = subModuleToRouteJson(rootModule, 0, 0);
    ofstream outFile(filename);
    if (outFile.is_open()) {
        outFile << RtJson.dump(4);
        outFile.close();
        std::cout << "保留布局后数据到" << filename << endl;
    }
    else {
        cerr << "Error opening file for writing: " << filename << endl;
    }
}

void outputLayoutToJson(const SubModuleNode& rootModule, const string& filename) {
    json astJson;
    astJson[rootModule.name] = subModuleToLayoutJson(rootModule, 0, 0);
    ofstream outFile(filename);
    if (outFile.is_open()) {
        outFile << astJson.dump(4);
        outFile.close();
        std::cout << "保留布局后数据到" << filename << endl;
    }
    else {
        cerr << "Error opening file for writing: " << filename << endl;
    }
}

// 为Point定义哈希函数，用于过孔去重
struct PointHash {
    size_t operator()(const Point& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};


// 路由函数：为网络生成斯坦纳树
void routeNet(Net& net) {
    if (net.pins.size() <= 1) return; // 少于2个引脚，无需布线

    // 确定水平层和垂直层
    int base_layer = net.pins[0]->layer;
    int horizontal_layer, vertical_layer;
    if (base_layer % 2 == 0) {
        horizontal_layer = base_layer;
        vertical_layer = base_layer + 1;
    }
    else {
        horizontal_layer = base_layer + 1;
        vertical_layer = base_layer;
    }

    // 收集引脚位置
    std::vector<Point> points;
    for (const auto& pin : net.pins) {
        points.push_back({ pin->pos.x, pin->pos.y });
    }

    int n = points.size();
    if (n == 0) return;

    // 构建完全图（曼哈顿距离）
    std::vector<std::vector<int>> graph(n, std::vector<int>(n, 0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            graph[i][j] = abs(points[i].x - points[j].x) + abs(points[i].y - points[j].y);
        }
    }

    // Prim算法构建最小生成树
    std::vector<int> parent(n, -1);
    std::vector<int> dist(n, INT_MAX);
    std::vector<bool> inMST(n, false);

    dist[0] = 0;
    for (int i = 0; i < n - 1; ++i) {
        int minDist = INT_MAX;
        int u = -1;
        for (int j = 0; j < n; ++j) {
            if (!inMST[j] && dist[j] < minDist) {
                minDist = dist[j];
                u = j;
            }
        }
        if (u == -1) break;
        inMST[u] = true;

        for (int v = 0; v < n; ++v) {
            if (!inMST[v] && graph[u][v] < dist[v]) {
                dist[v] = graph[u][v];
                parent[v] = u;
            }
        }
    }

    // 过孔去重集合
    std::unordered_set<Point, PointHash> vias_set;

    // 遍历最小生成树的每条边
    for (int v = 1; v < n; ++v) {
        int u = parent[v];
        if (u == -1) continue;

        Point A = points[u];
        Point B = points[v];

        // 如果两点重合，跳过
        if (A.x == B.x && A.y == B.y) continue;

        // 处理共线情况（水平或垂直）
        if (A.y == B.y) { // 水平共线
            Segment seg;
            seg.start = A;
            seg.end = B;
            seg.layer = horizontal_layer;
            net.segments.push_back(seg);
        }
        else if (A.x == B.x) { // 垂直共线
            Segment seg;
            seg.start = A;
            seg.end = B;
            seg.layer = vertical_layer;
            net.segments.push_back(seg);
        }
        else { // 需要转折
            // 选择路径：先水平后垂直
            Point via = { B.x, A.y };

            // 添加水平线段
            Segment seg_horiz;
            seg_horiz.start = A;
            seg_horiz.end = via;
            seg_horiz.layer = horizontal_layer;
            net.segments.push_back(seg_horiz);

            // 添加垂直线段
            Segment seg_vert;
            seg_vert.start = via;
            seg_vert.end = B;
            seg_vert.layer = vertical_layer;
            net.segments.push_back(seg_vert);

            // 添加过孔（去重）
            if (vias_set.find(via) == vias_set.end()) {
                net.vias.push_back(via);
                vias_set.insert(via);
            }
        }
    }
}

void rerouteConflictingNets(SubModuleNode& module);
void reRoute(Net& net, RoutingGrid& grid);
vector<string> builded_nets; // 用于记录已构建的nets名称
// 递归构建nets
void buildNets(shared_ptr<SubModuleNode> module) {
    // 先递归处理子模块
    for (auto& comp : module->components) {
        if (comp->pSubModuleNode) {
            if (find(builded_nets.begin(), builded_nets.end(), comp->type) == builded_nets.end()) buildNets(comp->pSubModuleNode);
            for (auto& sublayer : comp->pSubModuleNode->routing_grid.metal_layers) {
                for (int i = 0; i < sublayer.used.size(); i++) {
                    for (int j = 0; j < sublayer.used[0].size(); j++) {
                        if (sublayer.used[i][j]) {
                            module->routing_grid.metal_layers[sublayer.layer_id].used[i + comp->y][j + comp->x] = true;
                        }
                    }
                }
            }
        }
    }
    // 创建当前模块的nets
    for (auto& [net_name, idontcare] : module->comp_map)if (module->net_out_map.count(net_name) || module->net_in_map.count(net_name)) {
        auto net = make_shared<Net>();
        net->name = net_name;
        string ttyyppee = module->comp_map[net_name]->type;
        if (ttyyppee == "input" || ttyyppee == "output" || ttyyppee == "power") {
            auto selfpin = make_shared<Pin>();
            selfpin->pos = { module->comp_map[net_name]->x + module->comp_map[net_name]->width / 2, module->comp_map[net_name]->y + module->comp_map[net_name]->height / 2 };
            selfpin->layer = module->comp_map[net_name]->layer;
            net->pins.push_back(selfpin);
            module->routing_grid.via_space[selfpin->pos.y][selfpin->pos.x] = true; // 标记过孔位置
        }
        if (module->net_out_map.count(net_name)) {
            auto targets = module->net_out_map[net_name];
            if (ttyyppee != "input" && ttyyppee != "output" && ttyyppee != "wire" && ttyyppee != "power") continue;
            for (auto& target : targets) {  // 例如：target = o2
                if (module->comp_map.count(target)) {
                    auto target_comp = module->comp_map[target];
                    auto pin = make_shared<Pin>();
                    if (target_comp->type == "nmos" || target_comp->type == "pmos") {
                        if (target_comp->pMosNode->gate == net_name) {
                            pin->pos = { target_comp->x + target_comp->width / 2, target_comp->y + target_comp->height * 3 / 4 };
                            pin->layer = target_comp->layer;
                        }
                        else {
                            pin->pos = { target_comp->x + target_comp->width / 4, target_comp->y + target_comp->height / 2 };
                            pin->layer = target_comp->layer;

                        }
                    }
                    else if (net->name == "VCC" || net->name == "GND") {
                        int isvcc = (net->name == "VCC") ? 1 : 0;
                        pin->layer = target_comp->layer;
                        pin->pos = { target_comp->x + target_comp->width / 4, target_comp->y + target_comp->height - isvcc };
                    }
                    else {
                        cout << "不认识：" << ttyyppee << "类型的" << net_name << "的输出引脚" << target << endl;
                    }
                    net->pins.push_back(pin);
                    module->routing_grid.via_space[pin->pos.y][pin->pos.x] = true; // 标记过孔位置
                }
                else {
                    size_t dotpos = target.find('.');
                    if (dotpos == string::npos) {
                        cout << "布线时对于网络" + net_name + "的输出端口" + target + "未找到子模块" << endl;
                    }
                    else {
                        string submod_name = target.substr(0, dotpos);
                        string input_name = target.substr(dotpos + 1);
                        if (module->subModuleMap.count(submod_name)) {
                            int offsetx = module->comp_map[submod_name]->x;
                            int offsety = module->comp_map[submod_name]->y;
                            if (input_name == "VCC" || input_name == "GND") {
                                auto pin = make_shared<Pin>();
                                auto fuck = module->comp_map[submod_name]->pSubModuleNode->comp_map[input_name];
                                pin->pos = { offsetx + fuck->x + fuck->width / 2 , offsety + fuck->y + fuck->width / 2 };
                                pin->layer = fuck->layer;
                                net->pins.push_back(pin);
                                module->routing_grid.via_space[pin->pos.y][pin->pos.x] = true; // 标记过孔位置
                            }
                            else {
                                auto submod = module->subModuleMap[submod_name]->pSubModuleNode;
                                if (submod->comp_map.count(input_name)) {
                                    auto input_comp = submod->comp_map[input_name];
                                    auto pin = make_shared<Pin>();
                                    pin->pos = { offsetx + input_comp->x + input_comp->width / 2, offsety + input_comp->y + input_comp->height / 2 };
                                    pin->layer = input_comp->layer;
                                    net->pins.push_back(pin);
                                    module->routing_grid.via_space[pin->pos.y][pin->pos.x] = true; // 标记过孔位置
                                }
                            }
                        }
                        else {
                            cout << "布线时对于网络" + net_name + "的输出端口" + target + "未找到子模块" + submod_name << endl;
                        }
                    }
                }
            }
        }
        if (module->net_in_map.count(net_name)) {
            auto sources = module->net_in_map[net_name];
            if (ttyyppee != "input" && ttyyppee != "output" && ttyyppee != "wire" && ttyyppee != "power") {
                cout << "不认识：" << ttyyppee << "类型的" << net_name << "的输入引脚" << endl;
                continue;
            }
            for (auto& source : sources) {
                if (module->comp_map.count(source)) {
                    auto source_comp = module->comp_map[source];
                    if (source_comp->type != "nmos" && source_comp->type != "pmos") {
                        cout << "不认识：" << ttyyppee << "类型的" << net_name << "的输入引脚" << source << endl;
                        continue;
                    }
                    auto pin = make_shared<Pin>();
                    pin->pos = { source_comp->x + 5, source_comp->y + source_comp->height / 2 };
                    pin->layer = source_comp->layer;
                    net->pins.push_back(pin);
                    module->routing_grid.via_space[pin->pos.y][pin->pos.x] = true; // 标记过孔位置
                }
                else {
                    size_t dotpos = source.find('.');
                    if (dotpos == string::npos) {
                        cout << "布线时对于网络" + net_name + "的输入端口" + source + "未找到子模块" << endl;
                    }
                    else {
                        string submod_name = source.substr(0, dotpos);
                        string input_name = source.substr(dotpos + 1);
                        if (module->subModuleMap.count(submod_name)) {
                            int offsetx = module->comp_map[submod_name]->x;
                            int offsety = module->comp_map[submod_name]->y;
                            if (module->subModuleMap[submod_name]->pSubModuleNode->comp_map.count(input_name)) {
                                auto input_comp = module->subModuleMap[submod_name]->pSubModuleNode->comp_map[input_name];
                                auto pin = make_shared<Pin>();
                                pin->pos = { offsetx + input_comp->x + input_comp->width / 2, offsety + input_comp->y + input_comp->height / 2 };
                                pin->layer = input_comp->layer;
                                net->pins.push_back(pin);
								module->routing_grid.via_space[pin->pos.y][pin->pos.x] = true; // 标记过孔位置
                            }
                            else {
                                cout << "布线时对于网络" + net_name + "的输入端口" + source + "的子模块" + submod_name + "未找到输入端" + input_name << endl;
                            }
                        }
                        else {
                            cout << "布线时对于网络" + net_name + "的输入端口" + source + "未找到子模块" + submod_name << endl;
                        }
                    }
                }
            }
        }
        module->nets.push_back(net);
        builded_nets.push_back(module->module_name); // 记录已构建的nets类型
    }
    cout << "初始化布线" + module->module_name << endl;
    for (auto& net : module->nets) { 
        reRoute(*net, module->routing_grid); 
    }
    rerouteConflictingNets(*module);
    for (auto neet : module->nets) {
        markNetOnGrid(*neet, module->routing_grid);
    }
}

// 初始布局算法：网格布局，input和power在左边，除了output的其他在中间，output在右边
void initialLayout(shared_ptr<SubModuleNode> Module) {
    int x = 0, y = 0;
    // max_width = 总元件数量的平方根乘以平均元件宽度
    if (Module->components.empty()) return; // 如果没有组件，直接返回
    // 遍历计算平均宽度
    double total_width = 0;
    for (const auto& comp : Module->components) {
        total_width += comp->width;
    }
    int max_width = 1.5 * static_cast<int>(sqrt(Module->components.size())) * (total_width / Module->components.size());
    // 换行时x坐标增量
    int line_width = 0;

    // 布局输入和电源
    vector<shared_ptr<Component>> placed_components;
    // 布局线
    for (auto& comp : Module->components) {
        if (comp->type == "wire") {
            comp->x = -10000;
            comp->y = -10000;
            placed_components.push_back(comp);
        }
    }
    for (auto& comp : Module->components) {
        if (comp->type == "input" || comp->type == "power") {
            // 输入和电源在左边
            comp->x = x;
            comp->y = y;

            // 碰撞检测与位置调整
            bool has_overlap;
            do {
                has_overlap = false;
                for (const auto& existing : placed_components) {
                    if (comp->x < existing->x + existing->width && comp->x + comp->width > existing->x && comp->y < existing->y + existing->height && comp->y + comp->height > existing->y) {
                        has_overlap = true;
                        y += comp->height + 1; // 增加间距到2
                        comp->y = y;
                        if (y > max_width) {
                            y = 0;
                            x += line_width + 1;
                            line_width = 0;
                            comp->x = x;
                        }
                        break;
                    }
                }
            } while (has_overlap);

            line_width = max(line_width, comp->height);
            y += comp->height + 1; // 增加间距到2
            if (y > max_width) {
                y = 0;
                x += line_width + 1;
                line_width = 0;
            }
            placed_components.push_back(comp);
        }
    }
    y = 0;
    x += line_width + 1;
    line_width = 0; // 重置行高

    // 布局除了output的其他组件
    placed_components.clear();
    for (auto& comp : Module->components) {
        if (comp->type != "input" && comp->type != "power" && comp->type != "output" && comp->type != "wire") {
            // 其他组件在中间
            comp->x = x;
            comp->y = y;

            // 碰撞检测与位置调整
            bool has_overlap;
            do {
                has_overlap = false;
                for (const auto& existing : placed_components) {
                    if (comp->x < existing->x + existing->width && comp->x + comp->width > existing->x && comp->y < existing->y + existing->height && comp->y + comp->height > existing->y) {
                        has_overlap = true;
                        y += comp->height + 1;
                        comp->y = y;
                        if (y > max_width) {
                            y = 0;
                            x += line_width + 1;
                            line_width = 0;
                            comp->x = x;
                        }
                        break;
                    }
                }
            } while (has_overlap);

            line_width = max(line_width, comp->height);
            y += comp->height + 1;
            if (y > max_width) {
                y = 0;
                x += line_width + 1;
                line_width = 0;
            }
            placed_components.push_back(comp);
        }
    }
    y = 0;
    x += line_width + 1;
    line_width = 0; // 重置行高
    // 布局输出
    placed_components.clear();
    for (auto& comp : Module->components) {
        if (comp->type == "output") {
            // 输出在右边
            comp->x = x;
            comp->y = y;

            // 碰撞检测与位置调整
            bool has_overlap;
            do {
                has_overlap = false;
                for (const auto& existing : placed_components) {
                    if (comp->x < existing->x + existing->width && comp->x + comp->width > existing->x && comp->y < existing->y + existing->height && comp->y + comp->height > existing->y) {
                        has_overlap = true;
                        y += comp->height + 1;
                        comp->y = y;
                        if (y > max_width) {
                            y = 0;
                            x += line_width + 1;
                            line_width = 0;
                            comp->x = x;
                        }
                        break;
                    }
                }
            } while (has_overlap);

            line_width = max(line_width, comp->height);
            y += comp->height + 1;
            if (y > max_width) {
                y = 0;
                x += line_width + 1;
                line_width = 0;
            }
            placed_components.push_back(comp);
        }
    }
}

void layout(shared_ptr<SubModuleNode> Module) {
    // 计算初始边界
    int total_width = 0;

    for (const auto& comp : Module->components) {
        total_width += comp->width;
    }
    int total_height = 0;
    for (const auto& comp : Module->components) {
        total_height += comp->height;
    }
    int width_bound = total_width;
    int height_bound = total_height;

    for (auto& comp : Module->components) {
        // 如果是未布局过的类型的子模块，递归布局
        if (comp->pSubModuleNode) {
            if (Layouted_map.find(comp->type) == Layouted_map.end()) {
                layout(comp->pSubModuleNode);
                auto it = component_sizes.find(comp->type);
                if (it != component_sizes.end()) {
                    comp->width = it->second.first;
                    comp->height = it->second.second;
                }
            }
            // 如果布局过，则调用其内部布局信息（直接令pSubModuleNode为储存的那个，但是这样需要在输出位置时加上该子模块的偏移量）
            else {
                // 调用其内部布局信息
                comp->pSubModuleNode = Layouted_map[comp->type];
                comp->width = component_sizes[comp->type].first;
                comp->height = component_sizes[comp->type].second;
            }
        }
    }
    cout << "布局" + Module->module_name + "中……" << endl;
    initialLayout(Module);
    mixed_layout(Module->components, Module->in_map, Module->out_map, width_bound, height_bound);

    // 计算模块宽度、高度
    int min_x = 1000000, min_y = 1000000, max_x = -1000000, max_y = -1000000;
    for (const auto& comp : Module->components) {
        if (comp->type == "wire") continue;
        auto [left, bottom, right, top] = comp->bbox();
        min_x = min(min_x, comp->x);
        min_y = min(min_y, comp->y);
        max_x = max(max_x, comp->x + comp->width);
        max_y = max(max_y, comp->y + comp->height);
    }
    // 设置模块的宽度和高度
    int width = (max_x - min_x) * 1.1;
    int height = (max_y - min_y) * 1.1;
    component_sizes[Module->module_name] = { width, height };
    // 调整组件位置，使其相对于模块左上角对齐
    for (auto& comp : Module->components) {
        comp->x -= min_x;
        comp->y -= min_y;
    }

    // 初始化布线网
    Module->routing_grid = RoutingGrid(width, height, MAX_METAL_LAYER);

    // 将布局信息储存到Layouted_map
    Layouted_map[Module->module_name] = Module;
    std::cout << "布局模块" << Module->module_name << "完成，大小为" << int(width) << "x" << int(height) << endl;
}

shared_ptr<SubModuleNode> JsonToAST(const json& all_modules, const string& module_name) {
    // 检查缓存
    if (module_cache.find(module_name) != module_cache.end()) {
        return module_cache[module_name];
    }

    // 创建新模块节点
    auto module_node = make_shared<SubModuleNode>();
    module_node->name = module_name;
    module_node->module_name = module_name;
    module_cache[module_name] = module_node;

    // 检查模块是否存在
    if (!all_modules.contains(module_name)) {
        cerr << "错误：未找到模块定义: " << module_name << endl;
        return nullptr;
    }
    json module_json = all_modules[module_name];

    // 1. 处理端口
    if (module_json.contains("ports")) {
        for (auto& [name, data] : module_json["ports"].items()) {
            auto comp = make_shared<Component>();
            comp->name = name;
            comp->type = data["type"].get<string>();
            
            // 设置尺寸
            if (component_sizes.find(comp->type) != component_sizes.end()) {
                auto size = component_sizes[comp->type];
                comp->width = size.first;
                comp->height = size.second;
            }
            
            // 处理连接关系
            if (data.contains("in")) {
                for (auto& net : data["in"]) {
                    comp->in.push_back(net.get<string>());
                }
            }
            if (data.contains("out")) {
                for (auto& net : data["out"]) {
                    comp->out.push_back(net.get<string>());
                }
            }
            
            // 记录特殊端口
            if (comp->type == "input") module_node->inputPorts.push_back(name);
            if (comp->type == "output") module_node->outputPorts.push_back(name);
            if (comp->type == "wire") module_node->wirePorts.push_back(name);
            if (name == "VCC") module_node->isvcc = true;
            if (name == "GND") module_node->isgnd = true;
            
            module_node->components.push_back(comp);
            module_node->comp_map[name] = comp;
        }
    }

    // 2. 处理晶体管
    if (module_json.contains("mosfets")) {
        for (auto& [name, data] : module_json["mosfets"].items()) {
            auto comp = make_shared<Component>();
            comp->name = name;
            comp->type = data["type"].get<string>();
            
            // 设置MOS尺寸
            auto size = component_sizes[comp->type];
            comp->width = size.first;
            comp->height = size.second;
            
            // 创建MOS节点
            auto mos_node = make_shared<MosNode>();
            mos_node->gate = data["gate"].get<string>();
            mos_node->drain = data["drain"].get<string>();
            mos_node->source = data["source"].get<string>();
            comp->pMosNode = mos_node;
            
            // 设置连接关系
            comp->in = { mos_node->gate, mos_node->source };
            comp->out = { mos_node->drain };
            
            module_node->components.push_back(comp);
            module_node->comp_map[name] = comp;
            module_node->mosfets.push_back(name);
        }
    }

    // 3. 处理子模块
    if (module_json.contains("subModules") && !module_json["subModules"].is_null()) {
        for (auto& [inst_name, inst_data] : module_json["subModules"].items()) {
            string module_type = inst_data["module"].get<string>();
            
            // 创建子模块实例
            auto comp = make_shared<Component>();
            comp->name = inst_name;
            comp->type = module_type;
            
            // 递归构建子模块
            comp->pSubModuleNode = JsonToAST(all_modules, module_type);
            if (!comp->pSubModuleNode) {
                cerr << "错误：无法构建子模块 " << module_type << endl;
                continue;
            }
            
            // 设置初始尺寸（布局时会更新）
            auto size = component_sizes.find(module_type);
            if (size != component_sizes.end()) {
                comp->width = size->second.first;
                comp->height = size->second.second;
            } else {
                comp->width = 4;
                comp->height = 4;
            }
            
            // 处理连接关系
            if (inst_data.contains("connections")) {
                for (auto& [port_name, net_name] : inst_data["connections"].items()) {
                    // 获取端口类型
                    string port_type = "";
                    if (comp->pSubModuleNode->comp_map.find(port_name) != 
                        comp->pSubModuleNode->comp_map.end()) {
                        port_type = comp->pSubModuleNode->comp_map[port_name]->type;
                    }
                    
                    // 根据端口类型确定方向
                    if (port_type == "input" || port_type == "power") {
                        comp->in.push_back(net_name.get<string>());
                    } else if (port_type == "output") {
                        comp->out.push_back(net_name.get<string>());
                    } else if (!port_type.empty()) {
                        comp->in.push_back(net_name.get<string>());
                        comp->out.push_back(net_name.get<string>());
                    }
                }
            }
            
            module_node->components.push_back(comp);
            module_node->comp_map[inst_name] = comp;
            module_node->subModuleMap[inst_name] = comp;
        }
    }

    // 4. 构建连接映射
    for (auto& comp : module_node->components) {
        // 处理输出映射 (驱动网络)
        for (auto& net : comp->out) {
            module_node->in_map[net].push_back(comp);
        }
        
        // 处理输入映射 (消耗网络)
        for (auto& net : comp->in) {
            module_node->out_map[net].push_back(comp);
        }
    }

    return module_node;
}

// 计算两点曼哈顿距离
int manhattanDistance(const Point& a, const Point& b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

// 检查线段是否重叠
bool segmentsOverlap(const Segment& seg1, const Segment& seg2) {
    // 如果层不同，不可能重叠
    if (seg1.layer != seg2.layer) return false;

    // 检查水平线段重叠
    if (seg1.start.y == seg1.end.y && seg2.start.y == seg2.end.y &&
        seg1.start.y == seg2.start.y) {
        int seg1_min_x = min(seg1.start.x, seg1.end.x);
        int seg1_max_x = max(seg1.start.x, seg1.end.x);
        int seg2_min_x = min(seg2.start.x, seg2.end.x);
        int seg2_max_x = max(seg2.start.x, seg2.end.x);

        return seg1_max_x >= seg2_min_x && seg2_max_x >= seg1_min_x;
    }

    // 检查垂直线段重叠
    if (seg1.start.x == seg1.end.x && seg2.start.x == seg2.end.x &&
        seg1.start.x == seg2.start.x) {
        int seg1_min_y = min(seg1.start.y, seg1.end.y);
        int seg1_max_y = max(seg1.start.y, seg1.end.y);
        int seg2_min_y = min(seg2.start.y, seg2.end.y);
        int seg2_max_y = max(seg2.start.y, seg2.end.y);

        return seg1_max_y >= seg2_min_y && seg2_max_y >= seg1_min_y;
    }

    return false;
}

// 检查两个网络是否重叠
bool checkNetOverlap(Net& net1, Net& net2) {
    // 检查线段重叠
    for (const auto& seg1 : net1.segments) {
        for (const auto& seg2 : net2.segments) {
            if (segmentsOverlap(seg1, seg2)) {
                return true;
            }
        }
    }

    // 检查过孔重叠
    unordered_set<Point, PointHash> vias1(net1.vias.begin(), net1.vias.end());
    for (const auto& via : net2.vias) {
        for (const auto& via1 : vias1) {
            if (via == via1) {
                return true;
            }
        }
    }

    return false;
}

struct AStarNode {
    int x, y, layer;
    int g, f; // g: actual cost, f: g + heuristic
    bool operator>(const AStarNode& other) const {
        return f > other.f; // For min-heap
    }
};

vector<PathNode> findShortestPath(const Point& start, int start_layer,
    const Point& end, int end_layer,
    RoutingGrid& grid, Net& net) {
    // Get grid dimensions and layers
    int width = grid.width;
    int height = grid.height;
    int num_layers = grid.metal_layers.size();

    // Check if start/end are valid
    if (start.x < 0 || start.x >= width || start.y < 0 || start.y >= height ||
        end.x < 0 || end.x >= width || end.y < 0 || end.y >= height ||
        start_layer < 0 || start_layer >= num_layers ||
        end_layer < 0 || end_layer >= num_layers) {
        cout << "?";
        return {};
    }

    // Define movement directions: right, left, up, down
    vector<Point> directions = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };

    // 3D arrays for g-score and came_from
    vector<vector<vector<int>>> g_score(
        num_layers,
        vector<vector<int>>(height, vector<int>(width, INT_MAX))
    );

    vector<vector<vector<PathNode>>> came_from(
        num_layers,
        vector<vector<PathNode>>(height, vector<PathNode>(width, { -1, -1, -1 }))
    );

    // Priority queue (min-heap) for open nodes
    priority_queue<AStarNode, vector<AStarNode>, greater<AStarNode>> open_queue;

    // Initialize start node
    g_score[start_layer][start.y][start.x] = 0;
    int start_h = abs(start.x - end.x) + abs(start.y - end.y) + VIA_COST * abs(start_layer - end_layer);
    open_queue.push({ start.x, start.y, start_layer, 0, start_h });

    while (!open_queue.empty()) {
        AStarNode current = open_queue.top();
        open_queue.pop();

        // Skip if we found a better path already
        if (current.g > g_score[current.layer][current.y][current.x])
            continue;

        // Check if reached end
        if (current.x == end.x && current.y == end.y && current.layer == end_layer) {
            vector<PathNode> path;
            PathNode cur_node = { current.x, current.y, current.layer };

            // Reconstruct path backwards
            while (!(cur_node.x == start.x && cur_node.y == start.y && cur_node.layer == start_layer)) {
                path.push_back(cur_node);
                cur_node = came_from[cur_node.layer][cur_node.y][cur_node.x];
            }
            path.push_back({ start.x, start.y, start_layer });
            reverse(path.begin(), path.end());
            return path;
        }

        // Movement on same layer
        MetalLayer& layer_info = grid.metal_layers[current.layer];
        for (const Point& dir : directions) {
            // Check direction validity for this layer
            if (layer_info.is_horizontal && dir.y != 0) continue; // Horizontal layer: only x movement
            if (!layer_info.is_horizontal && dir.x != 0) continue; // Vertical layer: only y movement

            Point next_point = { current.x + dir.x, current.y + dir.y };

            // Check bounds
            if (next_point.x < 0 || next_point.x >= width ||
                next_point.y < 0 || next_point.y >= height)
                continue;

            bool is_self = false;
            for (auto& a : net.pins) {
                if (a->pos.x == next_point.x && a->pos.y == next_point.y) {
                    is_self = true;
                    break;
				}
            }

            // Check if position is free
            if (!grid.isPositionFree(current.layer, next_point)&&!is_self)
                continue;

            // Calculate new cost
            int new_g = current.g + 1;
            if (new_g < g_score[current.layer][next_point.y][next_point.x]) {
                g_score[current.layer][next_point.y][next_point.x] = new_g;
                int h = abs(next_point.x - end.x) + abs(next_point.y - end.y) +
                    VIA_COST * abs(current.layer - end_layer) + LAYER_COST * abs(current.layer - end_layer);
                int new_f = new_g + h;
                open_queue.push({ next_point.x, next_point.y, current.layer, new_g, new_f });
                came_from[current.layer][next_point.y][next_point.x] =
                { current.x, current.y, current.layer };
            }
        }

        // Layer switching (vias)
        for (int layer_offset : {-1, 1}) {
            int new_layer = current.layer + layer_offset;
            if (new_layer < 0 || new_layer >= num_layers) continue;

            // Same position, different layer
            Point same_point = { current.x, current.y };

            bool is_self = false;
            for (auto& a : net.pins) {
                if (a->pos.x == current.x && a->pos.y == current.y) {
                    is_self = true;
                    break;
                }
            }

            // Check via availability (via space)
            if (!grid.isViaFree(same_point) && !is_self)
                continue;

            // Check if new layer position is free
            if (!grid.isPositionFree(new_layer, same_point)&& !is_self)
                continue;

            // Calculate new cost (via cost)
            int new_g = current.g + VIA_COST;
            if (new_g < g_score[new_layer][same_point.y][same_point.x]) {
                g_score[new_layer][same_point.y][same_point.x] = new_g;
                int h = abs(same_point.x - end.x) + abs(same_point.y - end.y) + VIA_COST * abs(new_layer - end_layer)
                    + LAYER_COST * max(new_layer - end_layer, current.layer - end_layer);
                int new_f = new_g + h;
                open_queue.push({ same_point.x, same_point.y, new_layer, new_g, new_f });
                came_from[new_layer][same_point.y][same_point.x] =
                { current.x, current.y, current.layer };
            }
        }
    }
    cout << "||";
    return {}; // No path found
}

// 重新布线网络，避开障碍
void reRoute(Net& net, RoutingGrid& grid) {
    net.segments.clear();
    net.vias.clear();

    if (net.pins.size() <= 1) return;

    // 收集引脚位置和层
    vector<Point> pin_positions;
    vector<int> pin_layers;
    for (const auto& pin : net.pins) {
        pin_positions.push_back(pin->pos);
        pin_layers.push_back(pin->layer);
    }

    int n = pin_positions.size();
    vector<vector<int>> distances(n, vector<int>(n, INT_MAX));
    vector<vector<vector<PathNode>>> paths(n, vector<vector<PathNode>>(n));

    // 计算所有点对之间的最短路径
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            auto path = findShortestPath(
                pin_positions[i], pin_layers[i],
                pin_positions[j], pin_layers[j],
                grid,
                net
            );
            if (!path.empty()) {
                paths[i][j] = path;
                paths[j][i] = path;
                // 计算路径长度（实际代价）
                int dist = 0;
                for (size_t k = 1; k < path.size(); ++k) {
                    // 移动代价
                    if (path[k].x != path[k - 1].x || path[k].y != path[k - 1].y) {
                        dist += 1;
                    }
                    // 过孔代价
                    else if (path[k].layer != path[k - 1].layer) {
                        dist += 5;
                    }
                }
                distances[i][j] = dist;
                distances[j][i] = dist;
            }
        }
    }

    // 使用Prim算法构建最小生成树
    vector<bool> inTree(n, false);
    vector<int> minDist(n, INT_MAX);
    vector<int> parent(n, -1);
    minDist[0] = 0;

    for (int i = 0; i < n - 1; ++i) {
        int u = -1;
        for (int j = 0; j < n; ++j) {
            if (!inTree[j] && (u == -1 || minDist[j] < minDist[u])) {
                u = j;
            }
        }

        if (minDist[u] == INT_MAX) break; // 图不连通
        inTree[u] = true;

        for (int v = 0; v < n; ++v) {
            if (!inTree[v] && distances[u][v] < minDist[v]) {
                minDist[v] = distances[u][v];
                parent[v] = u;
            }
        }
    }

    // 过孔去重集合
    unordered_set<Point, PointHash> vias_set;

    // 为最小生成树的每条边添加路径
    for (int i = 1; i < n; ++i) {
        if (parent[i] == -1) continue;

        int u = parent[i];
        int v = i;
        const auto& path = paths[u][v];

        // 添加路径中的线段和过孔
        for (size_t j = 1; j < path.size(); ++j) {
            const auto& prev = path[j - 1];
            const auto& curr = path[j];

            // 添加线段（如果位置发生变化）
            if (prev.x != curr.x || prev.y != curr.y) {
                Segment seg;
                seg.start = { prev.x, prev.y };
                seg.end = { curr.x, curr.y };
                seg.layer = prev.layer; // 线段属于起始点的层
                net.segments.push_back(seg);
            }

            // 添加过孔（如果层发生变化）
            if (prev.layer != curr.layer) {
                Point via_pos = { prev.x, prev.y }; // 或 curr.x, curr.y 相同
                if (vias_set.find(via_pos) == vias_set.end()) {
                    net.vias.push_back(via_pos);
                    vias_set.insert(via_pos);
                }
            }
        }
    }
    // cout << ">";
}

// 拆线重排主函数
void rerouteConflictingNets(SubModuleNode& module) {
    bool conflictFound = true;
    int maxIterations = 10;
    cout << "拆线重布" << module.module_name << endl;
    // 对module.nets按照总线长升序排序
    sort(module.nets.begin(), module.nets.end(), [](const shared_ptr<Net>& a, const shared_ptr<Net>& b) {
        double lenA = 0, lenB = 0;
        for (const auto& seg : a->segments) {
            if (seg.start.x == seg.end.x) {
                lenA += abs(seg.start.y - seg.end.y);
            }
            else {
                lenA += abs(seg.start.x - seg.end.x);
            }
        }
        for (const auto& seg : b->segments) {
            if (seg.start.x == seg.end.x) {
                lenB += abs(seg.start.y - seg.end.y);
            }
            else {
                lenB += abs(seg.start.x - seg.end.x);
            }
        }
        return lenA < lenB;
        });

    while (conflictFound && maxIterations-- > 0) {
        cout << "[";
        conflictFound = false;
        for (int i = 0; i < module.nets.size(); i++) {
            for (int j = i + 1; j < module.nets.size(); j++) {
                Net& net1 = *module.nets[i];
                Net& net2 = *module.nets[j];

                if (checkNetOverlap(net1, net2)) {
                    conflictFound = true;
                    markNetOnGrid(net1, module.routing_grid);
                    // 重新布线net2
                    if (!component_sizes.count(module.module_name))cout << "不存在" << module.module_name << endl;
                    reRoute(net2, module.routing_grid);
                    markNetOnGrid(net2, module.routing_grid);
                    if (checkNetOverlap(net1, net2))cout << "x";
                    else cout << "=";
                }
            }
        }
        cout << "]\n";
    }
    if (conflictFound) cout << "无法实现无重叠，退出" << module.name + "的布线" << endl;
    else cout << "\n成功布线！" << endl;
}

int main(int argc, char* argv[]) {
    string filename = "output.json";       // 默认输入文件
    string module_name = "adder4";             // 模块名
    string layout_output = "Layout_after.json"; // 默认布局输出文件
    string route_output = "Route_after.json";   // 默认布线输出文件
    bool help_flag = false;

    // 解析命令行参数
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "-f" && i + 1 < argc) {
            filename = argv[++i];
        } else if (arg == "-m" && i + 1 < argc) {
            module_name = argv[++i];
        } else if (arg == "-n" && i + 1 < argc) {
            try {
                MIN_MOS_NUM = stod(argv[++i]);
                if (MIN_MOS_NUM <= 0) { cerr << "错误：最小MOS数量必须为正数\n"; return 1; }
            } catch (...) { cerr << "错误：无效的-n参数\n"; return 1; }
        } else if (arg == "-t" && i + 1 < argc) {
            try {
                SA_STEPS = stoi(argv[++i]);
                if (SA_STEPS <= 0) { cerr << "错误：退火步骤必须为正数\n"; return 1; }
            } catch (...) { cerr << "错误：无效的-t参数\n"; return 1; }
        } else if (arg == "-c" && i + 1 < argc) {
            try {
                CIRCLE = stoi(argv[++i]);
                if (CIRCLE <= 0) { cerr << "错误：循环次数必须为正数\n"; return 1; }
            } catch (...) { cerr << "错误：无效的-c参数\n"; return 1; }
        } else if (arg == "-i" && i + 1 < argc) {
            try {
                INIT_TEMP = stod(argv[++i]);
                if (INIT_TEMP <= 0) { cerr << "错误：初始温度必须为正数\n"; return 1; }
            } catch (...) { cerr << "错误：无效的-i参数\n"; return 1; }
        } else if (arg == "-l" && i + 1 < argc) {
            layout_output = argv[++i];
        } else if (arg == "-r" && i + 1 < argc) {
            route_output = argv[++i];
        } else if (arg == "-h") {
            help_flag = true;
        } else {
            cerr << "未知参数: " << arg << "\n使用-h查看帮助\n";
            return 1;
        }
    }

    // 显示帮助信息
    if (help_flag) {
        print_help();
        return 0;
    }

    // 读取JSON文件
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        return 1;
    }
    json j;
    file >> j;

    // 获取模块名（命令行未指定则提示输入）
    if (module_name.empty()) {
        std::cout << "Enter module name: ";
        cin >> module_name;
    }

    if (!j.contains(module_name)) {
        cerr << "模块不存在: " << module_name << endl;
        return 1;
    }

    // 创建AST根节点并执行处理流程
    shared_ptr<SubModuleNode> root = make_shared<SubModuleNode>();
    root->name = module_name;
    root->module_name = module_name;

    std::cout << "处理文件中……" << endl;
    root = JsonToAST(j, module_name);
    cout << "布局元件中……" << endl;
    layout(root);
    outputLayoutToJson(*root, layout_output);
    buildNets(root);
    outputRouteToJson(*root, route_output);
    return 0;
}

// 打印帮助信息
void print_help() {
    cout << "=== 布线布局程序参数说明 ===\n";
    cout << "-f <文件名>   指定输入JSON文件 (默认: design.json)\n";
    cout << "-m <模块名>   指定要处理的模块名(默认: top_module)\n";
    cout << "-n <数量>     设置最小MOS数量 (默认: 20)\n";
    cout << "-t <步骤>     设置退火算法迭代步骤 (默认: 1000)\n";
    cout << "-c <次数>     设置布局循环次数 (默认: 1)\n";
    cout << "-i <温度>     设置初始退火温度 (默认: 100000.0)\n";
    cout << "-l <文件名>   设置布局结果输出文件 (默认: Layout_after.json)\n";
    cout << "-r <文件名>   设置布线结果输出文件 (默认: Route_after.json)\n";
    cout << "-h            显示此帮助信息\n";
}
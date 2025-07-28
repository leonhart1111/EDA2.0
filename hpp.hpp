#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <regex>
#include <fstream>
#include <map>
#include <memory>
#include "json.hpp"

using json = nlohmann::ordered_json;

// 连接方向枚举
enum Direction { IN, OUT };

// 端口类型枚举
enum PortType { UNDEF, INPUT, OUTPUT, WIRE, POWER};
// power包括VCC、GND、CLK

// 晶体管类型
enum MosType { NMOS, PMOS};

// Token类型
enum TokenType {
    KEYWORD,
    USER_DEF,
    SYMBOL,
    NONE
};

// 前向声明
class Component;
class MosNode;
class ModuleNode;

// 连接结构体
struct Connection {
    std::shared_ptr<Component> component;  // 连接的元件
    Direction direction;                   // 连接方向
    std::string portName;                  // 元件端口名称

    Connection(std::shared_ptr<Component> comp, 
               Direction dir, 
               std::string name)
        : component(std::move(comp)), 
          direction(dir), 
          portName(std::move(name)) {}
    
    json toJSON() const;
};

// AST节点基类
class ASTNode {
public:
    virtual ~ASTNode() = default;
    virtual json toJSON() const = 0;
};

// 端口节点
class PortNode : public ASTNode {
public:
    std::string name;                      // 端口名称
    PortType type = UNDEF;                         // 端口类型
    std::vector<Connection> connections;   // 连接信息
    json toJSON() const override;
};

// 元件基类
class Component : public ASTNode {
public:
    virtual ~Component() = default;
    virtual std::string getName() const = 0;
};

// 晶体管节点
class MosNode : public Component {
public:
    std::string name;                     // 晶体管名称
    MosType mostype;                  // 晶体管类型 (nmos/pmos)
    std::string drain;
    std::string source;
    std::string gate;
    std::shared_ptr<PortNode> _drain;      // 漏极
    std::shared_ptr<PortNode> _gate;       // 栅极
    std::shared_ptr<PortNode> _source;     // 源极   
    std::string getName() const override { return name; }
    json toJSON() const override;
};

// 模块实例
class ModuleNode : ASTNode{
public:
    std::string module_name;
    bool isAtom; // 是否为基础模块
    std::vector<std::shared_ptr<PortNode>> ports;
    std::vector<std::shared_ptr<Component>> components;

    json toJSON() const override;
};

// 模块节点
class SubModuleNode : public Component {
public:
    std::string name;                     // 实例名称
    std::string module_name;                // 模块名称
    std::shared_ptr<ModuleNode> ptr;
    std::map<std::string, std::string> InNetMap;       // 端口列表
    std::map<std::string, std::string> OutNetMap;       // 端口列表

    std::string getName() const override { return name; }
    json toJSON() const override;
};

// Connection 的 JSON 序列化实现
json Connection::toJSON() const {
    json connJson;
    if (component) {
        connJson[component->getName()] = portName;
    }
    else throw std::runtime_error("没有找到连接的元件");
    return connJson;
}

// PortNode 的 JSON 序列化实现
json PortNode::toJSON() const {
    json connectionsJson;
    std::map<std::string, std::vector<json>> connectionMap;
    
    // 分类收集输入和输出连接
    for (const auto& conn : connections) {
        if (conn.direction == IN) {
            connectionMap["in"].push_back(conn.toJSON());
        } else {
            connectionMap["out"].push_back(conn.toJSON());
        }
    }
    
    // 构建端口JSON
    json portJson;
    portJson["type"] = [this]() {
        switch (type) {
            case INPUT: return "input";
            case OUTPUT: return "output";
            case WIRE: return "wire";
            case POWER: return "power";
            default: return "unknown";
        }
    }();
    
    portJson["connections"] = connectionMap;
    return portJson;
}

// MosNode 的 JSON 序列化实现
json MosNode::toJSON() const {
    json mosJson;
    mosJson["type"] = mostype == NMOS ? "nmos" : "pmos";
    mosJson["in"] = {
        {"gate", gate},
        {"source", source}
    };
    mosJson["out"] = {
        {"drain", drain},
    };
    return mosJson;
}

json SubModuleNode::toJSON() const {
    json inJson, outJson;
    for (const auto& [portName, netName] : InNetMap) {
        inJson[portName] = netName;
    }
    for (const auto& [portName, netName] : OutNetMap) {
        outJson[portName] = netName;
    }
    return {
        {"type", module_name},
        {"in", inJson},
        {"out", outJson}
    };
}

// ModuleNode 的 JSON 序列化实现
json ModuleNode::toJSON() const {
    json moduleJson;
    
    // 添加端口信息
    json portsJson;
    for (const auto& port : ports) {
        portsJson[port->name] = port->toJSON();
    }
    moduleJson["ports"] = portsJson;
    
    // 添加元件信息 如："n1":{"type":"mos", ports:["drain": "net1", "source": "net2", "gate": "input1"], 
    //               "adder1": {"type": "adder", ports:["input1": "net1", ......]}
    json componentsJson;
    for (const auto& comp : components) {
        if (auto mos = std::dynamic_pointer_cast<MosNode>(comp)) {
            componentsJson[mos->getName()] = mos->toJSON();
        } else if (auto mod = std::dynamic_pointer_cast<SubModuleNode>(comp)) {
            componentsJson[mod->getName()] = mod->toJSON();
        }
        else throw std::runtime_error("Error:Unknown component type of " + comp->getName());
    }
    moduleJson["components"] = componentsJson;
    
    // 添加模块属性
    moduleJson["isAtom"] = isAtom;
    
    return moduleJson;
}
import json
import copy
from math import pow

# 0 - low
# 1 - high
# 2 - Z
# 3 - X

Z = 2
X = 3
DEBUG = False

def state2str(state):
    if state == 0:
        return '0'
    elif state == 1:
        return '1'
    elif state == Z:
        return 'Z'
    elif state == X:
        return 'X'

def dic_show(outputs):
    for key in outputs.keys():
        print(f"{key}: {state2str(outputs[key])}")

def deserialize(keys, num):
    value_lis = []
    for i in range(len(keys)):
        value_lis.append(num & 0b11)
        num >>= 2
    value_lis = value_lis[::-1]

    dic = {}
    for id, key in enumerate(keys):
        dic[key] = value_lis[id]
    return dic

def serialize(dic):
    num = 0
    for value in dic.values():
        num += value
        num <<= 2
    return num >> 2

class Ports:
    def __init__(self, json_data):
        self.powers = {}
        self.inputs = {}
        self.outputs = {}
        self.wires = {}

        for key in json_data.keys():
            port = json_data[key]
            if port['type'] == "power":
                self.powers[key] = port['connections']
            elif port['type'] == "input":
                self.inputs[key] = port['connections']
            elif port['type'] == 'output':
                self.outputs[key] = port['connections']
                self.wires[key] = port['connections']       # 将output也当作wire
            elif port['type'] == "wire":
                self.wires[key] = port['connections']

    def reset(self):
        self.values = {}
        self.times = {}
        for key in self.wires.keys():
            self.values[key] = Z
            # self.times[key] = gap       # 时刻上为gap, 保证不会从它开始
        for key in self.powers.keys():
            if key == "VCC":
                self.values[key] = 1
            elif key == "GND":
                self.values[key] = 0
    
    def set_inputs(self, inputs, show=False):
        for key in inputs.keys():
            if DEBUG and show:
                print(f"set {key} as {state2str(inputs[key])}")
            self.values[key] = inputs[key]

    def update_port(self, name, state, show=False):
        """更新port状态, 如果更新, 将out_comp返回"""
        # TODO: 之后加timestamp
        origin = self.values[name]
        if origin == X or origin == state or state == Z:
            if DEBUG and show:
                print(f"no update to {name}: {state2str(origin)}")
            return []
        if origin == Z:
            self.values[name] = state
        else:
            self.values[name] = X
        
        if DEBUG and show:
            print(f"update {name}: {state2str(origin)} -> {state2str(self.values[name])}")

        if "out" not in self.wires[name]:
            return []
        return self.wires[name]['out']

    def get_outputs(self):
        out_dic = {}
        for key in self.outputs.keys():
            out_dic[key] = self.values[key]
        return out_dic

class Component:
    def __init__(self, json_data, name):
        self.name = name
        self.type = json_data['type']
        self.inPorts = json_data['in']
        self.outPorts = json_data['out']

class Module:
    def __init__(self, json_data, name):
        self.name = name
        self.top = False
        self.isAtom = json_data['isAtom']
        self.truth_tab = None

        self.components = {}
        sub_data = json_data['components']
        for key in sub_data:
            self.components[key] = Component(sub_data[key], key)

        self.ports = Ports(json_data['ports'])

    def sub_names(self):
        """获取这个模块有哪些子模块(非递归)"""
        names = []
        for key in self.components.keys():
            comp = self.components[key]
            if comp.type != "pmos" and comp.type != "nmos":
                names.append(comp.type)
        return names

    def _compute_mos(self, comp: Component):
        """对mos管结果进行计算"""
        g_port = comp.inPorts['gate']
        s_port = comp.inPorts['source']
        g_state = self.ports.values[g_port]
        s_state = self.ports.values[s_port]

        if DEBUG and self.top:
            print(f"update mos {comp.name}, src: {state2str(s_state)}, gate: {state2str(g_state)}")

        if g_state == X or s_state == X:
            res = X
        elif g_state == 2:
            res = 2
        else:
            if comp.type == "pmos":
                if g_state == 0:
                    res = s_state
                else:
                    res = Z
            elif comp.type == "nmos":
                if g_state == 1:
                    res = s_state
                else:
                    res = Z
        return {"drain": res}

    def _compute_module(self, comp: Component):
        """对子模块结果进行计算"""
        # 1. 实例化
        name = comp.name
        if not name in self.sub_modules:
            module = copy.deepcopy(self.sub_table[comp.type])
            self.sub_modules[name] = module
            module.simulate_init(self.sub_table)
        else:
            module = self.sub_modules[name]

        # 2. 输入准备
        inputs = {}
        for key in module.ports.inputs.keys():
            in_port = comp.inPorts[key]
            inputs[key] = self.ports.values[in_port]

        if DEBUG and self.top:
            print(f"update module {comp.name}")
            dic_show(inputs)

        # 3. 进行仿真并返回
        return module.simulate_step(inputs)

    def _compute(self, comp: Component):
        """获取各个in_port的状态, 更新out_port(如果更新则返回)"""
        # 1. 对于mos管, 直接计算
        if comp.type == "pmos" or comp.type == "nmos":
            outputs = self._compute_mos(comp)
        # 2. 对于子模块, 向下计算仿真
        else:
            outputs = self._compute_module(comp)

        # 3. 对输出的port进行更新
        for key, out_port in comp.outPorts.items():
            res = outputs[key]
            new_updates = self.ports.update_port(out_port, res, self.top)
            self.update_info += new_updates

    def _update_comp(self, comp_info):
        for key in comp_info.keys():
            name = key
            break
        port = comp_info[name]
        
        comp = self.components[name]
        assert port in comp.inPorts
        self._compute(comp)
    
    def simulate_init(self, sub_modules):
        if self.isAtom:
            self.simulate_all(sub_modules)
            return

        self.sub_modules = {}
        self.sub_table = sub_modules
        self.ports.reset()

    def simulate_step(self, inputs):
        if self.truth_tab is not None:
            rid = serialize(inputs)
            outputs = deserialize(self.ports.outputs.keys(), self.truth_tab[rid])
            return outputs
        
        self.ports.set_inputs(inputs, self.top)

        self.update_info = []
        for in_port in self.ports.inputs.values():
            for out_comp in in_port['out']:
                self.update_info.append(out_comp)
        for power in self.ports.powers.values():
            for out_comp in power['out']:
                self.update_info.append(out_comp)

        while len(self.update_info) > 0:
            self._update_comp(self.update_info[0])
            self.update_info = self.update_info[1:]

        return self.ports.get_outputs()

    def simulate(self, inputs, sub_modules):
        # 0. 是否有对应真值
        if self.truth_tab is not None:
            rid = serialize(inputs)
            print(rid)
            outputs = deserialize(self.ports.outputs.keys(), self.truth_tab[rid])
            return outputs

        # 1. 检查输入内容
        if inputs.keys() != self.ports.inputs.keys():
            print("Error: the inputs not match")
            exit()
        for value in inputs.values():
            if value not in [0, 1, Z, X]:
                print(f"Error: invalid value {value}")
                exit()

        # 2. 子模块处理
        self.sub_modules = {}           # 实例化的子模块列表
        self.sub_table = sub_modules    # 提供的子模块表

        # 3. 重置所有状态
        self.ports.reset()
        
        # 4. 设置输入
        self.ports.set_inputs(inputs)

        # 5. 开始仿真
        self.update_info = []       # 需要更新的组件及其接口
        ## 5.1. 从inputs找到需要更新的对象
        for in_port in self.ports.inputs.values():
            for out_comp in in_port['out']:
                self.update_info.append(out_comp)
        for power in self.ports.powers.values():
            for out_comp in power['out']:
                self.update_info.append(out_comp)

        ## 5.2. 迭代更新直到update_comp为空
        while len(self.update_info) > 0:
            self._update_comp(self.update_info[0])
            self.update_info = self.update_info[1:]

        # 6. 返回结果
        return self.ports.get_outputs()
    
    def show_truth_tab(self):
        if self.truth_tab is None:
            print("No truth table")
            return
        for id, num in enumerate(self.truth_tab):
            inputs = deserialize(self.ports.inputs.keys(), id)
            outputs = deserialize(self.ports.outputs.keys(), num)
            for key in inputs.keys():
                inputs[key] = state2str(inputs[key])
            for key in outputs.keys():
                outputs[key] = state2str(outputs[key])
            print(f"inputs: {inputs}, outputs: {outputs}")

    def simulate_all(self, sub_modules):
        if self.truth_tab is not None:
            return
        truth_tab = []
        keys = self.ports.inputs.keys()
        input_num = len(keys)
        
        row = int(pow(4, input_num))
        for num in range(row):
            inputs = deserialize(keys, num)
            outputs = self.simulate(inputs, sub_modules)
            num = serialize(outputs)
            truth_tab.append(num)
        self.truth_tab = truth_tab

class Circuit:
    def __init__(self, path):
        with open(path, "r") as file:
            data = json.load(file)
        
        self.modules = {}
        for key in data.keys():
            self.modules[key] = Module(data[key], key)

    def _sub_modules(self, module):
        """递归的构建需要的所有子模块并传入"""
        sub_modules = {}
        new_names = module.sub_names()
        for name in new_names:
            if name in sub_modules.keys():
                continue
            sub_module = self.modules[name]
            sub_modules[name] = sub_module
            new_names += sub_module.sub_names()
        return sub_modules

    def simulate(self, name, inputs):
        if name not in self.modules:
            print(f"Can't find module {name}")
        
        module = copy.deepcopy(self.modules[name])
        module.top = True
        sub_modules = self._sub_modules(module)
        outputs = module.simulate(inputs, sub_modules)
        return outputs

    def simulate_all(self, name):
        if name not in self.modules:
            print(f"Can't find module {name}")
        module = copy.deepcopy(self.modules[name])
        sub_modules = self._sub_modules(module)
        module.simulate_all(sub_modules)
        return module

if __name__ == "__main__":
    path = "./output.json"
    circuit = Circuit(path)

    name = "adder4"
    inputs = {
        "A0": 0,
        "A1": 0,
        "A2": 0,
        "A3": 1,
        "B0": 1,
        "B1": 0,
        "B2": 0,
        "B3": 1,
        "Cin": 0,
    }

    # name = "adder"
    # inputs = {
    #     "A": 1,
    #     "B": 1,
    #     "Cin": 1
    # }

    outputs = circuit.simulate(name, inputs)
    dic_show(outputs)

    # name = "My_and"
    # module = circuit.simulate_all(name)
    # module.show_truth_tab()
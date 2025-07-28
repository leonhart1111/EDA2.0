#include "hpp.hpp"

// 定义正则表达式
const std::regex keywords_regex(R"(module|input|output|wire|pmos|nmos|endmodule|include)");
const std::regex identifier_regex(R"([a-zA-Z_][a-zA-Z0-9_]*)");
//暂时无用
const std::regex number_regex(R"(\d+)");
const std::regex operator_regex(R"([,\.\(\)])");
const std::regex whitespace_regex(R"([ \t\n;]+)");
std::vector<std::string> Atoms;

using Token=std::pair<std::string,int>;

class Lexer {
public:
    Lexer(std::ifstream& f) : curLine(1),file(std::move(f)) {}
    Token pairCurToken(){
        Token token;int type;

        if (std::regex_match(current_token, keywords_regex)){
            type = KEYWORD;
        }
        else if (std::regex_match(current_token, identifier_regex)){
            type = USER_DEF;
        }

        token=std::make_pair(current_token,type);
        current_token.clear();
        
        return token;
    }
    std::string getLine(){
        return std::to_string(curLine);
    }
    Token getNextToken(){
        if(nextToken.first!=""){
            auto token=nextToken;
            nextToken.first="";
            return token;
        }
        // 逐个字符匹配,按顺序添加到词列表
        char c;
        while (file.get(c)) {
            if (c == ',' || c == '(' || c == ')'||c==';') {
                if (!current_token.empty()) {
                    nextToken=std::make_pair(std::string(1, c),SYMBOL);
                    return pairCurToken();
                }
                else{
                    return std::make_pair(std::string(1, c),SYMBOL);
                }
            } 
            else if (c==' '||c=='\n'||c=='\t'||c=='\r'||c=='\v') {
                if(c=='\n'||c=='\r'){
                    curLine++;
                }
                if (!current_token.empty()) {
                    return pairCurToken();
                }
            } 
            else {
                current_token += c;
            }
        }
        if (!current_token.empty()) {
            return pairCurToken();
        }
        else return std::make_pair("",NONE);
    }

private:
    std::string current_token;
    Token nextToken;//under analysis
    std::ifstream file;
    int curLine;
    // std::vector<Token> tokens;
    // size_t pos;
};

class Parser{
private:
    Lexer& lexer;
    Token token;//under analysis
    int pcount,ncount;
    std::shared_ptr<ModuleNode> moduleNode;
    std::vector<std::shared_ptr<ModuleNode>> modules;
public:
    Parser(Lexer& lexer):lexer(lexer),pcount(1),ncount(1){
        resetModule();
        moduleNode = std::make_shared<ModuleNode>();
    }
    void parse(){
        while((token=lexer.getNextToken()).second!=NONE){
            if(token.first=="include"){
                parseInclude();
            }
            if(token.first=="module"){
                parseModule();
            }
            if(token.first=="endmodule"){
                modules.push_back(moduleNode);
                resetModule();
                //分析新定义moduleNode
                moduleNode = std::make_shared<ModuleNode>();
            }
        }
    }
    json toJSON() const {
        json module_j;
        for(const auto&m:modules){
            module_j[m->module_name]=m->toJSON();
        }
        return module_j;
    }
    std::vector<std::shared_ptr<ModuleNode>> getModules() const {
        return modules;
    }
private:
    void parseInclude(){
        expect("\"");
        std::string filename=token.first;
        expect("\"");
        expect(";");
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Error:无法打开文件 " + filename);
        }
        Lexer lexer(file);
        Parser parser(lexer);
        parser.parse();
        modules.insert(modules.end(), parser.getModules().begin(), parser.getModules().end());
    }
    void parseModule(){
        moduleNode->module_name = lexer.getNextToken().first;
        moduleNode->isAtom = find(Atoms.begin(),Atoms.end(),moduleNode->module_name)!=Atoms.end();
        //TODO:删除无用port
        auto vcc=std::make_shared<PortNode>();
        auto gnd=std::make_shared<PortNode>();
        auto clk=std::make_shared<PortNode>();
        vcc->name = "VCC";
        gnd->name = "GND";
        clk->name = "CLK";
        vcc->type = POWER;
        gnd->type = POWER;
        clk->type = POWER;
        moduleNode->ports.push_back(vcc);
        moduleNode->ports.push_back(gnd);
        moduleNode->ports.push_back(clk);

        expect("(");
        while((token=lexer.getNextToken()).first!=")")
        {
            if(token.second==USER_DEF){
                auto portNode=std::make_shared<PortNode>();
                portNode->name = token.first;
                moduleNode->ports.push_back(portNode);
            }
            else if(token.first==","){
                continue;
            }
            else{
                throw std::runtime_error("Error:module参数中语法错误,Line "+lexer.getLine());
            }
        }
        while ((token = lexer.getNextToken()).first != "endmodule") {
            if (token.first == "input" || token.first == "output"|| token.first == "wire") {
                parsePort(token.first);
            }
            else if (token.first == "pmos" || token.first == "nmos") {
                parseMos(token.first);
            } 
            else if (token.first == "//"){
                parseNotes();
            }
            else if (token.second == USER_DEF){
                parseModuleNesting(token.first);
            }
            else if( token.second==NONE){
                throw std::runtime_error("Error:缺少endmodule,Line "+lexer.getLine());
            }
        }
        removeEmptyPort();
    }
    void parsePort(const std::string type){
        //需要分号h
        while((token=lexer.getNextToken()).first!=";"){
            if(token.second == USER_DEF){
                if(type== "wire"){
                    bool repeat_def_wire=false;
                    for (auto& port : moduleNode->ports) {
                        if (port->name == token.first && port->type==WIRE) {
                            std::cout<<"Warning:"<<"定义的wire类型中存在重复名称,Line "+lexer.getLine()<<std::endl;
                            repeat_def_wire=true;
                        }
                        else if (port->name == token.first && port->type==POWER) {
                            throw std::runtime_error("Error:VCC,GND,CLK是保留关键字,Line "+lexer.getLine());

                        }
                        else if (port->name == token.first && (port->type==INPUT || port->type==OUTPUT)){
                            throw std::runtime_error("Error不允许把输入/输出端口重定义为wire,Line "+lexer.getLine());
                        }
                    }
                    // 跳过重复定义
                    if(!repeat_def_wire){
                        auto wireNode=std::make_shared<PortNode>();
                        wireNode->name = token.first;
                        wireNode->type = WIRE; 
                        moduleNode->ports.push_back(wireNode);
                    }
                }
                else {
                    bool finded_port = false;
                    for (auto& port : moduleNode->ports) {
                        if (port->name == token.first) {
                            finded_port = true;
                            if(type == "input" || type == "output"){
                                if(port->type==POWER){
                                    throw std::runtime_error("Error:VCC,GND是保留关键字,Line "+lexer.getLine());
                                }
                                else if(port->type != UNDEF){
                                    throw std::runtime_error("Error:对端口类型的重复定义,Line "+lexer.getLine());
                                }
                                port->type = type == "input" ? INPUT : OUTPUT;
                            }
                            else{
                                throw std::runtime_error("Error端口类型错误,Line "+lexer.getLine());
                            }
                            break; // Exit the loop once the port is found and rewritten
                        }
                    }
                    if(!finded_port){
                        throw std::runtime_error("Error:声明的输入/输出端口未在module上定义,Line "+lexer.getLine());
                    }
                }
            }
            else if(token.first==","){
                continue;
            }
            else if(token.second == KEYWORD){
                throw std::runtime_error("Error:端口名不能为关键字,Line "+lexer.getLine());
            }
            else{
                throw std::runtime_error("Error:端口定义语法错误,Line "+lexer.getLine());
            }
        }
    }
    void parseMos(const std::string type){
        // 新建mos节点
        auto mosNode=std::make_shared<MosNode>();
        mosNode->mostype=(type=="pmos")?PMOS:NMOS;
        mosNode->name = (type=="pmos")?"p"+std::to_string(pcount++):"n"+std::to_string(ncount++);

        expect("(");
        mosNode->drain = lexer.getNextToken().first;
        expect(",");
        mosNode->source = lexer.getNextToken().first;
        expect(",");
        mosNode->gate = lexer.getNextToken().first;
        
        expect(")");
        expect(";");
        int def_port = 0;
        for(auto&p:moduleNode->ports){
            if(p->name==mosNode->drain||p->name==mosNode->source||p->name==mosNode->gate){
                def_port++;
            }
        }
        if(def_port<3){
            throw std::runtime_error("Error:语句中有未定义的端口名,Line "+lexer.getLine());
        }
        
        for(auto&port:moduleNode->ports){
            if(port->name == mosNode->drain){
                port->connections.push_back(Connection(mosNode,Direction::IN,"drain"));
                mosNode->_drain=port;
            }
            if(port->name == mosNode->source){
                port->connections.push_back(Connection(mosNode,Direction::OUT,"source"));
                mosNode->_source=port;
            }
            if(port->name == mosNode->gate){
                port->connections.push_back(Connection(mosNode,Direction::OUT,"gate"));
                mosNode->_gate=port;
            }
        } 
        moduleNode->components.push_back(mosNode);
    }
    void parseNotes(){
        do{
            token=lexer.getNextToken();
            if( token.second==NONE){
                throw std::runtime_error("Error:注释末尾必须加分号,Line "+lexer.getLine());
            }
        }while(token.first!=";" && token.first!=")" && token.first!="endmodule");
    }
    void parseModuleNesting(const std::string subModuleName){
        //必须在modules中已有定义, 否则必须在AtomModules中有声明
        bool found=false;
        std::vector<std::string> paras;//参数集
        Token instanceToken = lexer.getNextToken();
        if(instanceToken.second != USER_DEF){
            throw std::runtime_error("Error: 模块实例化未定义实例名, Line " + lexer.getLine());
        }
        for(auto&m:modules){
            if(m->module_name==subModuleName){
                found=true;
                auto submodule = std::make_shared<SubModuleNode>();
                submodule->module_name=subModuleName;
                submodule->name = instanceToken.first;
                // auto it = std::find_if(moduleNode->subModules.begin(),moduleNode->subModules.end(),[&subModuleName](const std::shared_ptr<ModuleNode>& subM){
                //     return subM->name == subModuleName;
                // });
                //添加子模块定义
                // if(it == moduleNode->subModules.end()){
                moduleNode->components.push_back(submodule);
                //moduleNode->subModuleCount++;
                //收集参数
                expect("(");
                while((token=lexer.getNextToken()).first!=")"){
                    if(token.second==USER_DEF){
                        paras.push_back(token.first);
                    }
                    else if(token.first==","){
                        continue;
                    }
                    else{
                        throw std::runtime_error("Error:实例化语法错误,Line "+lexer.getLine());
                    }
                }
                
                expect(";");
                int putSize=0;
                for(auto&p:m->ports){
                    if(p->type==INPUT || p->type==OUTPUT){
                        putSize++;
                    }
                }
                if(paras.size()!=putSize){
                    throw std::runtime_error("用于实例化的参数数量错误,应到" + std::to_string(putSize) + "人,实到" + std::to_string(paras.size()) + "人,Line "+lexer.getLine());
                }
                int iop_index=0;
                auto iop=m->ports[iop_index];
                while(iop->type!=INPUT&&iop->type!=OUTPUT){
                    iop=m->ports[++iop_index];
                }
                // 加入输入输出端口映射关系
                for(int i=0;i<paras.size();++i){
                    // 在模块端口中找到参数值
                    for(auto&p:moduleNode->ports){
                        if(p->name == paras[i]){        // p作为父模块的网络，对应子模块的参数paras[i](子模块的端口iop)
                            if(iop->type==INPUT){
                                submodule->InNetMap[iop->name]=p->name;
                                p->connections.push_back(Connection(submodule,Direction::OUT,iop->name));
                            }
                            else{
                                submodule->OutNetMap[iop->name]=p->name;
                                p->connections.push_back(Connection(submodule,Direction::IN,iop->name));
                            }
                        }
                    }
                    iop=m->ports[++iop_index];
                }

                // 加入
                found=true;
                break;
            }
            
        }
        if(!found){
            if(find(Atoms.begin(),Atoms.end(),subModuleName)==Atoms.end()){
                throw std::runtime_error("Error:未定义的模组被实例化,Line "+lexer.getLine());
            }
            else{
                throw std::runtime_error("目前没有实现基础模块的完全黑盒定义模式,Line "+lexer.getLine());
            }
        }
    }
    // 只接受期望的Token
    void expect(const std::string& expectedToken) {
        token = lexer.getNextToken();
        if (token.first != expectedToken) {
            throw std::runtime_error("Expected \"" + expectedToken + "\", but got \"" + token.first+"\",Line "+lexer.getLine());
        }
    }
    // 删除没有连接对象的端口：1.VCC和GND未使用 2.用户定义了未使用的端口
    void removeEmptyPort(){
        std::vector<std::shared_ptr<PortNode>> to_remove;
        // 收集需要删除的元素
        for (const auto& port : moduleNode->ports) {
            if (port != nullptr && port->connections.empty()) {
                if(port->type!=POWER){
                    std::cout<<"Warning:定义的端口未使用-"<<port->name<<",Line "+lexer.getLine()<<std::endl;
                }
                to_remove.push_back(port);
            }
        }
        // 删除收集到的元素
        for (const auto& port : to_remove) {
            moduleNode->ports.erase(
                std::remove(moduleNode->ports.begin(), moduleNode->ports.end(), port),
                moduleNode->ports.end()
            );
        }
    }
    // 分析新模组前的重置
    void resetModule(){
        pcount=1;
        ncount=1;
    }
};
// TODO：采用字节缓冲读取方法
// TODO：采用Parser实时读取方法 DONE
// PROBLEM: 采用保证不变的电路仿真是否可行


// int main(int argc, char* argv[]){
//     // 检查是否提供了文件名
//     std::string input_file = argv[1];
//     // std::string input_file="adders.v";
//     std::ifstream file(input_file);
//     std::ofstream output_file(std::string(input_file)+".json");
//     if(!file.is_open()){
//         std::cout << "文件打开失败" << std::endl;
//         exit(1);
//     }
//     Lexer lexer(file);
//     Parser parser(lexer);
//     parser.parse();
//     json ast=parser.toJSON();

//     for (auto& i : parser.getModules()) {
//         std::cout << "module " << i->name << std::endl;
//         // printf("module %s\n", i->name);
//         i->simulate_all();
//         std::cout << "\n";
//     }

//     output_file<<ast.dump(4);

//     output_file.close();
//     file.close();
// }

void options_helper() {
    std::cout << "You can use the following options\n";
    std::cout << "-h (help): 命令行选项实用信息\n";
    std::cout << "-f (file) <addr>: 需解析的文件路径\n";
    std::cout << "-o (output) <addr>: 输出json文件路径(不含扩展名)\n";
    exit(0);
}

int main(int argc, char* argv[]) {
    try {
        // 打开config.json
        std::ifstream configFile("config.json");
        if (!configFile.is_open()) {
            std::cerr << "Error: Could not open config.json" << std::endl;
            return 1;
        }
        json config;
        configFile >> config;
        configFile.close();
        Atoms = config["AtomModules"];
        std::map<std::string, std::string> options;
        if (argc == 1) {
            options["-f"] = "adder4.v";
            options["-o"] = "output";
            // options_helper();
        }
        else for (int i = 1; i < argc; i++) {
            std::string param(argv[i]);
            if (param[0] == '-') {
                if (param == "-h") {
                    options_helper();
                } else if (param == "-f") {
                    if (i != argc - 1) options[param] = argv[++i];
                    else options_helper();
                } else if (param == "-o") {
                    if (i != argc - 1) options[param] = argv[++i];
                    else options_helper();
                }
            } else {
                options_helper();
            }
        }

        for (auto& pair : options) {
            std::cout << pair.first <<  ": " << pair.second << "\n";
        }

        if (options.count("-f") == 0) {
            options_helper();
        }
        // 检查是否提供了文件名
        std::string input_file = options["-f"];
        std::ifstream file(input_file);

        // 读入文件相关
        if(!file.is_open()){
            std::cout << "fail to open " << options["-f"] << std::endl;
            exit(1);
        }
        Lexer lexer(file);
        Parser parser(lexer);
        parser.parse();
        json ast=parser.toJSON();
        file.close(); 
        std::string dump_name = input_file + ".json";
        if (options.count("-o")) {
            dump_name = options["-o"] + ".json";
        }
        std::cout << ast.dump(4) << std::endl;
        std::ofstream output_file(dump_name);
        output_file << ast.dump(4);
        output_file.close();
      
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "标准异常: " << e.what() << std::endl;
        return 1;
    }
    catch (const char* msg) {
        std::cerr << "字符串异常: " << msg << std::endl;
        return 1;
    }
    catch (...) {
        std::cerr << "未知类型异常" << std::endl;
        return 1;
    }
    return 0;
}
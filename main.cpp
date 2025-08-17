#include <iostream>
#include <string>
#include <vector>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_mixer.h>
#include <fstream>
#include <map>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <algorithm>
#include <bits/stdc++.h>
using namespace std;

// Forward declarations
class CircuitElement;
class Node;

const double EPSILON = 1e-12;
const int GRID_STEP = 20;
const SDL_Color BACKGROUND_COLOR = {0, 0, 0, 255};

// Tool enum for schematic editor
enum class Tool {
    Select, Wire, PlaceRes, PlaceCap, PlaceInd, PlaceDio, PlaceVolt, PlaceGnd
};

// Vector class for schematic editor
class Vec2 {
    double x_, y_;
public:
    Vec2(double x=0, double y=0) : x_(x), y_(y) {}
    double x() const { return x_; }
    double y() const { return y_; }
    void setX(double x) { x_ = x; }
    void setY(double y) { y_ = y; }

    Vec2 operator+(const Vec2& b) const { return Vec2(x_ + b.x_, y_ + b.y_); }
    Vec2 operator-(const Vec2& b) const { return Vec2(x_ - b.x_, y_ - b.y_); }
    Vec2 operator*(double s) const { return Vec2(x_ * s, y_ * s); }
    double length() const { return sqrt(x_*x_ + y_*y_); }
    Vec2 normalized() const { double l = length(); return l > 0 ? (*this)*(1.0/l) : *this; }
    Vec2 rotated(double angle) const {
        double rad = angle * M_PI / 180.0;
        return Vec2(x_ * cos(rad) - y_ * sin(rad), x_ * sin(rad) + y_ * cos(rad));
    }
};

// Component types for schematic editor
enum class ComponentType {
    Resistor, Capacitor, Inductor, Diode, VoltageSource, Ground
};

// Error class for circuit simulation
class CircuitError : public runtime_error {
public:
    explicit CircuitError(const string& message) : runtime_error(message) {}
};

// Node class for circuit simulation
class Node {
private:
    string name;
    int id;
    static int nextId;

public:
    explicit Node(const string& nodeName) : name(nodeName), id(nextId++) {}

    string getName() const { return name; }
    int getId() const { return id; }

    void rename(const string& newName) { name = newName; }

    bool operator==(const Node& other) const { return id == other.id; }
    bool operator<(const Node& other) const { return id < other.id; }
};

int Node::nextId = 0;

// Component class for schematic editor
class Component {
    Vec2 pos_;
    int rotation_;
    ComponentType type_;
    string name_;
    string value_;

public:
    Component(ComponentType type, const Vec2& pos = Vec2(), int rotation = 0,
              const string& name = "", const string& value = "")
            : type_(type), pos_(pos), rotation_(rotation), name_(name), value_(value) {}

    Vec2 getPos() const { return pos_; }
    int getRotation() const { return rotation_; }
    ComponentType getType() const { return type_; }
    string getName() const { return name_; }
    string getValue() const { return value_; }

    void setPos(const Vec2& pos) { pos_ = pos; }
    void setRotation(int rot) { rotation_ = rot % 360; }
    void setName(const string& name) { name_ = name; }
    void setValue(const string& value) { value_ = value; }

    vector<Vec2> getConnectionPoints() const {
        vector<Vec2> points;
        double offset = 10 * GRID_STEP;

        switch(type_) {
            case ComponentType::Resistor:
            case ComponentType::Capacitor:
            case ComponentType::Inductor:
            case ComponentType::Diode:
            case ComponentType::VoltageSource:
                points.push_back(Vec2(-offset, 0));
                points.push_back(Vec2(offset, 0));
                break;
            case ComponentType::Ground:
                points.push_back(Vec2(0, 0));
                points.push_back(Vec2(0, offset));
                break;
        }

        for(auto& p : points) {
            p = p.rotated(rotation_) + pos_;
        }

        return points;
    }
};

// Wire class for schematic editor
class Wire {
    vector<Vec2> pts_;
public:
    Wire() {}
    void addPoint(const Vec2& p) { pts_.push_back(p); }
    size_t size() const { return pts_.size(); }
    const Vec2& getPoint(size_t i) const { return pts_[i]; }
    void clear() { pts_.clear(); }
};

// Action history for schematic editor
struct Action {
    enum Type { AddComponent, DeleteComponent, AddWire, DeleteWire, EditComponent };
    Type type;
    Component* component;
    Wire* wire;
    string oldValue;
    string newValue;
};

class ActionHistory {
    vector<Action> undoStack;
    vector<Action> redoStack;

public:
    void addAction(const Action& action) {
        undoStack.push_back(action);
        redoStack.clear();
    }

    bool canUndo() const { return !undoStack.empty(); }
    bool canRedo() const { return !redoStack.empty(); }

    Action undo() {
        if (!canUndo()) return {};
        Action action = undoStack.back();
        undoStack.pop_back();
        redoStack.push_back(action);
        return action;
    }

    Action redo() {
        if (!canRedo()) return {};
        Action action = redoStack.back();
        redoStack.pop_back();
        undoStack.push_back(action);
        return action;
    }
};

// CircuitElement base class for circuit simulation
class CircuitElement {
protected:
    string name;
    shared_ptr<Node> node1;
    shared_ptr<Node> node2;
    double value;

public:
    CircuitElement(const string& elName, shared_ptr<Node> n1, shared_ptr<Node> n2, double val)
            : name(elName), node1(n1), node2(n2), value(val) {}

    virtual ~CircuitElement() = default;

    string getName() const { return name; }
    shared_ptr<Node> getNode1() const { return node1; }
    shared_ptr<Node> getNode2() const { return node2; }
    double getValue() const { return value; }

    virtual string getType() const = 0;

    virtual void stampMatrix(
            vector<vector<double>>& G,
            vector<vector<double>>& B,
            vector<vector<double>>& C,
            vector<vector<double>>& D,
            vector<double>& J,
            vector<double>& E,
            map<shared_ptr<Node>, int>& nodeIndexMap,
            int& extraVarIndex) = 0;

    virtual void stampTransient(
            vector<vector<double>>& G,
            vector<vector<double>>& B,
            vector<vector<double>>& C,
            vector<vector<double>>& D,
            vector<double>& J,
            vector<double>& E,
            const map<shared_ptr<Node>, int>& nodeIndexMap,
            int& extraVarIndex,
            double dt,
            double t) = 0;

    virtual void update(
            const map<shared_ptr<Node>, double>& solution,
            const map<shared_ptr<Node>, int>& nodeIndexMap,
            double dt) {}
};

// Resistor class for circuit simulation
class Resistor : public CircuitElement {
public:
    Resistor(const string& name, shared_ptr<Node> n1, shared_ptr<Node> n2, double resistance)
            : CircuitElement(name, n1, n2, resistance) {
        if (resistance <= 0) {
            throw CircuitError("Resistance cannot be zero or negative");
        }
    }

    string getType() const override { return "Resistor"; }

    void stampMatrix(vector<vector<double>>& G,
                     vector<vector<double>>& B,
                     vector<vector<double>>& C,
                     vector<vector<double>>& D,
                     vector<double>& J,
                     vector<double>& E,
                     map<shared_ptr<Node>, int>& nodeIndexMap,
                     int& extraVarIndex) override {

        double conductance = 1.0 / value;
        int n1 = nodeIndexMap[node1];
        int n2 = nodeIndexMap[node2];

        G[n1][n1] += conductance;
        G[n1][n2] -= conductance;
        G[n2][n1] -= conductance;
        G[n2][n2] += conductance;
    }

    void stampTransient(vector<vector<double>>& G,
                        vector<vector<double>>& B,
                        vector<vector<double>>& C,
                        vector<vector<double>>& D,
                        vector<double>& J,
                        vector<double>& E,
                        const map<shared_ptr<Node>, int>& nodeIndexMap,
                        int& extraVarIndex,
                        double dt,
                        double t) override {
        stampMatrix(G, B, C, D, J, E,
                    const_cast<map<shared_ptr<Node>, int>&>(nodeIndexMap),
                    extraVarIndex);
    }
};

// Capacitor class for circuit simulation
class Capacitor : public CircuitElement {
private:
    double current = 0.0;

public:
    double prevVoltage = 0.0;

    Capacitor(const string& name, shared_ptr<Node> n1, shared_ptr<Node> n2, double capacitance)
            : CircuitElement(name, n1, n2, capacitance) {
        if (capacitance <= 0) {
            throw CircuitError("Capacitance cannot be zero or negative");
        }
    }

    string getType() const override { return "Capacitor"; }

    double getCurrent() const { return current; }
    double getVoltage() const { return prevVoltage; }

    void stampMatrix(vector<vector<double>>& G,
                     vector<vector<double>>& B,
                     vector<vector<double>>& C,
                     vector<vector<double>>& D,
                     vector<double>& J,
                     vector<double>& E,
                     map<shared_ptr<Node>, int>& nodeIndexMap,
                     int& extraVarIndex) override {
    }

    void stampTransient(vector<vector<double>>& G,
                        vector<vector<double>>& B,
                        vector<vector<double>>& C,
                        vector<vector<double>>& D,
                        vector<double>& J,
                        vector<double>& E,
                        const map<shared_ptr<Node>, int>& nodeIndexMap,
                        int& extraVarIndex,
                        double dt,
                        double /*t*/) override {
        if (dt <= 0) return;

        int n1 = nodeIndexMap.count(node1) ? nodeIndexMap.at(node1) : -1;
        int n2 = nodeIndexMap.count(node2) ? nodeIndexMap.at(node2) : -1;

        double geq = value / dt;
        double ieq = geq * prevVoltage;

        if (n1 >= 0) {
            G[n1][n1] += geq;
            if (n2 >= 0) {
                G[n1][n2] -= geq;
                G[n2][n1] -= geq;
                G[n2][n2] += geq;
            }
            J[n1] += ieq;
            if (n2 >= 0) J[n2] -= ieq;
        }
        else if (n2 >= 0) {
            G[n2][n2] += geq;
            J[n2] -= ieq;
        }
    }

    void update(const map<shared_ptr<Node>, double>& solution,
                const map<shared_ptr<Node>, int>& /*nodeIndexMap*/,
                double dt) override {
        double v1 = solution.count(node1) ? solution.at(node1) : 0.0;
        double v2 = solution.count(node2) ? solution.at(node2) : 0.0;

        double newVoltage = v1 - v2;

        current = value * (newVoltage - prevVoltage) / dt;
        prevVoltage = newVoltage;
    }
};

// Inductor class for circuit simulation
class Inductor : public CircuitElement {
public:
    double voltage = 0.0;
    double prevCurrent = 0.0;

    Inductor(const string& name, shared_ptr<Node> n1, shared_ptr<Node> n2, double inductance)
            : CircuitElement(name, n1, n2, inductance) {
        if (inductance <= 0) {
            throw CircuitError("Inductance cannot be zero or negative");
        }
    }

    string getType() const override { return "Inductor"; }
    double getCurrent() const { return prevCurrent; }
    double getVoltage() const { return voltage; }

    void stampMatrix(vector<vector<double>>& G,
                     vector<vector<double>>& B,
                     vector<vector<double>>& C,
                     vector<vector<double>>& D,
                     vector<double>& J,
                     vector<double>& E,
                     map<shared_ptr<Node>, int>& nodeIndexMap,
                     int& extraVarIndex) override {
        int n1 = nodeIndexMap.count(node1) ? nodeIndexMap.at(node1) : -1;
        int n2 = nodeIndexMap.count(node2) ? nodeIndexMap.at(node2) : -1;
        int l = extraVarIndex++;

        if (n1 >= 0) { B[n1][l] += 1; C[l][n1] += 1; }
        if (n2 >= 0) { B[n2][l] -= 1; C[l][n2] -= 1; }

        D[l][l] = 0.0;
        E[l] = 0.0;
    }

    void stampTransient(vector<vector<double>>& G,
                        vector<vector<double>>& B,
                        vector<vector<double>>& C,
                        vector<vector<double>>& D,
                        vector<double>& J,
                        vector<double>& E,
                        const map<shared_ptr<Node>, int>& nodeIndexMap,
                        int& extraVarIndex,
                        double dt,
                        double ) override {
        int n1 = nodeIndexMap.count(node1) ? nodeIndexMap.at(node1) : -1;
        int n2 = nodeIndexMap.count(node2) ? nodeIndexMap.at(node2) : -1;
        int k  = extraVarIndex++;

        if (n1 >= 0) { B[n1][k] += 1; C[k][n1] += 1; }
        if (n2 >= 0) { B[n2][k] -= 1; C[k][n2] -= 1; }

        double Req = value / dt;
        D[k][k] += Req;
        E[k] += Req * prevCurrent;
    }

    void update(const map<shared_ptr<Node>, double>& solution,
                const map<shared_ptr<Node>, int>& nodeIndexMap,
                double ) override {
        double v1 = solution.count(node1) ? solution.at(node1) : 0.0;
        double v2 = solution.count(node2) ? solution.at(node2) : 0.0;
        voltage = v1 - v2;

        int currentVarIndex = static_cast<int>(nodeIndexMap.size());
        if (solution.count(nullptr) == 0 && currentVarIndex < static_cast<int>(solution.size())) {
            auto it = solution.begin();
            std::advance(it, currentVarIndex);
            prevCurrent = it->second;
        }
    }
};

// VoltageSource class for circuit simulation
class VoltageSource : public CircuitElement {
private:
    bool isDC = true;
    double amplitude = 0.0;
    double frequency = 0.0;
    double offset = 0.0;

public:
    VoltageSource(const string& name, shared_ptr<Node> n1, shared_ptr<Node> n2, double voltage)
            : CircuitElement(name, n1, n2, voltage) {}

    VoltageSource(const string& name, shared_ptr<Node> n1, shared_ptr<Node> n2,
                  double offset, double amplitude, double frequency)
            : CircuitElement(name, n1, n2, amplitude), isDC(false),
              amplitude(amplitude), frequency(frequency), offset(offset) {}

    string getType() const override { return isDC ? "DC Voltage Source" : "AC Voltage Source"; }

    double getVoltage(double time = 0.0) const {
        if (isDC) {
            return value;
        } else {
            return offset + amplitude * sin(2 * M_PI * frequency * time);
        }
    }

    void stampMatrix(vector<vector<double>>& G,
                     vector<vector<double>>& B,
                     vector<vector<double>>& C,
                     vector<vector<double>>& D,
                     vector<double>& J,
                     vector<double>& E,
                     map<shared_ptr<Node>, int>& nodeIndexMap,
                     int& extraVarIndex) override {

        int n1 = nodeIndexMap[node1];
        int n2 = nodeIndexMap[node2];
        int v = extraVarIndex++;

        B[n1][v] += 1;
        B[n2][v] -= 1;

        C[v][n1] += 1;
        C[v][n2] -= 1;

        E[v] += getVoltage();
    }

    void stampTransient(vector<vector<double>>& G,
                        vector<vector<double>>& B,
                        vector<vector<double>>& C,
                        vector<vector<double>>& D,
                        vector<double>& J,
                        vector<double>& E,
                        const map<shared_ptr<Node>, int>& nodeIndexMap,
                        int& extraVarIndex,
                        double dt,
                        double t) override {

        int n1 = nodeIndexMap.at(node1);
        int n2 = nodeIndexMap.at(node2);
        int v = extraVarIndex++;

        B[n1][v] += 1;
        B[n2][v] -= 1;

        C[v][n1] += 1;
        C[v][n2] -= 1;

        E[v] += getVoltage(t);
    }
};

// CurrentSource class for circuit simulation
class CurrentSource : public CircuitElement {
private:
    bool isDC = true;
    double amplitude = 0.0;
    double frequency = 0.0;
    double offset = 0.0;

public:
    CurrentSource(const string& name, shared_ptr<Node> n1, shared_ptr<Node> n2, double current)
            : CircuitElement(name, n1, n2, current) {}

    CurrentSource(const string& name, shared_ptr<Node> n1, shared_ptr<Node> n2,
                  double offset, double amplitude, double frequency)
            : CircuitElement(name, n1, n2, amplitude), isDC(false),
              amplitude(amplitude), frequency(frequency), offset(offset) {}

    string getType() const override { return isDC ? "DC Current Source" : "AC Current Source"; }

    double getCurrent(double time = 0.0) const {
        if (isDC) {
            return value;
        } else {
            return offset + amplitude * sin(2 * M_PI * frequency * time);
        }
    }

    void stampMatrix(vector<vector<double>>& G,
                     vector<vector<double>>& B,
                     vector<vector<double>>& C,
                     vector<vector<double>>& D,
                     vector<double>& J,
                     vector<double>& E,
                     map<shared_ptr<Node>, int>& nodeIndexMap,
                     int& extraVarIndex) override {

        int n1 = nodeIndexMap[node1];
        int n2 = nodeIndexMap[node2];

        J[n1] -= getCurrent();
        J[n2] += getCurrent();
    }

    void stampTransient(vector<vector<double>>& G,
                        vector<vector<double>>& B,
                        vector<vector<double>>& C,
                        vector<vector<double>>& D,
                        vector<double>& J,
                        vector<double>& E,
                        const map<shared_ptr<Node>, int>& nodeIndexMap,
                        int& extraVarIndex,
                        double dt,
                        double t) override {

        int n1 = nodeIndexMap.at(node1);
        int n2 = nodeIndexMap.at(node2);

        J[n1] -= getCurrent(t);
        J[n2] += getCurrent(t);
    }
};

// Diode class for circuit simulation
class Diode : public CircuitElement {
private:
    string model;
    double Is;
    double Vt;
    double n;
    double Vz;
    double Vprev;

public:
    Diode(const string& name, shared_ptr<Node> n1, shared_ptr<Node> n2,
          const string& model = "D", double Vz = 5.1)
            : CircuitElement(name, n1, n2, 0),
              model(model), Is(1e-14), Vt(0.0258), n(1.0), Vz(Vz), Vprev(0.7)
    {
        if (model != "D" && model != "Z") {
            throw CircuitError("Diode model must be either 'D' (standard) or 'Z' (Zener)");
        }
        if (model == "Z" && Vz <= 0) {
            throw CircuitError("Zener breakdown voltage must be positive");
        }
    }

    string getType() const override {
        return (model == "Z") ? "Zener Diode" : "Diode";
    }

    double calculateCurrent(double Vd) const {
        if (model == "D") {
            return Is * (exp(Vd / (n * Vt)) - 1);
        } else {
            if (Vd < -Vz) {
                return -Is * (exp(-(Vd + Vz) / (n * Vt)) - 1);
            } else if (Vd > 0) {
                return Is * (exp(Vd / (n * Vt)) - 1);
            } else {
                return -Is;
            }
        }
    }

    double calculateConductance(double Vd) const {
        if (model == "D") {
            return (Is / (n * Vt)) * exp(Vd / (n * Vt));
        } else {
            if (Vd < -Vz) {
                return (Is / (n * Vt)) * exp(-(Vd + Vz) / (n * Vt));
            } else if (Vd > 0) {
                return (Is / (n * Vt)) * exp(Vd / (n * Vt));
            } else {
                return 1e-12;
            }
        }
    }

    void stampMatrix(vector<vector<double>>& G,
                     vector<vector<double>>& B,
                     vector<vector<double>>& C,
                     vector<vector<double>>& D,
                     vector<double>& J,
                     vector<double>& E,
                     map<shared_ptr<Node>, int>& nodeIndexMap,
                     int& extraVarIndex) override {

        int n1 = nodeIndexMap[node1];
        int n2 = nodeIndexMap[node2];

        double g = calculateConductance(Vprev);
        double I = calculateCurrent(Vprev);
        double Ieq = I - g * Vprev;

        G[n1][n1] += g;
        G[n1][n2] -= g;
        G[n2][n1] -= g;
        G[n2][n2] += g;

        J[n1] -= Ieq;
        J[n2] += Ieq;
    }

    void stampTransient(vector<vector<double>>& G,
                        vector<vector<double>>& B,
                        vector<vector<double>>& C,
                        vector<vector<double>>& D,
                        vector<double>& J,
                        vector<double>& E,
                        const map<shared_ptr<Node>, int>& nodeIndexMap,
                        int& extraVarIndex,
                        double dt,
                        double t) override {

        int n1 = nodeIndexMap.at(node1);
        int n2 = nodeIndexMap.at(node2);

        double g = calculateConductance(Vprev);
        double I = calculateCurrent(Vprev);
        double Ieq = I - g * Vprev;

        G[n1][n1] += g;
        G[n1][n2] -= g;
        G[n2][n1] -= g;
        G[n2][n2] += g;

        J[n1] -= Ieq;
        J[n2] += Ieq;
    }

    void update(const map<shared_ptr<Node>, double>& solution,
                const map<shared_ptr<Node>, int>& nodeIndexMap,
                double ) override {
        double v1 = (nodeIndexMap.count(node1) ? solution.at(nodeIndexMap.find(node1)->first) : 0.0);
        double v2 = (nodeIndexMap.count(node2) ? solution.at(nodeIndexMap.find(node2)->first) : 0.0);
        Vprev = v1 - v2;
    }
};

// Ground class for circuit simulation
class Ground : public Node {
public:
    Ground() : Node("GND") {}
};

// Circuit class for circuit simulation
class Circuit {
private:
    map<string, shared_ptr<Node>> nodes;
    shared_ptr<Ground> ground;

    void ensureGroundExists() {
        if (!ground) {
            ground = make_shared<Ground>();
            nodes["GND"] = ground;
        }
    }

    void checkForDuplicateElement(const string& name) {
        for (const auto& element : elements) {
            if (element->getName() == name) {
                throw CircuitError("Element " + name + " already exists in the circuit");
            }
        }
    }
    shared_ptr<Node> getOrCreateNode(const string& name) {
        if (nodes.find(name) == nodes.end()) {
            nodes[name] = make_shared<Node>(name);
        }
        return nodes[name];
    }

public:
    Circuit() {
        ensureGroundExists();
    }

    vector<shared_ptr<CircuitElement>> elements;

    shared_ptr<Node> addNode(const string& name) {
        if (nodes.find(name) != nodes.end()) {
            throw CircuitError("Node " + name + " already exists");
        }
        auto newNode = make_shared<Node>(name);
        nodes[name] = newNode;
        return newNode;
    }

    void renameNode(const string& oldName, const string& newName) {
        if (nodes.find(oldName) == nodes.end()) {
            throw CircuitError("Node " + oldName + " does not exist");
        }
        if (nodes.find(newName) != nodes.end() && newName != oldName) {
            throw CircuitError("Node " + newName + " already exists");
        }

        auto node = nodes[oldName];
        nodes.erase(oldName);
        node->rename(newName);
        nodes[newName] = node;
    }

    shared_ptr<Node> getNode(const string& name) {
        if (nodes.find(name) == nodes.end()) {
            throw CircuitError("Node " + name + " not found");
        }
        return nodes[name];
    }

    vector<string> listNodes() const {
        vector<string> nodeNames;
        for (const auto& pair : nodes) {
            nodeNames.push_back(pair.first);
        }
        return nodeNames;
    }

    vector<string> listElements(const string& type = "") const {
        vector<string> elementInfo;
        for (const auto& element : elements) {
            if (type.empty() || element->getType().find(type) != string::npos) {
                elementInfo.push_back(element->getName() + ": " + element->getType());
            }
        }
        return elementInfo;
    }

    void addResistor(const string& name, const string& node1Name, const string& node2Name, double resistance) {
        checkForDuplicateElement(name);
        auto n1 = getOrCreateNode(node1Name);
        auto n2 = getOrCreateNode(node2Name);
        elements.push_back(make_shared<Resistor>(name, n1, n2, resistance));
    }

    void addCapacitor(const string& name, const string& node1Name, const string& node2Name, double capacitance) {
        checkForDuplicateElement(name);
        auto n1 = getOrCreateNode(node1Name);
        auto n2 = getOrCreateNode(node2Name);
        elements.push_back(make_shared<Capacitor>(name, n1, n2, capacitance));
    }

    void addInductor(const string& name, const string& node1Name, const string& node2Name, double inductance) {
        checkForDuplicateElement(name);
        auto n1 = getOrCreateNode(node1Name);
        auto n2 = getOrCreateNode(node2Name);
        elements.push_back(make_shared<Inductor>(name, n1, n2, inductance));
    }

    void addVoltageSource(const string& name, const string& node1Name, const string& node2Name, double voltage) {
        checkForDuplicateElement(name);
        auto n1 = getOrCreateNode(node1Name);
        auto n2 = getOrCreateNode(node2Name);
        elements.push_back(make_shared<VoltageSource>(name, n1, n2, voltage));
    }
    void addGround(const string& nodeName) {
        ensureGroundExists();

        if (nodeName == "GND") {
            throw CircuitError("Cannot name a node 'GND' as it's reserved for ground");
        }

        auto node = getOrCreateNode(nodeName);

        string vsourceName = "VGND_" + nodeName;
        addVoltageSource(vsourceName, nodeName, "GND", 0.0);
    }

    void addDiode(const string& name, const string& node1Name,
                  const string& node2Name, const string& model = "D",
                  double Vz = 5.1) {
        checkForDuplicateElement(name);
        auto n1 = getOrCreateNode(node1Name);
        auto n2 = getOrCreateNode(node2Name);

        if (model == "Z") {
            elements.push_back(make_shared<Diode>(name, n1, n2, model, Vz));
        } else {
            elements.push_back(make_shared<Diode>(name, n1, n2, model));
        }
    }
    void addCurrentSource(const string& name, const string& node1Name, const string& node2Name, double current) {
        checkForDuplicateElement(name);
        auto n1 = getOrCreateNode(node1Name);
        auto n2 = getOrCreateNode(node2Name);
        elements.push_back(make_shared<CurrentSource>(name, n1, n2, current));
    }

    void removeElement(const string& name) {
        auto it = find_if(elements.begin(), elements.end(),
                          [&name](const shared_ptr<CircuitElement>& el) { return el->getName() == name; });

        if (it == elements.end()) {
            throw CircuitError( "Error: Cannot delete resistor; component not found");
        }

        elements.erase(it);
    }
    void removeGround(const string& nodeName) {
        if (!ground) {
            throw CircuitError("No ground connection exists in the circuit");
        }

        string vsourceName = "VGND_" + nodeName;

        auto it = find_if(elements.begin(), elements.end(),
                          [&vsourceName](const shared_ptr<CircuitElement>& el) {
                              return el->getName() == vsourceName &&
                                     (el->getType() == "DC Voltage Source") &&
                                     el->getValue() == 0.0;
                          });

        if (it == elements.end()) {
            throw CircuitError("Node " + nodeName + " is not grounded");
        }

        elements.erase(it);
    }

    void removeResistor(const string& name) {
        auto it = find_if(elements.begin(), elements.end(),
                          [&name](const shared_ptr<CircuitElement>& el) {
                              return el->getName() == name && el->getType() == "Resistor";
                          });

        if (it == elements.end()) {
            throw CircuitError("Resistor " + name + " not found");
        }

        elements.erase(it);
    }

    void removeCapacitor(const string& name) {
        auto it = find_if(elements.begin(), elements.end(),
                          [&name](const shared_ptr<CircuitElement>& el) {
                              return el->getName() == name && el->getType() == "Capacitor";
                          });

        if (it == elements.end()) {
            throw CircuitError("Capacitor " + name + " not found");
        }

        elements.erase(it);
    }

    void removeInductor(const string& name) {
        auto it = find_if(elements.begin(), elements.end(),
                          [&name](const shared_ptr<CircuitElement>& el) {
                              return el->getName() == name && el->getType() == "Inductor";
                          });

        if (it == elements.end()) {
            throw CircuitError("Inductor " + name + " not found");
        }

        elements.erase(it);
    }

    void removeVoltageSource(const string& name) {
        auto it = find_if(elements.begin(), elements.end(),
                          [&name](const shared_ptr<CircuitElement>& el) {
                              return el->getName() == name &&
                                     (el->getType() == "DC Voltage Source" ||
                                      el->getType() == "AC Voltage Source");
                          });

        if (it == elements.end()) {
            throw CircuitError("Voltage source " + name + " not found");
        }

        elements.erase(it);
    }

    void removeCurrentSource(const string& name) {
        auto it = find_if(elements.begin(), elements.end(),
                          [&name](const shared_ptr<CircuitElement>& el) {
                              return el->getName() == name &&
                                     (el->getType() == "DC Current Source" ||
                                      el->getType() == "AC Current Source");
                          });

        if (it == elements.end()) {
            throw CircuitError("Current source " + name + " not found");
        }

        elements.erase(it);
    }

    void removeDiode(const string& name) {
        auto it = find_if(elements.begin(), elements.end(),
                          [&name](const shared_ptr<CircuitElement>& el) {
                              return el->getName() == name && el->getType() == "Diode";
                          });

        if (it == elements.end()) {
            throw CircuitError("Diode " + name + " not found");
        }

        elements.erase(it);
    }
    static void solveMatrix(vector<vector<double>>& A, vector<double>& b) {
        const int n = A.size();

        for (int i = 0; i < n; ++i) {
            double maxEl = abs(A[i][i]);
            int maxRow = i;
            for (int k = i + 1; k < n; ++k) {
                if (abs(A[k][i]) > maxEl) {
                    maxEl = abs(A[k][i]);
                    maxRow = k;
                }
            }

            for (int k = i; k < n; ++k) {
                swap(A[maxRow][k], A[i][k]);
            }
            swap(b[maxRow], b[i]);

            for (int k = i + 1; k < n; ++k) {

                if (fabs(A[i][i]) < EPSILON) {
                    throw CircuitError("Singular matrix in solver (division by zero)");
                }

                double c = -A[k][i] / A[i][i];
                for (int j = i; j < n; ++j) {
                    if (i == j) {
                        A[k][j] = 0;
                    } else {
                        A[k][j] += c * A[i][j];
                    }
                }
                b[k] += c * b[i];
            }
        }

        for (int i = n - 1; i >= 0; --i) {
            if (fabs(A[i][i]) < EPSILON) {
                throw CircuitError("Singular matrix in solver (division by zero)");
            }

            b[i] /= A[i][i];
            for (int k = i - 1; k >= 0; --k) {
                b[k] -= A[k][i] * b[i];
            }
        }
    }

    map<shared_ptr<Node>, double> analyzeDC() {
        ensureGroundExists();

        map<shared_ptr<Node>, int> nodeIndexMap;
        int nodeCount = 0;
        for (const auto& pair : nodes) {
            if (pair.first != "GND") {
                nodeIndexMap[pair.second] = nodeCount++;
            }
        }

        int extraVars = 0;
        for (const auto& element : elements) {
            if (element->getType() == "DC Voltage Source" ||
                element->getType() == "AC Voltage Source" ||
                element->getType() == "Inductor") {
                extraVars++;
            }
        }

        int matrixSize = nodeCount + extraVars;
        vector<vector<double>> G(nodeCount, vector<double>(nodeCount, 0.0));
        vector<vector<double>> B(nodeCount, vector<double>(extraVars, 0.0));
        vector<vector<double>> C(extraVars, vector<double>(nodeCount, 0.0));
        vector<vector<double>> D(extraVars, vector<double>(extraVars, 0.0));
        vector<double> J(nodeCount, 0.0);
        vector<double> E(extraVars, 0.0);

        int currentExtraVar = 0;
        for (const auto& element : elements) {
            element->stampMatrix(G, B, C, D, J, E, nodeIndexMap, currentExtraVar);
        }

        vector<vector<double>> A(matrixSize, vector<double>(matrixSize, 0.0));
        vector<double> b(matrixSize, 0.0);

        for (int i = 0; i < nodeCount; ++i) {
            for (int j = 0; j < nodeCount; ++j) {
                A[i][j] = G[i][j];
            }
            for (int j = 0; j < extraVars; ++j) {
                A[i][nodeCount + j] = B[i][j];
            }
            b[i] = J[i];
        }

        for (int i = 0; i < extraVars; ++i) {
            for (int j = 0; j < nodeCount; ++j) {
                A[nodeCount + i][j] = C[i][j];
            }
            for (int j = 0; j < extraVars; ++j) {
                A[nodeCount + i][nodeCount + j] = D[i][j];
            }
            b[nodeCount + i] = E[i];
        }

        solveMatrix(A, b);

        map<shared_ptr<Node>, double> nodeVoltages;
        nodeVoltages[ground] = 0.0;

        for (const auto& pair : nodeIndexMap) {
            nodeVoltages[pair.first] = b[pair.second];
        }

        return nodeVoltages;
    }

    vector<map<shared_ptr<Node>, double>> analyzeTransient(double tStep, double tStop, double tStart) {
        vector<map<shared_ptr<Node>, double>> results;

        auto initialSolution = analyzeDC();
        results.push_back(initialSolution);

        for (auto& element : elements) {
            if (auto cap = dynamic_cast<Capacitor*>(element.get())) {
                double v1 = initialSolution.count(cap->getNode1()) ? initialSolution.at(cap->getNode1()) : 0.0;
                double v2 = initialSolution.count(cap->getNode2()) ? initialSolution.at(cap->getNode2()) : 0.0;
                cap->prevVoltage = v1 - v2;
            }
            else if (auto ind = dynamic_cast<Inductor*>(element.get())) {
                ind->prevCurrent = 0.0;
            }
        }

        vector<double> currSolution;

        for (double t = tStart + tStep; t <= tStop; t += tStep) {

            map<shared_ptr<Node>, int> nodeIndexMap;
            int nodeCount = 0, extraVars = 0;
            for (const auto& pair : nodes) {
                if (pair.first != "GND")
                    nodeIndexMap[pair.second] = nodeCount++;
            }
            for (const auto& element : elements) {
                if (element->getType().find("Voltage Source") != string::npos ||
                    element->getType() == "Inductor")
                    extraVars++;
            }

            vector<double> prevSolution;
            if (t == tStart + tStep) {
                prevSolution.assign(nodeCount + extraVars, 0.0);
            } else {
                prevSolution = currSolution;
            }
            currSolution.assign(nodeCount + extraVars, 0.0);

            const int maxIter = 50;
            const double tol = 1e-6;

            for (int iter = 0; iter < maxIter; iter++) {

                vector<vector<double>> G(nodeCount, vector<double>(nodeCount, 0.0));
                vector<vector<double>> B(nodeCount, vector<double>(extraVars, 0.0));
                vector<vector<double>> C(extraVars, vector<double>(nodeCount, 0.0));
                vector<vector<double>> D(extraVars, vector<double>(extraVars, 0.0));
                vector<double> J(nodeCount, 0.0);
                vector<double> E(extraVars, 0.0);

                int currentExtraVar = 0;

                for (auto& element : elements) {
                    if (auto vs = dynamic_cast<VoltageSource*>(element.get())) {
                        vs->stampTransient(G, B, C, D, J, E, nodeIndexMap, currentExtraVar, tStep, t);
                        E[currentExtraVar - 1] = vs->getVoltage(t);
                    }
                    else if (auto cs = dynamic_cast<CurrentSource*>(element.get())) {
                        cs->stampTransient(G, B, C, D, J, E, nodeIndexMap, currentExtraVar, tStep, t);
                        double iCurr = cs->getCurrent(t);
                        int n1 = nodeIndexMap.count(cs->getNode1()) ? nodeIndexMap.at(cs->getNode1()) : -1;
                        int n2 = nodeIndexMap.count(cs->getNode2()) ? nodeIndexMap.at(cs->getNode2()) : -1;
                        if (n1 >= 0) J[n1] -= iCurr;
                        if (n2 >= 0) J[n2] += iCurr;
                    }
                    else {
                        element->stampTransient(G, B, C, D, J, E, nodeIndexMap, currentExtraVar, tStep, t);
                    }
                }

                int matrixSize = nodeCount + extraVars;
                vector<vector<double>> A(matrixSize, vector<double>(matrixSize, 0.0));
                vector<double> b(matrixSize, 0.0);

                for (int i = 0; i < nodeCount; i++) {
                    for (int j = 0; j < nodeCount; j++) A[i][j] = G[i][j];
                    for (int j = 0; j < extraVars; j++) A[i][nodeCount + j] = B[i][j];
                    b[i] = J[i];
                }
                for (int i = 0; i < extraVars; i++) {
                    for (int j = 0; j < nodeCount; j++) A[nodeCount + i][j] = C[i][j];
                    for (int j = 0; j < extraVars; j++) A[nodeCount + i][nodeCount + j] = D[i][j];
                    b[nodeCount + i] = E[i];
                }

                solveMatrix(A, b);
                currSolution = b;

                double maxDiff = 0.0;
                for (int i = 0; i < matrixSize; i++)
                    maxDiff = max(maxDiff, fabs(currSolution[i] - prevSolution[i]));

                if (maxDiff < tol) break;

                prevSolution = currSolution;

                map<shared_ptr<Node>, double> tmpSolution;
                tmpSolution[ground] = 0.0;
                for (const auto& pair : nodeIndexMap)
                    tmpSolution[pair.first] = currSolution[pair.second];

                for (auto& element : elements) {
                    if (element->getType().find("Diode") != string::npos) {
                        element->update(tmpSolution, nodeIndexMap, tStep);
                    }
                }
            }

            map<shared_ptr<Node>, double> finalSolution;
            finalSolution[ground] = 0.0;
            for (const auto& pair : nodeIndexMap)
                finalSolution[pair.first] = currSolution[pair.second];
            results.push_back(finalSolution);

            for (auto& element : elements) {
                if (element->getType() == "Capacitor" || element->getType() == "Inductor") {
                    element->update(finalSolution, nodeIndexMap, tStep);
                }
            }
        }
        return results;
    }
};

// Helper functions for schematic editor
vector<string> splitCommand(const string& command) {
    vector<string> tokens;
    string token;
    for (char ch : command) {
        if (isspace(ch)) {
            if (!token.empty()) {
                tokens.push_back(token);
                token.clear();
            }
        } else {
            token += ch;
        }
    }
    if (!token.empty()) {
        tokens.push_back(token);
    }
    return tokens;
}

// Splash screen function
void showSplashScreen() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        cerr << "SDL Init Error: " << SDL_GetError() << std::endl;
        return;
    }
    if (TTF_Init() < 0) {
        cerr << "TTF Init Error: " << TTF_GetError() << std::endl;
        SDL_Quit();
        return;
    }
    if (IMG_Init(IMG_INIT_PNG) == 0) {
        cerr << "IMG Init Error: " << IMG_GetError() << std::endl;
        TTF_Quit();
        SDL_Quit();
        return;
    }

    SDL_Window* splashWindow = SDL_CreateWindow(
            "Welcome",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            600, 400,
            SDL_WINDOW_SHOWN
    );
    if (!splashWindow) {
        cerr << "CreateWindow Error: " << SDL_GetError() << std::endl;
        IMG_Quit();
        TTF_Quit();
        SDL_Quit();
        return;
    }

    SDL_Renderer* splashRenderer = SDL_CreateRenderer(splashWindow, -1, SDL_RENDERER_ACCELERATED);
    if (!splashRenderer) {
        cerr << "CreateRenderer Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(splashWindow);
        IMG_Quit();
        TTF_Quit();
        SDL_Quit();
        return;
    }

    SDL_Surface* splashSurface = IMG_Load("splash.png");
    SDL_Texture* splashTexture = nullptr;
    if (splashSurface) {
        splashTexture = SDL_CreateTextureFromSurface(splashRenderer, splashSurface);
        SDL_FreeSurface(splashSurface);
    }
    else {
        cerr << "Failed to load splash image: " << IMG_GetError() << std::endl;
    }

    TTF_Font* splashFont = TTF_OpenFont("ITCBLKAD.ttf", 36);
    if (!splashFont) {
        cerr << "Font Load Error: " << TTF_GetError() << std::endl;
    }

    SDL_Color white = {250, 250, 250, 255};
    SDL_Surface* textSurface = nullptr;
    SDL_Texture* textTexture = nullptr;
    int textW = 0, textH = 0;
    if (splashFont) {
        textSurface = TTF_RenderText_Blended(splashFont, "HS spice", white);
        if (textSurface) {
            textTexture = SDL_CreateTextureFromSurface(splashRenderer, textSurface);
            textW = textSurface->w;
            textH = textSurface->h;
            SDL_FreeSurface(textSurface);
        }
    }

    SDL_SetRenderDrawColor(splashRenderer, 0, 0, 0, 255);
    SDL_RenderClear(splashRenderer);

    if (splashTexture) {
        SDL_Rect dstRect = {0, 0, 600, 400};
        SDL_RenderCopy(splashRenderer, splashTexture, nullptr, &dstRect);
    }

    if (textTexture) {
        SDL_Rect textRect = {(600 - textW) / 2, 0, textW, textH};
        SDL_RenderCopy(splashRenderer, textTexture, nullptr, &textRect);
    }

    SDL_RenderPresent(splashRenderer);

    SDL_Delay(3000);

    if (textTexture) SDL_DestroyTexture(textTexture);
    if (splashTexture) SDL_DestroyTexture(splashTexture);
    if (splashFont) TTF_CloseFont(splashFont);
    SDL_DestroyRenderer(splashRenderer);
    SDL_DestroyWindow(splashWindow);

    IMG_Quit();
    TTF_Quit();
    SDL_Quit();
}

// Frame class for SDL rendering
class Frame {
private:
    int number;
    vector<SDL_Event> frameevent;
    SDL_Renderer* renderer;
public:
    const std::vector<SDL_Event>& getEvents() const { return frameevent; }

    Frame(SDL_Renderer* renderer) : number(0), renderer(renderer) {}

    void renderframe() {
        SDL_RenderPresent(renderer);
    }

    void destroyFrame() {
        if (renderer != nullptr) {
            SDL_DestroyRenderer(renderer);
            renderer = nullptr;
        }
        frameevent.clear();
    }

    void clearEvent() {
        frameevent.clear();
    }

    bool nextEvent() {
        clearEvent();
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            frameevent.push_back(e);
        }
        return true;
    }

    bool containsEvent(SDL_EventType type) {
        for (const auto& e : frameevent) {
            if (e.type == type) {
                return true;
            }
        }
        return false;
    }

    void setNumber(int n) {
        number = n;
    }

    int getNumber() const {
        return number;
    }

    ~Frame() {
        destroyFrame();
    }
};

// Button base class
class Button {
private:
    string name;
    SDL_Rect rect;
    SDL_Texture* textTexture;
    SDL_Renderer* renderer;

protected:
    SDL_Rect getRect() const { return rect; }
    const string& getButtonName() const { return name; }
    SDL_Renderer* getRenderer() const { return renderer; }

public:
    Button(SDL_Renderer* ren, string name, int x, int y, int w, int h, TTF_Font* font)
            : renderer(ren), name(name), textTexture(nullptr) {
        rect = { x, y, w, h };
        SDL_Color white = { 255, 255, 255, 255 };
        SDL_Surface* surface = TTF_RenderText_Blended(font, name.c_str(), white);
        if (surface) {
            textTexture = SDL_CreateTextureFromSurface(renderer, surface);
            SDL_FreeSurface(surface);
        }
    }

    virtual ~Button() {
        if (textTexture) {
            SDL_DestroyTexture(textTexture);
        }
    }

    virtual void renderButton() {
        SDL_SetRenderDrawColor(renderer, 100, 100, 200, 255);
        SDL_RenderFillRect(renderer, &rect);

        if (textTexture) {
            int tw, th;
            SDL_QueryTexture(textTexture, nullptr, nullptr, &tw, &th);
            SDL_Rect textRect = {
                    rect.x + (rect.w - tw) / 2,
                    rect.y + (rect.h - th) / 2,
                    tw,
                    th
            };
            SDL_RenderCopy(renderer, textTexture, nullptr, &textRect);
        }
    }

    bool mouseIsHovering(int mouseX, int mouseY) {
        return (mouseX >= rect.x && mouseX <= rect.x + rect.w &&
                mouseY >= rect.y && mouseY <= rect.y + rect.h);
    }

    bool isClick(const SDL_Event& e) {
        if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
            return mouseIsHovering(e.button.x, e.button.y);
        }
        return false;
    }

    bool mouseIsHolding(const SDL_Event& e) {
        if (e.type == SDL_MOUSEMOTION && (e.motion.state & SDL_BUTTON_LMASK)) {
            return mouseIsHovering(e.motion.x, e.motion.y);
        }
        return false;
    }

    string getName() {
        return name;
    }

    virtual void doAction() = 0;
};

// Signal plot class for simulation results
class SignalPlot {
private:
    string name;
    vector<double> times;
    vector<double> voltages;
    SDL_Color color;

public:
    SignalPlot(const std::string& n,
               const std::vector<double>& t,
               const std::vector<double>& v,
               SDL_Color c = {0, 0, 255, 255})
            : name(n), times(t), voltages(v), color(c)
    {
        if (t.size() != v.size()) {
            throw runtime_error("Times and voltages vector sizes do not match");
        }
    }

    void render(SDL_Renderer* renderer, int width, int height) const {
        if (times.size() < 2) return;

        double minT = *std::min_element(times.begin(), times.end());
        double maxT = *std::max_element(times.begin(), times.end());
        double minV = *std::min_element(voltages.begin(), voltages.end());
        double maxV = *std::max_element(voltages.begin(), voltages.end());

        double rangeT = (maxT - minT == 0) ? 1 : (maxT - minT);
        double rangeV = (maxV - minV == 0) ? 1 : (maxV - minV);

        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

        for (size_t i = 1; i < times.size(); i++) {
            int x1 = static_cast<int>((times[i - 1] - minT) / rangeT * width);
            int y1 = height - static_cast<int>((voltages[i - 1] - minV) / rangeV * height);

            int x2 = static_cast<int>((times[i] - minT) / rangeT * width);
            int y2 = height - static_cast<int>((voltages[i] - minV) / rangeV * height);

            SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
        }
    }

    void saveToCSV(const std::string& filename) const {
        std::ofstream out(filename);
        if (!out.is_open()) {
            throw std::runtime_error("Cannot open file for writing: " + filename);
        }
        out << "Time,Voltage";
        for (size_t i = 0; i < times.size(); i++) {
            out << times[i] << "," << voltages[i] << "";
        }
        out.close();
    }

    const std::string& getName() const { return name; }
};

// Input box class for UI
class InputBox {
private:
    SDL_Rect rect;
    string text;
    SDL_Color bgColor, textColor;
    TTF_Font* font;
    bool active = false;
    Uint32 lastBlink = 0;
    bool showCursor = true;

public:
    InputBox(int x, int y, int w, int h, TTF_Font* f)
            : rect{ x, y, w, h }, font(f) {
        bgColor = { 200, 200, 200, 255 };
        textColor = { 0, 0, 0, 255 };
    }

    void setActive(bool state) { active = state; }
    bool isActive() const { return active; }
    SDL_Rect getRect() const { return rect; }
    string getText() const { return text; }

    bool isInside(int x, int y) const {
        return (x > rect.x && x < rect.x + rect.w &&
                y > rect.y && y < rect.y + rect.h);
    }

    bool handleEvent(const SDL_Event& e) {
        if (active && e.type == SDL_TEXTINPUT) {
            text += e.text.text;
        }

        if (active && e.type == SDL_KEYDOWN) {
            if (e.key.keysym.sym == SDLK_TAB) {
                return true;
            }

            if (e.key.keysym.sym == SDLK_BACKSPACE && !text.empty()) {
                text.pop_back();
            }
            else if (e.key.keysym.sym == SDLK_RETURN || e.key.keysym.sym == SDLK_ESCAPE) {
                active = false;
                SDL_StopTextInput();
            }
        }
        return false;
    }

    void render(SDL_Renderer* renderer) {
        if (active) SDL_SetRenderDrawColor(renderer, 180, 220, 255, 255);
        else SDL_SetRenderDrawColor(renderer, bgColor.r, bgColor.g, bgColor.b, bgColor.a);

        SDL_RenderFillRect(renderer, &rect);

        if (active) SDL_SetRenderDrawColor(renderer, 0, 120, 255, 255);
        else SDL_SetRenderDrawColor(renderer, 0, 0, 0, 125);
        SDL_RenderDrawRect(renderer, &rect);

        int textWidth = 0;
        if (!text.empty()) {
            SDL_Surface* surf = TTF_RenderUTF8_Blended(font, text.c_str(), textColor);
            SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, surf);
            SDL_Rect dst = { rect.x + 5, rect.y + (rect.h - surf->h) / 2, surf->w, surf->h };
            textWidth = surf->w;
            SDL_RenderCopy(renderer, tex, NULL, &dst);
            SDL_FreeSurface(surf);
            SDL_DestroyTexture(tex);
        }

        if (active) {
            if (SDL_GetTicks() - lastBlink > 500) {
                showCursor = !showCursor;
                lastBlink = SDL_GetTicks();
            }
            if (showCursor) {
                int cx = rect.x + 5 + textWidth + 2;
                SDL_RenderDrawLine(renderer, cx, rect.y + 5, cx, rect.y + rect.h - 5);
            }
        }
    }
};

// Transient analysis button
class TransientButton : public Button {
private:
    Circuit& circuit;
    bool ready;
    bool askingInput;
    vector<double> times;
    vector<double> voltages;
    string nodeName;
    TTF_Font* font;

    InputBox boxNode;
    InputBox boxTstart;
    InputBox boxTstop;
    InputBox boxStep;

public:
    TransientButton(SDL_Renderer* r, const std::string& text,
                    int x, int y, int w, int h, TTF_Font* f,
                    Circuit& c)
            : Button(r, text, x, y, w, h, f),
              circuit(c), ready(false), askingInput(false),
              boxNode(400, 150, 200, 35, f),
              boxTstart(400, 200, 200, 35, f),
              boxTstop(400, 250, 200, 35, f),
              boxStep(400, 300, 200, 35, f),
              font(f)
    {}

    void doAction() override {
        askingInput = true;
    }

    void setActiveBox(InputBox* box) {
        boxNode.setActive(false);
        boxTstart.setActive(false);
        boxTstop.setActive(false);
        boxStep.setActive(false);

        if (box) {
            box->setActive(true);
            SDL_StartTextInput();
        }
    }

    void handleEvent(const SDL_Event& e) {
        if (!askingInput) return;

        if (e.type == SDL_MOUSEBUTTONDOWN) {
            boxNode.setActive(boxNode.isInside(e.button.x, e.button.y));
            boxTstart.setActive(boxTstart.isInside(e.button.x, e.button.y));
            boxTstop.setActive(boxTstop.isInside(e.button.x, e.button.y));
            boxStep.setActive(boxStep.isInside(e.button.x, e.button.y));
            if (boxNode.isActive() || boxTstart.isActive() || boxTstop.isActive() || boxStep.isActive()) {
                SDL_StartTextInput();
            } else {
                SDL_StopTextInput();
            }
        }

        if (e.type == SDL_TEXTINPUT || e.type == SDL_KEYDOWN) {
            // ---- TAB navigation ----
            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_TAB) {
                if (boxNode.isActive()) {
                    setActiveBox(&boxTstart);
                } else if (boxTstart.isActive()) {
                    setActiveBox(&boxTstop);
                } else if (boxTstop.isActive()) {
                    setActiveBox(&boxStep);
                } else if (boxStep.isActive()) {
                    setActiveBox(&boxNode);
                }
                return;
            }

            boxNode.handleEvent(e);
            boxTstart.handleEvent(e);
            boxTstop.handleEvent(e);
            boxStep.handleEvent(e);

            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_RETURN) {
                runAnalysis();
                askingInput = false;
            }
        }
    }

    void renderLabel(SDL_Renderer* renderer, TTF_Font* font, const std::string& text, int x, int y) {
        SDL_Color color = {255, 255, 255, 255};
        SDL_Surface* surf = TTF_RenderUTF8_Blended(font, text.c_str(), color);
        SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, surf);
        SDL_Rect dst = { x, y, surf->w, surf->h };
        SDL_RenderCopy(renderer, tex, NULL, &dst);
        SDL_FreeSurface(surf);
        SDL_DestroyTexture(tex);
    }

    void renderInputs(SDL_Renderer* renderer, int screenWidth, int screenHeight) {
        if (!askingInput) return;

        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 150);
        SDL_Rect overlay = {0, 0, screenWidth, screenHeight};
        SDL_RenderFillRect(renderer, &overlay);

        renderLabel(renderer, font , "Node Name:", boxNode.getRect().x - 120, boxNode.getRect().y + 5);
        renderLabel(renderer, font, "tStart:", boxTstart.getRect().x - 80, boxTstart.getRect().y + 5);
        renderLabel(renderer, font, "tStop:", boxTstop.getRect().x - 80, boxTstop.getRect().y + 5);
        renderLabel(renderer, font, "Step:", boxStep.getRect().x - 80, boxStep.getRect().y + 5);

        boxNode.render(renderer);
        boxTstart.render(renderer);
        boxTstop.render(renderer);
        boxStep.render(renderer);
    }

    void runAnalysis() {
        nodeName = boxNode.getText();
        double tStart = std::stod(boxTstart.getText());
        double tStop  = std::stod(boxTstop.getText());
        double step   = std::stod(boxStep.getText());

        auto results = circuit.analyzeTransient(step, tStop, tStart);
        times.clear();
        voltages.clear();

        double t = tStart;
        for (auto& stepResult : results) {
            times.push_back(t);
            auto node = circuit.getNode(nodeName);
            voltages.push_back(stepResult.count(node) ? stepResult.at(node) : 0.0);
            t += step;
        }
        ready = true;
    }

    void renderIfReady(SDL_Renderer* renderer) {
        if (ready) {
            SignalPlot sig(nodeName, times, voltages, {255, 0, 0, 255});
            int w, h;
            SDL_GetRendererOutputSize(renderer, &w, &h);
            sig.render(renderer, w, h);
        }
    }
};

// Signal menu button
class SignalMenuButton : public Button {
private:
    TTF_Font* font;
    Circuit& circuit;
    bool menuOpen = false;
    vector<string> options = {"Transient", "Phase", "AC Sweep"};
    int optionHeight = 40;

public:
    function<void(const string&)> onOptionSelected;

    SignalMenuButton(SDL_Renderer* r, const string& text,
                     int x, int y, int w, int h,
                     TTF_Font* f, Circuit& c)
            : Button(r, text, x, y, w, h, f), font(f), circuit(c) {}

    void renderButton() override {
        SDL_Rect mainRect = getRect();
        SDL_Renderer* renderer = getRenderer();

        SDL_SetRenderDrawColor(renderer, 100, 100, 200, 255);
        SDL_RenderFillRect(renderer, &mainRect);

        SDL_Color white = {255, 255, 255, 255};
        SDL_Surface* surface = TTF_RenderText_Blended(font, getButtonName().c_str(), white);
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
        SDL_Rect textRect = {mainRect.x + (mainRect.w - surface->w) / 2,
                             mainRect.y + (mainRect.h - surface->h) / 2,
                             surface->w, surface->h};
        SDL_RenderCopy(renderer, texture, NULL, &textRect);
        SDL_FreeSurface(surface);
        SDL_DestroyTexture(texture);

        if (menuOpen) {
            for (size_t i = 0; i < options.size(); i++) {
                SDL_Rect optRect = {mainRect.x, mainRect.y + mainRect.h + static_cast<int>(i) * optionHeight,
                                    mainRect.w, optionHeight};
                SDL_SetRenderDrawColor(renderer, 80, 80, 180, 255);
                SDL_RenderFillRect(renderer, &optRect);
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
                SDL_RenderDrawRect(renderer, &optRect);

                SDL_Surface* optSurf = TTF_RenderText_Blended(font, options[i].c_str(), {255, 255, 255, 255});
                SDL_Texture* optTex = SDL_CreateTextureFromSurface(renderer, optSurf);
                SDL_Rect textRect = {optRect.x + 5, optRect.y + (optRect.h - optSurf->h) / 2,
                                     optSurf->w, optSurf->h};
                SDL_RenderCopy(renderer, optTex, NULL, &textRect);
                SDL_DestroyTexture(optTex);
                SDL_FreeSurface(optSurf);
            }
        }
    }

    void doAction() override {
        menuOpen = !menuOpen;
    }

    void handleOptionClick(int mouseX, int mouseY) {
        if (!menuOpen) return;

        SDL_Rect mainRect = getRect();
        for (size_t i = 0; i < options.size(); i++) {
            SDL_Rect optRect = {mainRect.x, mainRect.y + mainRect.h + static_cast<int>(i) * optionHeight,
                                mainRect.w, optionHeight};
            if (mouseX >= optRect.x && mouseX <= optRect.x + optRect.w &&
                mouseY >= optRect.y && mouseY <= optRect.y + optRect.h) {

                menuOpen = false;

                if (onOptionSelected) {
                    onOptionSelected(options[i]);
                }
            }
        }
    }
};

// Music toggle button
class MusicToggleButton : public Button {
private:
    bool isPlaying;
    Mix_Music* music;
public:
    MusicToggleButton(SDL_Renderer* ren, const string& n, int x, int y, int w, int h, TTF_Font* font, Mix_Music* mus)
            : Button(ren, n, x, y, w, h, font), isPlaying(true), music(mus) {}

    void doAction() override {
        if (isPlaying) {
            Mix_PauseMusic();
            isPlaying = false;
            cout << "Music Paused\n";
        }
        else {
            Mix_ResumeMusic();
            isPlaying = true;
            cout << "Music Resumed\n";
        }
    }
};

// Menu button
class MenuButton : public Button {
private:
    bool isOpen;
    vector<string> items;
    TTF_Font* font;
    SDL_Renderer* renderer;
    SDL_Rect rect;

public:
    MenuButton(SDL_Renderer* ren, const string& name, int x, int y, int w, int h,
               TTF_Font* f, const vector<string>& menuItems)
            : Button(ren, name, x, y, w, h, f),
              isOpen(false), items(menuItems), font(f), renderer(ren)
    {
        rect = {x, y, w, h};
    }

    void doAction() override {
        isOpen = !isOpen;
    }

    void renderMenu() {
        SDL_SetRenderDrawColor(renderer, 100, 100, 200, 255);
        SDL_RenderFillRect(renderer, &rect);

        SDL_Color white = {255, 255, 255, 255};
        SDL_Surface* surface = TTF_RenderText_Blended(font, getName().c_str(), white);
        if (surface) {
            SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
            int tw, th;
            SDL_QueryTexture(texture, nullptr, nullptr, &tw, &th);
            SDL_Rect textRect = {rect.x + (rect.w - tw) / 2, rect.y + (rect.h - th) / 2, tw, th};
            SDL_RenderCopy(renderer, texture, nullptr, &textRect);
            SDL_DestroyTexture(texture);
            SDL_FreeSurface(surface);
        }

        if (isOpen) {
            int itemHeight = rect.h;
            for (size_t i = 0; i < items.size(); ++i) {
                SDL_Rect itemRect = {rect.x, rect.y + rect.h * (int)(i + 1), rect.w, itemHeight};
                SDL_SetRenderDrawColor(renderer, 70, 70, 150, 255);
                SDL_RenderFillRect(renderer, &itemRect);

                SDL_Surface* surfaceItem = TTF_RenderText_Blended(font, items[i].c_str(), white);
                if (surfaceItem) {
                    SDL_Texture* textureItem = SDL_CreateTextureFromSurface(renderer, surfaceItem);
                    int tw, th;
                    SDL_QueryTexture(textureItem, nullptr, nullptr, &tw, &th);
                    SDL_Rect textRect = {itemRect.x + (itemRect.w - tw) / 2, itemRect.y + (itemRect.h - th) / 2, tw, th};
                    SDL_RenderCopy(renderer, textureItem, nullptr, &textRect);
                    SDL_DestroyTexture(textureItem);
                    SDL_FreeSurface(surfaceItem);
                }
            }
        }
    }

    bool handleClick(const SDL_Event& e) {
        if (isClick(e)) {
            doAction();
            return true;
        }
        if (isOpen && e.type == SDL_MOUSEBUTTONDOWN) {
            int index = (e.button.y - rect.y) / rect.h - 1;
            if (index >= 0 && index < (int)items.size()) {
                std::cout << "Selected: " << items[index] << "\n";
                isOpen = false;
                return true;
            }
        }
        return false;
    }
};

// Schematic editor drawing functions
double zoom = 1.0;
Vec2 pan(0, 0);
Tool currentTool = Tool::Select;
vector<Wire> wires;
vector<Component*> components;
Wire tempWire;
bool tempWireActive = false;
Component* selectedComponent = nullptr;
Wire* selectedWire = nullptr;
ActionHistory history;
bool panning = false;
Vec2 lastMouse(0, 0);
SDL_Texture* gridTex = nullptr;
bool showValueDialog = false;
string currentValueInput = "";
Component* componentToEdit = nullptr;

Vec2 screenFromWorld(const Vec2& w) {
    return Vec2(w.x() * zoom + pan.x(), w.y() * zoom + pan.y());
}

Vec2 worldFromScreen(const Vec2& s) {
    return Vec2((s.x() - pan.x()) / zoom, (s.y() - pan.y()) / zoom);
}

Vec2 snapToGrid(const Vec2& w) {
    return Vec2(round(w.x() / GRID_STEP) * GRID_STEP, round(w.y() / GRID_STEP) * GRID_STEP);
}

void drawGrid(SDL_Renderer* r, int W, int H) {
    double stepSmall = GRID_STEP * zoom;
    double stepBig = stepSmall * 5;

    double ox = fmod(pan.x(), stepSmall);
    double oy = fmod(pan.y(), stepSmall);

    SDL_SetRenderDrawColor(r, 40, 40, 40, 255);
    if (stepSmall >= 4) {
        for (double x = ox; x <= W; x += stepSmall)
            SDL_RenderDrawLine(r, (int)x, 0, (int)x, H);
        for (double y = oy; y <= H; y += stepSmall)
            SDL_RenderDrawLine(r, 0, (int)y, W, (int)y);
    }

    ox = fmod(pan.x(), stepBig);
    oy = fmod(pan.y(), stepBig);
    SDL_SetRenderDrawColor(r, 80, 80, 80, 255);
    for (double x = ox; x <= W; x += stepBig)
        SDL_RenderDrawLine(r, (int)x, 0, (int)x, H);
    for (double y = oy; y <= H; y += stepBig)
        SDL_RenderDrawLine(r, 0, (int)y, W, (int)y);
}

void drawResistor(SDL_Renderer* r, const Component& c) {
    auto conns = c.getConnectionPoints();
    Vec2 startWorld = conns[0];
    Vec2 endWorld   = conns[1];
    Vec2 centerWorld = c.getPos();

    double shortLength = 5;
    Vec2 dirStartW = (centerWorld - startWorld).normalized();
    Vec2 dirEndW   = (centerWorld - endWorld).normalized();

    Vec2 shortStartWorld = startWorld + dirStartW * shortLength;
    Vec2 shortEndWorld   = endWorld + dirEndW * shortLength;

    Vec2 start = screenFromWorld(startWorld);
    Vec2 shortStart = screenFromWorld(shortStartWorld);
    Vec2 end = screenFromWorld(endWorld);
    Vec2 shortEnd = screenFromWorld(shortEndWorld);

    SDL_SetRenderDrawColor(r, 255, 150, 0, 255);
    Vec2 center = screenFromWorld(centerWorld);
    Vec2 dir = (end - start).normalized();
    Vec2 perp(-dir.y(), dir.x());
    double segLen = 8 * zoom;
    int segCount = 5;
    double H = 8 * zoom;

    Vec2 current = center - dir * (segLen * segCount / 2);
    int sign = 1;
    for(int i = 0; i < segCount; i++) {
        Vec2 next = current + dir * segLen;
        Vec2 mid = (current + next) * 0.5 + perp * (H * sign);
        SDL_RenderDrawLine(r, (int)current.x(), (int)current.y(),
                           (int)mid.x(), (int)mid.y());
        SDL_RenderDrawLine(r, (int)mid.x(), (int)mid.y(),
                           (int)next.x(), (int)next.y());
        current = next;
        sign *= -1;
    }
}

void drawCapacitor(SDL_Renderer* r, const Component& c) {
    auto conns = c.getConnectionPoints();
    Vec2 start = screenFromWorld(conns[0]);
    Vec2 end = screenFromWorld(conns[1]);
    Vec2 center = screenFromWorld(c.getPos());

    SDL_SetRenderDrawColor(r, 0, 255, 0, 255);
    Vec2 dir = (end - start).normalized();
    Vec2 perp(-dir.y(), dir.x());
    double plateLen = 15 * zoom;

    Vec2 plate1Start = center - perp * plateLen;
    Vec2 plate1End = center + perp * plateLen;
    Vec2 plate2Start = plate1Start + dir * (5 * zoom);
    Vec2 plate2End = plate1End + dir * (5 * zoom);

    SDL_RenderDrawLine(r, (int)plate1Start.x(), (int)plate1Start.y(),
                       (int)plate1End.x(),   (int)plate1End.y());

    SDL_RenderDrawLine(r, (int)plate2Start.x(), (int)plate2Start.y(),
                       (int)plate2End.x(),   (int)plate2End.y());
}

void drawInductor(SDL_Renderer* r, const Component& c) {
    auto conns = c.getConnectionPoints();
    Vec2 start  = screenFromWorld(conns[0]);
    Vec2 end    = screenFromWorld(conns[1]);
    Vec2 center = screenFromWorld(c.getPos());

    SDL_SetRenderDrawColor(r, 255, 0, 0, 255);
    Vec2 dir  = (end - start).normalized();
    Vec2 perp(-dir.y(), dir.x());
    double radius   = 8 * zoom;
    int turns       = 5;
    double turnDist = 5 * zoom;

    Vec2 current = center - dir * (turns * turnDist / 2);
    for(int i = 0; i < turns; i++) {
        Vec2 next = current + dir * turnDist;
        Vec2 mid  = (current + next) * 0.5;

        for(int a = 0; a <= 180; a += 10) {
            double angle = (dir.x() > 0 ? a : -a) * M_PI / 180.0;
            Vec2 p1 = mid + perp * (radius * sin(angle))
                      + dir  * (radius * cos(angle));
            Vec2 p2 = mid + perp * (radius * sin((a + 10) * M_PI / 180))
                      + dir  * (radius * cos((a + 10) * M_PI / 180));
            SDL_RenderDrawLine(r, (int)p1.x(), (int)p1.y(),
                               (int)p2.x(), (int)p2.y());
        }
        current = next;
    }
}

void drawDiode(SDL_Renderer* r, const Component& c) {
    auto conns  = c.getConnectionPoints();
    Vec2 start  = screenFromWorld(conns[0]);
    Vec2 end    = screenFromWorld(conns[1]);
    Vec2 center = screenFromWorld(c.getPos());

    SDL_SetRenderDrawColor(r, 255, 165, 0, 255);
    Vec2 dir  = (end - start).normalized();
    Vec2 perp(-dir.y(), dir.x());
    double size = 15 * zoom;

    Vec2 p1 = center - dir * size;
    Vec2 p2 = center + dir * size + perp * size;
    Vec2 p3 = center + dir * size - perp * size;
    SDL_RenderDrawLine(r, (int)p1.x(), (int)p1.y(), (int)p2.x(), (int)p2.y());
    SDL_RenderDrawLine(r, (int)p2.x(), (int)p2.y(), (int)p3.x(), (int)p3.y());
    SDL_RenderDrawLine(r, (int)p3.x(), (int)p3.y(), (int)p1.x(), (int)p1.y());

    SDL_RenderDrawLine(r,
                       (int)(center.x() - 25 + dir.x() * size * 0.5),
                       (int)(center.y() + dir.y() * size * 0.5 - perp.y() * size),
                       (int)(center.x() - 25 + dir.x() * size * 0.5),
                       (int)(center.y() + dir.y() * size * 0.5 + perp.y() * size)
    );
}

void drawVoltageSource(SDL_Renderer* r, const Component& c) {
    auto conns  = c.getConnectionPoints();
    Vec2 start  = screenFromWorld(conns[0]);
    Vec2 end    = screenFromWorld(conns[1]);
    Vec2 center = screenFromWorld(c.getPos());

    SDL_SetRenderDrawColor(r, 128, 0, 128, 255);
    double radius = 12 * zoom;

    for(int a = 0; a < 360; a += 10) {
        double angle1 = a * M_PI / 180.0;
        double angle2 = (a+10) * M_PI / 180.0;
        SDL_RenderDrawLine(r,
                           (int)(center.x() + radius * cos(angle1)),
                           (int)(center.y() + radius * sin(angle1)),
                           (int)(center.x() + radius * cos(angle2)),
                           (int)(center.y() + radius * sin(angle2))
        );
    }

    SDL_RenderDrawLine(r,
                       (int)(center.x() - radius * 0.5), (int)center.y(),
                       (int)(center.x() + radius * 0.5), (int)center.y()
    );
    SDL_RenderDrawLine(r,
                       (int)center.x(), (int)(center.y() - radius * 0.5),
                       (int)center.x(), (int)(center.y() + radius * 0.5)
    );

    SDL_RenderDrawLine(r,
                       (int)(center.x() - radius * 0.5), (int)(center.y() + radius * 0.7),
                       (int)(center.x() + radius * 0.5), (int)(center.y() + radius * 0.7)
    );
}

void drawGround(SDL_Renderer* r, const Component& c) {
    auto conns = c.getConnectionPoints();
    Vec2 top = screenFromWorld(conns[0]);
    Vec2 bottom = screenFromWorld(conns[1]);

    SDL_SetRenderDrawColor(r, 0, 200, 255, 255);
    double triangleWidth  = 20 * zoom;
    double triangleHeight = 25 * zoom;

    Vec2 triangleTop(top.x(), top.y());
    Vec2 triangleLeft (top.x() - triangleWidth/2, triangleTop.y() + triangleHeight);
    Vec2 triangleRight(top.x() + triangleWidth/2, triangleTop.y() + triangleHeight);

    SDL_RenderDrawLine(r, (int)triangleTop.x(),   (int)triangleTop.y(),
                       (int)triangleLeft.x(), (int)triangleLeft.y());
    SDL_RenderDrawLine(r, (int)triangleTop.x(),   (int)triangleTop.y(),
                       (int)triangleRight.x(),(int)triangleRight.y());
    SDL_RenderDrawLine(r, (int)triangleLeft.x(),  (int)triangleLeft.y(),
                       (int)triangleRight.x(),(int)triangleRight.y());

    int lineCount     = 3;
    double lineSpacing = 5 * zoom;
    double startY     = triangleTop.y() + triangleHeight + 2;

    for(int i = 1; i <= lineCount; i++) {
        double lineWidth = triangleWidth * (1.0 - (i-1)*0.3);
        SDL_RenderDrawLine(r,
                           (int)(top.x() - lineWidth/2), (int)(startY + (i-1)*lineSpacing),
                           (int)(top.x() + lineWidth/2), (int)(startY + (i-1)*lineSpacing)
        );
    }
}

void drawWire(SDL_Renderer* r, const Wire& w) {
    SDL_SetRenderDrawColor(r, 0, 0, 255, 255);
    for(size_t i = 1; i < w.size(); ++i) {
        Vec2 a = screenFromWorld(w.getPoint(i-1));
        Vec2 b = screenFromWorld(w.getPoint(i));
        SDL_RenderDrawLine(r, (int)a.x(), (int)a.y(), (int)b.x(), (int)b.y());
    }
}

void drawText(SDL_Renderer* renderer, TTF_Font* font, const string& text, int x, int y) {
    SDL_Color color = {255, 255, 255, 255};
    SDL_Surface* surface = TTF_RenderText_Blended(font, text.c_str(), color);
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_Rect dstRect = {x, y, surface->w, surface->h};
    SDL_FreeSurface(surface);
    SDL_RenderCopy(renderer, texture, NULL, &dstRect);
    SDL_DestroyTexture(texture);
}

void drawValueDialog(SDL_Renderer* renderer, TTF_Font* font) {
    if (!showValueDialog) return;

    int W, H;
    SDL_GetRendererOutputSize(renderer, &W, &H);

    // Draw semi-transparent background
    SDL_Rect bgRect = {W/2 - 200, H/2 - 100, 400, 200};
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 200);
    SDL_RenderFillRect(renderer, &bgRect);
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    SDL_RenderDrawRect(renderer, &bgRect);

    // Draw title
    drawText(renderer, font, "Edit Value", W/2 - 50, H/2 - 80);

    // Draw input box
    SDL_Rect inputRect = {W/2 - 150, H/2 - 30, 300, 40};
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
    SDL_RenderFillRect(renderer, &inputRect);
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    SDL_RenderDrawRect(renderer, &inputRect);

    // Draw current input text
    drawText(renderer, font, currentValueInput, W/2 - 140, H/2 - 20);

    // Draw OK button
    SDL_Rect okRect = {W/2 - 180, H/2 + 40, 120, 40};
    SDL_SetRenderDrawColor(renderer, 50, 150, 50, 255);
    SDL_RenderFillRect(renderer, &okRect);
    SDL_SetRenderDrawColor(renderer, 100, 200, 100, 255);
    SDL_RenderDrawRect(renderer, &okRect);
    drawText(renderer, font, "OK", W/2 - 140, H/2 + 50);

    // Draw Cancel button
    SDL_Rect cancelRect = {W/2 + 60, H/2 + 40, 120, 40};
    SDL_SetRenderDrawColor(renderer, 150, 50, 50, 255);
    SDL_RenderFillRect(renderer, &cancelRect);
    SDL_SetRenderDrawColor(renderer, 200, 100, 100, 255);
    SDL_RenderDrawRect(renderer, &cancelRect);
    drawText(renderer, font, "Cancel", W/2 + 80, H/2 + 50);
}

void drawToolMenu(SDL_Renderer* ren, TTF_Font* font, Tool currentTool) {
    SDL_Rect menuRect = {10, 10, 200, 250};
    SDL_SetRenderDrawColor(ren, 30, 30, 30, 200);
    SDL_RenderFillRect(ren, &menuRect);
    SDL_SetRenderDrawColor(ren, 80, 80, 80, 255);
    SDL_RenderDrawRect(ren, &menuRect);

    drawText(ren, font, "Tools", 20, 15);

    vector<pair<string, Tool>> tools = {
            {"ESC: Select", Tool::Select},
            {"W: Wire", Tool::Wire},
            {"R: Resistor", Tool::PlaceRes},
            {"C: Capacitor", Tool::PlaceCap},
            {"L: Inductor", Tool::PlaceInd},
            {"D: Diode", Tool::PlaceDio},
            {"V: Voltage", Tool::PlaceVolt},
            {"G: Ground", Tool::PlaceGnd},
            {"Z: Undo", Tool::Select},
            {"Y: Redo", Tool::Select}
    };

    for(size_t i = 0; i < tools.size(); i++) {
        SDL_Color color = (currentTool == tools[i].second) ?
                          SDL_Color{100, 200, 255, 255} :
                          SDL_Color{255, 255, 255, 255};
        drawText(ren, font, tools[i].first, 20, 40 + i*25);
    }
}

double distancePointToSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
    Vec2 ab = b - a;
    Vec2 ap = p - a;
    double t = max(0.0, min(1.0, ((ap.x() * ab.x()) + (ap.y() * ab.y())) / ((ab.x()*ab.x()) + (ab.y()*ab.y()))));
    Vec2 closest = a + ab * t;
    return (p - closest).length();
}

bool isPointInRect(int x, int y, SDL_Rect rect) {
    return x >= rect.x && x <= rect.x + rect.w &&
           y >= rect.y && y <= rect.y + rect.h;
}

// Main function
int main(int argc, char* argv[]) {
    showSplashScreen();
    Circuit circuit;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
        std::cerr << "SDL Init Error: " << SDL_GetError() << "";
        return -1;
    }
    if (TTF_Init() < 0) {
        std::cerr << "TTF Init Error: " << TTF_GetError() << "";
        SDL_Quit();
        return -1;
    }
    if (IMG_Init(IMG_INIT_PNG) == 0) {
        std::cerr << "IMG Init Error: " << IMG_GetError() << "";
        TTF_Quit();
        SDL_Quit();
        return -1;
    }
    if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) < 0) {
        std::cerr << "SDL_mixer Init Error: " << Mix_GetError() << "";
        IMG_Quit();
        TTF_Quit();
        SDL_Quit();
        return -1;
    }

    SDL_DisplayMode dm;
    SDL_GetCurrentDisplayMode(0, &dm);

    SDL_Window* window = SDL_CreateWindow(
            "Circuit Simulator & Schematic Editor",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            dm.w, dm.h,
            SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

    if (!window) {
        std::cerr << "CreateWindow Error: " << SDL_GetError() << "";
        Mix_CloseAudio();
        IMG_Quit();
        TTF_Quit();
        SDL_Quit();
        return -1;
    }

    SDL_Surface* iconSurface = IMG_Load("background.png");
    if (iconSurface) {
        SDL_SetWindowIcon(window, iconSurface);
        SDL_FreeSurface(iconSurface);
    }
    else {
        cerr << "Icon Load Error: " << IMG_GetError() << "\n";
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        cerr << "CreateRenderer Error: " << SDL_GetError() << "";
        SDL_DestroyWindow(window);
        Mix_CloseAudio();
        IMG_Quit();
        TTF_Quit();
        SDL_Quit();
        return -1;
    }

    Frame frame(renderer);
    TTF_Font* font = TTF_OpenFont(R"(ITCBLKAD.ttf)", 16);
    TTF_Font* smallFont = TTF_OpenFont(R"(ITCBLKAD.ttf)", 12);
    if (!font || !smallFont) {
        cerr << "Font Error: " << TTF_GetError() << "";
    }

    Mix_Music* bgMusic = Mix_LoadMUS("downwithisrael.mp3");
    if (!bgMusic) {
        cerr << "Failed to load music: " << Mix_GetError() << "\n";
    }
    else {
        Mix_PlayMusic(bgMusic, -1);
    }

    // Create UI buttons
    MusicToggleButton musicBtn(renderer, "Toggle Music", 0, 0, 150, 50, font, bgMusic);
    SignalMenuButton signalBtn(renderer, "signal", 151, 0, 150, 50, font, circuit);
    TransientButton tBtn(renderer, "Transient", 151, 200, 150, 50, font, circuit);

    vector<string> elements = {
            "Voltage Source (v)",
            "Current Source (c)",
            "Diode (d)",
            "Resistor (r)",
            "Ground (gnd)",
            "Capacitor (cap)",
            "Inductor (ind)"
    };
    MenuButton menuBtn(renderer, "Menu", 302, 0, 120, 50, font, elements);

    signalBtn.onOptionSelected = [&](const string& option) {
        if (option == "Transient") {
            tBtn.doAction();
        } else if (option == "Phase") {
            // Handle phase analysis
        } else if (option == "AC Sweep") {
            // Handle AC sweep analysis
        }
    };

    SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP);

    bool running = true;
    bool f11Pressed = false;

    while (running) {
        running = frame.nextEvent();

        if (frame.containsEvent(SDL_QUIT)) {
            running = false;
        }

        for (const auto& e : frame.getEvents()) {
            if (menuBtn.handleClick(e)) {
                // Menu selection handled
            }

            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) {
                running = false;
            }

            if (e.key.keysym.sym == SDLK_F11 && !f11Pressed) {
                f11Pressed = true;
                Uint32 flags = SDL_GetWindowFlags(window);
                if (flags & SDL_WINDOW_FULLSCREEN_DESKTOP) {
                    SDL_SetWindowFullscreen(window, 0);
                    SDL_SetWindowBordered(window, SDL_TRUE);
                    SDL_SetWindowSize(window, 1280, 720);
                    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
                }
                else {
                    SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP);
                }
            }
            if (e.type == SDL_KEYUP && e.key.keysym.sym == SDLK_F11) {
                f11Pressed = false;
            }

            if (signalBtn.isClick(e)) {
                signalBtn.doAction();
            }
            if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                signalBtn.handleOptionClick(e.button.x, e.button.y);
            }
            tBtn.handleEvent(e);
            if (musicBtn.isClick(e)) {
                musicBtn.doAction();
            }

            // Schematic editor event handling
            if(e.type == SDL_MOUSEWHEEL) {
                int mx, my;
                SDL_GetMouseState(&mx, &my);
                Vec2 before = worldFromScreen(Vec2(mx, my));
                double factor = (e.wheel.y > 0) ? 1.1 : 0.9;
                zoom = max(0.1, min(zoom * factor, 10.0));
                Vec2 after = worldFromScreen(Vec2(mx, my));
                Vec2 delta = after - before;
                pan = pan - delta * zoom;
            }
            else if(e.type == SDL_MOUSEBUTTONDOWN) {
                if(e.button.button == SDL_BUTTON_MIDDLE) {
                    panning = true;
                    lastMouse = Vec2(e.button.x, e.button.y);
                }

                if(e.button.button == SDL_BUTTON_LEFT) {
                    if (showValueDialog) {
                        int W, H;
                        SDL_GetRendererOutputSize(renderer, &W, &H);
                        SDL_Rect okRect = {W/2 - 180, H/2 + 40, 120, 40};
                        SDL_Rect cancelRect = {W/2 + 60, H/2 + 40, 120, 40};

                        if (isPointInRect(e.button.x, e.button.y, okRect)) {
                            // Save the edited value
                            if (componentToEdit) {
                                Action action;
                                action.type = Action::EditComponent;
                                action.component = componentToEdit;
                                action.oldValue = componentToEdit->getValue();
                                action.newValue = currentValueInput;
                                history.addAction(action);

                                componentToEdit->setValue(currentValueInput);
                            }
                            showValueDialog = false;
                        }
                        else if (isPointInRect(e.button.x, e.button.y, cancelRect)) {
                            showValueDialog = false;
                        }
                    }
                    else {
                        Vec2 w = snapToGrid(worldFromScreen(Vec2(e.button.x, e.button.y)));

                        if(currentTool == Tool::Wire) {
                            if(!tempWireActive) {
                                tempWireActive = true;
                                tempWire = Wire();
                                tempWire.addPoint(w);
                            }
                            else {
                                Vec2 prev = tempWire.getPoint(tempWire.size()-1);
                                if(prev.x() != w.x() && prev.y() != w.y()) {
                                    tempWire.addPoint(Vec2(w.x(), prev.y()));
                                }
                                tempWire.addPoint(w);
                            }
                        }
                        else if(currentTool != Tool::Select) {
                            ComponentType type;
                            string value;

                            switch(currentTool) {
                                case Tool::PlaceRes:
                                    type = ComponentType::Resistor;
                                    value = "1k";
                                    break;
                                case Tool::PlaceCap:
                                    type = ComponentType::Capacitor;
                                    value = "10u";
                                    break;
                                case Tool::PlaceInd:
                                    type = ComponentType::Inductor;
                                    value = "100m";
                                    break;
                                case Tool::PlaceDio:
                                    type = ComponentType::Diode;
                                    value = "D";
                                    break;
                                case Tool::PlaceVolt:
                                    type = ComponentType::VoltageSource;
                                    value = "5V";
                                    break;
                                case Tool::PlaceGnd:
                                    type = ComponentType::Ground;
                                    value = "GND";
                                    break;
                                default: break;
                            }

                            Component* newComp = new Component(type, w, 0, "", value);
                            components.push_back(newComp);

                            Action action;
                            action.type = Action::AddComponent;
                            action.component = newComp;
                            history.addAction(action);
                        }
                        else {
                            selectedComponent = nullptr;
                            selectedWire = nullptr;
                            Vec2 mouseWorld = worldFromScreen(Vec2(e.button.x, e.button.y));

                            const double bodySelectRadius = 10;
                            const double pinSelectRadius  = 5;

                            for (auto* comp : components) {
                                Vec2 pos = comp->getPos();
                                if ((pos - mouseWorld).length() <= bodySelectRadius) {
                                    selectedComponent = comp;
                                    break;
                                }

                                auto conns = comp->getConnectionPoints();
                                for (const auto& p : conns) {
                                    if ((p - mouseWorld).length() <= pinSelectRadius) {
                                        selectedComponent = comp;
                                        break;
                                    }
                                }
                                if (selectedComponent) break;
                            }

                            if (!selectedComponent) {
                                const double wireSelectThreshold = 3.0;
                                for (auto& wire : wires) {
                                    for (size_t i = 1; i < wire.size(); i++) {
                                        Vec2 a = wire.getPoint(i-1);
                                        Vec2 b = wire.getPoint(i);
                                        double dist = distancePointToSegment(mouseWorld, a, b);
                                        if (dist <= wireSelectThreshold) {
                                            selectedWire = &wire;
                                            break;
                                        }
                                    }
                                    if (selectedWire) break;
                                }
                            }
                        }
                    }
                }

                if(e.button.button == SDL_BUTTON_RIGHT) {
                    if (showValueDialog) {
                        showValueDialog = false;
                    }
                    else if (selectedComponent) {
                        showValueDialog = true;
                        currentValueInput = selectedComponent->getValue();
                        componentToEdit = selectedComponent;
                    }
                    else if(tempWireActive && tempWire.size() > 1) {
                        wires.push_back(tempWire);

                        Action action;
                        action.type = Action::AddWire;
                        action.wire = &wires.back();
                        history.addAction(action);

                        tempWireActive = false;
                    }
                }
            }
            else if(e.type == SDL_MOUSEBUTTONUP) {
                if(e.button.button == SDL_BUTTON_MIDDLE) panning = false;
            }
            else if(e.type == SDL_MOUSEMOTION) {
                if(panning) {
                    Vec2 cur(e.motion.x, e.motion.y);
                    Vec2 d = cur - lastMouse;
                    pan = pan + d;
                    lastMouse = cur;
                }
            }
            else if(e.type == SDL_KEYDOWN) {
                if (showValueDialog) {
                    if (e.key.keysym.sym == SDLK_RETURN) {
                        if (componentToEdit) {
                            Action action;
                            action.type = Action::EditComponent;
                            action.component = componentToEdit;
                            action.oldValue = componentToEdit->getValue();
                            action.newValue = currentValueInput;
                            history.addAction(action);

                            componentToEdit->setValue(currentValueInput);
                        }
                        showValueDialog = false;
                    }
                    else if (e.key.keysym.sym == SDLK_ESCAPE) {
                        showValueDialog = false;
                    }
                    else if (e.key.keysym.sym == SDLK_BACKSPACE && !currentValueInput.empty()) {
                        currentValueInput.pop_back();
                    }
                }
                else {
                    switch(e.key.keysym.sym) {
                        case SDLK_ESCAPE:
                            currentTool = Tool::Select;
                            tempWireActive = false;
                            selectedComponent = nullptr;
                            break;
                        case SDLK_r: currentTool = Tool::PlaceRes; break;
                        case SDLK_w: currentTool = Tool::Wire; break;
                        case SDLK_c: currentTool = Tool::PlaceCap; break;
                        case SDLK_l: currentTool = Tool::PlaceInd; break;
                        case SDLK_d: currentTool = Tool::PlaceDio; break;
                        case SDLK_v: currentTool = Tool::PlaceVolt; break;
                        case SDLK_g: currentTool = Tool::PlaceGnd; break;
                        case SDLK_LEFT:
                            if(selectedComponent)
                                selectedComponent->setRotation(selectedComponent->getRotation() - 90);
                            break;
                        case SDLK_RIGHT:
                            if(selectedComponent)
                                selectedComponent->setRotation(selectedComponent->getRotation() + 90);
                            break;
                        case SDLK_DELETE:
                        case SDLK_BACKSPACE:
                            if (selectedComponent) {
                                Action action;
                                action.type = Action::DeleteComponent;
                                action.component = selectedComponent;
                                history.addAction(action);

                                auto it = std::find(components.begin(), components.end(), selectedComponent);
                                if (it != components.end()) {
                                    delete *it;
                                    components.erase(it);
                                    selectedComponent = nullptr;
                                }
                            }
                            else if (selectedWire) {
                                Action action;
                                action.type = Action::DeleteWire;
                                action.wire = selectedWire;
                                history.addAction(action);

                                auto it = std::find_if(wires.begin(), wires.end(),
                                                       [&](const Wire& w) { return &w == selectedWire; });
                                if (it != wires.end()) {
                                    wires.erase(it);
                                    selectedWire = nullptr;
                                }
                            }
                            break;
                        case SDLK_z:
                            if (SDL_GetModState() & KMOD_CTRL) {
                                Action action = history.undo();
                                switch (action.type) {
                                    case Action::AddComponent: {
                                        auto it = std::find(components.begin(), components.end(), action.component);
                                        if (it != components.end()) {
                                            delete *it;
                                            components.erase(it);
                                        }
                                        break;
                                    }
                                    case Action::DeleteComponent:
                                        components.push_back(action.component);
                                        break;
                                    case Action::AddWire: {
                                        auto it = std::find_if(wires.begin(), wires.end(),
                                                               [&](const Wire& w) { return &w == action.wire; });
                                        if (it != wires.end()) {
                                            wires.erase(it);
                                        }
                                        break;
                                    }
                                    case Action::DeleteWire:
                                        wires.push_back(*action.wire);
                                        break;
                                    case Action::EditComponent:
                                        if (action.component) {
                                            action.component->setValue(action.oldValue);
                                        }
                                        break;
                                }
                            }
                            break;
                        case SDLK_y:
                            if (SDL_GetModState() & KMOD_CTRL) {
                                Action action = history.redo();
                                switch (action.type) {
                                    case Action::AddComponent:
                                        components.push_back(action.component);
                                        break;
                                    case Action::DeleteComponent: {
                                        auto it = std::find(components.begin(), components.end(), action.component);
                                        if (it != components.end()) {
                                            delete *it;
                                            components.erase(it);
                                        }
                                        break;
                                    }
                                    case Action::AddWire:
                                        wires.push_back(*action.wire);
                                        break;
                                    case Action::DeleteWire: {
                                        auto it = std::find_if(wires.begin(), wires.end(),
                                                               [&](const Wire& w) { return &w == action.wire; });
                                        if (it != wires.end()) {
                                            wires.erase(it);
                                        }
                                        break;
                                    }
                                    case Action::EditComponent:
                                        if (action.component) {
                                            action.component->setValue(action.newValue);
                                        }
                                        break;
                                }
                            }
                            break;
                    }
                }
            }
            else if (e.type == SDL_TEXTINPUT && showValueDialog) {
                currentValueInput += e.text.text;
            }
        }

        // Rendering
        SDL_SetRenderDrawColor(renderer, BACKGROUND_COLOR.r, BACKGROUND_COLOR.g,
                               BACKGROUND_COLOR.b, BACKGROUND_COLOR.a);
        SDL_RenderClear(renderer);

        int W, H;
        SDL_GetRendererOutputSize(renderer, &W, &H);
        drawGrid(renderer, W, H);

        // Draw schematic components
        for(const auto& wire : wires) drawWire(renderer, wire);
        if(tempWireActive) drawWire(renderer, tempWire);

        for (const auto* comp : components) {
            switch (comp->getType()) {
                case ComponentType::Resistor: drawResistor(renderer, *comp); break;
                case ComponentType::Capacitor: drawCapacitor(renderer, *comp); break;
                case ComponentType::Inductor: drawInductor(renderer, *comp); break;
                case ComponentType::Diode: drawDiode(renderer, *comp); break;
                case ComponentType::VoltageSource: drawVoltageSource(renderer, *comp); break;
                case ComponentType::Ground: drawGround(renderer, *comp); break;
            }

            if(comp->getValue() != "") {
                Vec2 pos = screenFromWorld(comp->getPos());
                drawText(renderer, smallFont, comp->getValue(),
                         (int)pos.x() + 20, (int)pos.y() - 10);
            }
        }

        if(selectedComponent) {
            auto conns = selectedComponent->getConnectionPoints();
            for(const auto& p : conns) {
                Vec2 sp = screenFromWorld(p);
                SDL_Rect rect = {(int)sp.x()-3, (int)sp.y()-3, 6, 6};
                SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
                SDL_RenderFillRect(renderer, &rect);
            }
        }

        if(selectedWire) {
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
            for(size_t i = 1; i < selectedWire->size(); i++) {
                Vec2 a = screenFromWorld(selectedWire->getPoint(i-1));
                Vec2 b = screenFromWorld(selectedWire->getPoint(i));
                SDL_RenderDrawLine(renderer, (int)a.x(), (int)a.y(), (int)b.x(), (int)b.y());
            }
        }

        // Draw UI elements
        drawToolMenu(renderer, font, currentTool);
        drawValueDialog(renderer, font);

        signalBtn.renderButton();
        musicBtn.renderButton();
        tBtn.renderInputs(renderer, W, H);
        tBtn.renderIfReady(renderer);
        menuBtn.renderMenu();

        string status = "Tool: ";
        switch(currentTool) {
            case Tool::Select: status += "Select"; break;
            case Tool::Wire: status += "Wire"; break;
            case Tool::PlaceRes: status += "Resistor"; break;
            case Tool::PlaceCap: status += "Capacitor"; break;
            case Tool::PlaceInd: status += "Inductor"; break;
            case Tool::PlaceDio: status += "Diode"; break;
            case Tool::PlaceVolt: status += "Voltage Source"; break;
            case Tool::PlaceGnd: status += "Ground"; break;
        }
        drawText(renderer, font, status, 10, H - 30);

        frame.renderframe();
        SDL_RenderPresent(renderer);
    }

    // Cleanup
    Mix_FreeMusic(bgMusic);
    TTF_CloseFont(font);
    TTF_CloseFont(smallFont);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    Mix_CloseAudio();
    IMG_Quit();
    TTF_Quit();
    for (auto* comp : components) {
        delete comp;
    }
    components.clear();
    SDL_Quit();
    return 0;
}
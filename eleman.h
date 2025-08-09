//
// Created by asus on 5/31/2025.
//
#pragma once;

#ifndef PROJECT_ELEMAN_H
#define PROJECT_ELEMAN_H

#endif //PROJECT_ELEMAN_H

#ianclude <iostream>
#include "eleman.h"
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <algorithm>
using namespace std;

class CircuitElement;
class Node;

const double EPSILON = 1e-12;

class CircuitError : public runtime_error {
public:
    explicit CircuitError(const string& message) : runtime_error(message) {}
};

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
    virtual void stampMatrix(vector<vector<double>>& G,
                             vector<vector<double>>& B,
                             vector<vector<double>>& C,
                             vector<vector<double>>& D,
                             vector<double>& J,
                             vector<double>& E,
                             map<shared_ptr<Node>, int>& nodeIndexMap,
                             int& extraVarIndex) = 0;

    virtual void update(double dt) {}
};

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
};

class Capacitor : public CircuitElement {
private:
    double current = 0.0;

public:
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

    void update(double dt) override {
    }

    void stampTransient(vector<vector<double>>& G,
                        vector<double>& J,
                        map<shared_ptr<Node>, int>& nodeIndexMap,
                        double dt) {
        if (dt <= 0) return;

        int n1 = nodeIndexMap[node1];
        int n2 = nodeIndexMap[node2];

        double geq = value / dt;
        double ieq = geq * prevVoltage;

        G[n1][n1] += geq;
        G[n1][n2] -= geq;
        G[n2][n1] -= geq;
        G[n2][n2] += geq;

        J[n1] += ieq;
        J[n2] -= ieq;
    }

    double prevVoltage = 0.0;
};

class Inductor : public CircuitElement {
private:
    double voltage = 0.0;

public:
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

        int n1 = nodeIndexMap[node1];
        int n2 = nodeIndexMap[node2];
        int l = extraVarIndex++;

        B[n1][l] += 1;
        B[n2][l] -= 1;

        C[l][n1] += 1;
        C[l][n2] -= 1;

        D[l][l] -= value;

        E[l] -= value * prevCurrent;
    }

    void update(double dt) override {
        prevCurrent = voltage * dt / value;
    }

    double prevCurrent = 0.0;
};

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
};

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
};


class Diode : public CircuitElement {
private:
    string model;
    double Is;
    double Vt;
    double n;
    double Vz;

public:
    Diode(const string& name, shared_ptr<Node> n1, shared_ptr<Node> n2,
          const string& model = "D", double Vz = 5.1)
            : CircuitElement(name, n1, n2, 0), model(model),
              Is(1e-14), Vt(0.0258), n(1.0), Vz(Vz) {
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
                return -Is * (exp(-(Vd + Vz) / (n * Vt)) - 1 + Vz/Vt);
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

        double Vd = 0.0;
        if (nodeIndexMap.count(node1) && nodeIndexMap.count(node2)) {
            Vd = 0.7;
        }

        double I = calculateCurrent(Vd);
        double g = calculateConductance(Vd);

        int n1 = nodeIndexMap[node1];
        int n2 = nodeIndexMap[node2];

        G[n1][n1] += g;
        G[n1][n2] -= g;
        G[n2][n1] -= g;
        G[n2][n2] += g;

        J[n1] -= (I - g * Vd);
        J[n2] += (I - g * Vd);
    }
};

class Ground : public Node {
public:
    Ground() : Node("GND") {}
};

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

    vector<map<shared_ptr<Node>, double>> analyzeTransient(double tStep, double tStop, double tStart = 0.0) {
        vector<map<shared_ptr<Node>, double>> results;

        auto initialSolution = analyzeDC();
        results.push_back(initialSolution);

        for (auto& element : elements) {
            if (auto cap = dynamic_cast<Capacitor*>(element.get())) {
                cap->prevVoltage = initialSolution[cap->getNode1()] - initialSolution[cap->getNode2()];
            }
            else if (auto ind = dynamic_cast<Inductor*>(element.get())) {
                ind->prevCurrent = 0.0;
            }
        }

        for (double t = tStart + tStep; t <= tStop; t += tStep) {
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

            vector<vector<double>> G(nodeCount, vector<double>(nodeCount, 0.0));
            vector<vector<double>> B(nodeCount, vector<double>(extraVars, 0.0));
            vector<vector<double>> C(extraVars, vector<double>(nodeCount, 0.0));
            vector<vector<double>> D(extraVars, vector<double>(extraVars, 0.0));
            vector<double> J(nodeCount, 0.0);
            vector<double> E(extraVars, 0.0);

            int currentExtraVar = 0;
            for (auto& element : elements) {
                element->stampMatrix(G, B, C, D, J, E, nodeIndexMap, currentExtraVar);

                if (auto cap = dynamic_cast<Capacitor*>(element.get())) {
                    cap->stampTransient(G, J, nodeIndexMap, tStep);
                }
            }

            int matrixSize = nodeCount + extraVars;
            vector<vector<double>> A(matrixSize, vector<double>(matrixSize, 0.0));
            vector<double> b(matrixSize, 0.0);

            for (int i = 0; i < nodeCount; i++) {
                for (int j = 0; j < nodeCount; j++) {
                    A[i][j] = G[i][j];
                }
                for (int j = 0; j < extraVars; j++) {
                    A[i][nodeCount + j] = B[i][j];
                }
                b[i] = J[i];
            }

            for (int i = 0; i < extraVars; i++) {
                for (int j = 0; j < nodeCount; j++) {
                    A[nodeCount + i][j] = C[i][j];
                }
                for (int j = 0; j < extraVars; j++) {
                    A[nodeCount + i][nodeCount + j] = D[i][j];
                }
                b[nodeCount + i] = E[i];
            }

            solveMatrix(A, b);

            map<shared_ptr<Node>, double> solution;
            solution[ground] = 0.0;
            for (const auto& pair : nodeIndexMap) {
                solution[pair.first] = b[pair.second];
            }
            results.push_back(solution);

            for (auto& element : elements) {
                if (auto cap = dynamic_cast<Capacitor*>(element.get())) {
                    double v1 = solution[cap->getNode1()];
                    double v2 = solution[cap->getNode2()];
                    cap->prevVoltage = v1 - v2;
                }
                else if (auto ind = dynamic_cast<Inductor*>(element.get())) {
                }
                element->update(tStep);
            }
        }

        return results;
    }
    vector<shared_ptr<CircuitElement>> elements;
};

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
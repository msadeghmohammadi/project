//
// Created by asus on 5/31/2025.
//
#pragma once;

#ifndef PROJECT_ELEMAN_H
#define PROJECT_ELEMAN_H

#endif //PROJECT_ELEMAN_H

#include <iostream>
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

// Forward declarations
class CircuitElement;
class Node;

// Global constants
const double EPSILON = 1e-12;

// Exception class for circuit errors
class CircuitError : public std::runtime_error {
public:
    explicit CircuitError(const std::string& message) : std::runtime_error(message) {}
};

// Node class to represent connection points in the circuit
class Node {
private:
    std::string name;
    int id;
    static int nextId;

public:
    explicit Node(const std::string& nodeName) : name(nodeName), id(nextId++) {}

    std::string getName() const { return name; }
    int getId() const { return id; }

    void rename(const std::string& newName) { name = newName; }

    bool operator==(const Node& other) const { return id == other.id; }
    bool operator<(const Node& other) const { return id < other.id; }
};

int Node::nextId = 0;

// Base class for all circuit elements
class CircuitElement {
protected:
    std::string name;
    std::shared_ptr<Node> node1;
    std::shared_ptr<Node> node2;
    double value;

public:
    CircuitElement(const std::string& elName, std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, double val)
            : name(elName), node1(n1), node2(n2), value(val) {}

    virtual ~CircuitElement() = default;

    std::string getName() const { return name; }
    std::shared_ptr<Node> getNode1() const { return node1; }
    std::shared_ptr<Node> getNode2() const { return node2; }
    double getValue() const { return value; }

    virtual std::string getType() const = 0;
    virtual void stampMatrix(std::vector<std::vector<double>>& G,
                             std::vector<std::vector<double>>& B,
                             std::vector<std::vector<double>>& C,
                             std::vector<std::vector<double>>& D,
                             std::vector<double>& J,
                             std::vector<double>& E,
                             std::map<std::shared_ptr<Node>, int>& nodeIndexMap,
                             int& extraVarIndex) = 0;

    virtual void update(double dt) {} // For dynamic elements
};

// Resistor class
class Resistor : public CircuitElement {
public:
    Resistor(const std::string& name, std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, double resistance)
            : CircuitElement(name, n1, n2, resistance) {
        if (resistance <= 0) {
            throw CircuitError("Resistance cannot be zero or negative");
        }
    }

    std::string getType() const override { return "Resistor"; }

    void stampMatrix(std::vector<std::vector<double>>& G,
                     std::vector<std::vector<double>>& B,
                     std::vector<std::vector<double>>& C,
                     std::vector<std::vector<double>>& D,
                     std::vector<double>& J,
                     std::vector<double>& E,
                     std::map<std::shared_ptr<Node>, int>& nodeIndexMap,
                     int& extraVarIndex) override {

        double conductance = 1.0 / value;
        int n1 = nodeIndexMap[node1];
        int n2 = nodeIndexMap[node2];

        // Stamp conductance matrix
        G[n1][n1] += conductance;
        G[n1][n2] -= conductance;
        G[n2][n1] -= conductance;
        G[n2][n2] += conductance;
    }
};

// Capacitor class
class Capacitor : public CircuitElement {
private:
    double voltage = 0.0; // Initial voltage

public:
    Capacitor(const std::string& name, std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, double capacitance)
            : CircuitElement(name, n1, n2, capacitance) {
        if (capacitance <= 0) {
            throw CircuitError("Capacitance cannot be zero or negative");
        }
    }

    std::string getType() const override { return "Capacitor"; }

    void stampMatrix(std::vector<std::vector<double>>& G,
                     std::vector<std::vector<double>>& B,
                     std::vector<std::vector<double>>& C,
                     std::vector<std::vector<double>>& D,
                     std::vector<double>& J,
                     std::vector<double>& E,
                     std::map<std::shared_ptr<Node>, int>& nodeIndexMap,
                     int& extraVarIndex) override {

        // Using backward Euler approximation: I = C*(Vnew - Vold)/dt
        // This is handled in the update method
    }

    void update(double dt) override {
        // Update logic for transient analysis would go here
    }
};

// Inductor class
class Inductor : public CircuitElement {
private:
    double current = 0.0; // Initial current

public:
    Inductor(const std::string& name, std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, double inductance)
            : CircuitElement(name, n1, n2, inductance) {
        if (inductance <= 0) {
            throw CircuitError("Inductance cannot be zero or negative");
        }
    }

    std::string getType() const override { return "Inductor"; }

    void stampMatrix(std::vector<std::vector<double>>& G,
                     std::vector<std::vector<double>>& B,
                     std::vector<std::vector<double>>& C,
                     std::vector<std::vector<double>>& D,
                     std::vector<double>& J,
                     std::vector<double>& E,
                     std::map<std::shared_ptr<Node>, int>& nodeIndexMap,
                     int& extraVarIndex) override {

        // For MNA, we need to add an extra variable for inductor current
        int n1 = nodeIndexMap[node1];
        int n2 = nodeIndexMap[node2];
        int l = extraVarIndex++;

        // Stamp G matrix
        // No direct stamp for inductor in G

        // Stamp B matrix
        B[n1][l] += 1;
        B[n2][l] -= 1;

        // Stamp C matrix (B transpose)
        C[l][n1] += 1;
        C[l][n2] -= 1;

        // Stamp D matrix
        D[l][l] -= value; // -L for backward Euler

        // Stamp E vector (right hand side)
        E[l] -= value * current; // -L*I_old for backward Euler
    }

    void update(double dt) override {
        // Update current based on new voltages
        // This would be handled after solving the matrix
    }
};

// Voltage source class
class VoltageSource : public CircuitElement {
private:
    bool isDC = true;
    double amplitude = 0.0;
    double frequency = 0.0;
    double offset = 0.0;

public:
    VoltageSource(const std::string& name, std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, double voltage)
            : CircuitElement(name, n1, n2, voltage) {}

    VoltageSource(const std::string& name, std::shared_ptr<Node> n1, std::shared_ptr<Node> n2,
                  double offset, double amplitude, double frequency)
            : CircuitElement(name, n1, n2, amplitude), isDC(false),
              amplitude(amplitude), frequency(frequency), offset(offset) {}

    std::string getType() const override { return isDC ? "DC Voltage Source" : "AC Voltage Source"; }

    double getVoltage(double time = 0.0) const {
        if (isDC) {
            return value;
        } else {
            return offset + amplitude * std::sin(2 * M_PI * frequency * time);
        }
    }

    void stampMatrix(std::vector<std::vector<double>>& G,
                     std::vector<std::vector<double>>& B,
                     std::vector<std::vector<double>>& C,
                     std::vector<std::vector<double>>& D,
                     std::vector<double>& J,
                     std::vector<double>& E,
                     std::map<std::shared_ptr<Node>, int>& nodeIndexMap,
                     int& extraVarIndex) override {

        int n1 = nodeIndexMap[node1];
        int n2 = nodeIndexMap[node2];
        int v = extraVarIndex++;

        // Stamp B matrix
        B[n1][v] += 1;
        B[n2][v] -= 1;

        // Stamp C matrix (B transpose)
        C[v][n1] += 1;
        C[v][n2] -= 1;

        // Stamp E vector (right hand side)
        E[v] += getVoltage(); // For DC analysis
    }
};

// Current source class
class CurrentSource : public CircuitElement {
private:
    bool isDC = true;
    double amplitude = 0.0;
    double frequency = 0.0;
    double offset = 0.0;

public:
    CurrentSource(const std::string& name, std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, double current)
            : CircuitElement(name, n1, n2, current) {}

    CurrentSource(const std::string& name, std::shared_ptr<Node> n1, std::shared_ptr<Node> n2,
                  double offset, double amplitude, double frequency)
            : CircuitElement(name, n1, n2, amplitude), isDC(false),
              amplitude(amplitude), frequency(frequency), offset(offset) {}

    std::string getType() const override { return isDC ? "DC Current Source" : "AC Current Source"; }

    double getCurrent(double time = 0.0) const {
        if (isDC) {
            return value;
        } else {
            return offset + amplitude * std::sin(2 * M_PI * frequency * time);
        }
    }

    void stampMatrix(std::vector<std::vector<double>>& G,
                     std::vector<std::vector<double>>& B,
                     std::vector<std::vector<double>>& C,
                     std::vector<std::vector<double>>& D,
                     std::vector<double>& J,
                     std::vector<double>& E,
                     std::map<std::shared_ptr<Node>, int>& nodeIndexMap,
                     int& extraVarIndex) override {

        int n1 = nodeIndexMap[node1];
        int n2 = nodeIndexMap[node2];

        // Stamp J vector (right hand side)
        J[n1] -= getCurrent(); // Current leaving node 1
        J[n2] += getCurrent(); // Current entering node 2
    }
};


class Diode : public CircuitElement {
private:
    std::string model;
    double Is;     // Saturation current
    double Vt;     // Thermal voltage
    double n;      // Ideality factor
    double Vz;     // Zener breakdown voltage (for Zener diodes)

public:
    Diode(const std::string& name, std::shared_ptr<Node> n1, std::shared_ptr<Node> n2,
          const std::string& model = "D", double Vz = 5.1)
            : CircuitElement(name, n1, n2, 0), model(model),
              Is(1e-14), Vt(0.0258), n(1.0), Vz(Vz) {
        if (model != "D" && model != "Z") {
            throw CircuitError("Diode model must be either 'D' (standard) or 'Z' (Zener)");
        }
        if (model == "Z" && Vz <= 0) {
            throw CircuitError("Zener breakdown voltage must be positive");
        }
    }

    std::string getType() const override {
        return (model == "Z") ? "Zener Diode" : "Diode";
    }

    // Diode current equation
    double calculateCurrent(double Vd) const {
        if (model == "D") {
            // Standard diode equation
            return Is * (exp(Vd / (n * Vt)) - 1);
        } else {
            // Zener diode equation (simplified model)
            if (Vd < -Vz) {
                // Breakdown region
                return -Is * (exp(-(Vd + Vz) / (n * Vt)) - 1 + Vz/Vt);
            } else if (Vd > 0) {
                // Forward bias
                return Is * (exp(Vd / (n * Vt)) - 1);
            } else {
                // Reverse bias, before breakdown
                return -Is;
            }
        }
    }

    // Derivative of diode current
    double calculateConductance(double Vd) const {
        if (model == "D") {
            return (Is / (n * Vt)) * exp(Vd / (n * Vt));
        } else {
            if (Vd < -Vz) {
                return (Is / (n * Vt)) * exp(-(Vd + Vz) / (n * Vt));
            } else if (Vd > 0) {
                return (Is / (n * Vt)) * exp(Vd / (n * Vt));
            } else {
                return 1e-12; // Small conductance for numerical stability
            }
        }
    }

    void stampMatrix(std::vector<std::vector<double>>& G,
                     std::vector<std::vector<double>>& B,
                     std::vector<std::vector<double>>& C,
                     std::vector<std::vector<double>>& D,
                     std::vector<double>& J,
                     std::vector<double>& E,
                     std::map<std::shared_ptr<Node>, int>& nodeIndexMap,
                     int& extraVarIndex) override {

        // Initial guess for diode voltage (0.7V for forward bias)
        double Vd = 0.0;
        if (nodeIndexMap.count(node1) && nodeIndexMap.count(node2)) {
            // In a real implementation, we'd use the current node voltages
            // For now we'll use 0.7V as initial guess
            Vd = 0.7;
        }

        double I = calculateCurrent(Vd);
        double g = calculateConductance(Vd);

        int n1 = nodeIndexMap[node1];
        int n2 = nodeIndexMap[node2];

        // Stamp the conductance matrix
        G[n1][n1] += g;
        G[n1][n2] -= g;
        G[n2][n1] -= g;
        G[n2][n2] += g;

        // Stamp the current vector
        J[n1] -= (I - g * Vd);
        J[n2] += (I - g * Vd);
    }
};

// Ground node (special case)
class Ground : public Node {
public:
    Ground() : Node("GND") {}
};
double parseValueWithUnit(const std::string& valueStr) {
    if (valueStr.empty()) {
        throw CircuitError("Empty value provided");
    }

    // Find the first non-digit character (except . and e/E for scientific notation)
    size_t unitPos = valueStr.find_first_not_of("0123456789.eE-+");

    double value;
    try {
        if (unitPos == std::string::npos) {
            // No unit specified
            value = std::stod(valueStr);
        } else {
            // Extract numeric part
            value = std::stod(valueStr.substr(0, unitPos));

            // Get unit suffix
            std::string unit = valueStr.substr(unitPos);

            // Convert to lowercase for case-insensitive comparison
            std::transform(unit.begin(), unit.end(), unit.begin(), ::tolower);

            // Apply unit multiplier
            if (unit == "k") {
                value *= 1e3;  // kilo
            } else if (unit == "meg") {
                value *= 1e6;  // mega
            } else if (unit == "g") {
                value *= 1e9;  // giga
            } else if (unit == "m") {
                value *= 1e-3; // milli
            } else if (unit == "u") {
                value *= 1e-6; // micro
            } else if (unit == "n") {
                value *= 1e-9; // nano
            } else if (unit == "p") {
                value *= 1e-12; // pico
            } else {
                throw CircuitError("Unknown unit '" + unit + "'");
            }
        }
    } catch (const std::invalid_argument&) {
        throw CircuitError("Invalid numeric value '" + valueStr + "'");
    } catch (const std::out_of_range&) {
        throw CircuitError("Value '" + valueStr + "' is out of range");
    }

    return value;
}
// Circuit class to hold all elements and nodes
class Circuit {
private:
    std::map<std::string, std::shared_ptr<Node>> nodes;
    std::shared_ptr<Ground> ground;

    // Helper methods
    void ensureGroundExists() {
        if (!ground) {
            ground = std::make_shared<Ground>();
            nodes["GND"] = ground;
        }
    }

    void checkForDuplicateElement(const std::string& name) {
        for (const auto& element : elements) {
            if (element->getName() == name) {
                throw CircuitError("Element " + name + " already exists in the circuit");
            }
        }
    }
    std::shared_ptr<Node> getOrCreateNode(const std::string& name) {
        if (nodes.find(name) == nodes.end()) {
            nodes[name] = std::make_shared<Node>(name);
        }
        return nodes[name];
    }
public:
    Circuit() {
        ensureGroundExists();
    }

    std::shared_ptr<Node> addNode(const std::string& name) {
        if (nodes.find(name) != nodes.end()) {
            throw CircuitError("Node " + name + " already exists");
        }
        auto newNode = std::make_shared<Node>(name);
        nodes[name] = newNode;
        return newNode;
    }

    void renameNode(const std::string& oldName, const std::string& newName) {
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

    std::shared_ptr<Node> getNode(const std::string& name) {
        if (nodes.find(name) == nodes.end()) {
            throw CircuitError("Node " + name + " not found");
        }
        return nodes[name];
    }

    std::vector<std::string> listNodes() const {
        std::vector<std::string> nodeNames;
        for (const auto& pair : nodes) {
            nodeNames.push_back(pair.first);
        }
        return nodeNames;
    }

    std::vector<std::string> listElements(const std::string& type = "") const {
        std::vector<std::string> elementInfo;
        for (const auto& element : elements) {
            if (type.empty() || element->getType().find(type) != std::string::npos) {
                elementInfo.push_back(element->getName() + ": " + element->getType());
            }
        }
        return elementInfo;
    }

    void addResistor(const std::string& name, const std::string& node1Name,
                     const std::string& node2Name, const std::string& valueStr) {
        checkForDuplicateElement(name);
        auto n1 = getOrCreateNode(node1Name);
        auto n2 = getOrCreateNode(node2Name);

        double resistance = parseValueWithUnit(valueStr);
        if (resistance <= 0) {
            throw CircuitError("Resistance must be positive");
        }

        elements.push_back(std::make_shared<Resistor>(name, n1, n2, resistance));
    }

    void addCapacitor(const std::string& name, const std::string& node1Name,
                      const std::string& node2Name, const std::string& valueStr) {
        checkForDuplicateElement(name);
        auto n1 = getOrCreateNode(node1Name);
        auto n2 = getOrCreateNode(node2Name);

        double capacitance = parseValueWithUnit(valueStr);
        if (capacitance <= 0) {
            throw CircuitError("Capacitance must be positive");
        }

        elements.push_back(std::make_shared<Capacitor>(name, n1, n2, capacitance));
    }

    void addInductor(const std::string& name, const std::string& node1Name,
                     const std::string& node2Name, const std::string& valueStr) {
        checkForDuplicateElement(name);
        auto n1 = getOrCreateNode(node1Name);
        auto n2 = getOrCreateNode(node2Name);

        double inductance = parseValueWithUnit(valueStr);
        if (inductance <= 0) {
            throw CircuitError("Inductance must be positive");
        }

        elements.push_back(std::make_shared<Inductor>(name, n1, n2, inductance));
    }

    void addVoltageSource(const std::string& name, const std::string& node1Name,
                          const std::string& node2Name, const std::string& valueStr) {
        checkForDuplicateElement(name);
        auto n1 = getOrCreateNode(node1Name);
        auto n2 = getOrCreateNode(node2Name);

        double voltage = parseValueWithUnit(valueStr);
        elements.push_back(std::make_shared<VoltageSource>(name, n1, n2, voltage));
    }
    void addGround(const std::string& nodeName) {
        ensureGroundExists(); // Make sure we have a ground node

        if (nodeName == "GND") {
            throw CircuitError("Cannot name a node 'GND' as it's reserved for ground");
        }

        auto node = getOrCreateNode(nodeName);

        // Connect the node to ground with a 0V voltage source
        // This is a standard SPICE technique for grounding nodes
        std::string vsourceName = "VGND_" + nodeName;
        addVoltageSource(vsourceName, nodeName, "GND", "0.0");
    }

    void addDiode(const std::string& name, const std::string& node1Name,
                  const std::string& node2Name, const std::string& model = "D",
                  double Vz = 5.1) {
        checkForDuplicateElement(name);
        auto n1 = getOrCreateNode(node1Name);
        auto n2 = getOrCreateNode(node2Name);

        if (model == "Z") {
            elements.push_back(std::make_shared<Diode>(name, n1, n2, model, Vz));
        } else {
            elements.push_back(std::make_shared<Diode>(name, n1, n2, model));
        }
    }
    void addCurrentSource(const std::string& name, const std::string& node1Name,
                          const std::string& node2Name, const std::string& valueStr) {
        checkForDuplicateElement(name);
        auto n1 = getOrCreateNode(node1Name);
        auto n2 = getOrCreateNode(node2Name);

        double current = parseValueWithUnit(valueStr);
        elements.push_back(std::make_shared<CurrentSource>(name, n1, n2, current));
    }

    void removeElement(const std::string& name) {
        auto it = std::find_if(elements.begin(), elements.end(),
                               [&name](const std::shared_ptr<CircuitElement>& el) { return el->getName() == name; });

        if (it == elements.end()) {
            throw CircuitError("Element " + name + " not found");
        }

        elements.erase(it);
    }
    void removeGround(const std::string& nodeName) {
        if (!ground) {
            throw CircuitError("No ground connection exists in the circuit");
        }

        // Find the 0V voltage source that connects this node to ground
        std::string vsourceName = "VGND_" + nodeName;

        auto it = std::find_if(elements.begin(), elements.end(),
                               [&vsourceName](const std::shared_ptr<CircuitElement>& el) {
                                   return el->getName() == vsourceName &&
                                          (el->getType() == "DC Voltage Source") &&
                                          el->getValue() == 0.0;
                               });

        if (it == elements.end()) {
            throw CircuitError("Node " + nodeName + " is not grounded");
        }

        elements.erase(it);
    }

    void removeResistor(const std::string& name) {
        auto it = std::find_if(elements.begin(), elements.end(),
                               [&name](const std::shared_ptr<CircuitElement>& el) {
                                   return el->getName() == name && el->getType() == "Resistor";
                               });

        if (it == elements.end()) {
            throw CircuitError("Resistor " + name + " not found");
        }

        elements.erase(it);
    }

    void removeCapacitor(const std::string& name) {
        auto it = std::find_if(elements.begin(), elements.end(),
                               [&name](const std::shared_ptr<CircuitElement>& el) {
                                   return el->getName() == name && el->getType() == "Capacitor";
                               });

        if (it == elements.end()) {
            throw CircuitError("Capacitor " + name + " not found");
        }

        elements.erase(it);
    }

    void removeInductor(const std::string& name) {
        auto it = std::find_if(elements.begin(), elements.end(),
                               [&name](const std::shared_ptr<CircuitElement>& el) {
                                   return el->getName() == name && el->getType() == "Inductor";
                               });

        if (it == elements.end()) {
            throw CircuitError("Inductor " + name + " not found");
        }

        elements.erase(it);
    }

    void removeVoltageSource(const std::string& name) {
        auto it = std::find_if(elements.begin(), elements.end(),
                               [&name](const std::shared_ptr<CircuitElement>& el) {
                                   return el->getName() == name &&
                                          (el->getType() == "DC Voltage Source" ||
                                           el->getType() == "AC Voltage Source");
                               });

        if (it == elements.end()) {
            throw CircuitError("Voltage source " + name + " not found");
        }

        elements.erase(it);
    }

    void removeCurrentSource(const std::string& name) {
        auto it = std::find_if(elements.begin(), elements.end(),
                               [&name](const std::shared_ptr<CircuitElement>& el) {
                                   return el->getName() == name &&
                                          (el->getType() == "DC Current Source" ||
                                           el->getType() == "AC Current Source");
                               });

        if (it == elements.end()) {
            throw CircuitError("Current source " + name + " not found");
        }

        elements.erase(it);
    }

    void removeDiode(const std::string& name) {
        auto it = std::find_if(elements.begin(), elements.end(),
                               [&name](const std::shared_ptr<CircuitElement>& el) {
                                   return el->getName() == name && el->getType() == "Diode";
                               });

        if (it == elements.end()) {
            throw CircuitError("Diode " + name + " not found");
        }

        elements.erase(it);
    }
    // Matrix solver using Gaussian elimination
    static void solveMatrix(std::vector<std::vector<double>>& A, std::vector<double>& b) {
        const int n = A.size();

        // Forward elimination
        for (int i = 0; i < n; ++i) {
            // Search for maximum in this column
            double maxEl = std::abs(A[i][i]);
            int maxRow = i;
            for (int k = i + 1; k < n; ++k) {
                if (std::abs(A[k][i]) > maxEl) {
                    maxEl = std::abs(A[k][i]);
                    maxRow = k;
                }
            }

            // Swap maximum row with current row
            for (int k = i; k < n; ++k) {
                std::swap(A[maxRow][k], A[i][k]);
            }
            std::swap(b[maxRow], b[i]);

            // Make all rows below this one 0 in current column
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

        // Back substitution
        for (int i = n - 1; i >= 0; --i) {
            b[i] /= A[i][i];
            for (int k = i - 1; k >= 0; --k) {
                b[k] -= A[k][i] * b[i];
            }
        }
    }

    // DC analysis
    std::map<std::shared_ptr<Node>, double> analyzeDC() {
        ensureGroundExists();

        // Create node index map (excluding ground)
        std::map<std::shared_ptr<Node>, int> nodeIndexMap;
        int nodeCount = 0;
        for (const auto& pair : nodes) {
            if (pair.first != "GND") {
                nodeIndexMap[pair.second] = nodeCount++;
            }
        }

        // Count extra variables needed (for voltage sources and inductors)
        int extraVars = 0;
        for (const auto& element : elements) {
            if (element->getType() == "DC Voltage Source" ||
                element->getType() == "AC Voltage Source" ||
                element->getType() == "Inductor") {
                extraVars++;
            }
        }

        // Initialize MNA matrices
        int matrixSize = nodeCount + extraVars;
        std::vector<std::vector<double>> G(nodeCount, std::vector<double>(nodeCount, 0.0));
        std::vector<std::vector<double>> B(nodeCount, std::vector<double>(extraVars, 0.0));
        std::vector<std::vector<double>> C(extraVars, std::vector<double>(nodeCount, 0.0));
        std::vector<std::vector<double>> D(extraVars, std::vector<double>(extraVars, 0.0));
        std::vector<double> J(nodeCount, 0.0);
        std::vector<double> E(extraVars, 0.0);

        // Stamp all elements into the matrix
        int currentExtraVar = 0;
        for (const auto& element : elements) {
            element->stampMatrix(G, B, C, D, J, E, nodeIndexMap, currentExtraVar);
        }

        // Combine into one big matrix for solving
        std::vector<std::vector<double>> A(matrixSize, std::vector<double>(matrixSize, 0.0));
        std::vector<double> b(matrixSize, 0.0);

        // Fill G and B parts
        for (int i = 0; i < nodeCount; ++i) {
            for (int j = 0; j < nodeCount; ++j) {
                A[i][j] = G[i][j];
            }
            for (int j = 0; j < extraVars; ++j) {
                A[i][nodeCount + j] = B[i][j];
            }
            b[i] = J[i];
        }

        // Fill C and D parts
        for (int i = 0; i < extraVars; ++i) {
            for (int j = 0; j < nodeCount; ++j) {
                A[nodeCount + i][j] = C[i][j];
            }
            for (int j = 0; j < extraVars; ++j) {
                A[nodeCount + i][nodeCount + j] = D[i][j];
            }
            b[nodeCount + i] = E[i];
        }

        // Solve the matrix
        solveMatrix(A, b);

        // Extract node voltages (ground is 0)
        std::map<std::shared_ptr<Node>, double> nodeVoltages;
        nodeVoltages[ground] = 0.0;

        for (const auto& pair : nodeIndexMap) {
            nodeVoltages[pair.first] = b[pair.second];
        }

        return nodeVoltages;
    }

    // Transient analysis
    std::vector<std::map<std::shared_ptr<Node>, double>> analyzeTransient(double tStep, double tStop, double tStart = 0.0) {
        // This would implement the transient analysis using backward Euler
        // For simplicity, we'll just return the DC solution in this base version
        std::vector<std::map<std::shared_ptr<Node>, double>> results;
        results.push_back(analyzeDC());
        return results;
    }

    std::vector<std::shared_ptr<CircuitElement>> elements;
};

// Helper functions for command parsing
std::vector<std::string> splitCommand(const std::string& command) {
    std::vector<std::string> tokens;
    std::string token;
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

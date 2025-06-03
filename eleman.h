//
// Created by asus on 5/31/2025.
//

#ifndef PROJECT_ELEMAN_H
#define PROJECT_ELEMAN_H

#endif //PROJECT_ELEMAN_H


class eleman{
protected:
    string name;
    string type;
    int node1, node2;
    double value;
public:
    CircuitElement(string n, string t, int n1, int n2, double v)
    : name(n), type(t), node1(n1), node2(n2), value(v) {}
    virtual ~CircuitElement() {}

    string getName() const { return name; }
    string getType() const { return type; }
    int getNode1() const { return node1; }
    int getNode2() const { return node2; }
    double getValue() const { return value; }

    virtual void stampMatrix(MatrixXd& G, MatrixXd& B, MatrixXd& C, MatrixXd& D,
                             VectorXd& I, VectorXd& E, VectorXd& V, VectorXd& J,
                             int& numVars, map<int,int>& nodeMap) = 0;
};

class Resistor : public CircuitElement {
public:
    Resistor(string n, int n1, int n2, double r)
            : CircuitElement(n, "Resistor", n1, n2, r) {}

    void stampMatrix(MatrixXd& G, MatrixXd& B, MatrixXd& C, MatrixXd& D,
                     VectorXd& I, VectorXd& E, VectorXd& V, VectorXd& J,
                     int& numVars, map<int,int>& nodeMap) override {
        double conductance = 1.0 / value;
        int n1 = nodeMap[node1];
        int n2 = nodeMap[node2];

        if(node1 != 0) {
            G(n1-1, n1-1) += conductance;
            if(node2 != 0) G(n1-1, n2-1) -= conductance;
        }
        if(node2 != 0) {
            G(n2-1, n2-1) += conductance;
            if(node1 != 0) G(n2-1, n1-1) -= conductance;
        }
    }
};

class Capacitor : public CircuitElement {
public:
    Capacitor(string n, int n1, int n2, double c)
            : CircuitElement(n, "Capacitor", n1, n2, c) {}

    void stampMatrix(MatrixXd& G, MatrixXd& B, MatrixXd& C, MatrixXd& D,
                     VectorXd& I, VectorXd& E, VectorXd& V, VectorXd& J,
                     int& numVars, map<int,int>& nodeMap) override {
    }
};

class Inductor : public CircuitElement {
public:
    Inductor(string n, int n1, int n2, double l)
            : CircuitElement(n, "Inductor", n1, n2, l) {}

    void stampMatrix(MatrixXd& G, MatrixXd& B, MatrixXd& C, MatrixXd& D,
                     VectorXd& I, VectorXd& E, VectorXd& V, VectorXd& J,
                     int& numVars, map<int,int>& nodeMap) override {
    }
};

class VoltageSource : public CircuitElement {
public:
    VoltageSource(string n, int n1, int n2, double v)
            : CircuitElement(n, "VoltageSource", n1, n2, v) {}

    void stampMatrix(MatrixXd& G, MatrixXd& B, MatrixXd& C, MatrixXd& D,
                     VectorXd& I, VectorXd& E, VectorXd& V, VectorXd& J,
                     int& numVars, map<int,int>& nodeMap) override {
        int currentIndex = numVars++;
        int n1 = nodeMap[node1];
        int n2 = nodeMap[node2];

        if(node1 != 0) B(n1-1, currentIndex) = 1;
        if(node2 != 0) B(n2-1, currentIndex) = -1;
        C(currentIndex, n1-1) = 1;
        C(currentIndex, n2-1) = -1;
        E(currentIndex) = value;
    }
};

class CurrentSource : public CircuitElement {
public:
    CurrentSource(string n, int n1, int n2, double i)
            : CircuitElement(n, "CurrentSource", n1, n2, i) {}

    void stampMatrix(MatrixXd& G, MatrixXd& B, MatrixXd& C, MatrixXd& D,
                     VectorXd& I, VectorXd& E, VectorXd& V, VectorXd& J,
                     int& numVars, map<int,int>& nodeMap) override {
        int n1 = nodeMap[node1];
        int n2 = nodeMap[node2];

        if(node1 != 0) J(n1-1) -= value;
        if(node2 != 0) J(n2-1) += value;
    }
};
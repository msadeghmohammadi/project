#include <iostream>
#include "eleman.h"
#include <string>
#include <memory>
#include <algorithm>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>
using namespace std;

class frame{
private:
    int number;
    vector<SDL_Event>frameevent;
public:
    void renderframe(){

    }

    void destoryframe(){

    }

    void clearEvent(){

    }

    void nextEvent(){

    }

    void containsEvent(SDL_EventType){

    }
};

class Button{
private:
    string name;
    SDL_Rect rect;
    SDL_Text text;
    int x;
    int y;
    int width;
    int heigh;
public:
    void renderbutton(){

    }

    void mouseishovering(){

    }

    void isclick(){

    }

    void mouseishucking(){

    }

    virtual void doAction() = 0;

};
int main() {
    Circuit circuit;

    while (true) {
        cout << "> ";
        string command;
        getline(cin, command);

        try {
            auto tokens = splitCommand(command);
            if (tokens.empty()) continue;

            if (tokens[0] == "help") {
                cout << "Available commands:" << endl;
                cout << "  add node <name> - Add a new node" << endl;
                cout << "  add R <name> <node1> <node2> <value> - Add resistor" << endl;
                cout << "  add C <name> <node1> <node2> <value> - Add capacitor" << endl;
                cout << "  add L <name> <node1> <node2> <value> - Add inductor" << endl;
                cout << "  add V <name> <node1> <node2> <value> - Add voltage source" << endl;
                cout << "  add I <name> <node1> <node2> <value> - Add current source" << endl;
                cout << "  remove <element_name> - Remove an element" << endl;
                cout << "  rename node <old_name> <new_name> - Rename a node" << endl;
                cout << "  list nodes - List all nodes" << endl;
                cout << "  list elements [type] - List all elements (optionally filtered by type)" << endl;
                cout << "  analyze dc - Perform DC analysis" << endl;
                cout << "  analyze transient <t_step> <t_stop> [t_start] - Perform transient analysis" <<endl;
                cout << "  exit - Exit the program" << endl;
            }
            else if (tokens[0] == "add") {
                if (tokens.size() < 2) {
                    cout << "Error: Missing arguments for 'add'" << endl;
                    continue;
                }

                if (tokens[1] == "node") {
                    if (tokens.size() != 3) {
                        cout << "Error: Usage: add node <name>" << endl;
                        continue;
                    }
                    circuit.addNode(tokens[2]);
                    cout << "Added node " << tokens[2] << endl;
                }
                else if (tokens[1] == "R") {
                    if (tokens.size() != 6) {
                        cout << "Error: Usage: add R <name> <node1> <node2> <value>" << endl;
                        continue;
                    }
                    if(tokens[1]=="r"){
                        cout<<" Error: Element "<<tokens[2]<<" not found in library"<<endl;
                        continue;
                    }
                    circuit.addResistor(tokens[2], tokens[3], tokens[4], stod(tokens[5]));
                    cout << "Added resistor " << tokens[2] << endl;
                }
                else if (tokens[1] == "C") {
                    if (tokens.size() != 6) {
                        cout << "Error: Usage: add C <name> <node1> <node2> <value>" << endl;
                        continue;
                    }
                    circuit.addCapacitor(tokens[2], tokens[3], tokens[4], stod(tokens[5]));
                    cout << "Added capacitor " << tokens[2] << endl;
                }
                else if (tokens[1] == "D") {
                    if (tokens.size() < 5) {
                        cout << "Error: Usage: add D <name> <node1> <node2> [model] [Vz]" << endl;
                        cout << "  model: 'D' (default) or 'Z' (Zener)" << endl;
                        cout << "  Vz: Zener breakdown voltage (required for Zener, default 5.1V)" << endl;
                        continue;
                    }

                    try {
                        string model = (tokens.size() >= 6) ? tokens[5] : "D";
                        if (model == "Z") {
                            double Vz = (tokens.size() >= 7) ? stod(tokens[6]) : 5.1;
                            circuit.addDiode(tokens[2], tokens[3], tokens[4], model, Vz);
                            cout << "Added Zener diode " << tokens[2] << " with Vz=" << Vz << "V" << endl;
                        } else {
                            circuit.addDiode(tokens[2], tokens[3], tokens[4], model);
                            cout << "Added diode " << tokens[2] << endl;
                        }
                    } catch (const CircuitError& e) {
                        cout << "Error: " << e.what() << endl;
                    }
                }
                else if (tokens[1] == "L") {
                    if (tokens.size() != 6) {
                        cout << "Error: Usage: add L <name> <node1> <node2> <value>" << endl;
                        continue;
                    }
                    circuit.addInductor(tokens[2], tokens[3], tokens[4], stod(tokens[5]));
                    cout << "Added inductor " << tokens[2] <<endl;
                }
                else if (tokens[1] == "V") {
                    if (tokens.size() != 6) {
                        cout << "Error: Usage: add V <name> <node1> <node2> <value>" << endl;
                        continue;
                    }
                    circuit.addVoltageSource(tokens[2], tokens[3], tokens[4], stod(tokens[5]));
                    cout << "Added voltage source " << tokens[2] << endl;
                }
                else if (tokens[1] == "I") {
                    if (tokens.size() != 6) {
                        cout << "Error: Usage: add I <name> <node1> <node2> <value>" << endl;
                        continue;
                    }
                    circuit.addCurrentSource(tokens[2], tokens[3], tokens[4], stod(tokens[5]));
                    cout << "Added current source " << tokens[2] << endl;
                }
                else if (tokens[0] == "add" && tokens[1] == "GND") {
                    if (tokens.size() != 3) {
                        cout << "Error: Usage: add GND <node>" << endl;
                        continue;
                    }

                    try {
                        circuit.addGround(tokens[2]);
                        cout << "Grounded node " << tokens[2] << endl;
                    } catch (const CircuitError& e) {
                        cout << "Error: " << e.what() << endl;
                    }
                }
                else {
                    cout << "Error: Unknown element type " << tokens[1] << endl;
                }
            }
            else if (tokens[0] == "delete" && tokens[1] == "GND") {
                if (tokens.size() != 3) {
                    cout << "Error: Usage: delete GND <node>" << endl;
                    continue;
                }

                try {
                    circuit.removeGround(tokens[2]);
                    cout << "Removed ground connection from node " << tokens[2] << endl;
                } catch (const CircuitError& e) {
                    cout << "Error: " << e.what() << endl;
                }
            }
            else if (tokens[0] == "delete") {
                if (tokens.size() != 2) {
                    cout << "Error: Usage: delete <element_name>" << endl;
                    continue;
                }

                auto it = find_if(circuit.elements.begin(), circuit.elements.end(),
                                       [&tokens](const shared_ptr<CircuitElement>& el) {
                                           return el->getName() == tokens[1];
                                       });

                if (it == circuit.elements.end()) {
                    cout << "Error: Element " << tokens[1] << " not found" << endl;
                    continue;
                }

                string type = (*it)->getType();

                try {
                    if (type == "Resistor") {
                        circuit.removeResistor(tokens[1]);
                    }
                    else if (type == "Capacitor") {
                        circuit.removeCapacitor(tokens[1]);
                    }
                    else if (type == "Inductor") {
                        circuit.removeInductor(tokens[1]);
                    }
                    else if (type == "DC Voltage Source" || type == "AC Voltage Source") {
                        circuit.removeVoltageSource(tokens[1]);
                    }
                    else if (type == "DC Current Source" || type == "AC Current Source") {
                        circuit.removeCurrentSource(tokens[1]);
                    }
                    else if (type == "Diode") {
                        circuit.removeDiode(tokens[1]);
                    }
                    else {
                        circuit.removeElement(tokens[1]);
                    }
                    cout << "Deleted " << type << " " << tokens[1] << endl;
                } catch (const CircuitError& e) {
                    cout << "Error: " << e.what() << endl;
                }
            }
            else if (tokens[0] == "rename" && tokens.size() > 1 && tokens[1] == "node") {
                if (tokens.size() != 4) {
                    cout << "Error: Usage: rename node <old_name> <new_name>" << endl;
                    continue;
                }
                circuit.renameNode(tokens[2], tokens[3]);
                cout << "Renamed node " << tokens[2] << " to " << tokens[3] << endl;
            }
            else if (tokens[0] == "list") {
                if (tokens.size() < 2) {
                    cout << "Error: Usage: list nodes|elements [type]" << endl;
                    continue;
                }

                if (tokens[1] == "nodes") {
                    auto nodes = circuit.listNodes();
                    cout << "Nodes:" << endl;
                    for (const auto& node : nodes) {
                        cout << "  " << node << endl;
                    }
                }
                else if (tokens[1] == "elements") {
                    string type = tokens.size() > 2 ? tokens[2] : "";
                    auto elements = circuit.listElements(type);
                    cout << "Elements" << (type.empty() ? "" : " (" + type + ")") << ":" << endl;
                    for (const auto& el : elements) {
                        cout << "  " << el << endl;
                    }
                }
                else {
                    cout << "Error: Unknown list type " << tokens[1] << endl;
                }
            }
            else if (tokens[0] == "analyze") {
                if (tokens.size() < 2) {
                    cout << "Error: Usage: analyze dc|transient ..." << endl;
                    continue;
                }

                if (tokens[1] == "dc") {
                    auto results = circuit.analyzeDC();
                    cout << "DC Analysis Results:" << endl;
                    for (const auto& pair : results) {
                        cout << "  Node " << pair.first->getName() << ": " << pair.second << " V" << endl;
                    }
                }
                else if (tokens[1] == "transient") {
                    if (tokens.size() < 4) {
                        cout << "Error: Usage: analyze transient <t_step> <t_stop> [t_start]" << endl;
                        continue;
                    }
                    double tStep =  stod(tokens[2]);
                    double tStop =  stod(tokens[3]);
                    double tStart = tokens.size() > 4 ? stod(tokens[4]) : 0.0;

                    auto results = circuit.analyzeTransient(tStep, tStop, tStart);
                    cout << "Transient Analysis Results (last time point):" << endl;
                    for (const auto& pair : results.back()) {
                        cout << "  Node " << pair.first->getName() << ": " << pair.second << " V" << endl;
                    }
                }
                else {
                    cout << "Error: Unknown analysis type " << tokens[1] << endl;
                }
            }
            else if (tokens[0] == "exit") {
                break;
            }
            else {
               cout << "Error: Syntax error" << tokens[0] << endl;
            }
        } catch (const CircuitError& e) {
            cout << "Circuit Error: " << e.what() << endl;
        } catch (const exception& e) {
            cout << "Error: " << e.what() << endl;
        }
    }

    return 0;
}
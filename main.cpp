#include <iostream>
#include "eleman.h"
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <algorithm>


using namespace std;
int main() {
    Circuit circuit;

    std::cout << "Simple Circuit Simulator (MNA)" << std::endl;
    std::cout << "Type 'help' for commands" << std::endl;

    while (true) {
        std::cout << "> ";
        std::string command;
        std::getline(std::cin, command);

        try {
            auto tokens = splitCommand(command);
            if (tokens.empty()) continue;

            if (tokens[0] == "help") {
                std::cout << "Available commands:" << std::endl;
                std::cout << "  add node <name> - Add a new node" << std::endl;
                std::cout << "  add R <name> <node1> <node2> <value> - Add resistor" << std::endl;
                std::cout << "  add C <name> <node1> <node2> <value> - Add capacitor" << std::endl;
                std::cout << "  add L <name> <node1> <node2> <value> - Add inductor" << std::endl;
                std::cout << "  add V <name> <node1> <node2> <value> - Add voltage source" << std::endl;
                std::cout << "  add I <name> <node1> <node2> <value> - Add current source" << std::endl;
                std::cout << "  remove <element_name> - Remove an element" << std::endl;
                std::cout << "  rename node <old_name> <new_name> - Rename a node" << std::endl;
                std::cout << "  list nodes - List all nodes" << std::endl;
                std::cout << "  list elements [type] - List all elements (optionally filtered by type)" << std::endl;
                std::cout << "  analyze dc - Perform DC analysis" << std::endl;
                std::cout << "  analyze transient <t_step> <t_stop> [t_start] - Perform transient analysis" << std::endl;
                std::cout << "  exit - Exit the program" << std::endl;
            }
            else if (tokens[0] == "add") {
                if (tokens.size() < 2) {
                    std::cout << "Error: Missing arguments for 'add'" << std::endl;
                    continue;
                }

                if (tokens[1] == "node") {
                    if (tokens.size() != 3) {
                        std::cout << "Error: Usage: add node <name>" << std::endl;
                        continue;
                    }
                    circuit.addNode(tokens[2]);
                    std::cout << "Added node " << tokens[2] << std::endl;
                }
                else if (tokens[1] == "R") {
                    if (tokens.size() != 6) {
                        std::cout << "Error: Usage: add R <name> <node1> <node2> <value>" << std::endl;
                        continue;
                    }
                    try {
                        circuit.addResistor(tokens[2], tokens[3], tokens[4], tokens[5]);
                        std::cout << "Added resistor " << tokens[2] << " = " << tokens[5] << std::endl;
                    } catch (const CircuitError& e) {
                        std::cout << "Error: " << e.what() << std::endl;
                    }
                }
                else if (tokens[1] == "C") {
                    if (tokens.size() != 6) {
                        std::cout << "Error: Usage: add C <name> <node1> <node2> <value>" << std::endl;
                        continue;
                    }
                    circuit.addCapacitor(tokens[2], tokens[3], tokens[4], tokens[5]);
                    std::cout << "Added capacitor " << tokens[2] << " = " << tokens[5] << std::endl;
                }
                else if (tokens[1] == "D") {
                    if (tokens.size() < 5) {
                        std::cout << "Error: Usage: add D <name> <node1> <node2> [model] [Vz]" << std::endl;
                        std::cout << "  model: 'D' (default) or 'Z' (Zener)" << std::endl;
                        std::cout << "  Vz: Zener breakdown voltage (required for Zener, default 5.1V)" << std::endl;
                        continue;
                    }

                    try {
                        std::string model = (tokens.size() >= 6) ? tokens[5] : "D";
                        if (model == "Z") {
                            double Vz = (tokens.size() >= 7) ? std::stod(tokens[6]) : 5.1;
                            circuit.addDiode(tokens[2], tokens[3], tokens[4], model, Vz);
                            std::cout << "Added Zener diode " << tokens[2] << " with Vz=" << Vz << "V" << std::endl;
                        } else {
                            circuit.addDiode(tokens[2], tokens[3], tokens[4], model);
                            std::cout << "Added diode " << tokens[2] << std::endl;
                        }
                    } catch (const CircuitError& e) {
                        std::cout << "Error: " << e.what() << std::endl;
                    }
                }
                else if (tokens[1] == "L") {
                    if (tokens.size() != 6) {
                        std::cout << "Error: Usage: add L <name> <node1> <node2> <value>" << std::endl;
                        continue;
                    }
                    circuit.addInductor(tokens[2], tokens[3], tokens[4], tokens[5]);
                    std::cout << "Added inductor " << tokens[2] << " = " << tokens[5] << std::endl;
                }
                else if (tokens[1] == "V") {
                    if (tokens.size() != 6) {
                        std::cout << "Error: Usage: add V <name> <node1> <node2> <value>" << std::endl;
                        continue;
                    }
                    circuit.addVoltageSource(tokens[2], tokens[3], tokens[4], tokens[5]);
                    std::cout << "Added voltage source " << tokens[2] << " = " << tokens[5] << std::endl;
                }
                else if (tokens[1] == "I") {
                    if (tokens.size() != 6) {
                        std::cout << "Error: Usage: add I <name> <node1> <node2> <value>" << std::endl;
                        continue;
                    }
                    circuit.addCurrentSource(tokens[2], tokens[3], tokens[4], tokens[5]);
                    std::cout << "Added current source " << tokens[2] << " = " << tokens[5] << std::endl;
                }
                else if (tokens[1] == "GND") {
                    if (tokens.size() != 3) {
                        std::cout << "Error: Usage: add GND <node>" << std::endl;
                        continue;
                    }

                    try {
                        circuit.addGround(tokens[2]);
                        std::cout << "Grounded node " << tokens[2] << std::endl;
                    } catch (const CircuitError& e) {
                        std::cout << "Error: " << e.what() << std::endl;
                    }
                }
                else {
                    std::cout << "Error: Unknown element type " << tokens[1] << std::endl;
                }
            }
            else if (tokens[0] == "delete" && tokens[1] == "GND") {
                if (tokens.size() != 3) {
                    std::cout << "Error: Usage: delete GND <node>" << std::endl;
                    continue;
                }

                try {
                    circuit.removeGround(tokens[2]);
                    std::cout << "Removed ground connection from node " << tokens[2] << std::endl;
                } catch (const CircuitError& e) {
                    std::cout << "Error: " << e.what() << std::endl;
                }
            }
            else if (tokens[0] == "delete") {
                if (tokens.size() != 2) {
                    std::cout << "Error: Usage: delete <element_name>" << std::endl;
                    continue;
                }

                // Try to find the element first to give specific error messages
                auto it = std::find_if(circuit.elements.begin(), circuit.elements.end(),
                                       [&tokens](const std::shared_ptr<CircuitElement>& el) {
                                           return el->getName() == tokens[1];
                                       });

                if (it == circuit.elements.end()) {
                    std::cout << "Error: Element " << tokens[1] << " not found" << std::endl;
                    continue;
                }

                std::string type = (*it)->getType();

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
                        circuit.removeElement(tokens[1]); // Fallback for unknown types
                    }
                    std::cout << "Deleted " << type << " " << tokens[1] << std::endl;
                } catch (const CircuitError& e) {
                    std::cout << "Error: " << e.what() << std::endl;
                }
            }
            else if (tokens[0] == "rename" && tokens.size() > 1 && tokens[1] == "node") {
                if (tokens.size() != 4) {
                    std::cout << "Error: Usage: rename node <old_name> <new_name>" << std::endl;
                    continue;
                }
                circuit.renameNode(tokens[2], tokens[3]);
                std::cout << "Renamed node " << tokens[2] << " to " << tokens[3] << std::endl;
            }
            else if (tokens[0] == "list") {
                if (tokens.size() < 2) {
                    std::cout << "Error: Usage: list nodes|elements [type]" << std::endl;
                    continue;
                }

                if (tokens[1] == "nodes") {
                    auto nodes = circuit.listNodes();
                    std::cout << "Nodes:" << std::endl;
                    for (const auto& node : nodes) {
                        std::cout << "  " << node << std::endl;
                    }
                }
                else if (tokens[1] == "elements") {
                    std::string type = tokens.size() > 2 ? tokens[2] : "";
                    auto elements = circuit.listElements(type);
                    std::cout << "Elements" << (type.empty() ? "" : " (" + type + ")") << ":" << std::endl;
                    for (const auto& el : elements) {
                        std::cout << "  " << el << std::endl;
                    }
                }
                else {
                    std::cout << "Error: Unknown list type " << tokens[1] << std::endl;
                }
            }
            else if (tokens[0] == "analyze") {
                if (tokens.size() < 2) {
                    std::cout << "Error: Usage: analyze dc|transient ..." << std::endl;
                    continue;
                }

                if (tokens[1] == "dc") {
                    auto results = circuit.analyzeDC();
                    std::cout << "DC Analysis Results:" << std::endl;
                    for (const auto& pair : results) {
                        std::cout << "  Node " << pair.first->getName() << ": " << pair.second << " V" << std::endl;
                    }
                }
                else if (tokens[1] == "transient") {
                    if (tokens.size() < 4) {
                        std::cout << "Error: Usage: analyze transient <t_step> <t_stop> [t_start]" << std::endl;
                        continue;
                    }
                    double tStep = std::stod(tokens[2]);
                    double tStop = std::stod(tokens[3]);
                    double tStart = tokens.size() > 4 ? std::stod(tokens[4]) : 0.0;

                    auto results = circuit.analyzeTransient(tStep, tStop, tStart);
                    std::cout << "Transient Analysis Results (last time point):" << std::endl;
                    for (const auto& pair : results.back()) {
                        std::cout << "  Node " << pair.first->getName() << ": " << pair.second << " V" << std::endl;
                    }
                }
                else {
                    std::cout << "Error: Unknown analysis type " << tokens[1] << std::endl;
                }
            }
            else if (tokens[0] == "exit") {
                break;
            }
            else {
                std::cout << "Error: Unknown command " << tokens[0] << std::endl;
            }
        } catch (const CircuitError& e) {
            std::cout << "Circuit Error: " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cout << "Error: " << e.what() << std::endl;
        }
    }

    return 0;
}
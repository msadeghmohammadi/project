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

class Frame{
private:
    int number;
    vector<SDL_Event>frameevent;
    SDL_Renderer* renderer;
public:
    const std::vector<SDL_Event>& getEvents() const { return frameevent; }

    Frame(SDL_Renderer* renderer) : number(0), renderer(renderer) {}

    void renderframe(){
        SDL_RenderPresent(renderer);
    }

    void destroyFrame() {
        if (renderer != nullptr) {
            SDL_DestroyRenderer(renderer);
            renderer = nullptr;
        }

        frameevent.clear();
    }

    void clearEvent(){
        frameevent.clear();
    }

    bool nextEvent(){
        clearEvent();

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            frameevent.push_back(e);
        }
        return true;
    }

    bool containsEvent(SDL_EventType type){
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

class Button {
private:
    string name;
    SDL_Rect rect;
    SDL_Texture* textTexture;
    SDL_Renderer* renderer;

public:
    Button(SDL_Renderer* ren, string name, int x, int y, int w, int h, TTF_Font* font)
            : renderer(ren), name(name), textTexture(nullptr) {
        rect = { x, y, w, h };

        SDL_Color white = {255, 255, 255, 255};
        SDL_Surface* surface = TTF_RenderText_Blended(font, name.c_str(), white);
        if(surface) {
            textTexture = SDL_CreateTextureFromSurface(renderer, surface);
            SDL_FreeSurface(surface);
        }
    }

    ~Button() {
        if(textTexture) {
            SDL_DestroyTexture(textTexture);
        }
    }

    void renderButton() {
        SDL_SetRenderDrawColor(renderer, 100, 100, 200, 255);
        SDL_RenderFillRect(renderer, &rect);

        if(textTexture) {
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

    string getName(){
        return name;
    }
    virtual void doAction() = 0;
};

int main(int argc, char* argv[]) {
    Circuit circuit;

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL Init Error: " << SDL_GetError() << "";
        return -1;
    }
    if (TTF_Init() < 0) {
        std::cerr << "TTF Init Error: " << TTF_GetError() << "";
        SDL_Quit();
        return -1;
    }
    SDL_DisplayMode dm;
    SDL_GetCurrentDisplayMode(0, &dm);

    SDL_Window* window = SDL_CreateWindow(
            "Fullscreen App",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            dm.w, dm.h,
            SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

    if (!window) {
        std::cerr << "CreateWindow Error: " << SDL_GetError() << "";
        TTF_Quit();
        SDL_Quit();
        return -1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::cerr << "CreateRenderer Error: " << SDL_GetError() << "";
        SDL_DestroyWindow(window);
        TTF_Quit();
        SDL_Quit();
        return -1;
    }

    Frame frame(renderer);
    TTF_Font* font = TTF_OpenFont("ITCEDSCR.TTF", 16);
    if (!font) {
        std::cerr << "Font Error: " << TTF_GetError() << "";
    }

    class TestButton : public Button {
    public:
        TestButton(SDL_Renderer* r, const string& n, int x, int y, int w, int h, TTF_Font* f)
                : Button(r, n, x, y, w, h, f) {}
        void doAction() override {
            std::cout << "Button Clicked: " << getName() << "";
        }
    };
    TestButton btn(renderer, "signal", 100, 100, 150, 50, font);

    SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP);

    bool running = true;
    bool f11Pressed = false;
    string inputBuffer;

    while (running) {
        running = frame.nextEvent();

        if (frame.containsEvent(SDL_QUIT)) {
            running = false;
        }

        for (const auto& e : frame.getEvents()) {
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
                } else {
                    SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP);
                }
            }

            if (e.type == SDL_KEYUP) {
                if (e.key.keysym.sym == SDLK_F11) {
                    f11Pressed = false;
                }
            }

            if (btn.isClick(e)) {
                btn.doAction();
            }
        }

//        cout << "> ";
//        string command;
//        getline(cin, command);
//
//        try {
//            auto tokens = splitCommand(command);
//            if (tokens.empty()) continue;
//
//            if (tokens[0] == "help") {
//                cout << "Available commands:" << endl;
//                cout << "  add node <name> - Add a new node" << endl;
//                cout << "  add R <name> <node1> <node2> <value> - Add resistor" << endl;
//                cout << "  add C <name> <node1> <node2> <value> - Add capacitor" << endl;
//                cout << "  add L <name> <node1> <node2> <value> - Add inductor" << endl;
//                cout << "  add V <name> <node1> <node2> <value> - Add voltage source" << endl;
//                cout << "  add I <name> <node1> <node2> <value> - Add current source" << endl;
//                cout << "  remove <element_name> - Remove an element" << endl;
//                cout << "  rename node <old_name> <new_name> - Rename a node" << endl;
//                cout << "  list nodes - List all nodes" << endl;
//                cout << "  list elements [type] - List all elements (optionally filtered by type)" << endl;
//                cout << "  analyze dc - Perform DC analysis" << endl;
//                cout << "  analyze transient <t_step> <t_stop> [t_start] - Perform transient analysis" <<endl;
//                cout << "  exit - Exit the program" << endl;
//            }
//            else if (tokens[0] == "add") {
//                if (tokens.size() < 2) {
//                    cout << "Error: Missing arguments for 'add'" << endl;
//                    continue;
//                }
//
//                if (tokens[1] == "node") {
//                    if (tokens.size() != 3) {
//                        cout << "Error: Usage: add node <name>" << endl;
//                        continue;
//                    }
//                    circuit.addNode(tokens[2]);
//                    cout << "Added node " << tokens[2] << endl;
//                }
//                else if (tokens[1] == "R") {
//                    if (tokens.size() != 6) {
//                        cout << "Error: Usage: add R <name> <node1> <node2> <value>" << endl;
//                        continue;
//                    }
//                    if(tokens[1]=="r"){
//                        cout<<" Error: Element "<<tokens[2]<<" not found in library"<<endl;
//                        continue;
//                    }
//                    circuit.addResistor(tokens[2], tokens[3], tokens[4], stod(tokens[5]));
//                    cout << "Added resistor " << tokens[2] << endl;
//                }
//                else if (tokens[1] == "C") {
//                    if (tokens.size() != 6) {
//                        cout << "Error: Usage: add C <name> <node1> <node2> <value>" << endl;
//                        continue;
//                    }
//                    circuit.addCapacitor(tokens[2], tokens[3], tokens[4], stod(tokens[5]));
//                    cout << "Added capacitor " << tokens[2] << endl;
//                }
//                else if (tokens[1] == "D") {
//                    if (tokens.size() < 5) {
//                        cout << "Error: Usage: add D <name> <node1> <node2> [model] [Vz]" << endl;
//                        cout << "  model: 'D' (default) or 'Z' (Zener)" << endl;
//                        cout << "  Vz: Zener breakdown voltage (required for Zener, default 5.1V)" << endl;
//                        continue;
//                    }
//
//                    try {
//                        string model = (tokens.size() >= 6) ? tokens[5] : "D";
//                        if (model == "Z") {
//                            double Vz = (tokens.size() >= 7) ? stod(tokens[6]) : 5.1;
//                            circuit.addDiode(tokens[2], tokens[3], tokens[4], model, Vz);
//                            cout << "Added Zener diode " << tokens[2] << " with Vz=" << Vz << "V" << endl;
//                        } else {
//                            circuit.addDiode(tokens[2], tokens[3], tokens[4], model);
//                            cout << "Added diode " << tokens[2] << endl;
//                        }
//                    } catch (const CircuitError& e) {
//                        cout << "Error: " << e.what() << endl;
//                    }
//                }
//                else if (tokens[1] == "L") {
//                    if (tokens.size() != 6) {
//                        cout << "Error: Usage: add L <name> <node1> <node2> <value>" << endl;
//                        continue;
//                    }
//                    circuit.addInductor(tokens[2], tokens[3], tokens[4], stod(tokens[5]));
//                    cout << "Added inductor " << tokens[2] <<endl;
//                }
//                else if (tokens[1] == "V") {
//                    if (tokens.size() != 6) {
//                        cout << "Error: Usage: add V <name> <node1> <node2> <value>" << endl;
//                        continue;
//                    }
//                    circuit.addVoltageSource(tokens[2], tokens[3], tokens[4], stod(tokens[5]));
//                    cout << "Added voltage source " << tokens[2] << endl;
//                }
//                else if (tokens[1] == "I") {
//                    if (tokens.size() != 6) {
//                        cout << "Error: Usage: add I <name> <node1> <node2> <value>" << endl;
//                        continue;
//                    }
//                    circuit.addCurrentSource(tokens[2], tokens[3], tokens[4], stod(tokens[5]));
//                    cout << "Added current source " << tokens[2] << endl;
//                }
//                else if (tokens[0] == "add" && tokens[1] == "GND") {
//                    if (tokens.size() != 3) {
//                        cout << "Error: Usage: add GND <node>" << endl;
//                        continue;
//                    }
//
//                    try {
//                        circuit.addGround(tokens[2]);
//                        cout << "Grounded node " << tokens[2] << endl;
//                    } catch (const CircuitError& e) {
//                        cout << "Error: " << e.what() << endl;
//                    }
//                }
//                else {
//                    cout << "Error: Unknown element type " << tokens[1] << endl;
//                }
//            }
//            else if (tokens[0] == "delete" && tokens[1] == "GND") {
//                if (tokens.size() != 3) {
//                    cout << "Error: Usage: delete GND <node>" << endl;
//                    continue;
//                }
//
//                try {
//                    circuit.removeGround(tokens[2]);
//                    cout << "Removed ground connection from node " << tokens[2] << endl;
//                } catch (const CircuitError& e) {
//                    cout << "Error: " << e.what() << endl;
//                }
//            }
//            else if (tokens[0] == "delete") {
//                if (tokens.size() != 2) {
//                    cout << "Error: Usage: delete <element_name>" << endl;
//                    continue;
//                }
//
//                auto it = find_if(circuit.elements.begin(), circuit.elements.end(),
//                                       [&tokens](const shared_ptr<CircuitElement>& el) {
//                                           return el->getName() == tokens[1];
//                                       });
//
//                if (it == circuit.elements.end()) {
//                    cout << "Error: Element " << tokens[1] << " not found" << endl;
//                    continue;
//                }
//
//                string type = (*it)->getType();
//
//                try {
//                    if (type == "Resistor") {
//                        circuit.removeResistor(tokens[1]);
//                    }
//                    else if (type == "Capacitor") {
//                        circuit.removeCapacitor(tokens[1]);
//                    }
//                    else if (type == "Inductor") {
//                        circuit.removeInductor(tokens[1]);
//                    }
//                    else if (type == "DC Voltage Source" || type == "AC Voltage Source") {
//                        circuit.removeVoltageSource(tokens[1]);
//                    }
//                    else if (type == "DC Current Source" || type == "AC Current Source") {
//                        circuit.removeCurrentSource(tokens[1]);
//                    }
//                    else if (type == "Diode") {
//                        circuit.removeDiode(tokens[1]);
//                    }
//                    else {
//                        circuit.removeElement(tokens[1]);
//                    }
//                    cout << "Deleted " << type << " " << tokens[1] << endl;
//                } catch (const CircuitError& e) {
//                    cout << "Error: " << e.what() << endl;
//                }
//            }
//            else if (tokens[0] == "rename" && tokens.size() > 1 && tokens[1] == "node") {
//                if (tokens.size() != 4) {
//                    cout << "Error: Usage: rename node <old_name> <new_name>" << endl;
//                    continue;
//                }
//                circuit.renameNode(tokens[2], tokens[3]);
//                cout << "Renamed node " << tokens[2] << " to " << tokens[3] << endl;
//            }
//            else if (tokens[0] == "list") {
//                if (tokens.size() < 2) {
//                    cout << "Error: Usage: list nodes|elements [type]" << endl;
//                    continue;
//                }
//
//                if (tokens[1] == "nodes") {
//                    auto nodes = circuit.listNodes();
//                    cout << "Nodes:" << endl;
//                    for (const auto& node : nodes) {
//                        cout << "  " << node << endl;
//                    }
//                }
//                else if (tokens[1] == "elements") {
//                    string type = tokens.size() > 2 ? tokens[2] : "";
//                    auto elements = circuit.listElements(type);
//                    cout << "Elements" << (type.empty() ? "" : " (" + type + ")") << ":" << endl;
//                    for (const auto& el : elements) {
//                        cout << "  " << el << endl;
//                    }
//                }
//                else {
//                    cout << "Error: Unknown list type " << tokens[1] << endl;
//                }
//            }
//            else if (tokens[0] == "analyze") {
//                if (tokens.size() < 2) {
//                    cout << "Error: Usage: analyze dc|transient ..." << endl;
//                    continue;
//                }
//
//                if (tokens[1] == "dc") {
//                    auto results = circuit.analyzeDC();
//                    cout << "DC Analysis Results:" << endl;
//                    for (const auto& pair : results) {
//                        cout << "  Node " << pair.first->getName() << ": " << pair.second << " V" << endl;
//                    }
//                }
//                else if (tokens[1] == "transient") {
//                    if (tokens.size() < 4) {
//                        cout << "Error: Usage: analyze transient <t_step> <t_stop> [t_start]" << endl;
//                        continue;
//                    }
//                    double tStep =  stod(tokens[2]);
//                    double tStop =  stod(tokens[3]);
//                    double tStart = tokens.size() > 4 ? stod(tokens[4]) : 0.0;
//
//                    auto results = circuit.analyzeTransient(tStep, tStop, tStart);
//                    cout << "Transient Analysis Results (last time point):" << endl;
//                    for (const auto& pair : results.back()) {
//                        cout << "  Node " << pair.first->getName() << ": " << pair.second << " V" << endl;
//                    }
//                }
//                else {
//                    cout << "Error: Unknown analysis type " << tokens[1] << endl;
//                }
//            }
//            else if (tokens[0] == "exit") {
//                break;
//            }
//            else {
//               cout << "Error: Syntax error" << tokens[0] << endl;
//            }
//        } catch (const CircuitError& e) {
//            cout << "Circuit Error: " << e.what() << endl;
//        } catch (const exception& e) {
//            cout << "Error: " << e.what() << endl;
//        }

        SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
        SDL_RenderClear(renderer);
        btn.renderButton();
        frame.renderframe();
    }

    TTF_CloseFont(font);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();
    return 0;
}
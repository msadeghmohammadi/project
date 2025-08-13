#include <iostream>
#include "eleman.h"
#include <string>
#include <vector>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_mixer.h>
#include <fstream>

using namespace std;
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
            "Fullscreen App",
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
    if (!font) {
        cerr << "Font Error: " << TTF_GetError() << "";
    }

    Mix_Music* bgMusic = Mix_LoadMUS("downwithisrael.mp3");
    if (!bgMusic) {
        cerr << "Failed to load music: " << Mix_GetError() << "\n";
    }
    else {
        Mix_PlayMusic(bgMusic, -1);
    }

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

        } else if (option == "AC Sweep") {

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
                // انتخاب منو هندل شد
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
        }
        SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
        SDL_RenderClear(renderer);
        signalBtn.renderButton();
        musicBtn.renderButton();
        int w, h;
        SDL_GetRendererOutputSize(renderer, &w, &h);
        tBtn.renderInputs(renderer, w, h);
        tBtn.renderIfReady(renderer);
        frame.renderframe();
        menuBtn.renderMenu();
        SDL_RenderPresent(renderer);
    }

    Mix_FreeMusic(bgMusic);
    TTF_CloseFont(font);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    Mix_CloseAudio();
    IMG_Quit();
    TTF_Quit();
    SDL_Quit();
    return 0;
}
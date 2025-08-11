#include <iostream>
#include "eleman.h"
#include <string>
#include <memory>
#include <algorithm>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>
#include <fstream>

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

class signal {
private:
    string fileLocation;
    ifstream file;
    int chunkSize;
    vector<pair<double,double>> currentChunk;
    int probeNodeIndex = -1;

public:
    signal(const string& loc, int chunk, int nodeIndex)
            : fileLocation(loc), chunkSize(chunk), probeNodeIndex(nodeIndex) {}

    void evaluateChunkSize() {
        file.open(fileLocation);
        if (!file.is_open()) throw runtime_error("Cannot open signal file");
        file.clear();
        file.seekg(0, ios::beg);
    }

    bool readNextChunk() {
        currentChunk.clear();
        double t, v;
        int count = 0;
        while (count < chunkSize && (file >> t >> v)) {
            currentChunk.push_back({t, v});
            count++;
        }
        return !currentChunk.empty();
    }

    void plotNextChunk(SDL_Renderer* renderer, int width, int height) {
        if (currentChunk.empty()) return;

        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);

        double minV = currentChunk[0].second;
        double maxV = minV;
        for (auto& p : currentChunk) {
            minV = min(minV, p.second);
            maxV = max(maxV, p.second);
        }
        double rangeV = (maxV - minV == 0) ? 1 : (maxV - minV);

        for (size_t i = 1; i < currentChunk.size(); i++) {
            int x1 = (i - 1) * width / chunkSize;
            int y1 = height - static_cast<int>((currentChunk[i - 1].second - minV) / rangeV * height);
            int x2 = i * width / chunkSize;
            int y2 = height - static_cast<int>((currentChunk[i].second - minV) / rangeV * height);
            SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
        }
    }
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
    TestButton btn(renderer, "signal", 0, 0, 150, 50, font);

    double tStart   = 0.0;     // زمان شروع شبیه‌سازی
    double tStop    = 5e-3;    // زمان پایان شبیه‌سازی (اینجا 5 میلی‌ثانیه)
    double tStep    = 1e-6;

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
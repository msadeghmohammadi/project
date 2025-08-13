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
        std::cerr << "SDL Init Error: " << SDL_GetError() << std::endl;
        return;
    }
    if (TTF_Init() < 0) {
        std::cerr << "TTF Init Error: " << TTF_GetError() << std::endl;
        SDL_Quit();
        return;
    }
    if (IMG_Init(IMG_INIT_PNG) == 0) {
        std::cerr << "IMG Init Error: " << IMG_GetError() << std::endl;
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
        std::cerr << "CreateWindow Error: " << SDL_GetError() << std::endl;
        IMG_Quit();
        TTF_Quit();
        SDL_Quit();
        return;
    }

    SDL_Renderer* splashRenderer = SDL_CreateRenderer(splashWindow, -1, SDL_RENDERER_ACCELERATED);
    if (!splashRenderer) {
        std::cerr << "CreateRenderer Error: " << SDL_GetError() << std::endl;
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
        std::cerr << "Failed to load splash image: " << IMG_GetError() << std::endl;
    }

    TTF_Font* splashFont = TTF_OpenFont("ITCBLKAD.ttf", 36);
    if (!splashFont) {
        std::cerr << "Font Load Error: " << TTF_GetError() << std::endl;
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

    // رندر کردن splash screen
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

    SDL_Delay(3000);  // صبر 3 ثانیه

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

    void renderButton() {
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
        isOpen = !isOpen; // باز و بسته کردن منو
    }

    void renderMenu() {
        // --- کشیدن دکمه اصلی ---
        SDL_SetRenderDrawColor(renderer, 100, 100, 200, 255); // پس‌زمینه دکمه
        SDL_RenderFillRect(renderer, &rect);

        // متن دکمه اصلی
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

        // --- کشیدن آیتم‌ها اگر باز باشند ---
        if (isOpen) {
            int itemHeight = rect.h;
            for (size_t i = 0; i < items.size(); ++i) {
                SDL_Rect itemRect = {rect.x, rect.y + rect.h * (int)(i + 1), rect.w, itemHeight};
                SDL_SetRenderDrawColor(renderer, 70, 70, 150, 255); // پس‌زمینه آیتم
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
        if (isClick(e)) { // کلیک روی دکمه اصلی
            doAction();
            return true;
        }
        if (isOpen && e.type == SDL_MOUSEBUTTONDOWN) { // کلیک روی آیتم‌ها
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
        std::cerr << "Icon Load Error: " << IMG_GetError() << "\n";
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::cerr << "CreateRenderer Error: " << SDL_GetError() << "";
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
        std::cerr << "Font Error: " << TTF_GetError() << "";
    }

    Mix_Music* bgMusic = Mix_LoadMUS("downwithisrael.mp3");
    if (!bgMusic) {
        std::cerr << "Failed to load music: " << Mix_GetError() << "\n";
    }
    else {
        Mix_PlayMusic(bgMusic, -1);
    }

    class TestButton : public Button {
    public:
        TestButton(SDL_Renderer* r, const string& n, int x, int y, int w, int h, TTF_Font* f)
                : Button(r, n, x, y, w, h, f) {}
        void doAction() override {
            std::cout << "Button Clicked: " << getName() << "\n";
        }
    };
    TestButton btn(renderer, "signal", 100, 100, 150, 50, font);

    MusicToggleButton musicBtn(renderer, "Toggle Music", 100, 200, 200, 50, font, bgMusic);

    SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP);

    bool running = true;
    bool f11Pressed = false;
    vector<string> elements = {
            "Voltage Source (v)",
            "Current Source (c)",
            "Diode (d)",
            "Resistor (r)",
            "Ground (gnd)",
            "Capacitor (cap)",
            "Inductor (ind)"
    };

    MenuButton menuBtn(renderer, "Menu", dm.w - 150, 20, 120, 40, font, elements);

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

            if (btn.isClick(e)) {
                btn.doAction();
            }
            if (musicBtn.isClick(e)) {
                musicBtn.doAction();
            }
        }
        SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
        SDL_RenderClear(renderer);

        btn.renderButton();
        musicBtn.renderButton();
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

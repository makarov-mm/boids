// Boids 3D / Swarm Intelligence - native Win32 + OpenGL viewer.
//
// No GLFW / GLAD: the project builds with nothing but the Windows SDK.
//
// Controls:
//   left-drag  orbit camera         mouse wheel  zoom
//   Space      pause                Up/Down      sim speed (substeps/frame)
//   O          obstacle on/off      P            predators on/off
//   R          reset                H            hide HUD       Esc  quit

#include "gl_api.hpp"   // first: windows.h + OpenGL

#include <windowsx.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <format>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "boids.hpp"
#include "mat4.hpp"
#include "renderer.hpp"

namespace {
constexpr int   CLIENT_W     = 1280;   // initial client size at 96 DPI
constexpr int   CLIENT_H     = 800;
constexpr DWORD WINDOW_STYLE = WS_OVERLAPPEDWINDOW;
constexpr int   BOIDS        = 12000;
constexpr int   PREDATORS    = 3;      // P toggles between this and none
constexpr int   MAX_SUBSTEPS = 8;

// WGL_ARB_create_context / WGL_EXT_swap_control (not in the Windows SDK headers).
constexpr int WGL_CONTEXT_MAJOR_VERSION_ARB    = 0x2091;
constexpr int WGL_CONTEXT_MINOR_VERSION_ARB    = 0x2092;
constexpr int WGL_CONTEXT_PROFILE_MASK_ARB     = 0x9126;
constexpr int WGL_CONTEXT_CORE_PROFILE_BIT_ARB = 0x0001;
using PFNWGLCREATECONTEXTATTRIBSARBPROC = HGLRC(WINAPI*)(HDC, HGLRC, const int*);
using PFNWGLSWAPINTERVALEXTPROC         = BOOL(WINAPI*)(int);

BoidsParams simulationParams() {
    BoidsParams p;
    p.boids     = BOIDS;
    p.predators = PREDATORS;
    return p;
}

std::wstring widen(std::string_view utf8) {
    const int n = MultiByteToWideChar(CP_UTF8, 0, utf8.data(), static_cast<int>(utf8.size()), nullptr, 0);
    std::wstring w(static_cast<std::size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8.data(), static_cast<int>(utf8.size()), w.data(), n);
    return w;
}
} // namespace

// =============================================================================
//  The window, its OpenGL context, input and the frame loop.
// =============================================================================
class Viewer {
public:
    explicit Viewer(HINSTANCE inst);
    ~Viewer();

    Viewer(const Viewer&)            = delete;
    Viewer& operator=(const Viewer&) = delete;

    void run(int showCmd);

private:
    static LRESULT CALLBACK windowProc(HWND h, UINT msg, WPARAM wp, LPARAM lp);
    [[nodiscard]] std::optional<LRESULT> onMessage(UINT msg, WPARAM wp, LPARAM lp);   // nullopt: default handling
    void onKey(WPARAM key);

    void createWindow(HINSTANCE inst);
    void createContext();
    void release() noexcept;   // idempotent: GL objects, context, window
    void onResize(int width, int height);
    void frame();
    void updateFps();
    [[nodiscard]] std::vector<std::string> hudLines() const;
    [[nodiscard]] float dpiScale() const noexcept { return static_cast<float>(GetDpiForWindow(hwnd_)) / USER_DEFAULT_SCREEN_DPI; }

    HWND  hwnd_ = nullptr;
    HDC   dc_   = nullptr;
    HGLRC glrc_ = nullptr;

    Boids                   sim_{ simulationParams() };
    OrbitCamera             camera_;
    std::optional<Renderer> renderer_;

    bool  dragging_ = false, paused_ = false, showHud_ = true, quit_ = false;
    POINT lastMouse_{};
    int   substeps_ = 1;

    double                                fps_    = 0;
    int                                   frames_ = 0;
    std::chrono::steady_clock::time_point fpsStart_ = std::chrono::steady_clock::now();

    std::exception_ptr pendingError_;   // thrown inside the window procedure, rethrown by run()
};

Viewer::Viewer(HINSTANCE inst) {
    try {
        createWindow(inst);
        createContext();
        renderer_.emplace();
    } catch (...) {
        release();
        throw;
    }
    renderer_->setWorld(sim_.params());

    const BoidsParams& p = sim_.params();
    const Vec3 ext = p.boundsMax - p.boundsMin;
    camera_.target   = (p.boundsMin + p.boundsMax) * 0.5f;
    camera_.distance = std::max({ ext.x, ext.y, ext.z }) * 1.7f;

    RECT rc{};
    GetClientRect(hwnd_, &rc);
    onResize(rc.right - rc.left, rc.bottom - rc.top);
}

Viewer::~Viewer() {
    release();
}

void Viewer::release() noexcept {
    renderer_.reset();   // its GL objects need the context
    if (glrc_) {
        wglMakeCurrent(nullptr, nullptr);
        wglDeleteContext(glrc_);
        glrc_ = nullptr;
    }
    if (dc_) {
        ReleaseDC(hwnd_, dc_);
        dc_ = nullptr;
    }
    if (hwnd_) {
        SetWindowLongPtrW(hwnd_, GWLP_USERDATA, 0);   // no more messages to this object
        DestroyWindow(hwnd_);
        hwnd_ = nullptr;
    }
}

void Viewer::createWindow(HINSTANCE inst) {
    const WNDCLASSEXW wc{
        .cbSize        = sizeof(WNDCLASSEXW),
        .style         = CS_OWNDC,   // OpenGL keeps one device context
        .lpfnWndProc   = windowProc,
        .hInstance     = inst,
        .hCursor       = LoadCursorW(nullptr, IDC_ARROW),
        .lpszClassName = L"BoidsWindow",
    };
    if (!RegisterClassExW(&wc)) throw std::runtime_error("RegisterClassEx failed");

    // 1280 x 800 at 96 DPI, scaled to the system DPI; WM_DPICHANGED follows the monitor.
    const UINT dpi = GetDpiForSystem();
    RECT rect{ 0, 0, MulDiv(CLIENT_W, static_cast<int>(dpi), USER_DEFAULT_SCREEN_DPI),
                     MulDiv(CLIENT_H, static_cast<int>(dpi), USER_DEFAULT_SCREEN_DPI) };
    AdjustWindowRectExForDpi(&rect, WINDOW_STYLE, FALSE, 0, dpi);

    hwnd_ = CreateWindowExW(0, wc.lpszClassName, L"Boids 3D", WINDOW_STYLE, CW_USEDEFAULT, CW_USEDEFAULT,
                            rect.right - rect.left, rect.bottom - rect.top, nullptr, nullptr, inst, this);
    if (!hwnd_) throw std::runtime_error("CreateWindowEx failed");
}

void Viewer::createContext() {
    dc_ = GetDC(hwnd_);
    if (!dc_) throw std::runtime_error("GetDC failed");

    const PIXELFORMATDESCRIPTOR pfd{
        .nSize        = sizeof(PIXELFORMATDESCRIPTOR),
        .nVersion     = 1,
        .dwFlags      = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
        .iPixelType   = PFD_TYPE_RGBA,
        .cColorBits   = 32,
        .cDepthBits   = 24,
        .cStencilBits = 8,
        .iLayerType   = PFD_MAIN_PLANE,
    };
    const int format = ChoosePixelFormat(dc_, &pfd);
    if (format == 0 || !SetPixelFormat(dc_, format, &pfd)) throw std::runtime_error("No suitable pixel format for OpenGL");

    // A legacy context first: wglGetProcAddress needs a current one. Then
    // the 3.3 core profile the shaders are written for, when the driver has
    // WGL_ARB_create_context (all current ones do); else keep the legacy one.
    glrc_ = wglCreateContext(dc_);
    if (!glrc_ || !wglMakeCurrent(dc_, glrc_)) throw std::runtime_error("Could not create an OpenGL context");
    if (const auto createContextAttribs = reinterpret_cast<PFNWGLCREATECONTEXTATTRIBSARBPROC>(
            wglGetProcAddress("wglCreateContextAttribsARB"))) {
        constexpr int attribs[] = { WGL_CONTEXT_MAJOR_VERSION_ARB, 3, WGL_CONTEXT_MINOR_VERSION_ARB, 3,
                                    WGL_CONTEXT_PROFILE_MASK_ARB, WGL_CONTEXT_CORE_PROFILE_BIT_ARB, 0 };
        if (const HGLRC core = createContextAttribs(dc_, nullptr, attribs)) {
            if (wglMakeCurrent(dc_, core)) {
                wglDeleteContext(glrc_);
                glrc_ = core;
            } else {
                wglDeleteContext(core);
                wglMakeCurrent(dc_, glrc_);
            }
        }
    }

    GLint major = 0, minor = 0;   // stay 0 below OpenGL 3.0
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);
    if (major < 3 || (major == 3 && minor < 3)) {
        const auto* version = reinterpret_cast<const char*>(glGetString(GL_VERSION));
        throw std::runtime_error(std::format("OpenGL 3.3 is required; this driver offers {}.\n"
                                             "Run on a machine with real GPU drivers (not over Remote Desktop).",
                                             version ? version : "unknown"));
    }
    if (const char* missing = gl::load())
        throw std::runtime_error(std::format("Required OpenGL function is missing: {}", missing));

    if (const auto swapInterval = reinterpret_cast<PFNWGLSWAPINTERVALEXTPROC>(wglGetProcAddress("wglSwapIntervalEXT")))
        swapInterval(1);   // vsync
}

// =============================================================================
//  Frame loop
// =============================================================================
void Viewer::run(int showCmd) {
    ShowWindow(hwnd_, showCmd);
    UpdateWindow(hwnd_);

    MSG msg{};
    while (!quit_) {
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) quit_ = true;
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        if (pendingError_) std::rethrow_exception(std::exchange(pendingError_, nullptr));
        if (quit_) break;

        // Minimized: nothing to show, so sleep until a message arrives
        // instead of spinning the simulation flat out.
        if (IsIconic(hwnd_)) { WaitMessage(); continue; }

        frame();
    }
}

void Viewer::frame() {
    if (!paused_)
        for (int i = 0; i < substeps_; ++i) sim_.step();

    renderer_->drawScene(camera_, sim_);
    if (showHud_) renderer_->drawHud(hudLines(), dpiScale());
    SwapBuffers(dc_);

    updateFps();
}

void Viewer::updateFps() {
    ++frames_;
    const auto   now     = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - fpsStart_).count();
    if (elapsed < 0.5) return;

    fps_      = frames_ / elapsed;
    frames_   = 0;
    fpsStart_ = now;
    const std::wstring title = std::format(L"Boids 3D  |  {} boids  |  {:.0f} fps  |  x{}{}",
                                           sim_.count(), fps_, substeps_, paused_ ? L"  [PAUSED]" : L"");
    SetWindowTextW(hwnd_, title.c_str());
}

std::vector<std::string> Viewer::hudLines() const {
    return {
        "Boids 3D / Swarm Intelligence",
        std::format("fps {:.0f}", fps_),
        std::format("{} boids   {} predators{}", sim_.count(), sim_.predatorPos().size(),
                    sim_.params().obstacle ? "   obstacle" : ""),
        std::format("speed x{}{}", substeps_, paused_ ? "   [PAUSED]" : ""),
        "",
        "drag / wheel  orbit / zoom",
        "space    pause",
        "up / dn  sim speed",
        "O   obstacle on/off",
        "P   predators on/off",
        "R   reset    H  hide HUD",
        "esc      quit",
    };
}

// =============================================================================
//  Messages
// =============================================================================
LRESULT CALLBACK Viewer::windowProc(HWND h, UINT msg, WPARAM wp, LPARAM lp) {
    if (msg == WM_NCCREATE) {
        const auto* cs = reinterpret_cast<const CREATESTRUCTW*>(lp);
        SetWindowLongPtrW(h, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(cs->lpCreateParams));
    }
    if (auto* self = reinterpret_cast<Viewer*>(GetWindowLongPtrW(h, GWLP_USERDATA))) {
        // An exception must not unwind through user32: park it for run().
        try {
            if (const auto result = self->onMessage(msg, wp, lp)) return *result;
        } catch (...) {
            self->pendingError_ = std::current_exception();
            return 0;
        }
    }
    return DefWindowProcW(h, msg, wp, lp);
}

std::optional<LRESULT> Viewer::onMessage(UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_SIZE:
        onResize(LOWORD(lp), HIWORD(lp));
        return 0;
    case WM_DPICHANGED: {   // moved to a monitor with another scale: take the suggested size
        const auto* r = reinterpret_cast<const RECT*>(lp);
        SetWindowPos(hwnd_, nullptr, r->left, r->top, r->right - r->left, r->bottom - r->top, SWP_NOZORDER | SWP_NOACTIVATE);
        return 0;
    }
    case WM_LBUTTONDOWN:
        dragging_  = true;
        lastMouse_ = { GET_X_LPARAM(lp), GET_Y_LPARAM(lp) };
        SetCapture(hwnd_);
        return 0;
    case WM_LBUTTONUP:
        dragging_ = false;
        ReleaseCapture();
        return 0;
    case WM_CAPTURECHANGED:
        dragging_ = false;
        return 0;
    case WM_MOUSEMOVE: {
        const POINT p{ GET_X_LPARAM(lp), GET_Y_LPARAM(lp) };
        if (dragging_) camera_.rotate(static_cast<float>(p.x - lastMouse_.x), static_cast<float>(p.y - lastMouse_.y));
        lastMouse_ = p;
        return 0;
    }
    case WM_MOUSEWHEEL:
        camera_.zoom(static_cast<float>(GET_WHEEL_DELTA_WPARAM(wp)) / WHEEL_DELTA);
        return 0;
    case WM_KEYDOWN:
        onKey(wp);
        return 0;
    case WM_CLOSE:
        quit_ = true;   // the loop ends; the destructor tears down GL, then the window
        return 0;
    default:
        return std::nullopt;
    }
}

void Viewer::onResize(int width, int height) {
    if (width <= 0 || height <= 0) return;   // 0 x 0 while minimized
    camera_.aspect = static_cast<float>(width) / static_cast<float>(height);
    if (renderer_) renderer_->resize(width, height);
}

void Viewer::onKey(WPARAM key) {
    switch (key) {
    case VK_SPACE:  paused_ = !paused_; break;
    case VK_UP:     substeps_ = std::min(substeps_ + 1, MAX_SUBSTEPS); break;
    case VK_DOWN:   substeps_ = std::max(substeps_ - 1, 1); break;
    case 'O':
        sim_.setObstacle(!sim_.params().obstacle);
        renderer_->setWorld(sim_.params());
        break;
    case 'P':       sim_.setPredators(sim_.predatorPos().empty() ? PREDATORS : 0); break;
    case 'R':       sim_.reset(); break;
    case 'H':       showHud_ = !showHud_; break;
    case VK_ESCAPE: quit_ = true; break;
    default:        break;
    }
}

int WINAPI wWinMain(HINSTANCE inst, HINSTANCE, PWSTR, int showCmd) {
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);   // Windows 10 1703+

    try {
        Viewer viewer(inst);
        viewer.run(showCmd);
        return 0;
    } catch (const std::exception& e) {
        MessageBoxW(nullptr, widen(e.what()).c_str(), L"Boids 3D", MB_ICONERROR | MB_OK);
        return 1;
    }
}

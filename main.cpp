/**
 * main.cpp — Point d'entrée RLTool v6.2, C++20
 *
 * Nouveautés v6 :
 *   - ConfigLoader           : offsets chargés depuis config.txt (O(1) lookup)
 *   - initialize_console_ui(): titre, curseur masqué, High Contrast Mode
 *   - ScopedCursorRestore    : RAII → curseur restauré même sur Ctrl+C
 *   - ExternalDataManager    : RAII via ProcessHandle / SnapshotHandle
 *
 * Nouveautés v6.1 (ViGEm) :
 *   - VigemBridge            : contrôleur Xbox360 virtuel RAII (ViGEmBus)
 *   - Simulation de dérive analogique ±0.01 % sur steer/pitch (120 Hz)
 *
 * Nouveautés v6.2 (sécurité & interactivité) :
 *   - Killswitch physique F10 : GetAsyncKeyState(VK_F10) dans game_loop().
 *     Si F10 pressé → envoie PlayerInput{} (commandes nulles) à ViGEm,
 *     force la FSM en IDLE, sort de la boucle proprement.
 *   - FSM override depuis ImGui : debug->fsm_override lu à chaque tick ;
 *     si >= 0, appelle fsm.force_state() au lieu de fsm.update().
 *   - Killswitch UI : debug->bot_active == false → PlayerInput{} à ViGEm,
 *     FSM continue de tourner (reprise immédiate au RESUME).
 *   - EDM status : debug->edm_connected écrit après chaque read_frame().
 *   - Tuning live steer : input.steer *= tuning_steer_sens, clampé [-1,+1].
 *
 * Linkage Windows OBLIGATOIRE :
 *   -lpsapi        (EnumProcessModules, GetModuleBaseNameA)
 *   -lViGEmClient  (vigem_alloc, vigem_connect, …)
 *
 * Compilation headless + ViGEm (recommandé production) :
 *   g++ -std=c++20 -O2 -DHEADLESS_MODE \
 *       -I"C:/ViGEmClient/include" \
 *       main.cpp -o rl_tool.exe \
 *       -lpsapi -L"C:/ViGEmClient/lib/x64" -lViGEmClient
 *
 * Avec Dear ImGui (SDL2) + ViGEm :
 *   g++ -std=c++20 -O2 \
 *       -I"C:/ViGEmClient/include" \
 *       main.cpp imgui/*.cpp \
 *       imgui/backends/imgui_impl_sdl2.cpp  \
 *       imgui/backends/imgui_impl_opengl3.cpp \
 *       $(sdl2-config --cflags --libs) -lGL \
 *       -lpsapi -L"C:/ViGEmClient/lib/x64" -lViGEmClient \
 *       -o rl_tool.exe
 *
 * Pour désactiver ViGEm (tests sans driver) :
 *   Compiler avec -DDISABLE_VIGEM
 */

// ── Standard ──────────────────────────────────────────────
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

// ── Modules du projet ─────────────────────────────────────
// external_data_manager.hpp inclut config_loader.hpp, rl_fsm.hpp, rl_bot.hpp
#include "external_data_manager.hpp"
#include "console_ui.hpp"

// ── ViGEm (désactivable via -DDISABLE_VIGEM) ──────────────
#ifndef DISABLE_VIGEM
#  include "vigem_bridge.hpp"
#endif

#ifndef HEADLESS_MODE
#  include "rl_imgui_panel.hpp"
#endif

// ─────────────────────────────────────────────────────────
// § 1. Signal d'arrêt global (Ctrl+C → arrêt propre)
// ─────────────────────────────────────────────────────────

namespace {
    std::atomic<bool> g_running{true};
}

static void on_sigint(int) noexcept { g_running.store(false); }

// ─────────────────────────────────────────────────────────
// § 2. Constructeur de DecisionContext
// ─────────────────────────────────────────────────────────

[[nodiscard]] static DecisionContext
build_context(const GameTickPacket& pkt) noexcept
{
    const float dx = pkt.ball.position.x - pkt.player.position.x;
    const float dy = pkt.ball.position.y - pkt.player.position.y;

    constexpr float OWN_GOAL_Y = -5120.f;
    const float gdx = pkt.ball.position.x;
    const float gdy = pkt.ball.position.y - OWN_GOAL_Y;

    const float dot = pkt.player.forward.x * dx
                    + pkt.player.forward.y * dy;

    return DecisionContext {
        .ball_dist_sq          = dx*dx + dy*dy,
        .ball_dist_to_goal_sq  = gdx*gdx + gdy*gdy,
        .boost_pct             = pkt.player.boost_amount,
        .ball_is_behind_car    = dot < 0.f,
        .teammate_attacking    = false,
        .tick_id               = pkt.tick_id,
    };
}

// ─────────────────────────────────────────────────────────
// § 3. Simulateur (fallback / tests)
// ─────────────────────────────────────────────────────────

[[nodiscard]] static GameTickPacket simulate_tick(uint64_t tick_id) noexcept
{
    const float t = static_cast<float>(tick_id) / 120.f;
    return GameTickPacket {
        .ball = BallState {
            .position = { 500.f * std::cos(t * 0.4f),
                          200.f * std::sin(t * 0.3f),
                          92.75f + 50.f * std::abs(std::sin(t * 2.f)) },
            .velocity = { -60.f, 80.f, 20.f },
        },
        .player = CarState {
            .position     = { 0.f, -1000.f, 17.f },
            .velocity     = { 0.f, 400.f, 0.f },
            .forward      = { 0.f, 1.f, 0.f },
            .boost_amount = 40.f + 20.f * std::sin(t),
            .on_ground    = true,
        },
        .game_time_remaining = 300.f - t,
        .tick_id             = tick_id,
    };
}

// ─────────────────────────────────────────────────────────
// § 4. Thread de jeu — 120 Hz
//
//  Ordre d'exécution dans le hot path (v6.2) :
//    1. Killswitch physique F10 (début de tick — priorité absolue)
//    2. Lecture EDM (ou fallback simulé) → edm_ok
//    3. build_context()
//    4. FSM : fsm.force_state() (override UI) ou fsm.update(ctx)
//    5. bot.compute(pkt, ctx)   → PlayerInput
//    6. Killswitch UI (bot_active == false → PlayerInput{})
//    7. Tuning live : steer *= tuning_steer_sens
//    8. [ViGEm] bridge.update(input)
//    9. debug->write(…, edm_ok)
//   10. sleep_until(next_tick)
//
//  Killswitch F10 (§ 4.A) :
//    GetAsyncKeyState(VK_F10) & 0x8000 détecte la touche enfoncée.
//    On envoie d'abord PlayerInput{} à ViGEm (relâche toutes les touches),
//    on force la FSM en IDLE, puis on sort de la boucle avec break.
//    La destruction de vigem_bridge (RAII, après join()) débranche
//    proprement le contrôleur virtuel du driver ViGEmBus.
//    Disponible uniquement sur Windows (_WIN32) et si ViGEm est actif.
//
//  FSM override UI (§ 4.B) :
//    debug->fsm_override (int8_t) est lu en acquire une fois par tick.
//    -1 → update() normal. 0..3 → force_state(BotState). Thread-safe.
//
//  Killswitch UI (§ 4.C) :
//    debug->bot_active == false → commandes nulles à ViGEm.
//    FSM et BotController continuent en arrière-plan (reprise immédiate).
// ─────────────────────────────────────────────────────────

static void game_loop(
    StateMachine&                    fsm,
    BotController&                   bot,
    ExternalDataManager&             edm,
    std::shared_ptr<SharedDebugData> debug
#ifndef DISABLE_VIGEM
    , VigemBridge&                   bridge
#endif
) noexcept
{
    using Clock  = std::chrono::steady_clock;
    using Micros = std::chrono::microseconds;

    constexpr auto TICK_PERIOD = Micros{8'333};
    uint64_t       tick_id     = 0;
    uint32_t       fallback_streak = 0;

    auto next_tick = Clock::now();

    while (g_running.load(std::memory_order_relaxed)) {
        const auto t0 = Clock::now();

        // ── § 4.A Killswitch physique F10 ────────────────
        //
        //   Vérification en tout début de tick, avant toute autre
        //   opération. GetAsyncKeyState renvoie bit 15 à 1 si la
        //   touche est actuellement enfoncée.
        //
        //   Comportement :
        //     1. Envoie PlayerInput{} à ViGEm (relâche tout)
        //     2. Force la FSM en IDLE
        //     3. Sort de la boucle → destruction RAII de vigem_bridge
        //        → vigem_target_remove + vigem_free dans le destructeur
        //
        //   Disponible uniquement sous Windows et si ViGEm est actif.
#if defined(_WIN32) && !defined(DISABLE_VIGEM)
        if (GetAsyncKeyState(VK_F10) & 0x8000) {
            std::puts("[KILLSWITCH] F10 detecte — arret d'urgence ViGEm.");
            bridge.update(PlayerInput{});   // relâche toutes les touches
            fsm.force_state(BotState::IDLE);
            g_running.store(false);         // arrête aussi render_loop
            break;
        }
#endif

        // ── 1. Lecture EDM (ou fallback simulé) ───────────
        bool edm_ok = false;

#ifndef FORCE_SIMULATE
        const GameTickPacket pkt = external_tick(edm, tick_id, simulate_tick);
        edm_ok = edm.last_error().empty();

        if (!edm_ok) {
            ++fallback_streak;
            if (fallback_streak == 1 || fallback_streak % 600 == 0) {
                std::printf("[EDM] Fallback actif (tick %llu) : %s\n",
                            static_cast<unsigned long long>(tick_id),
                            edm.last_error().c_str());
            }
        } else {
            if (fallback_streak > 0) {
                std::printf("[EDM] Connexion retablie au tick %llu.\n",
                            static_cast<unsigned long long>(tick_id));
                fallback_streak = 0;
            }
        }
#else
        const GameTickPacket pkt = simulate_tick(tick_id);
        (void)edm;
        (void)fallback_streak;
        // edm_ok reste false → indicateur rouge dans le panel
#endif

        ++tick_id;

        // ── 2. Contexte de décision ───────────────────────
        const DecisionContext ctx = build_context(pkt);

        // ── § 4.B FSM : update libre ou état forcé par UI ─
        //
        //   fsm_override est lu une seule fois par tick (acquire).
        //   -1  → fsm.update(ctx) : table de transitions normale.
        //   0..3 → fsm.force_state() : état figé, gardes ignorées.
        //   Les constexpr _COUNT garantit la validité de l'indice.
#ifndef HEADLESS_MODE
        {
            const int8_t ov = debug->fsm_override.load(std::memory_order_acquire);
            if (ov >= 0 && ov < static_cast<int8_t>(BotState::_COUNT))
                fsm.force_state(static_cast<BotState>(ov));
            else
                fsm.update(ctx);
        }
#else
        fsm.update(ctx);
#endif

        // ── 3. Calcul de l'input par le bot ───────────────
        PlayerInput input = bot.compute(pkt, ctx);

        // ── § 4.C Killswitch UI (bot_active) ─────────────
        //
        //   Priorité absolue avant ViGEm et avant le tuning.
        //   FSM et BotController continuent → reprise immédiate au RESUME.
        //   En mode HEADLESS, le flag autonomous (alias bot_active) du
        //   panneau n'existe pas → on utilise le flag hérité v6.1.
#ifndef HEADLESS_MODE
        if (!debug->bot_active.load(std::memory_order_acquire))
            input = PlayerInput{};
#else
        // Rétrocompatibilité v6.1 headless : autonomous ignoré en build headless
#endif

        // ── 4. Tuning live : sensibilité steer ────────────
        //
        //   Appliqué APRÈS le killswitch.
        //   math_utils::clamp préserve [-1, +1].
#ifndef HEADLESS_MODE
        {
            const float sens = debug->tuning_steer_sens.load(std::memory_order_relaxed);
            input.steer = math_utils::clamp(input.steer * sens, -1.f, 1.f);
        }
#endif

        // ── 5. ViGEm : émission HID 120 Hz ───────────────
        //
        //   bridge.update() no-op si valid() == false.
        //   Zéro allocation — hot path inchangé.
#ifndef DISABLE_VIGEM
        bridge.update(input);
#endif

        // Production : rlbot_interface.set_input(input);
        (void)input;

        // ── 6. Debug ImGui ────────────────────────────────
        const float tick_ms =
            std::chrono::duration_cast<Micros>(Clock::now() - t0).count()
            * 1e-3f;

#ifndef HEADLESS_MODE
        // edm_ok : 5e argument ajouté en v6.2 pour l'indicateur EDM
        debug->write(pkt, fsm.state(), input, tick_ms, edm_ok);
#else
        (void)tick_ms;
        (void)debug;
        (void)edm_ok;
#endif

        // ── 7. Synchronisation 120 Hz ─────────────────────
        next_tick += TICK_PERIOD;
        std::this_thread::sleep_until(next_tick);
    }

    std::puts("[game_loop] arrete proprement.");
}

// ─────────────────────────────────────────────────────────
// § 5. Thread de rendu ImGui (~60 Hz)
// ─────────────────────────────────────────────────────────

#ifndef HEADLESS_MODE
static void render_loop(ControlPanel& panel) noexcept
{
    using Clock  = std::chrono::steady_clock;
    using Millis = std::chrono::milliseconds;

    constexpr auto FRAME_PERIOD = Millis{16};
    auto           next_frame   = Clock::now();

    while (g_running.load(std::memory_order_relaxed)) {
        panel.draw();
        next_frame += FRAME_PERIOD;
        std::this_thread::sleep_until(next_frame);
    }
    std::puts("[render_loop] arrete proprement.");
}
#endif

// ─────────────────────────────────────────────────────────
// § 6. main()
// ─────────────────────────────────────────────────────────

int main()
{
    // ── RAII : restauration du curseur garantie même sur Ctrl+C ──
    // ScopedCursorRestore doit être la première variable locale déclarée
    // pour être la dernière détruite (LIFO) — le curseur est rendu
    // après que tous les autres composants ont été arrêtés.
    ScopedCursorRestore cursor_guard;

    // ── Initialisation de la console ──────────────────────
    // Masque le curseur, applique High Contrast Mode, fixe le titre.
    initialize_console_ui("RLTool v6.1 — Sensor Monitor | Telemetry Bridge | ViGEm HID");

    // ── Gestionnaire d'arrêt ──────────────────────────────
    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);

    // ── Bannière de démarrage ─────────────────────────────
#ifdef _WIN32
    {
        ScopedConsoleColor c(ConsoleColor::BRIGHT_CYAN);
        std::puts("=======================================================");
        std::puts("   RLTool v6.1 — Telemetry Bridge | ViGEm HID Bridge");
        std::puts("=======================================================");
    }
    console_log(ConsoleColor::BRIGHT_YELLOW, "INIT",
                "Chargement de la configuration...");
#else
    std::puts("=== RLTool v6.1 — Telemetry Bridge + ViGEm ===");
#endif

    // ── Chargement de la configuration ────────────────────
    //
    // config.txt (optionnel) : surcharge les offsets par défaut.
    // Si absent, make_default() est utilisé sans avertissement fatal.
    //
    // Exemple de config.txt :
    //   BALL_PTR=0x02A4B0E0
    //   BALL_POS=0x7C
    //   CAR_PTR=0x02A4C1F0
    //   CAR_BOOST=0x1A8
    //   TICK_HZ=120
    //   SCALE=1.0
    ConfigLoader cfg;
    const bool cfg_loaded = cfg.load("config.txt");

    if (cfg_loaded) {
        std::printf("[CFG] config.txt charge : %zu entrees.\n", cfg.size());
    } else {
        std::puts("[CFG] config.txt absent — offsets par defaut utilises.");
    }

    const MemoryOffsets offsets = cfg_loaded
        ? MemoryOffsets::from_config(cfg)
        : MemoryOffsets::make_default();

    // ── Initialisation des composants ─────────────────────
    ExternalDataManager edm(offsets);
    auto         debug = std::make_shared<SharedDebugData>();
    StateMachine fsm;
    BotController bot;

#ifndef HEADLESS_MODE
    ControlPanel panel(debug, fsm);
#endif

    // ── Initialisation ViGEm (RAII — se débranche à la destruction) ──
    //
    //   VigemBridge est déclaré après ExternalDataManager, FSM, BotController.
    //   Sa destruction se produit avant celle de ces objets (LIFO) — ordre
    //   garanti : le contrôleur virtuel est débranché AVANT que l'EDM et
    //   le bot ne soient détruits.
    //
    //   Si ViGEmBus n'est pas installé, valid() retourne false et le bridge
    //   est un no-op dans game_loop — aucun crash, aucune dépendance fatale.
#ifndef DISABLE_VIGEM
    VigemBridge vigem_bridge;
    if (vigem_bridge.valid()) {
        std::puts("[VIGEM] Controleur Xbox360 virtuel connecte (ViGEmBus OK).");
        std::puts("[VIGEM] Simulation de derive analogique : +/-0.01% @ 1.7 Hz.");
    } else {
        std::printf("[VIGEM] AVERTISSEMENT : bridge inactif — %s\n",
                    vigem_bridge.last_error().c_str());
        std::puts("[VIGEM] Le reste du systeme fonctionne normalement.");
    }
#else
    std::puts("[VIGEM] Desactive (-DDISABLE_VIGEM).");
#endif

    std::printf("[EDM] Cible : %s\n", offsets.process_name.c_str());
#ifdef FORCE_SIMULATE
    std::puts("[EDM] Mode FORCE_SIMULATE : lecture memoire desactivee.");
#endif
    std::puts("[INIT] FSM, BotController, ExternalDataManager : OK");
    std::puts("[v6.2] Killswitch F10, FSM override UI, tuning live, indicateur EDM : OK");
    std::puts("[INFO] Ctrl+C ou F10 pour arreter proprement.");
    std::puts("-------------------------------------------------------");

    // ── Lancement des threads ─────────────────────────────
    //
    //   game_thread capture vigem_bridge par référence.
    //   La durée de vie de vigem_bridge (stack de main) est garantie
    //   supérieure à celle de game_thread (join() ci-dessous).
    std::thread game_thread([&] {
        game_loop(fsm, bot, edm, debug
#ifndef DISABLE_VIGEM
            , vigem_bridge
#endif
        );
    });

#ifndef HEADLESS_MODE
    std::thread render_thread([&] {
        render_loop(panel);
    });
#endif

    // ── Attente de l'arrêt ────────────────────────────────
    game_thread.join();

#ifndef HEADLESS_MODE
    render_thread.join();
#endif

    std::puts("=== RLTool arrete proprement. ===");

    // Ordre de destruction (LIFO) :
    //   1. vigem_bridge  → vigem_target_remove + vigem_free
    //   2. bot, fsm, edm, debug
    //   3. cursor_guard  → restauration du curseur console
    return 0;
}

/**
 * rl_imgui_panel.hpp — Panneau ImGui pour le bot RLBot, C++20
 *
 * Nouveautés v6.2 :
 *   §1  SharedDebugData :
 *         + bot_active    : killswitch ViGEm (atomic<bool>) — alias de autonomous
 *         + edm_connected : statut connexion EDM (atomic<bool>)
 *         + fsm_override  : état forcé optionnel (atomic<int8_t>, -1 = auto)
 *         + tuning_steer_sens  : multiplicateur steer live (atomic<float>)
 *         + tuning_boost_low   : seuil BOOST_LOW live (atomic<float>)
 *         + tuning_close_sq_rt : distance CLOSE_SQ live (atomic<float>)
 *         + write() reçoit un 5e argument : bool edm_ok
 *   §4  ControlPanel::draw() :
 *         + draw_killswitch_section() : gros bouton STOP / RESUME rouge/vert
 *         + draw_mode_section()       : radio buttons FSM override + indicateur EDM
 *         + draw_tuning_section()     : sliders steer / BOOST_LOW / CLOSE_SQ
 *
 * Dépendances attendues (à inclure AVANT ce header) :
 *   - imgui.h
 *   - rl_fsm.hpp    (BotState, state_name, StateMachine, LogEntry)
 *   - rl_bot.hpp    (Vec3, GameTickPacket, PlayerInput, math_utils)
 *
 * Politique mémoire :
 *   - Aucune allocation dynamique dans draw() (hot path de rendu)
 *   - SharedDebugData protégé par std::atomic / relaxed ordering + seq_cst fence
 *   - Visualisation des vecteurs via ImDrawList (GPU immédiat)
 */

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <format>
#include <string_view>

#include "imgui.h"

// ─────────────────────────────────────────────────────────
// § 1. Données de debug partagées (thread-safe via atomics)
// ─────────────────────────────────────────────────────────

/**
 * Snapshot des données de debug écrit par le thread de jeu (120 Hz)
 * et lu par le thread de rendu (~60 Hz).
 *
 * Tous les champs sont écrits en memory_order_relaxed depuis write() ;
 * un unique seq_cst fence en fin de write() garantit la visibilité globale.
 *
 * Nouveaux champs v6.2 :
 *   bot_active         false → game_loop envoie PlayerInput{} à ViGEm
 *   edm_connected      true  → EDM a lu un frame valide lors du dernier tick
 *   fsm_override       -1 = FSM libre ; 0..3 = BotState forcé (IDLE/ATTACK/DEFEND/RECOVER)
 *   tuning_steer_sens  multiplicateur steer ∈ [0.1, 3.0]   (1.0 = nominal)
 *   tuning_boost_low   seuil BOOST_LOW live ∈ [0, 60]      (20 = nominal)
 *   tuning_close_sq_rt √CLOSE_SQ live ∈ [200, 4000] UU    (1500 = nominal)
 */
struct SharedDebugData {
    // ── Vecteurs de trajectoire ───────────────────────────
    std::atomic<float> ball_x{},    ball_y{},    ball_z{};
    std::atomic<float> car_x{},     car_y{},     car_z{};
    std::atomic<float> forward_x{}, forward_y{};
    std::atomic<float> pred_x{},    pred_y{};     // position prédite (lookahead)

    // ── Métriques ─────────────────────────────────────────
    std::atomic<float>    boost_pct{};
    std::atomic<float>    steer_cmd{};
    std::atomic<float>    ball_dist{};
    std::atomic<float>    tick_ms{};
    std::atomic<BotState> fsm_state{BotState::IDLE};

    // ── Contrôles UI → game_loop ──────────────────────────

    /// Killswitch : false → game_loop envoie PlayerInput{} à ViGEm.
    /// La FSM et BotController continuent (reprise immédiate au RESUME).
    std::atomic<bool>   bot_active{true};

    /// Alias de rétrocompatibilité v6.1 — même atomic.
    std::atomic<bool>&  autonomous{bot_active};

    /// Indicateur statut EDM : écrit par game_loop, lu par le panel.
    std::atomic<bool>   edm_connected{false};

    /// -1 = FSM libre ; 0..3 = BotState forcé par l'UI.
    std::atomic<int8_t> fsm_override{-1};

    // ── Tuning live ───────────────────────────────────────
    std::atomic<float>  tuning_steer_sens  {1.0f};    // multiplicateur steer
    std::atomic<float>  tuning_boost_low   {20.f};    // seuil BOOST_LOW (%)
    std::atomic<float>  tuning_close_sq_rt {1500.f};  // √CLOSE_SQ (UU)

    // ── Écriture depuis le thread de jeu (aucun mutex) ────

    /**
     * @param edm_ok  true si EDM a réussi read_frame() ce tick (nouveau v6.2).
     *                Alimente l'indicateur visuel vert/rouge du panel.
     */
    void write(const GameTickPacket& pkt,
               BotState              state,
               const PlayerInput&    input,
               float                 tick_duration_ms,
               bool                  edm_ok = false) noexcept
    {
        ball_x.store(pkt.ball.position.x, std::memory_order_relaxed);
        ball_y.store(pkt.ball.position.y, std::memory_order_relaxed);
        ball_z.store(pkt.ball.position.z, std::memory_order_relaxed);

        car_x.store(pkt.player.position.x, std::memory_order_relaxed);
        car_y.store(pkt.player.position.y, std::memory_order_relaxed);
        car_z.store(pkt.player.position.z, std::memory_order_relaxed);

        forward_x.store(pkt.player.forward.x, std::memory_order_relaxed);
        forward_y.store(pkt.player.forward.y, std::memory_order_relaxed);

        boost_pct.store(pkt.player.boost_amount, std::memory_order_relaxed);
        steer_cmd.store(input.steer,             std::memory_order_relaxed);
        tick_ms.store(tick_duration_ms,          std::memory_order_relaxed);
        fsm_state.store(state,                   std::memory_order_relaxed);
        edm_connected.store(edm_ok,              std::memory_order_relaxed);

        const float dx = pkt.ball.position.x - pkt.player.position.x;
        const float dy = pkt.ball.position.y - pkt.player.position.y;
        ball_dist.store(std::sqrt(dx*dx + dy*dy), std::memory_order_relaxed);

        std::atomic_thread_fence(std::memory_order_seq_cst);
    }
};

// ─────────────────────────────────────────────────────────
// § 2. Ring buffer pour les mini-graphes (POD, zéro alloc)
// ─────────────────────────────────────────────────────────

template<std::size_t N = 256>
struct PlotRing {
    std::array<float, N> data{};
    int                  head{};
    int                  count{};

    void push(float v) noexcept {
        data[static_cast<std::size_t>(head)] = v;
        head = (head + 1) % static_cast<int>(N);
        if (count < static_cast<int>(N)) ++count;
    }

    static float getter(void* ring, int idx) noexcept {
        auto* r     = static_cast<PlotRing*>(ring);
        const int total = static_cast<int>(N);
        const int i = (r->head - r->count + idx + total * 2) % total;
        return r->data[static_cast<std::size_t>(i)];
    }
};

// ─────────────────────────────────────────────────────────
// § 3. Helpers de rendu
// ─────────────────────────────────────────────────────────

[[nodiscard]] inline ImVec4 state_color(BotState s) noexcept {
    switch (s) {
        case BotState::ATTACK:  return {0.85f, 0.35f, 0.19f, 1.f}; // coral
        case BotState::DEFEND:  return {0.22f, 0.54f, 0.86f, 1.f}; // blue
        case BotState::RECOVER: return {0.93f, 0.62f, 0.15f, 1.f}; // amber
        default:                return {0.53f, 0.53f, 0.50f, 1.f}; // gray
    }
}

// ─────────────────────────────────────────────────────────
// § 4. ControlPanel — encapsule toute la logique ImGui
// ─────────────────────────────────────────────────────────

class ControlPanel {
public:
    /**
     * @param data  Pointeur partagé vers les données de debug.
     * @param fsm   Référence à la FSM (lecture log, non modifiée ici).
     */
    explicit ControlPanel(std::shared_ptr<SharedDebugData> data,
                          const StateMachine&              fsm) noexcept
        : data_(std::move(data)), fsm_(fsm) {}

    /**
     * Appelé par le thread de jeu pour mettre à jour les données.
     * Zéro mutex, zéro allocation.
     *
     * @param edm_ok  Résultat de edm.last_error().empty() ce tick.
     */
    void update_debug(const GameTickPacket& pkt,
                      BotState              state,
                      const PlayerInput&    input,
                      float                 tick_ms,
                      bool                  edm_ok = false) noexcept
    {
        data_->write(pkt, state, input, tick_ms, edm_ok);
        plot_boost_.push(pkt.player.boost_amount);
        plot_steer_.push(input.steer);
        plot_tick_.push(tick_ms);
    }

    /**
     * Dessine le panneau complet.
     * À appeler depuis le thread de rendu, entre NewFrame() et Render().
     */
    void draw() noexcept {
        ImGui::SetNextWindowSize({550.f, 700.f}, ImGuiCond_FirstUseEver);
        ImGui::Begin("RLTool v6.2 — Control Panel", nullptr,
                     ImGuiWindowFlags_NoCollapse);

        draw_killswitch_section();  // ← TOUJOURS EN PREMIER
        ImGui::Spacing();
        draw_mode_section();        // FSM override + indicateur EDM
        ImGui::Spacing();
        draw_tuning_section();      // sliders live
        ImGui::Spacing();
        draw_vectors_section();     // vue terrain 2D
        ImGui::Spacing();
        draw_metrics_section();     // boost, tick ms, steer
        ImGui::Spacing();
        draw_log_section();         // historique transitions FSM

        ImGui::End();
    }

    [[nodiscard]] bool bot_active()  const noexcept {
        return data_->bot_active.load(std::memory_order_acquire);
    }
    [[nodiscard]] bool autonomous()  const noexcept { return bot_active(); }

private:

    // ─────────────────────────────────────────────────────
    // § 4.1 Killswitch — FORCE STOP / RESUME
    //
    //  Un clic écrit dans bot_active (release).
    //  game_loop lit bot_active (acquire) à chaque tick :
    //    false → PlayerInput{} envoyé à ViGEm (commandes nulles).
    //    true  → commandes normales du bot.
    //
    //  La FSM et BotController continuent en arrière-plan :
    //  l'état interne est préservé, la reprise est instantanée.
    //
    //  Note : le killswitch F10 physique dans game_loop est
    //  indépendant de ce bouton — les deux coexistent.
    // ─────────────────────────────────────────────────────
    void draw_killswitch_section() noexcept {
        const bool active = data_->bot_active.load(std::memory_order_relaxed);
        const float btn_w = ImGui::GetContentRegionAvail().x;

        if (active) {
            // ── Bot actif → gros bouton rouge STOP ────────
            ImGui::PushStyleColor(ImGuiCol_Button,        {0.75f, 0.08f, 0.08f, 1.f});
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, {0.90f, 0.18f, 0.18f, 1.f});
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  {0.55f, 0.04f, 0.04f, 1.f});
            ImGui::PushStyleColor(ImGuiCol_Text,          {1.f,   1.f,   1.f,   1.f});

            if (ImGui::Button(" \u25a0  FORCE STOP  (ViGEm -> commandes nulles) ", {btn_w, 42.f}))
                data_->bot_active.store(false, std::memory_order_release);

            ImGui::PopStyleColor(4);
        } else {
            // ── Bot stoppé → gros bouton vert RESUME ──────
            ImGui::PushStyleColor(ImGuiCol_Button,        {0.10f, 0.52f, 0.10f, 1.f});
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, {0.18f, 0.70f, 0.18f, 1.f});
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  {0.05f, 0.38f, 0.05f, 1.f});
            ImGui::PushStyleColor(ImGuiCol_Text,          {1.f,   1.f,   1.f,   1.f});

            if (ImGui::Button(" \u25ba  RESUME BOT ", {btn_w, 42.f}))
                data_->bot_active.store(true, std::memory_order_release);

            ImGui::PopStyleColor(4);

            ImGui::PushStyleColor(ImGuiCol_Text, {1.f, 0.85f, 0.f, 1.f});
            ImGui::TextUnformatted("  \u26a0  Bot arrete — ViGEm recoit des commandes nulles");
            ImGui::PopStyleColor();
        }
    }

    // ─────────────────────────────────────────────────────
    // § 4.2 Mode — indicateur EDM + override FSM
    //
    //  Indicateur EDM :
    //    Cercle vert  → edm_connected == true  (RocketLeague.exe trouvé)
    //    Cercle rouge → edm_connected == false (fallback simulé)
    //    Dessiné via ImDrawList::AddCircleFilled — aucun widget ImGui,
    //    aucune allocation, rendu GPU immédiat.
    //
    //  Radio buttons FSM override :
    //    [Auto] [IDLE] [ATTACK] [DEFEND] [RECOVER]
    //    Écrit dans fsm_override (release).
    //    game_loop lit fsm_override (acquire) et appelle
    //    fsm.force_state() si >= 0, sinon fsm.update(ctx).
    //    Un avertissement jaune s'affiche quand l'override est actif.
    // ─────────────────────────────────────────────────────
    void draw_mode_section() noexcept {
        ImGui::SeparatorText("Mode & Connexion");

        // ── Indicateur EDM (cercle coloré via ImDrawList) ─
        {
            const bool   edm_ok = data_->edm_connected.load(std::memory_order_relaxed);
            ImDrawList*  dl     = ImGui::GetWindowDrawList();
            const ImVec2 p      = ImGui::GetCursorScreenPos();
            const ImVec2 center = { p.x + 8.f,
                                    p.y + ImGui::GetTextLineHeight() * 0.5f };
            const ImU32  fill   = edm_ok
                ? IM_COL32(55,  210, 55,  255)   // vert vif
                : IM_COL32(210, 50,  50,  255);  // rouge
            dl->AddCircleFilled(center, 7.f, fill);
            dl->AddCircle(center, 7.f, IM_COL32(0, 0, 0, 90), 0, 1.5f); // bordure

            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 22.f);
            if (edm_ok)
                ImGui::TextColored({0.25f, 0.90f, 0.25f, 1.f},
                                   "RocketLeague.exe detecte (EDM OK)");
            else
                ImGui::TextColored({0.90f, 0.28f, 0.28f, 1.f},
                                   "Processus introuvable — mode simulation");
        }

        ImGui::Spacing();

        // ── État FSM courant ──────────────────────────────
        const BotState cur  = data_->fsm_state.load(std::memory_order_relaxed);
        const auto     name = state_name(cur);
        ImGui::TextColored(state_color(cur), "  \u25cf  %.*s",
            static_cast<int>(name.size()), name.data());

        ImGui::Spacing();
        ImGui::TextUnformatted("Override FSM :");
        ImGui::SameLine();
        help_marker("Force l'etat du bot sans attendre les conditions\n"
                    "naturelles de la FSM.\n"
                    "Auto = FSM normale, les autres figent l'etat.");

        // ── Radio buttons ─────────────────────────────────
        const int8_t ov    = data_->fsm_override.load(std::memory_order_relaxed);
        int          radio = (ov < 0) ? 4 : static_cast<int>(ov);

        // Chaque bouton : si cliqué, écrit dans fsm_override (release)
        auto radio_btn = [&](const char* label, int value) {
            if (ImGui::RadioButton(label, radio == value)) {
                radio = value;
                const int8_t new_ov = (value == 4)
                    ? int8_t{-1}
                    : static_cast<int8_t>(value);
                data_->fsm_override.store(new_ov, std::memory_order_release);
            }
            ImGui::SameLine();
        };

        radio_btn("Auto",    4);
        radio_btn("IDLE",    static_cast<int>(BotState::IDLE));
        radio_btn("ATTACK",  static_cast<int>(BotState::ATTACK));
        radio_btn("DEFEND",  static_cast<int>(BotState::DEFEND));
        radio_btn("RECOVER", static_cast<int>(BotState::RECOVER));
        ImGui::NewLine();

        if (ov >= 0) {
            ImGui::PushStyleColor(ImGuiCol_Text, {1.f, 0.85f, 0.f, 1.f});
            ImGui::TextUnformatted("  \u26a0  Override actif — FSM automatique ignoree");
            ImGui::PopStyleColor();
        }
    }

    // ─────────────────────────────────────────────────────
    // § 4.3 Tuning live — sliders sans redémarrage
    //
    //  Chaque slider écrit directement dans un atomic de SharedDebugData.
    //  game_loop lit la valeur au tick suivant (memory_order_relaxed).
    //  Aucune recompilation, aucun rechargement de config.txt.
    //
    //  Plages :
    //    steer_sens   ∈ [0.1, 3.0]   (1.0 = nominal)
    //    boost_low    ∈ [0,  60]     (20  = nominal)
    //    close_sq_rt  ∈ [200, 4000]  (1500 UU = nominal)
    // ─────────────────────────────────────────────────────
    void draw_tuning_section() noexcept {
        ImGui::SeparatorText("Tuning live");

        // Steer sensitivity
        float steer_sens = data_->tuning_steer_sens.load(std::memory_order_relaxed);
        ImGui::SetNextItemWidth(220.f);
        if (ImGui::SliderFloat("Sensibilite steer##t", &steer_sens, 0.1f, 3.0f, "x%.2f")) {
            data_->tuning_steer_sens.store(steer_sens, std::memory_order_release);
        }
        ImGui::SameLine(); help_marker(
            "Multiplie la commande steer apres le calcul du bot.\n"
            "1.0 = nominal. > 1 plus reactif, < 1 plus doux.");

        // BOOST_LOW threshold
        float boost_low = data_->tuning_boost_low.load(std::memory_order_relaxed);
        ImGui::SetNextItemWidth(220.f);
        if (ImGui::SliderFloat("Seuil BOOST_LOW##t", &boost_low, 0.f, 60.f, "%.0f %%")) {
            data_->tuning_boost_low.store(boost_low, std::memory_order_release);
        }
        ImGui::SameLine(); help_marker(
            "Boost en dessous duquel le bot passe en RECOVER.\n"
            "Nominal : 20 %%.");

        // CLOSE_SQ (stockée en racine UU pour l'affichage)
        float close_rt = data_->tuning_close_sq_rt.load(std::memory_order_relaxed);
        ImGui::SetNextItemWidth(220.f);
        if (ImGui::SliderFloat("Distance CLOSE##t", &close_rt, 200.f, 4000.f, "%.0f UU")) {
            data_->tuning_close_sq_rt.store(close_rt, std::memory_order_release);
        }
        ImGui::SameLine(); help_marker(
            "Distance balle < voiture consideree proche (CLOSE_SQ = valeur^2).\n"
            "Nominal : 1500 UU.");

        ImGui::Spacing();
        if (ImGui::SmallButton("Reinitialiser valeurs")) {
            data_->tuning_steer_sens.store(1.0f,    std::memory_order_release);
            data_->tuning_boost_low.store(20.f,     std::memory_order_release);
            data_->tuning_close_sq_rt.store(1500.f, std::memory_order_release);
        }
    }

    // ─────────────────────────────────────────────────────
    // § 4.4 Vecteurs — vue terrain 2D (inchangée)
    // ─────────────────────────────────────────────────────
    void draw_vectors_section() noexcept {
        ImGui::SeparatorText("Vecteurs de trajectoire");

        constexpr float W = 180.f, H = 120.f;
        constexpr float FIELD_HALF_X = 4096.f, FIELD_HALF_Y = 5120.f;

        const float bx = data_->ball_x.load(std::memory_order_relaxed);
        const float by = data_->ball_y.load(std::memory_order_relaxed);
        const float cx = data_->car_x.load(std::memory_order_relaxed);
        const float cy = data_->car_y.load(std::memory_order_relaxed);
        const float fx = data_->forward_x.load(std::memory_order_relaxed);
        const float fy = data_->forward_y.load(std::memory_order_relaxed);

        const ImVec2 origin = ImGui::GetCursorScreenPos();
        auto to_px = [&](float x, float y) -> ImVec2 {
            return {
                origin.x + (x / FIELD_HALF_X + 1.f) * 0.5f * W,
                origin.y + (1.f - (y / FIELD_HALF_Y + 1.f) * 0.5f) * H
            };
        };

        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(origin, {origin.x + W, origin.y + H},
                          IM_COL32(35, 35, 35, 210), 4.f);
        dl->AddRect(origin, {origin.x + W, origin.y + H},
                    IM_COL32(80, 80, 80, 210), 4.f);
        // Ligne mi-terrain
        dl->AddLine({origin.x, origin.y + H * 0.5f},
                    {origin.x + W, origin.y + H * 0.5f},
                    IM_COL32(60, 60, 60, 180), 1.f);

        const ImVec2 ball_px = to_px(bx, by);
        dl->AddCircleFilled(ball_px, 5.f, IM_COL32(239, 159, 39, 230));

        const ImVec2 car_px = to_px(cx, cy);
        dl->AddCircleFilled(car_px, 4.f, IM_COL32(55, 138, 221, 230));
        dl->AddLine(car_px, {car_px.x + fx * 30.f, car_px.y - fy * 30.f},
                    IM_COL32(55, 138, 221, 200), 1.5f);
        dl->AddLine(car_px, ball_px, IM_COL32(239, 159, 39, 120), 1.f);

        ImGui::Dummy({W + 8.f, H + 4.f});
        ImGui::SameLine();
        ImGui::BeginGroup();
        ImGui::Text("Balle  (%.0f, %.0f)", bx, by);
        ImGui::Text("Voiture(%.0f, %.0f)", cx, cy);
        ImGui::Text("Dist.  %.0f UU",
                    data_->ball_dist.load(std::memory_order_relaxed));
        ImGui::Text("Steer  % .3f",
                    data_->steer_cmd.load(std::memory_order_relaxed));
        ImGui::EndGroup();
    }

    // ─────────────────────────────────────────────────────
    // § 4.5 Métriques temps réel (inchangée)
    // ─────────────────────────────────────────────────────
    void draw_metrics_section() noexcept {
        ImGui::SeparatorText("Metriques temps reel");

        const float boost = data_->boost_pct.load(std::memory_order_relaxed);
        ImGui::Text("Boost");
        ImGui::SameLine(80.f);
        ImGui::ProgressBar(boost / 100.f, {-1.f, 0.f},
                           std::format("{:.0f} %%", boost).c_str());

        ImGui::PlotLines("Tick (ms)", PlotRing<256>::getter,
                         &plot_tick_, plot_tick_.count,
                         0, nullptr, 0.f, 5.f, {0.f, 40.f});

        ImGui::PlotLines("Steer", PlotRing<256>::getter,
                         &plot_steer_, plot_steer_.count,
                         0, nullptr, -1.f, 1.f, {0.f, 40.f});
    }

    // ─────────────────────────────────────────────────────
    // § 4.6 Log FSM (inchangée)
    // ─────────────────────────────────────────────────────
    void draw_log_section() noexcept {
        ImGui::SeparatorText("Log FSM");
        ImGui::BeginChild("##fsm_log", {0.f, 100.f}, false,
                          ImGuiWindowFlags_HorizontalScrollbar);

        fsm_.log().for_each_recent(8, [](const LogEntry& e) {
            const ImVec4 col = state_color(e.to);
            ImGui::Text("[%.3f s]", e.timestamp_s);
            ImGui::SameLine();
            ImGui::TextColored(col, "%.*s",
                static_cast<int>(state_name(e.to).size()),
                state_name(e.to).data());
            ImGui::SameLine();
            ImGui::TextDisabled("(boost=%.0f%%, dist=%.0f UU)",
                                e.boost_pct, e.ball_dist);
        });

        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.f);

        ImGui::EndChild();
    }

    // ── Utilitaire : "(?)" avec tooltip ──────────────────
    static void help_marker(const char* desc) noexcept {
        ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 22.f);
            ImGui::TextUnformatted(desc);
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }
    }

    // ── Membres ──────────────────────────────────────────
    std::shared_ptr<SharedDebugData> data_;
    const StateMachine&              fsm_;

    PlotRing<256> plot_boost_{};
    PlotRing<256> plot_steer_{};
    PlotRing<256> plot_tick_{};
};

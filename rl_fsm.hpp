/**
 * rl_fsm.hpp — Finite State Machine zero-allocation, C++20
 *
 * Architecture :
 *  - États typés via enum class + std::variant (pas de vtable, pas d'heap)
 *  - Transitions évaluées à chaque tick par table de règles constexpr
 *  - std::atomic<State> pour lecture thread-safe depuis ImGui
 *  - RingBuffer<LogEntry, N> pour le log lock-free (SPSC)
 *  - SharedContext partagé via std::shared_ptr (owner unique : BotController)
 *
 * Nouveautés v6.2 :
 *  - StateMachine::force_state(BotState) : override externe depuis l'UI ImGui
 *    ou depuis un killswitch physique (F10). Écrit directement dans l'atomic
 *    current_state_ sans passer par la table de transitions.
 *    Thread-safe : memory_order_release visible immédiatement par le thread de jeu.
 */

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <optional>
#include <string_view>
#include <variant>

// ─────────────────────────────────────────────────────────
// § 1. Définition des états
// ─────────────────────────────────────────────────────────

enum class BotState : uint8_t {
    IDLE    = 0,
    ATTACK  = 1,
    DEFEND  = 2,
    RECOVER = 3,
    _COUNT
};

[[nodiscard]] constexpr std::string_view state_name(BotState s) noexcept {
    switch (s) {
        case BotState::IDLE:    return "IDLE";
        case BotState::ATTACK:  return "ATTACK";
        case BotState::DEFEND:  return "DEFEND";
        case BotState::RECOVER: return "RECOVER";
        default:                return "???";
    }
}

// ─────────────────────────────────────────────────────────
// § 2. Contexte de décision (snapshot par tick, trivial)
// ─────────────────────────────────────────────────────────

struct DecisionContext {
    float    ball_dist_sq{};          // distance² voiture→balle (UU²)
    float    ball_dist_to_goal_sq{};  // distance² balle→cage propre (UU²)
    float    boost_pct{};             // [0, 100]
    bool     ball_is_behind_car{};    // balle côté défensif ?
    bool     teammate_attacking{};    // allié déjà en attaque ?
    uint64_t tick_id{};
};

// ─────────────────────────────────────────────────────────
// § 3. Table de transitions constexpr
//
//  Chaque règle est évaluée dans l'ordre ; la première qui
//  retourne true déclenche la transition vers next_state.
//  Pas de heap, pas de std::function dynamique : lambdas
//  stockées comme pointeurs de fonction bruts.
// ─────────────────────────────────────────────────────────

using GuardFn = bool (*)(const DecisionContext&) noexcept;

struct TransitionRule {
    BotState  from;
    BotState  to;
    GuardFn   guard;
};

// Seuils physiques (calibrés sur Rocket League UU)
namespace Thresholds {
    inline constexpr float CLOSE_SQ        = 1500.f * 1500.f;
    inline constexpr float VERY_CLOSE_SQ   = 800.f  * 800.f;
    inline constexpr float GOAL_DANGER_SQ  = 3000.f * 3000.f;
    inline constexpr float BOOST_LOW       = 20.f;
    inline constexpr float BOOST_OK        = 50.f;
}

// Table déclarative — lisible, zéro runtime overhead
inline constexpr std::array<TransitionRule, 9> TRANSITION_TABLE {{
    // ── Depuis IDLE ──────────────────────────────────────
    { BotState::IDLE, BotState::ATTACK,
        [](const DecisionContext& c) noexcept {
            return !c.ball_is_behind_car
                && c.boost_pct > Thresholds::BOOST_LOW
                && !c.teammate_attacking; } },

    { BotState::IDLE, BotState::DEFEND,
        [](const DecisionContext& c) noexcept {
            return c.ball_is_behind_car
                && c.ball_dist_to_goal_sq < Thresholds::GOAL_DANGER_SQ; } },

    { BotState::IDLE, BotState::RECOVER,
        [](const DecisionContext& c) noexcept {
            return c.boost_pct < Thresholds::BOOST_LOW; } },

    // ── Depuis ATTACK ────────────────────────────────────
    { BotState::ATTACK, BotState::DEFEND,
        [](const DecisionContext& c) noexcept {
            return c.ball_is_behind_car
                && c.ball_dist_to_goal_sq < Thresholds::GOAL_DANGER_SQ; } },

    { BotState::ATTACK, BotState::RECOVER,
        [](const DecisionContext& c) noexcept {
            return c.boost_pct < Thresholds::BOOST_LOW; } },

    { BotState::ATTACK, BotState::IDLE,
        [](const DecisionContext& c) noexcept {
            return c.ball_dist_sq > Thresholds::CLOSE_SQ
                && !c.ball_is_behind_car; } },

    // ── Depuis DEFEND ────────────────────────────────────
    { BotState::DEFEND, BotState::IDLE,
        [](const DecisionContext& c) noexcept {
            return !c.ball_is_behind_car
                || c.ball_dist_to_goal_sq > Thresholds::GOAL_DANGER_SQ; } },

    { BotState::DEFEND, BotState::RECOVER,
        [](const DecisionContext& c) noexcept {
            return c.boost_pct < Thresholds::BOOST_LOW; } },

    // ── Depuis RECOVER ───────────────────────────────────
    { BotState::RECOVER, BotState::IDLE,
        [](const DecisionContext& c) noexcept {
            return c.boost_pct >= Thresholds::BOOST_OK; } },
}};

// ─────────────────────────────────────────────────────────
// § 4. Ring buffer SPSC pour le log (zero-allocation)
// ─────────────────────────────────────────────────────────

struct LogEntry {
    float     timestamp_s{};
    BotState  from{};
    BotState  to{};
    float     boost_pct{};
    float     ball_dist{};
};

template<std::size_t Cap = 64>
    requires ((Cap & (Cap - 1)) == 0)
class LogRing {
public:
    void push(const LogEntry& e) noexcept {
        const auto h = head_.load(std::memory_order_relaxed);
        buffer_[h & (Cap - 1)] = e;
        head_.store(h + 1, std::memory_order_release);
    }

    // Lit les N dernières entrées (côté ImGui, thread de rendu)
    template<typename Fn>
    void for_each_recent(std::size_t n, Fn&& fn) const noexcept {
        const auto h = head_.load(std::memory_order_acquire);
        const auto count = std::min(n, static_cast<std::size_t>(h));
        for (std::size_t i = count; i > 0; --i)
            fn(buffer_[(h - i) & (Cap - 1)]);
    }

private:
    alignas(64) std::atomic<uint64_t>   head_{0};
    std::array<LogEntry, Cap>           buffer_{};
};

// ─────────────────────────────────────────────────────────
// § 5. StateMachine — orchestre transitions + log
// ─────────────────────────────────────────────────────────

class StateMachine {
public:
    explicit StateMachine(float start_time = 0.f) noexcept
        : current_time_(start_time) {}

    /**
     * Évalue la table de transitions sur le contexte courant.
     * Appelé à chaque tick (hot path) : O(|transitions|), zéro alloc.
     *
     * @param ctx    Snapshot de décision du tick courant.
     * @param dt_s   Durée du tick en secondes.
     * @return       true si une transition a eu lieu.
     */
    bool update(const DecisionContext& ctx, float dt_s = 1.f / 120.f) noexcept {
        current_time_ += dt_s;
        const BotState cur = current_state_.load(std::memory_order_relaxed);

        for (const auto& rule : TRANSITION_TABLE) {
            if (rule.from == cur && rule.guard(ctx)) {
                transition_to(cur, rule.to, ctx);
                return true;
            }
        }
        return false;
    }

    // Lecture thread-safe depuis ImGui
    [[nodiscard]] BotState state() const noexcept {
        return current_state_.load(std::memory_order_acquire);
    }

    /**
     * Force l'état FSM depuis un thread externe (UI ImGui ou killswitch physique).
     *
     * Contourne délibérément la table de transitions : l'état est écrasé
     * immédiatement sans évaluation des gardes.
     *
     * Thread-safety :
     *   - memory_order_release garantit que le thread de jeu voit le nouvel
     *     état au prochain tick (via memory_order_relaxed sur le load de update()).
     *   - Aucun mutex, aucune allocation — utilisable depuis le thread de rendu
     *     ou depuis un handler de signal/hotkey.
     *
     * Usage :
     *   fsm.force_state(BotState::IDLE);    // UI : bouton IDLE
     *   fsm.force_state(BotState::DEFEND);  // killswitch F10 → mode défensif
     *
     * @param s   État cible. Doit être < BotState::_COUNT.
     */
    void force_state(BotState s) noexcept {
        current_state_.store(s, std::memory_order_release);
    }

    const LogRing<64>& log() const noexcept { return log_; }

private:
    void transition_to(BotState from, BotState to,
                       const DecisionContext& ctx) noexcept {
        current_state_.store(to, std::memory_order_release);
        log_.push(LogEntry {
            .timestamp_s = current_time_,
            .from        = from,
            .to          = to,
            .boost_pct   = ctx.boost_pct,
            .ball_dist   = sqrtf(ctx.ball_dist_sq),
        });
    }

    alignas(64) std::atomic<BotState> current_state_{BotState::IDLE};
    LogRing<64>                        log_{};
    float                              current_time_{};
};

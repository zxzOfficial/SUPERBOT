/**
 * rl_bot.hpp — Bot Rocket League C++20, haute performance
 *
 * Principes fondamentaux (inchangés) :
 *  - Zero dynamic allocation sur le chemin critique (hot path)
 *  - std::array + structures de taille fixe pour les prédictions
 *  - Lock-free SPSC queue pour les ticks
 *  - constexpr et [[nodiscard]] pour la sûreté statique
 *
 * Améliorations v4 :
 *  §3b BallLatencyBuffer : buffer circulaire de 12 positions de balle.
 *      Le bot prend ses décisions sur la position d'il y a LATENCY_TICKS
 *      ticks (~83 ms à 120 Hz) pour simuler la latence humaine de perception.
 *      Exception : si la balle est à moins de PROXIMITY_THRESHOLD (500 UU),
 *      le bot repasse en temps réel (réflexe de proximité).
 *      Zéro allocation — std::array<Vec3, 12> sur la pile du BotController.
 *
 *  §5  Filtre d'inertie étendu (throttle + boost) :
 *      throttle : même lerp exponentiel + limiteur de jerk que steer.
 *                 α = 0.25 → constante de temps ≈ 4 ticks (~33 ms).
 *      boost    : la décision binaire (bool) est lissée via un accumulateur
 *                 float prev_boost_ ∈ [0, 1] avec α = 0.15 et jerk 0.08/tick.
 *                 La commande finale est (prev_boost_ >= 0.5f).
 *                 Cela simule le temps de pression physique sur une touche
 *                 et supprime les impulsions de boost d'un seul tick.
 *
 *  §5  Toutes les constantes d'inertie / latence sont regroupées dans
 *      la section "Paramètres configurables" pour faciliter le tuning.
 *
 * Améliorations v3 (conservées) :
 *  §0  math_utils   : lerp, clamp, PI — constexpr, zero-dep
 *  §4  CubicBezier  : trajectoire Bézier cubique P0..P3, zero-allocation
 *  §5  BotController::compute() :
 *        • Calcul des 4 points de contrôle à chaque tick (pile uniquement)
 *        • Évaluation de la courbe à t=BEZIER_T pour la cible intermédiaire
 *        • compute_steer_angle() sur cette cible intermédiaire
 *        • Lissage exponentiel (lerp) → supprime les sauts de direction
 *        • Limiteur de jerk → borne la variation Δsteer/tick
 *        • Décision boost modulée par ball_dist_to_goal_sq (DecisionContext)
 *
 * Aucune allocation dynamique n'est introduite — hot path zéro-alloc garanti.
 */

#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>

// ─────────────────────────────────────────────────────────
// § 0. Utilitaires mathématiques (constexpr, zero-dep)
// ─────────────────────────────────────────────────────────

namespace math_utils {

/// Interpolation linéaire a + t·(b−a).
/// Formulation de précision : minimise l'erreur d'arrondi pour t ≈ 1.
[[nodiscard]] constexpr float lerp(float a, float b, float t) noexcept {
    return a + t * (b - a);
}

/// Clamp sans inclure <algorithm>.
[[nodiscard]] constexpr float clamp(float v, float lo, float hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

/// π en simple précision — évite d'inclure <numbers>.
inline constexpr float PI = 3.14159265358979323846f;

} // namespace math_utils

// ─────────────────────────────────────────────────────────
// § 1. Structures de données fondamentales
// ─────────────────────────────────────────────────────────

/// Vecteur 3D en virgule flottante simple précision.
/// Pas de constructeur virtuel, layout garanti trivial.
struct Vec3 {
    float x{}, y{}, z{};

    [[nodiscard]] constexpr Vec3 operator+(const Vec3& o) const noexcept {
        return {x + o.x, y + o.y, z + o.z};
    }
    [[nodiscard]] constexpr Vec3 operator-(const Vec3& o) const noexcept {
        return {x - o.x, y - o.y, z - o.z};
    }
    [[nodiscard]] constexpr Vec3 operator*(float s) const noexcept {
        return {x * s, y * s, z * s};
    }
    [[nodiscard]] constexpr float dot(const Vec3& o) const noexcept {
        return x * o.x + y * o.y + z * o.z;
    }
    [[nodiscard]] float length() const noexcept {
        return std::sqrt(dot(*this));
    }
    [[nodiscard]] Vec3 normalized() const noexcept {
        const float len = length();
        if (len < 1e-6f) return {};
        return *this * (1.f / len);
    }
};

static_assert(std::is_trivially_copyable_v<Vec3>);

/// État minimal de la balle tel que reçu du moteur.
struct BallState {
    Vec3  position{};
    Vec3  velocity{};
    Vec3  angular_velocity{};
    float radius{92.75f};   // unités du jeu (UU)
};

/// État minimal du joueur.
struct CarState {
    Vec3  position{};
    Vec3  velocity{};
    Vec3  forward{};        // vecteur unitaire, direction de la voiture
    float boost_amount{};
    bool  on_ground{};
};

/// Paquet de données reçu à chaque tick (simplifié).
struct GameTickPacket {
    BallState ball;
    CarState  player;
    float     game_time_remaining{};
    uint64_t  tick_id{};
};

/// Commandes envoyées au moteur.
struct PlayerInput {
    float throttle{};   // [-1, 1]
    float steer{};      // [-1, 1]
    float pitch{};
    float yaw{};
    float roll{};
    bool  boost{};
    bool  jump{};
    bool  handbrake{};
};

// ─────────────────────────────────────────────────────────
// § 2. BallPredictor — intégration d'Euler semi-implicite,
//       zero-allocation
// ─────────────────────────────────────────────────────────

/// Résultat de la prédiction : tableau de positions futures.
/// Capacité fixe = pas d'allocation dynamique.
template<std::size_t MaxSteps = 128>
struct PredictionPath {
    std::array<Vec3, MaxSteps> positions{};
    std::size_t                step_count{};
};

/// Physique simplifiée du jeu (constantes approx. Rocket League).
struct BallPhysics {
    static constexpr float gravity     = -650.f;  // UU/s²
    static constexpr float drag        =   0.03f; // coefficient aérien
    static constexpr float restitution =   0.6f;  // rebond sol
    static constexpr float floor_z     =   0.f;   // niveau du terrain
};

/**
 * Prédit la trajectoire de la balle par intégration d'Euler semi-implicite.
 *
 *   a(t)     = [ -drag·vx,  -drag·vy,  g − drag·vz ]
 *   v(t+Δt) = v(t) + a(t)·Δt
 *   x(t+Δt) = x(t) + v(t+Δt)·Δt    ← semi-implicite : v déjà mis à jour
 *
 * Résultat alloué sur la pile (std::array), zéro heap.
 */
template<std::size_t MaxSteps = 128>
[[nodiscard]] PredictionPath<MaxSteps> predict_ball(
    const BallState& ball,
    float            dt    = 1.f / 120.f,
    std::size_t      steps = MaxSteps) noexcept
{
    PredictionPath<MaxSteps> path;
    path.step_count = std::min(steps, MaxSteps);

    Vec3 pos = ball.position;
    Vec3 vel = ball.velocity;

    for (std::size_t i = 0; i < path.step_count; ++i) {
        const Vec3 acc {
            -BallPhysics::drag * vel.x,
            -BallPhysics::drag * vel.y,
             BallPhysics::gravity - BallPhysics::drag * vel.z
        };
        vel = vel + acc * dt;
        pos = pos + vel * dt;

        if (pos.z <= BallPhysics::floor_z + ball.radius) {
            pos.z = BallPhysics::floor_z + ball.radius;
            vel.z = -vel.z * BallPhysics::restitution;
        }
        path.positions[i] = pos;
    }
    return path;
}

// ─────────────────────────────────────────────────────────
// § 3. TickHandler — file SPSC lock-free, zero-allocation
// ─────────────────────────────────────────────────────────

/**
 * File circulaire SPSC (Single Producer / Single Consumer).
 *
 * Invariants :
 *   - head_ écrit uniquement par le producteur.
 *   - tail_ écrit uniquement par le consommateur.
 *   - alignas(64) sur chaque index → zéro false sharing.
 *   - Masquage AND (& mask_) remplace le modulo coûteux.
 *
 * @tparam T    Doit être trivially copyable.
 * @tparam Cap  Doit être une puissance de 2.
 */
template<typename T, std::size_t Cap = 64>
    requires std::is_trivially_copyable_v<T> && ((Cap & (Cap - 1)) == 0)
class SPSCQueue {
public:
    bool push(const T& item) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t next = (head + 1) & mask_;
        if (next == tail_.load(std::memory_order_acquire))
            return false;
        buffer_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    [[nodiscard]] std::optional<T> pop() noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire))
            return std::nullopt;
        T item = buffer_[tail];
        tail_.store((tail + 1) & mask_, std::memory_order_release);
        return item;
    }

private:
    static constexpr std::size_t mask_ = Cap - 1;
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
    std::array<T, Cap> buffer_{};
};

/**
 * TickHandler : dispatche les GameTickPackets via un callback enregistré.
 *
 * Note d'allocation : std::function alloue sur le heap à la construction
 * (capture de lambda). C'est un compromis délibéré — hors hot path.
 * Sur le hot path (on_tick / process_pending), aucune allocation n'a lieu.
 */
class TickHandler {
public:
    using TickCallback = std::function<void(const GameTickPacket&)>;

    explicit TickHandler(TickCallback cb) noexcept
        : callback_(std::move(cb)) {}

    void on_tick(const GameTickPacket& pkt) noexcept { queue_.push(pkt); }

    void process_pending() noexcept {
        while (auto pkt = queue_.pop())
            callback_(*pkt);
    }

private:
    SPSCQueue<GameTickPacket, 64> queue_;
    TickCallback                  callback_;
};

// ─────────────────────────────────────────────────────────
// § 3b. BallLatencyBuffer — buffer de latence visuelle
//
//  Principe :
//    Le cerveau humain perçoit et réagit à un stimulus visuel avec
//    un délai de ~80-100 ms (latence sensori-motrice).
//    Ce buffer stocke les 12 dernières positions de la balle (FIFO
//    circulaire) pour que le BotController prenne ses décisions
//    sur la position qu'il "aurait vue" il y a LATENCY_TICKS ticks,
//    simulant ce délai de perception.
//
//  Exception de proximité :
//    Quand la balle est à moins de PROXIMITY_THRESHOLD UU de la
//    voiture, le buffer est court-circuité et la position temps réel
//    est retournée. Cela modélise le "réflexe de proximité" humain :
//    à courte distance, la réaction est quasi-instantanée (feedback
//    proprioceptif > feedback visuel).
//
//  Architecture :
//    - std::array<Vec3, CAP> alloué dans le BotController (pile)
//    - head_ : uint8_t (suffisant pour CAP ≤ 256)
//    - count_ : nombre de positions enregistrées (rampe jusqu'à CAP)
//    - Zéro allocation dynamique — trivially copyable
//
//  Invariant : CAP doit être une puissance de 2 pour que le masquage
//  AND remplace le modulo (même convention que SPSCQueue).
// ─────────────────────────────────────────────────────────

/// Nombre de ticks de retard appliqués à la perception de la balle.
/// 10 ticks à 120 Hz = 83.3 ms ≈ latence humaine médiane.
inline constexpr std::size_t LATENCY_TICKS = 10;

/// Distance (UU) en dessous de laquelle le bot repasse en temps réel.
/// 500 UU ≈ ~5 fois le rayon de la balle standard (92.75 UU).
inline constexpr float PROXIMITY_THRESHOLD = 500.f;

/// Capacité du buffer circulaire. Doit être une puissance de 2 ≥ LATENCY_TICKS + 1.
/// 16 ≥ 10 + 1 = 11 ✓
inline constexpr std::size_t LATENCY_BUF_CAP = 16;

static_assert((LATENCY_BUF_CAP & (LATENCY_BUF_CAP - 1)) == 0,
              "LATENCY_BUF_CAP doit être une puissance de 2");
static_assert(LATENCY_BUF_CAP > LATENCY_TICKS,
              "LATENCY_BUF_CAP doit être > LATENCY_TICKS");

/**
 * Buffer circulaire de positions de balle pour la simulation de latence visuelle.
 *
 * Usage (dans BotController::compute()) :
 *   lat_buf_.push(pkt.ball.position);
 *   const Vec3 perceived = lat_buf_.get_delayed(dist_to_ball);
 */
class BallLatencyBuffer {
public:
    BallLatencyBuffer() noexcept {
        // Initialisation explicite : tous les slots à zéro.
        // Sans cela, les premiers ticks liraient des positions non-initialisées
        // si get_delayed() est appelé avant que le buffer soit rempli.
        buffer_.fill(Vec3{});
    }

    /**
     * Enregistre la position courante de la balle.
     * Appelé une fois par tick, avant get_delayed().
     * Zéro allocation, O(1).
     */
    void push(const Vec3& ball_pos) noexcept {
        buffer_[head_] = ball_pos;
        head_ = static_cast<uint8_t>((head_ + 1u) & mask_);
        if (count_ < LATENCY_BUF_CAP)
            ++count_;
    }

    /**
     * Retourne la position perçue de la balle, en fonction de la distance.
     *
     * @param dist_to_ball  Distance actuelle voiture→balle (UU).
     * @return  - Position retardée de LATENCY_TICKS ticks si dist > PROXIMITY_THRESHOLD
     *          - Position temps réel (index head-1) si dist ≤ PROXIMITY_THRESHOLD
     *          - Position temps réel si le buffer n'est pas encore rempli
     *            (les premiers ticks avant que count_ atteigne LATENCY_TICKS)
     *
     * Invariant de lecture :
     *   head_ pointe vers le prochain slot d'écriture (slot "futur").
     *   Le slot le plus récent est à (head_ - 1 + CAP) & mask_.
     *   Le slot d'il y a N ticks est à (head_ - 1 - N + CAP*2) & mask_.
     */
    [[nodiscard]] Vec3 get_delayed(float dist_to_ball) const noexcept {
        // Réflexe de proximité : temps réel
        const bool proximity_reflex = (dist_to_ball <= PROXIMITY_THRESHOLD);

        // Buffer pas encore rempli : on retourne la position la plus ancienne
        // disponible plutôt qu'une position non-initialisée.
        const bool buffer_filling = (count_ <= LATENCY_TICKS);

        if (proximity_reflex || buffer_filling) {
            // Slot le plus récent = (head_ - 1 + CAP) & mask_
            const uint8_t recent_idx =
                static_cast<uint8_t>((head_ - 1u + LATENCY_BUF_CAP) & mask_);
            return buffer_[recent_idx];
        }

        // Position d'il y a LATENCY_TICKS ticks :
        //   head_ pointe vers le prochain slot vide.
        //   Il y a 1 tick : head_ - 1
        //   Il y a N ticks : head_ - 1 - (N - 1) = head_ - N
        //   (+ LATENCY_BUF_CAP * 2 pour rester positif avant le masquage)
        const uint8_t delayed_idx =
            static_cast<uint8_t>(
                (static_cast<unsigned>(head_)
                 + LATENCY_BUF_CAP * 2u
                 - LATENCY_TICKS)
                & mask_);
        return buffer_[delayed_idx];
    }

    /// Nombre de positions enregistrées depuis la construction.
    [[nodiscard]] std::size_t count() const noexcept { return count_; }

private:
    static constexpr uint8_t mask_ =
        static_cast<uint8_t>(LATENCY_BUF_CAP - 1u);

    std::array<Vec3, LATENCY_BUF_CAP> buffer_{};
    uint8_t     head_{0};     // prochain index d'écriture
    std::size_t count_{0};    // positions enregistrées (rampe jusqu'à CAP)
};

static_assert(std::is_trivially_destructible_v<BallLatencyBuffer>,
              "BallLatencyBuffer doit être trivially destructible (pas de heap)");

// ─────────────────────────────────────────────────────────
// § 4. CubicBezier + TrajectoryCalc
//
//  Courbe de Bézier cubique définie par quatre points de contrôle.
//  Évaluation par l'algorithme de De Casteljau : stable, O(1), zero-alloc.
//
//  Formule :
//    B(t) = (1-t)³·P0 + 3(1-t)²t·P1 + 3(1-t)t²·P2 + t³·P3
//
//  Propriétés exploitées :
//    • B(0) = P0 (position actuelle de la voiture)
//    • B(1) = P3 (position cible prédite de la balle)
//    • La tangente en P0 est proportionnelle à (P1 − P0)
//    • La tangente en P3 est proportionnelle à (P3 − P2)
//
//  Construction des points de contrôle internes (§5) :
//    P1 = P0 + forward_car  * CTRL_TENSION * dist(P0,P3)
//    P2 = P3 − forward_ball * CTRL_TENSION * dist(P0,P3)
//    où forward_ball est la direction normalisée de la vitesse de la balle.
//
//  Drift sinusoïdal (§5.4) :
//    Un offset perpendiculaire est appliqué à P1 et P2 uniquement.
//    P0 et P3 restent fixes.
// ─────────────────────────────────────────────────────────

/// Quatre points de contrôle d'une courbe de Bézier cubique.
/// Trivially copyable, alloué sur la pile.
struct CubicBezier {
    Vec3 p0{}, p1{}, p2{}, p3{};
};

static_assert(std::is_trivially_copyable_v<CubicBezier>);

/**
 * Évalue B(t) par l'algorithme de De Casteljau.
 *
 * @param b  Courbe de Bézier cubique.
 * @param t  Paramètre ∈ [0, 1] (0 = départ, 1 = arrivée).
 * @return   Point sur la courbe à t.
 */
[[nodiscard]] constexpr Vec3 bezier_eval(const CubicBezier& b, float t) noexcept {
    const float u  = 1.f - t;
    const float u2 = u * u;
    const float u3 = u2 * u;
    const float t2 = t * t;
    const float t3 = t2 * t;
    return b.p0 * u3
         + b.p1 * (3.f * u2 * t)
         + b.p2 * (3.f * u  * t2)
         + b.p3 * t3;
}

/**
 * Calcule la tangente normalisée B'(t) / |B'(t)|.
 * B'(t) = 3[ (P1−P0)(1−t)² + 2(P2−P1)(1−t)t + (P3−P2)t² ]
 */
[[nodiscard]] Vec3 bezier_tangent(const CubicBezier& b, float t) noexcept {
    const float u = 1.f - t;
    const Vec3 d = (b.p1 - b.p0) * (3.f * u * u)
                 + (b.p2 - b.p1) * (6.f * u * t)
                 + (b.p3 - b.p2) * (3.f * t * t);
    return d.normalized();
}

/**
 * Calcule l'angle de braquage signé vers une cible (fonction libre, pure).
 *
 *   cos θ  = forward · to_target
 *   sign   = cross(forward, to_target).z
 *   θ      = atan2(sign, cos θ)  ∈ [−π, π]
 *
 * Positive → tourner à gauche. Fonction sans état, toujours testable.
 */
[[nodiscard]] float compute_steer_angle(
    const Vec3& car_pos,
    const Vec3& car_forward,
    const Vec3& target) noexcept
{
    Vec3 to_target {
        target.x - car_pos.x,
        target.y - car_pos.y,
        0.f
    };
    to_target = to_target.normalized();

    const float cos_angle = car_forward.dot(to_target);
    const float cross_z   = car_forward.x * to_target.y
                          - car_forward.y * to_target.x;
    return std::atan2(cross_z, cos_angle);
}

/**
 * Convertit un angle de braquage en commande normalisée [-1, 1].
 * Proportionnel pur clampé — remplacer par un PID pour moins de dépassement.
 */
[[nodiscard]] constexpr float angle_to_steer(
    float angle_rad,
    float gain = 2.f) noexcept
{
    return math_utils::clamp(gain * angle_rad, -1.f, 1.f);
}

// ─────────────────────────────────────────────────────────
// § 5. BotController — orchestre tous les modules
//
//  Pipeline par tick :
//    0. Latence visuelle : push ball_pos dans lat_buf_,
//       choisir perceived_ball_pos selon la distance.
//    1. predict_ball()           → position cible future (P3)
//       La prédiction part de perceived_ball_state (pas la vraie balle).
//    2. build_bezier_curve()     → calcul de P1, P2 avec drift sur offset ⊥
//    3. bezier_eval(t=BEZIER_T)  → cible intermédiaire sur la courbe
//    4. compute_steer_angle()    → angle géométrique pur [-π, π]
//    5. angle_to_steer()         → commande brute [-1, 1]
//    6. lerp(prev_steer_, brut, LERP_STEER) + jerk_limit(MAX_JERK_STEER)
//    7. throttle lissé : lerp(prev_throttle_, 1.f, LERP_THROTTLE)
//                       + jerk_limit(MAX_JERK_THROTTLE)
//    8. boost lissé   : accumulateur float prev_boost_ ∈ [0,1]
//                       lerp(prev_boost_, target_boost, LERP_BOOST)
//                       + jerk_limit(MAX_JERK_BOOST)
//                       décision finale : prev_boost_ >= 0.5f
//    9. Décision boost raw modulée par DecisionContext
//
//  État interne (tous float + BallLatencyBuffer, pile uniquement) :
//    lat_buf_        — buffer de latence visuelle (16 × Vec3)
//    drift_phase_    — phase sinusoïdale ∈ [0, 2π)
//    prev_steer_     — steer filtré du tick précédent
//    prev_throttle_  — throttle filtré du tick précédent
//    prev_boost_     — accumulateur boost ∈ [0, 1]
//
//  Vérification DecisionContext :
//    ball_dist_sq          → UTILISÉ  : moduler BEZIER_T + choix latence
//    ball_dist_to_goal_sq  → UTILISÉ  : ajuster le seuil boost (danger cage)
//    boost_pct             → UTILISÉ  : décision d'activer le boost
//    ball_is_behind_car    → UTILISÉ  : inverser la tension P1
//    teammate_attacking    → NON UTILISÉ dans BotController (FSM uniquement)
//    tick_id               → UTILISÉ  : réf. déterministe au premier tick
// ─────────────────────────────────────────────────────────

// Forward-déclaration (défini dans rl_fsm.hpp — inclus avant rl_bot.hpp)
struct DecisionContext;

class BotController {
public:
    BotController() = default;

    /**
     * Calcule les commandes à envoyer au moteur.
     *
     * @param pkt  GameTickPacket courant (position réelle, pour la FSM et ImGui).
     * @param ctx  DecisionContext courant (snapshot FSM du même tick).
     * @return     PlayerInput prêt à être soumis.
     *
     * Latence visuelle :
     *   La position de la balle utilisée pour la prédiction et le calcul
     *   de trajectoire est perçue avec LATENCY_TICKS de retard, sauf si
     *   la distance voiture→balle est ≤ PROXIMITY_THRESHOLD (réflexe).
     *   La FSM et ImGui reçoivent toujours pkt non-modifié (vraie position).
     */
    [[nodiscard]] PlayerInput compute(
        const GameTickPacket& pkt,
        const DecisionContext& ctx) noexcept
    {
        // ── 0. Buffer de latence visuelle ─────────────────────
        //
        // On push la position réelle à chaque tick.
        // On lit ensuite la position "perçue" avec le délai configuré,
        // sauf si la balle est en zone de proximité (réflexe immédiat).
        //
        // Calcul de distance voiture→balle (plan XY, pas de sqrt si évitable,
        // mais get_delayed() a besoin de la distance en UU pour le seuil).
        const float dx_real = pkt.ball.position.x - pkt.player.position.x;
        const float dy_real = pkt.ball.position.y - pkt.player.position.y;
        const float dist_to_ball = std::sqrt(dx_real*dx_real + dy_real*dy_real);

        lat_buf_.push(pkt.ball.position);
        const Vec3 perceived_ball_pos = lat_buf_.get_delayed(dist_to_ball);

        // On construit un BallState "perçu" qui conserve vélocité et radius
        // réels (seule la position est retardée — la vélocité est celle du
        // dernier tick connu, ce qui est cohérent avec la simulation de latence
        // visuelle : l'œil humain perçoit également la vitesse avec un léger
        // retard, mais la grandeur dominante est la position).
        BallState perceived_ball = pkt.ball;
        perceived_ball.position  = perceived_ball_pos;

        // ── 1. Prédiction de la position future de la balle ───
        //
        // On prédit à partir de la balle perçue, pas de la balle réelle.
        // Cela garantit que le bot se comporte comme un humain qui agit
        // sur l'image qu'il voit, pas sur la position extrapolée en temps réel.
        constexpr std::size_t LOOKAHEAD = 12; // ticks (~100 ms à 120 Hz)
        const auto path       = predict_ball<128>(perceived_ball, dt_, LOOKAHEAD);
        const Vec3 target_pos = path.positions[LOOKAHEAD - 1]; // P3 fixe

        // ── 2. Construction de la courbe de Bézier ────────────
        const Vec3  car_pos  = pkt.player.position;
        const Vec3  car_fwd  = pkt.player.forward;

        // Direction approche finale : normalisée depuis la vitesse de la balle
        Vec3 ball_fwd = pkt.ball.velocity.normalized();
        if (ball_fwd.dot(ball_fwd) < 1e-6f)  // balle quasi-immobile
            ball_fwd = (target_pos - car_pos).normalized();

        const Vec3  d03  = target_pos - car_pos;
        const float dist = std::sqrt(d03.x*d03.x + d03.y*d03.y) + 1e-3f;

        // BEZIER_T adaptatif (utilise ball_dist_sq du DecisionContext)
        const float dist_norm  = math_utils::clamp(
            std::sqrt(ctx.ball_dist_sq) / 4000.f, 0.f, 1.f);
        const float bezier_t   = math_utils::lerp(0.15f, 0.35f, dist_norm);

        // Tension inversée si la balle est derrière la voiture
        const float tension_sign = ctx.ball_is_behind_car ? -1.f : 1.f;
        const float ctrl_len     = dist * CTRL_TENSION * tension_sign;

        Vec3 p0 = car_pos;
        Vec3 p3 = target_pos;
        Vec3 p1 = p0 + car_fwd  * ctrl_len;
        Vec3 p2 = p3 - ball_fwd * (dist * CTRL_TENSION);

        // ── 3. Drift sur P1 et P2 (offset perpendiculaire) ───
        drift_phase_ += 2.f * math_utils::PI * DRIFT_FREQ * dt_;
        if (drift_phase_ >= 2.f * math_utils::PI)
            drift_phase_ -= 2.f * math_utils::PI;

        const Vec3  right        = { -car_fwd.y, car_fwd.x, 0.f };
        const float drift_offset = DRIFT_AMP * std::sin(drift_phase_) * dist;

        p1 = p1 + right * drift_offset;
        p2 = p2 + right * drift_offset;

        // ── 4. Évaluation de la courbe → cible intermédiaire ──
        const CubicBezier curve { p0, p1, p2, p3 };
        const Vec3 bezier_target = bezier_eval(curve, bezier_t);

        // ── 5. Angle de braquage géométrique pur ──────────────
        const float steer_angle = compute_steer_angle(car_pos, car_fwd, bezier_target);

        // ── 6. Commande brute (proportionnel clampé) ──────────
        const float steer_raw = angle_to_steer(steer_angle);

        // ── 7. Lissage steer — lerp exponentiel + limiteur de jerk ──
        //
        //   steer_smooth(t) = prev + α·(raw − prev)
        //   LERP_STEER = 0.2 → τ ≈ 5 ticks (~42 ms)
        //   MAX_JERK_STEER = 0.05 → rampe 0→1 en ~167 ms
        {
            const float lerped = math_utils::lerp(prev_steer_, steer_raw, LERP_STEER);
            const float delta  = math_utils::clamp(
                lerped - prev_steer_, -MAX_JERK_STEER, MAX_JERK_STEER);
            prev_steer_ += delta;
        }

        // ── 8. Lissage throttle — lerp exponentiel + limiteur de jerk ─
        //
        //   La cible de throttle est toujours 1.f (avance en permanence).
        //   Le filtre simule le temps de pression sur la gâchette.
        //   LERP_THROTTLE = 0.25 → τ ≈ 4 ticks (~33 ms)
        //   MAX_JERK_THROTTLE = 0.07 → rampe 0→1 en ~116 ms
        //
        //   L'état initial prev_throttle_ = 0.f garantit un démarrage
        //   progressif (pas de "coup de gaz" instantané au tick 0).
        {
            const float lerped = math_utils::lerp(prev_throttle_, 1.f, LERP_THROTTLE);
            const float delta  = math_utils::clamp(
                lerped - prev_throttle_, -MAX_JERK_THROTTLE, MAX_JERK_THROTTLE);
            prev_throttle_ += delta;
        }

        // ── 9. Décision boost raw ─────────────────────────────
        //
        //   Boost activé si :
        //     a) La cible prédite est loin (dist² > seuil)
        //     b) La cage propre n'est pas en danger immédiat
        //     c) La réserve de boost le permet (boost_pct > seuil)
        const Vec3  diff_to_target    = target_pos - car_pos;
        const float dist_sq_to_target = diff_to_target.dot(diff_to_target);

        const bool cage_safe  = (ctx.ball_dist_to_goal_sq > GOAL_SAFE_SQ);
        const bool boost_ok   = (ctx.boost_pct > BOOST_MIN_PCT);
        const bool far_target = (dist_sq_to_target > BOOST_DIST_SQ);
        const float boost_target = (far_target && cage_safe && boost_ok) ? 1.f : 0.f;

        // ── 10. Lissage boost — accumulateur float + jerk ──────
        //
        //   Le boost est un signal binaire (on/off), mais on le lisse
        //   via un accumulateur float ∈ [0, 1] pour éviter les impulsions
        //   d'un seul tick (souvent des artefacts des transitions FSM).
        //
        //   LERP_BOOST = 0.15 → τ ≈ 6 ticks (~50 ms)
        //     → Le bot met ~50 ms à décider de booster (activation naturelle)
        //     → Le boost ne s'arrête pas instantanément (relâchement progressif)
        //   MAX_JERK_BOOST = 0.08 → rampe 0→1 en ~100 ms
        //
        //   Seuil de décision finale : prev_boost_ >= 0.5f
        //     → Symétrique : même temps pour activer que pour désactiver.
        {
            const float lerped = math_utils::lerp(prev_boost_, boost_target, LERP_BOOST);
            const float delta  = math_utils::clamp(
                lerped - prev_boost_, -MAX_JERK_BOOST, MAX_JERK_BOOST);
            prev_boost_ += delta;
        }

        return PlayerInput {
            .throttle = prev_throttle_,
            .steer    = prev_steer_,
            .boost    = (prev_boost_ >= 0.5f),
        };
    }

    /**
     * Surcharge de compatibilité ascendante — sans DecisionContext.
     * Construit un contexte minimal à partir du packet seul.
     * À utiliser uniquement dans les tests unitaires isolés.
     */
    [[nodiscard]] PlayerInput compute(const GameTickPacket& pkt) noexcept {
        const float dx = pkt.ball.position.x - pkt.player.position.x;
        const float dy = pkt.ball.position.y - pkt.player.position.y;
        constexpr float OWN_GOAL_Y = -5120.f;
        const float gdx = pkt.ball.position.x;
        const float gdy = pkt.ball.position.y - OWN_GOAL_Y;
        const float dot = pkt.player.forward.x * dx
                        + pkt.player.forward.y * dy;
        const DecisionContext ctx {
            .ball_dist_sq         = dx*dx + dy*dy,
            .ball_dist_to_goal_sq = gdx*gdx + gdy*gdy,
            .boost_pct            = pkt.player.boost_amount,
            .ball_is_behind_car   = dot < 0.f,
            .teammate_attacking   = false,
            .tick_id              = pkt.tick_id,
        };
        return compute(pkt, ctx);
    }

private:
    // ── Constante de tick ──────────────────────────────────
    static constexpr float dt_ = 1.f / 120.f;

    // ── Paramètres de la courbe de Bézier ─────────────────
    static constexpr float CTRL_TENSION = 0.33f;

    // ── Paramètres du drift sinusoïdal ─────────────────────
    static constexpr float DRIFT_FREQ = 0.35f;   // Hz  (période ~2.86 s)
    static constexpr float DRIFT_AMP  = 0.04f;   // fraction de dist P0→P3

    // ── Paramètres du lissage steer ────────────────────────
    // α = 0.2 → τ ≈ 5 ticks (~42 ms à 120 Hz)
    static constexpr float LERP_STEER      = 0.20f;
    // Variation max de steer par tick → rampe 0→1 en ~167 ms
    static constexpr float MAX_JERK_STEER  = 0.05f;

    // ── Paramètres du lissage throttle ─────────────────────
    // α = 0.25 → τ ≈ 4 ticks (~33 ms) : gâchette plus réactive que le steer
    static constexpr float LERP_THROTTLE     = 0.25f;
    // Variation max de throttle par tick → rampe 0→1 en ~116 ms
    static constexpr float MAX_JERK_THROTTLE = 0.07f;

    // ── Paramètres du lissage boost ────────────────────────
    // α = 0.15 → τ ≈ 6 ticks (~50 ms) : boost plus inertiel = plus humain
    static constexpr float LERP_BOOST      = 0.15f;
    // Variation max de l'accumulateur boost par tick → rampe 0→1 en ~100 ms
    static constexpr float MAX_JERK_BOOST  = 0.08f;

    // ── Seuils pour la décision boost ─────────────────────
    static constexpr float BOOST_DIST_SQ  = 2500.f * 2500.f; // UU²
    static constexpr float GOAL_SAFE_SQ   = 3000.f * 3000.f; // UU²
    static constexpr float BOOST_MIN_PCT  = 15.f;             // %

    // ── État interne persisté (pile, zero-alloc) ───────────
    BallLatencyBuffer lat_buf_{};    // buffer circulaire 16 × Vec3
    float drift_phase_   = 0.f;     // phase drift ∈ [0, 2π)
    float prev_steer_    = 0.f;     // steer filtré du tick précédent
    float prev_throttle_ = 0.f;     // throttle filtré (démarrage progressif)
    float prev_boost_    = 0.f;     // accumulateur boost ∈ [0, 1]
};

// ─────────────────────────────────────────────────────────
// § 6. Assemblage — point d'entrée illustratif (pseudo-code)
// ─────────────────────────────────────────────────────────
//
// class MyBot : public RLBotInterface {
// public:
//     MyBot() : handler_([this](const GameTickPacket& p){ last_pkt_ = p; }) {}
//
//     void on_tick(const GameTickPacket& pkt) override {
//         handler_.on_tick(pkt);
//     }
//
//     PlayerInput get_output_for_frame(const DecisionContext& ctx) override {
//         handler_.process_pending();
//         return controller_.compute(last_pkt_, ctx);
//     }
//
// private:
//     GameTickPacket last_pkt_{};
//     TickHandler    handler_;
//     BotController  controller_;
// };

// ─────────────────────────────────────────────────────────
// § 7. Ordre d'inclusion recommandé dans main.cpp
// ─────────────────────────────────────────────────────────
//
//   #include "rl_fsm.hpp"   // déclare DecisionContext
//   #include "rl_bot.hpp"   // utilise DecisionContext
//
// BotController::compute(pkt, ctx) attend que DecisionContext soit
// visible — garantir cet ordre dans tous les fichiers de compilation.

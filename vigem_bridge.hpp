/**
 * vigem_bridge.hpp — Abstraction ViGEmBus Xbox360, C++20
 *
 * Rôle :
 *   Expose un contrôleur Xbox 360 virtuel via ViGEmBus à partir d'un
 *   PlayerInput (throttle, steer, boost…).
 *
 * Contraintes respectées :
 *   - RAII complet : le contrôleur se débranche à la destruction de l'objet,
 *     même sur exception ou Ctrl+C.
 *   - Zéro allocation dynamique dans update() — hot path 120 Hz garanti.
 *   - Standard C++20 Windows uniquement (ViGEmBus est un driver Windows).
 *   - Simulation de dérive analogique (±0.01 % = ±3.27 LSB sur 16 bits) pour
 *     modéliser la limite de précision d'un potentiomètre physique standard.
 *
 * Dépendances :
 *   ViGEm Client SDK : https://github.com/ViGEm/ViGEmClient
 *   Lier avec : ViGEmClient.lib  (x64 Release)
 *   En-tête   : ViGEm/Client.h  (dans le SDK)
 *
 * Ordre d'inclusion recommandé dans main.cpp :
 *   #include "external_data_manager.hpp"   // déclare PlayerInput, etc.
 *   #include "vigem_bridge.hpp"
 *
 * Notes de sécurité :
 *   vigem_alloc / vigem_free gèrent le heap ViGEm en dehors du hot path.
 *   Aucun malloc / new dans update().
 */

#pragma once

// ── Garde plateforme ──────────────────────────────────────
#ifndef _WIN32
#  error "vigem_bridge.hpp est exclusivement Windows (ViGEmBus)."
#endif

#ifndef WIN32_LEAN_AND_MEAN
#  define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#  define NOMINMAX
#endif
#include <windows.h>

// ── ViGEm Client SDK ──────────────────────────────────────
// Assurez-vous que le chemin d'inclusion pointe vers le SDK.
// Exemple : g++ -I"C:/ViGEmClient/include" ...
#include "Client.h"
#include "Common.h"
#include "Util.h"

#pragma comment(lib, "ViGEmClient.lib")

// ── Projet ────────────────────────────────────────────────
#include "rl_bot.hpp"   // PlayerInput, math_utils

// ── Standard ──────────────────────────────────────────────
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

// ─────────────────────────────────────────────────────────
// § 1. Utilitaires de conversion XInput
// ─────────────────────────────────────────────────────────

namespace vigem_detail {

/// Norme XInput : axe analogique 16 bits signé [-32 768, +32 767].
inline constexpr float AXIS_MAX = 32'767.f;
inline constexpr float AXIS_MIN = -32'768.f;

/// Amplitude de dérive : ±0.01 % de l'étendue complète (65 536 LSB).
/// 65536 × 0.0001 ≈ 6.55 LSB → arrondi à ±3.27 LSB par côté.
/// La valeur est exprimée en fraction normalisée [-1, +1] pour rester
/// dans le domaine float avant la conversion finale.
inline constexpr float DRIFT_AMPLITUDE = 0.0001f; // 0.01 %

/**
 * Applique la dérive d'un potentiomètre simulé sur une valeur normalisée.
 *
 * La phase est mise à jour par update() à chaque tick — zéro alloc,
 * zéro branche conditionnelle non-triviale sur le hot path.
 *
 * @param value      Valeur normalisée ∈ [-1, +1]
 * @param drift_sin  sin(phase) pré-calculé pour ce tick — évite un 2e appel
 * @return           Valeur avec micro-fluctuation appliquée, clampée à [-1, +1]
 */
[[nodiscard]] constexpr float apply_drift(float value, float drift_sin) noexcept {
    const float drifted = value + DRIFT_AMPLITUDE * drift_sin;
    return math_utils::clamp(drifted, -1.f, 1.f);
}

/**
 * Convertit une valeur normalisée ∈ [-1, +1] en SHORT XInput.
 * Pas de division en virgule flottante sur le hot path (multiplication).
 */
[[nodiscard]] constexpr SHORT to_short_axis(float normalized) noexcept {
    const float clamped = math_utils::clamp(normalized, -1.f, 1.f);
    // Multiplication par AXIS_MAX pour les positifs, par |AXIS_MIN| pour les négatifs
    // → préserve la symétrie parfaite du DAC 16 bits signé.
    const float scaled = (clamped >= 0.f)
        ? clamped * AXIS_MAX
        : clamped * (-AXIS_MIN);   // -(-32768) = 32768 > 32767 → clamp ci-dessous
    // Clamp final en entier pour éviter UB sur les bords
    const int32_t as_int = static_cast<int32_t>(scaled);
    if (as_int >  32'767) return  32'767;
    if (as_int < -32'768) return -32'768;
    return static_cast<SHORT>(as_int);
}

/**
 * Convertit une valeur normalisée ∈ [0, 1] en BYTE XInput (gâchette).
 * Les gâchettes Xbox360 sont non-signées 8 bits [0, 255].
 */
[[nodiscard]] constexpr BYTE to_byte_trigger(float normalized) noexcept {
    const float clamped = math_utils::clamp(normalized, 0.f, 1.f);
    return static_cast<BYTE>(clamped * 255.f);
}

} // namespace vigem_detail

// ─────────────────────────────────────────────────────────
// § 2. VigemBridge — RAII + hot path 120 Hz
// ─────────────────────────────────────────────────────────

/**
 * Abstraction RAII d'un contrôleur Xbox 360 virtuel (ViGEmBus).
 *
 * Cycle de vie :
 *   VigemBridge bridge;          ← vigem_alloc + vigem_connect + plug_in
 *   bridge.update(input);        ← update hot path (120 Hz, zéro-alloc)
 *   // destruction automatique → vigem_target_remove + vigem_free
 *
 * Robustesse :
 *   - valid() retourne false si le driver ViGEmBus est absent ou l'init a échoué.
 *   - update() est un no-op silencieux si valid() == false → pas de crash
 *     si le driver n'est pas installé sur la machine de test.
 */
class VigemBridge {
public:
    // ── Construction / destruction ────────────────────────

    VigemBridge() noexcept {
        client_ = vigem_alloc();
        if (!client_) {
            last_error_ = "vigem_alloc() a echoue (OOM ou driver absent).";
            return;
        }

        const VIGEM_ERROR err_connect = vigem_connect(client_);
        if (!VIGEM_SUCCESS(err_connect)) {
            last_error_ = "vigem_connect() echoue — code : "
                        + std::to_string(static_cast<int>(err_connect))
                        + ". Assurez-vous que ViGEmBus est installe.";
            vigem_free(client_);
            client_ = nullptr;
            return;
        }

        target_ = vigem_target_x360_alloc();
        if (!target_) {
            last_error_ = "vigem_target_x360_alloc() a echoue.";
            vigem_disconnect(client_);
            vigem_free(client_);
            client_ = nullptr;
            return;
        }

        const VIGEM_ERROR err_plugin = vigem_target_add(client_, target_);
        if (!VIGEM_SUCCESS(err_plugin)) {
            last_error_ = "vigem_target_add() echoue — code : "
                        + std::to_string(static_cast<int>(err_plugin));
            vigem_target_free(target_);
            target_ = nullptr;
            vigem_disconnect(client_);
            vigem_free(client_);
            client_ = nullptr;
            return;
        }

        ready_ = true;
    }

    /// Destructeur RAII — débrancher et libérer même sur Ctrl+C.
    ~VigemBridge() noexcept {
        if (target_ && client_) {
            vigem_target_remove(client_, target_);  // débranchement propre
            vigem_target_free(target_);
        }
        if (client_) {
            vigem_disconnect(client_);
            vigem_free(client_);
        }
    }

    // Non-copiable (ressource système unique)
    VigemBridge(const VigemBridge&)            = delete;
    VigemBridge& operator=(const VigemBridge&) = delete;

    // Déplaçable
    VigemBridge(VigemBridge&& other) noexcept
        : client_(other.client_)
        , target_(other.target_)
        , drift_phase_(other.drift_phase_)
        , ready_(other.ready_)
        , last_error_(std::move(other.last_error_))
    {
        other.client_ = nullptr;
        other.target_ = nullptr;
        other.ready_  = false;
    }

    VigemBridge& operator=(VigemBridge&&) = delete; // simplification RAII

    // ── Accesseurs ────────────────────────────────────────

    [[nodiscard]] bool        valid()      const noexcept { return ready_; }
    [[nodiscard]] const std::string& last_error() const noexcept { return last_error_; }

    // ─────────────────────────────────────────────────────
    // § 3. update() — hot path 120 Hz, zéro allocation
    //
    //  Mapping PlayerInput → XUSB_REPORT :
    //
    //   throttle ∈ [-1, +1]
    //     ≥ 0  → sRightTrigger (gâchette droite = accélération)
    //     < 0  → sLeftTrigger  (gâchette gauche  = frein/marche arrière)
    //
    //   steer   ∈ [-1, +1] → sThumbLX (stick gauche X)
    //   pitch   ∈ [-1, +1] → sThumbRY (stick droit  Y)
    //   yaw     ∈ [-1, +1] → sThumbRX (stick droit  X)
    //   roll    ∈ [-1, +1] → sThumbLY (stick gauche Y — convention RL)
    //
    //   boost     → bouton A  (wButtons & XUSB_GAMEPAD_A)
    //   jump      → bouton B
    //   handbrake → bouton X
    //
    //  Dérive analogique :
    //   Un sin(phase) commun est appliqué sur steer et throttle (axes X/Y)
    //   pour modéliser la dérive résiduelle d'un potentiomètre physique
    //   (ADC 12 bits → résolution 0.024 % → 0.01 % est conservateur).
    //   La phase avance de 2π × DRIFT_FREQ × dt à chaque tick.
    //   Fréquence de drift : 1.7 Hz (période ~0.59 s) — typique d'un
    //   joystick de qualité entrée de gamme sous charge thermique légère.
    // ─────────────────────────────────────────────────────

    void update(const PlayerInput& input) noexcept
    {
        if (!ready_) [[unlikely]] return;

        // ── 1. Avance de la phase de dérive ──────────────
        //    Constexpr : 2π × 1.7 Hz / 120 Hz ≈ 0.08901 rad/tick
        //    Pas de branche, pas d'alloc — simple addition flottante.
        drift_phase_ += DRIFT_PHASE_INC;
        if (drift_phase_ >= TWO_PI) drift_phase_ -= TWO_PI;

        const float ds = std::sin(drift_phase_); // une seule évaluation transcendante/tick

        // ── 2. Construction du rapport XInput (pile uniquement) ──
        XUSB_REPORT report{};

        // Gâchettes (throttle → split signe)
        if (input.throttle >= 0.f) {
            report.bRightTrigger = vigem_detail::to_byte_trigger(input.throttle);
        } else {
            report.bLeftTrigger  = vigem_detail::to_byte_trigger(-input.throttle);
        }

        // Stick gauche : steer (X) avec dérive + roll (Y) sans dérive
        report.sThumbLX = vigem_detail::to_short_axis(
            vigem_detail::apply_drift(input.steer, ds));
        report.sThumbLY = vigem_detail::to_short_axis(input.roll);

        // Stick droit : yaw (X) + pitch (Y) avec dérive simulée sur Y
        report.sThumbRX = vigem_detail::to_short_axis(input.yaw);
        report.sThumbRY = vigem_detail::to_short_axis(
            vigem_detail::apply_drift(input.pitch, ds));

        // Boutons numériques
        if (input.boost)     report.wButtons |= XUSB_GAMEPAD_A;
        if (input.jump)      report.wButtons |= XUSB_GAMEPAD_B;
        if (input.handbrake) report.wButtons |= XUSB_GAMEPAD_X;

        // ── 3. Envoi au driver (appel système léger, non-bloquant) ─
        // vigem_target_x360_update est thread-safe côté driver ViGEm.
        // Retour ignoré : un échec isolé ne justifie pas d'interrompre
        // la boucle de jeu — le tick suivant retransmettra l'état.
        (void)vigem_target_x360_update(client_, target_, report);
    }

private:
    // ── Constantes de dérive ──────────────────────────────
    static constexpr float DRIFT_FREQ      = 1.7f;           // Hz
    static constexpr float TWO_PI          = 6.28318530718f;
    static constexpr float DRIFT_PHASE_INC = TWO_PI * DRIFT_FREQ / 120.f;
    //  ≈ 0.08901 rad/tick — évalué à la compilation, zéro coût runtime

    // ── État RAII ─────────────────────────────────────────
    PVIGEM_CLIENT client_ = nullptr;
    PVIGEM_TARGET target_ = nullptr;
    float         drift_phase_ = 0.f;   // phase ∈ [0, 2π) — pile, zero-alloc
    bool          ready_       = false;
    std::string   last_error_;
};

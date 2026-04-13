/**
 * external_data_manager.hpp — Lecteur de télémétrie non-intrusif, C++20
 *
 * Principe :
 *   Lit les données d'un processus cible via ReadProcessMemory (Windows).
 *   Le handle est ouvert, utilisé pour la copie, puis fermé automatiquement
 *   via RAII (ProcessHandle) — zéro ressource système résiduelle garanti
 *   même en cas d'erreur de lecture ou d'exception.
 *
 * Architecture :
 *   - ProcessHandle       : RAII wrapper sur HANDLE Windows (CloseHandle garanti)
 *   - SnapshotHandle      : RAII wrapper sur CreateToolhelp32Snapshot
 *   - MemoryOffsets       : table d'adresses configurables (make_default / from_config)
 *   - ExternalDataManager : ouvre / lit / ferme via RAII à chaque cycle
 *   - TelemetryFrame      : snapshot POD produit à chaque lecture
 *   - ConfigLoader        : les offsets peuvent être chargés depuis config.txt
 *
 * Couplage avec rl_bot.hpp :
 *   TelemetryFrame::to_game_tick_packet() convertit le snapshot en
 *   GameTickPacket compatible avec BotController::compute() et la FSM.
 *
 * Linkage Windows (OBLIGATOIRE) :
 *   Ce fichier émet automatiquement :
 *     #pragma comment(lib, "psapi.lib")
 *   ce qui suffit avec MSVC et MinGW/MSYS2.
 *
 *   Avec g++ (MinGW ou cross-compiler), ajouter explicitement en ligne de commande :
 *     -lpsapi
 *   Exemple complet :
 *     g++ -std=c++20 -O2 -DHEADLESS_MODE
 *         -I"C:/ViGEmClient/include"
 *         main.cpp -o rl_tool.exe
 *         -lpsapi
 *         -L"C:/ViGEmClient/lib/x64" -lViGEmClient
 *
 *   Sans -lpsapi : erreur de linkage sur EnumProcessModules / GetModuleBaseNameA.
 *   Sans -lViGEmClient : erreur de linkage sur vigem_alloc / vigem_connect.
 *
 * Sécurité :
 *   PROCESS_VM_READ est le droit minimal : aucune écriture possible dans le
 *   processus cible. Aucun shellcode, aucun hook — lecture mémoire pure.
 *
 * Portabilité :
 *   Sur Linux/macOS, ce fichier est un no-op (stub avec last_error_ renseigné).
 *   Remplacer read_frame() par une implémentation /proc/<pid>/mem ou ptrace.
 */

#pragma once

// ── Ordre important : rl_fsm.hpp avant rl_bot.hpp ─────────
#include "rl_fsm.hpp"
#include "rl_bot.hpp"
#include "config_loader.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>

// ── Détection plateforme ───────────────────────────────────
#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#  include <tlhelp32.h>
#  include <psapi.h>
#  pragma comment(lib, "psapi.lib")
   using NativeHandle = HANDLE;
   inline constexpr NativeHandle INVALID_NATIVE_HANDLE = nullptr;
#else
#  include <cstdio>
   using NativeHandle = int;
   inline constexpr NativeHandle INVALID_NATIVE_HANDLE = -1;
#  warning "ExternalDataManager : ReadProcessMemory non disponible hors Windows."
#endif

// ─────────────────────────────────────────────────────────
// § 1. Quaternion (orientation 3D)
// ─────────────────────────────────────────────────────────

struct Quat {
    float w{1.f}, x{}, y{}, z{};

    /// Convertit le quaternion en vecteur "forward" (axe Y local → monde).
    [[nodiscard]] Vec3 to_forward() const noexcept {
        return Vec3 {
            2.f * (x*y - w*z),
            1.f - 2.f * (x*x + z*z),
            2.f * (y*z + w*x)
        };
    }

    /// Convertit en vecteur "up" (axe Z local → monde).
    [[nodiscard]] Vec3 to_up() const noexcept {
        return Vec3 {
            2.f * (x*z + w*y),
            2.f * (y*z - w*x),
            1.f - 2.f * (x*x + y*y)
        };
    }
};

static_assert(std::is_trivially_copyable_v<Quat>);

// ─────────────────────────────────────────────────────────
// § 2. RAII wrapper sur le handle de processus Windows
//
//  Garantit que CloseHandle() est appelé exactement une fois,
//  même si une lecture échoue à mi-parcours.
//  Non-copiable, déplaçable (move semantics).
// ─────────────────────────────────────────────────────────

#ifdef _WIN32

class ProcessHandle {
public:
    /// Constructeur : ouvre un handle PROCESS_VM_READ sur le PID donné.
    explicit ProcessHandle(DWORD pid) noexcept
        : handle_(OpenProcess(PROCESS_VM_READ, FALSE, pid))
    {}

    /// Constructeur depuis un HANDLE déjà ouvert (ownership transféré).
    explicit ProcessHandle(HANDLE h) noexcept : handle_(h) {}

    /// Destructeur : fermeture garantie — le cœur du RAII.
    ~ProcessHandle() noexcept {
        close();
    }

    // Non-copiable : un seul propriétaire du handle
    ProcessHandle(const ProcessHandle&)            = delete;
    ProcessHandle& operator=(const ProcessHandle&) = delete;

    // Déplaçable : transfert de propriété
    ProcessHandle(ProcessHandle&& other) noexcept
        : handle_(other.handle_) { other.handle_ = nullptr; }

    ProcessHandle& operator=(ProcessHandle&& other) noexcept {
        if (this != &other) {
            close();
            handle_       = other.handle_;
            other.handle_ = nullptr;
        }
        return *this;
    }

    /// Retourne true si le handle est valide (ouverture réussie).
    [[nodiscard]] bool valid()  const noexcept { return handle_ != nullptr; }
    [[nodiscard]] HANDLE get()  const noexcept { return handle_; }

    /// Libération explicite anticipée (optionnel — le destructeur suffit).
    void close() noexcept {
        if (handle_ != nullptr) {
            CloseHandle(handle_);
            handle_ = nullptr;
        }
    }

private:
    HANDLE handle_ = nullptr;
};

/// RAII wrapper sur un snapshot TH32 (CreateToolhelp32Snapshot).
class SnapshotHandle {
public:
    explicit SnapshotHandle() noexcept
        : handle_(CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0))
    {}

    ~SnapshotHandle() noexcept {
        if (handle_ != INVALID_HANDLE_VALUE)
            CloseHandle(handle_);
    }

    SnapshotHandle(const SnapshotHandle&)            = delete;
    SnapshotHandle& operator=(const SnapshotHandle&) = delete;
    SnapshotHandle(SnapshotHandle&&)                 = delete;
    SnapshotHandle& operator=(SnapshotHandle&&)      = delete;

    [[nodiscard]] bool   valid() const noexcept { return handle_ != INVALID_HANDLE_VALUE; }
    [[nodiscard]] HANDLE get()   const noexcept { return handle_; }

private:
    HANDLE handle_;
};

#endif // _WIN32

// ─────────────────────────────────────────────────────────
// § 3. Table d'offsets mémoire
// ─────────────────────────────────────────────────────────

struct ObjectOffsets {
    uintptr_t ptr_offset{};
    uintptr_t pos_offset{};
    uintptr_t vel_offset{};
    uintptr_t ang_vel_offset{};
    uintptr_t rot_offset{};
    uintptr_t boost_offset{};
    uintptr_t grounded_offset{};
};

struct MemoryOffsets {
    std::string  process_name{"RocketLeague.exe"};
    uintptr_t    module_base_offset{};

    ObjectOffsets ball{};
    ObjectOffsets car{};

    uintptr_t time_remaining_ptr_offset{};
    uintptr_t time_remaining_field_offset{};

    /// Offsets par défaut — à calibrer avec Cheat Engine.
    [[nodiscard]] static MemoryOffsets make_default() noexcept {
        MemoryOffsets o;
        o.process_name             = "RocketLeague.exe";
        o.ball.ptr_offset          = 0x02A4B0E0;
        o.ball.pos_offset          = 0x7C;
        o.ball.vel_offset          = 0x88;
        o.ball.ang_vel_offset      = 0x94;
        o.ball.rot_offset          = 0xA0;
        o.car.ptr_offset           = 0x02A4C1F0;
        o.car.pos_offset           = 0x7C;
        o.car.vel_offset           = 0x88;
        o.car.rot_offset           = 0xA0;
        o.car.boost_offset         = 0x1A8;
        o.car.grounded_offset      = 0x1B0;
        o.time_remaining_ptr_offset   = 0x02A50000;
        o.time_remaining_field_offset = 0x10;
        return o;
    }

    /**
     * Construit les offsets depuis un ConfigLoader déjà chargé.
     * Les clés absentes conservent les valeurs par défaut.
     *
     * Exemple de config.txt :
     *   PROCESS_NAME=RocketLeague.exe
     *   BALL_PTR=0x02A4B0E0
     *   BALL_POS=0x7C
     *   CAR_PTR=0x02A4C1F0
     *   CAR_BOOST=0x1A8
     */
    static MemoryOffsets from_config(const ConfigLoader& cfg) noexcept {
        MemoryOffsets o = make_default();

        // Surcharger uniquement les clés présentes dans le fichier
        o.ball.ptr_offset     = cfg.get_value_or_default("BALL_PTR",      o.ball.ptr_offset);
        o.ball.pos_offset     = cfg.get_value_or_default("BALL_POS",      o.ball.pos_offset);
        o.ball.vel_offset     = cfg.get_value_or_default("BALL_VEL",      o.ball.vel_offset);
        o.ball.ang_vel_offset = cfg.get_value_or_default("BALL_ANG_VEL",  o.ball.ang_vel_offset);
        o.ball.rot_offset     = cfg.get_value_or_default("BALL_ROT",      o.ball.rot_offset);

        o.car.ptr_offset      = cfg.get_value_or_default("CAR_PTR",       o.car.ptr_offset);
        o.car.pos_offset      = cfg.get_value_or_default("CAR_POS",       o.car.pos_offset);
        o.car.vel_offset      = cfg.get_value_or_default("CAR_VEL",       o.car.vel_offset);
        o.car.rot_offset      = cfg.get_value_or_default("CAR_ROT",       o.car.rot_offset);
        o.car.boost_offset    = cfg.get_value_or_default("CAR_BOOST",     o.car.boost_offset);
        o.car.grounded_offset = cfg.get_value_or_default("CAR_GROUNDED",  o.car.grounded_offset);

        o.time_remaining_ptr_offset   =
            cfg.get_value_or_default("TIME_PTR",   o.time_remaining_ptr_offset);
        o.time_remaining_field_offset =
            cfg.get_value_or_default("TIME_FIELD", o.time_remaining_field_offset);

        return o;
    }
};

// ─────────────────────────────────────────────────────────
// § 4. TelemetryFrame — snapshot brut de la mémoire cible
// ─────────────────────────────────────────────────────────

struct TelemetryFrame {
    Vec3  ball_pos{};
    Vec3  ball_vel{};
    Vec3  ball_ang_vel{};
    Quat  ball_rot{};

    Vec3  car_pos{};
    Vec3  car_vel{};
    Quat  car_rot{};
    float car_boost{};
    bool  car_on_ground{};

    float game_time_remaining{};
    bool  valid{false};

    [[nodiscard]] GameTickPacket to_game_tick_packet(uint64_t tick_id) const noexcept {
        return GameTickPacket {
            .ball = BallState {
                .position         = ball_pos,
                .velocity         = ball_vel,
                .angular_velocity = ball_ang_vel,
                .radius           = 92.75f,
            },
            .player = CarState {
                .position     = car_pos,
                .velocity     = car_vel,
                .forward      = car_rot.to_forward(),
                .boost_amount = car_boost,
                .on_ground    = car_on_ground,
            },
            .game_time_remaining = game_time_remaining,
            .tick_id             = tick_id,
        };
    }
};

// ─────────────────────────────────────────────────────────
// § 5. ExternalDataManager
//
//  Politique de handle (RAII) :
//    Chaque appel à read_frame() :
//      1. Résout le PID via SnapshotHandle (RAII TH32)
//      2. Ouvre ProcessHandle (RAII OpenProcess)
//      3. Lit tous les champs via ReadProcessMemory
//      4. ProcessHandle est détruit en fin de scope → CloseHandle garanti
//
//    Aucun handle n'est maintenu entre deux cycles.
//    Même en cas de return anticipé (pid=0, base=0), le destructeur
//    RAII assure la fermeture — aucun CloseHandle() manuel nécessaire.
// ─────────────────────────────────────────────────────────

class ExternalDataManager {
public:
    explicit ExternalDataManager(
        MemoryOffsets offsets = MemoryOffsets::make_default()) noexcept
        : offsets_(std::move(offsets)) {}

    // Non-copiable
    ExternalDataManager(const ExternalDataManager&)            = delete;
    ExternalDataManager& operator=(const ExternalDataManager&) = delete;
    ExternalDataManager(ExternalDataManager&&)                 = default;
    ExternalDataManager& operator=(ExternalDataManager&&)      = default;

    /**
     * Lit un snapshot complet depuis le processus cible.
     *
     * Toute la gestion du handle est RAII :
     *   ProcessHandle ph(pid);  ← ouverture
     *   ...                     ← lectures
     * }                         ← CloseHandle automatique (fin de scope)
     *
     * @return TelemetryFrame avec valid=true si OK, false sinon.
     */
    [[nodiscard]] TelemetryFrame read_frame() noexcept {
        TelemetryFrame frame;

#ifdef _WIN32
        // ── 1. Résolution du PID (RAII snapshot TH32) ────────
        const DWORD pid = find_process_id(offsets_.process_name);
        if (pid == 0) {
            last_error_ = "Processus introuvable : " + offsets_.process_name;
            return frame;  // aucun handle ouvert → rien à fermer
        }

        // ── 2. Ouverture RAII du handle (fermeture garantie) ──
        //
        //    ProcessHandle ph possède le HANDLE.
        //    Quand ph sort de scope (return, ou fin de bloc),
        //    ~ProcessHandle() appelle CloseHandle() automatiquement.
        //    Aucun CloseHandle() manuel dans ce chemin de code.
        {
            ProcessHandle ph(pid);
            if (!ph.valid()) {
                last_error_ = "OpenProcess echoue (PROCESS_VM_READ) — code : "
                            + std::to_string(GetLastError());
                return frame;
                // ph sort de scope ici → CloseHandle() sur nullptr (no-op safe)
            }

            // ── 3. Résolution de la base du module ───────────
            const uintptr_t base = get_module_base(ph.get(), pid,
                                                   offsets_.process_name);
            if (base == 0) {
                last_error_ = "Base du module introuvable.";
                return frame;
                // ph sort de scope → CloseHandle() garanti
            }

            // ── 4. Lectures RPM ──────────────────────────────
            bool ok = true;

            // Balle
            uintptr_t ball_ptr = 0;
            ok &= rpm<uintptr_t>(ph.get(), base + offsets_.ball.ptr_offset, ball_ptr);
            if (ok && ball_ptr) {
                ok &= rpm<Vec3>(ph.get(), ball_ptr + offsets_.ball.pos_offset,     frame.ball_pos);
                ok &= rpm<Vec3>(ph.get(), ball_ptr + offsets_.ball.vel_offset,     frame.ball_vel);
                ok &= rpm<Vec3>(ph.get(), ball_ptr + offsets_.ball.ang_vel_offset, frame.ball_ang_vel);
                ok &= rpm<Quat>(ph.get(), ball_ptr + offsets_.ball.rot_offset,     frame.ball_rot);
            }

            // Voiture
            uintptr_t car_ptr = 0;
            ok &= rpm<uintptr_t>(ph.get(), base + offsets_.car.ptr_offset, car_ptr);
            if (ok && car_ptr) {
                ok &= rpm<Vec3> (ph.get(), car_ptr + offsets_.car.pos_offset,      frame.car_pos);
                ok &= rpm<Vec3> (ph.get(), car_ptr + offsets_.car.vel_offset,      frame.car_vel);
                ok &= rpm<Quat> (ph.get(), car_ptr + offsets_.car.rot_offset,      frame.car_rot);
                ok &= rpm<float>(ph.get(), car_ptr + offsets_.car.boost_offset,    frame.car_boost);
                ok &= rpm<bool> (ph.get(), car_ptr + offsets_.car.grounded_offset, frame.car_on_ground);
            }

            // Temps de jeu
            uintptr_t time_ptr = 0;
            ok &= rpm<uintptr_t>(ph.get(), base + offsets_.time_remaining_ptr_offset, time_ptr);
            if (ok && time_ptr)
                ok &= rpm<float>(ph.get(), time_ptr + offsets_.time_remaining_field_offset,
                                 frame.game_time_remaining);

            frame.valid = ok;
            if (!ok)
                last_error_ = "ReadProcessMemory : lecture partielle.";
            else
                last_error_.clear();

        } // ← ph sort de scope ici → CloseHandle() garanti

#else
        last_error_ = "ExternalDataManager non supporte sur cette plateforme.";
        (void)frame;
#endif

        return frame;
    }

    [[nodiscard]] const std::string& last_error()  const noexcept { return last_error_; }
    [[nodiscard]] const MemoryOffsets& offsets()   const noexcept { return offsets_; }
    void set_offsets(MemoryOffsets offsets) noexcept { offsets_ = std::move(offsets); }

private:
    MemoryOffsets offsets_;
    std::string   last_error_;

#ifdef _WIN32
    template<typename T>
    [[nodiscard]] static bool rpm(HANDLE h, uintptr_t addr, T& out) noexcept {
        SIZE_T bytes_read = 0;
        const BOOL ok = ReadProcessMemory(
            h,
            reinterpret_cast<LPCVOID>(addr),
            &out,
            sizeof(T),
            &bytes_read);
        return ok && (bytes_read == sizeof(T));
    }

    /**
     * Trouve le PID du processus cible via RAII SnapshotHandle.
     * La snapshot TH32 est fermée automatiquement à la fin de la fonction.
     */
    [[nodiscard]] static DWORD find_process_id(const std::string& name) noexcept {
        SnapshotHandle snap;  // ← RAII : CreateToolhelp32Snapshot
        if (!snap.valid()) return 0;

        PROCESSENTRY32W entry{};
        entry.dwSize = sizeof(entry);

        DWORD pid = 0;
        if (Process32FirstW(snap.get(), &entry)) {
            do {
                char narrow[MAX_PATH]{};
                WideCharToMultiByte(CP_ACP, 0,
                    entry.szExeFile, -1,
                    narrow, MAX_PATH,
                    nullptr, nullptr);
                if (name == narrow) {
                    pid = entry.th32ProcessID;
                    break;
                }
            } while (Process32NextW(snap.get(), &entry));
        }

        // snap.~SnapshotHandle() → CloseHandle() garanti ici
        return pid;
    }

    [[nodiscard]] static uintptr_t get_module_base(
        HANDLE h, DWORD /*pid*/, const std::string& module_name) noexcept
    {
        HMODULE modules[1024]{};
        DWORD   needed = 0;

        if (!EnumProcessModules(h, modules,
                                static_cast<DWORD>(sizeof(modules)),
                                &needed))
            return 0;

        const DWORD count = needed / sizeof(HMODULE);
        for (DWORD i = 0; i < count; ++i) {
            char name[MAX_PATH]{};
            if (GetModuleBaseNameA(h, modules[i], name, MAX_PATH)) {
                if (module_name == name)
                    return reinterpret_cast<uintptr_t>(modules[i]);
            }
        }
        return 0;
    }
#endif
};

// ─────────────────────────────────────────────────────────
// § 6. Adaptateur : ExternalDataManager → GameTickPacket
// ─────────────────────────────────────────────────────────

template<typename FallbackFn>
[[nodiscard]] GameTickPacket external_tick(
    ExternalDataManager& edm,
    uint64_t             tick_id,
    FallbackFn&&         fallback) noexcept
{
    const TelemetryFrame frame = edm.read_frame();
    if (frame.valid)
        return frame.to_game_tick_packet(tick_id);
    return fallback(tick_id);
}

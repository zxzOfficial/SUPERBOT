# RLTool v6.1 — Architecture C++20 haute performance

Bot Rocket League avec bridge HID virtuel (ViGEmBus), conçu autour de trois principes non négociables : **zéro allocation dynamique sur le chemin critique**, **communication inter-thread lock-free**, et **séparation stricte entre le thread de calcul et le thread de rendu**.

---

## Table des matières

1. [Structure du projet](#structure-du-projet)
2. [Graphe d'inclusion](#graphe-dinclusion)
3. [Architecture globale](#architecture-globale)
4. [Description de chaque fichier](#description-de-chaque-fichier)
5. [Spécifications techniques](#spécifications-techniques)
6. [Prérequis](#prérequis)
7. [Compilation](#compilation)
8. [Fonctionnement — du contexte à la décision](#fonctionnement--du-contexte-à-la-décision)
9. [Bridge ViGEm — mapping HID](#bridge-vigem--mapping-hid)
10. [Seuils de décision calibrés](#seuils-de-décision-calibrés)
11. [Extension et contribution](#extension-et-contribution)

---

## Structure du projet

```
.
├── config.txt                  # Offsets mémoire (calibrés Cheat Engine)
├── config_loader.hpp           # Parser config.txt → unordered_map O(1)
├── rl_fsm.hpp                  # FSM, DecisionContext, table de transitions
├── rl_bot.hpp                  # Physique, prédiction balle, BotController
├── external_data_manager.hpp   # Lecture mémoire RAII (ReadProcessMemory)
├── vigem_bridge.hpp            # Contrôleur Xbox360 virtuel (ViGEmBus)
├── console_ui.hpp              # Console Windows RAII (couleurs, curseur)
├── rl_imgui_panel.hpp          # Panneau Dear ImGui (debug, métriques)
└── main.cpp                    # Point d'entrée, orchestration des threads
```

---

## Graphe d'inclusion

L'ordre d'inclusion est contraint et doit être respecté dans `main.cpp`.
Chaque flèche signifie "inclut" :

```
config_loader.hpp          (aucune dépendance interne)
       │
       ▼
rl_fsm.hpp                 (aucune dépendance interne)
       │
       ▼
rl_bot.hpp         ←────── rl_fsm.hpp  (DecisionContext)
       │
       ▼
external_data_manager.hpp  ←── rl_fsm.hpp + rl_bot.hpp + config_loader.hpp
       │
       ▼
vigem_bridge.hpp           ←── rl_bot.hpp  (PlayerInput, math_utils)
       │
       ▼
console_ui.hpp             (aucune dépendance interne)
       │
       ▼
rl_imgui_panel.hpp         ←── imgui.h + rl_bot.hpp + rl_fsm.hpp
       │
       ▼
main.cpp                   ←── tous les headers ci-dessus
```

**Dans `main.cpp`, l'ordre obligatoire est :**

```cpp
#include "external_data_manager.hpp"   // tire rl_fsm, rl_bot, config_loader
#include "console_ui.hpp"
#ifndef DISABLE_VIGEM
#  include "vigem_bridge.hpp"          // tire rl_bot
#endif
#ifndef HEADLESS_MODE
#  include "rl_imgui_panel.hpp"        // tire imgui, rl_bot, rl_fsm
#endif
```

> `external_data_manager.hpp` inclut déjà `rl_fsm.hpp`, `rl_bot.hpp` et `config_loader.hpp` — il n'est pas nécessaire de les ré-inclure manuellement.

---

## Architecture globale

### Vue d'ensemble des threads

```
┌───────────────────────────────────────────┐     ┌──────────────────────────────┐
│          Thread de jeu (120 Hz)           │     │   Thread de rendu (60 Hz)    │
│                                           │     │                              │
│  ExternalDataManager::read_frame()        │     │  ImGui::NewFrame()           │
│  (ou simulate_tick en fallback)           │     │       │                      │
│       │                                   │     │       ▼                      │
│       ▼                                   │     │  ControlPanel::draw()        │
│  build_context()                          │     │   ├─ état FSM (atomic read)  │
│       │                                   │     │   ├─ vecteurs 2D terrain     │
│       ▼                                   │     │   ├─ graphes boost/steer     │
│  StateMachine::update()                   │     │   └─ log des transitions     │
│       │                                   │     │                              │
│       ▼                                   │     │  ImGui::Render()             │
│  BotController::compute()                 │     └──────────────────────────────┘
│       │                                   │              ▲
│       ▼                                   │              │
│  VigemBridge::update(input)  ←── HID 120Hz│              │
│       │                                   │   atomics relaxed + seq_cst fence
│       ▼                                   │
│  SharedDebugData::write() ────────────────┼──────────────┘
│       │                                   │
│       ▼                                   │
│  sleep_until(next_tick)                   │
└───────────────────────────────────────────┘
```

### Ordre d'exécution dans le hot path (par tick)

```
1. ExternalDataManager::read_frame()   →  TelemetryFrame (RAII handle)
2. build_context()                     →  DecisionContext
3. StateMachine::update(ctx)           →  transition FSM éventuelle
4. BotController::compute(pkt, ctx)    →  PlayerInput
5. VigemBridge::update(input)          →  émission HID (< 1 µs CPU→driver)
6. SharedDebugData::write()            →  debug ImGui (si !HEADLESS_MODE)
7. sleep_until(next_tick)              →  précision µs (steady_clock)
```

Le point 5 est volontairement placé **avant** le write de debug pour garantir la latence la plus faible possible entre le calcul et l'émission HID.

---

## Description de chaque fichier

### `config.txt`
Fichier de configuration texte chargé au démarrage par `ConfigLoader`. Format `CLE=VALEUR` (hex ou décimal). Les clés absentes conservent les valeurs par défaut de `MemoryOffsets::make_default()`.

Clés supportées : `BALL_PTR`, `BALL_POS`, `BALL_VEL`, `BALL_ANG_VEL`, `BALL_ROT`, `CAR_PTR`, `CAR_POS`, `CAR_VEL`, `CAR_ROT`, `CAR_BOOST`, `CAR_GROUNDED`, `TIME_PTR`, `TIME_FIELD`, `PROCESS_NAME`, `SCALE`, `TICK_HZ`.

---

### `config_loader.hpp`
Parser léger de fichiers de configuration. Architecture :
- Parsing unique au démarrage (cold path)
- Stockage dans `std::unordered_map<string, uintptr_t>` → lookup O(1)
- Les `float` sont stockés en bits IEEE 754 via `std::bit_cast`
- `get_value_or_default()` et `get_float_or()` sont `noexcept`, zéro allocation

---

### `rl_fsm.hpp`
Machine à états finis zero-allocation.

**Contient :**
- `enum class BotState` : `IDLE`, `ATTACK`, `DEFEND`, `RECOVER`
- `struct DecisionContext` : snapshot par tick (distances², boost, flags)
- `TRANSITION_TABLE` : `std::array<TransitionRule, 9>` constexpr avec gardes `bool(*)(const DecisionContext&) noexcept`
- `LogRing<64>` : ring buffer SPSC pour l'historique des transitions
- `StateMachine` : évalue la table à chaque tick, log les transitions, expose `state()` thread-safe via `std::atomic<BotState>`

---

### `rl_bot.hpp`
Cœur du bot — physique, prédiction, contrôleur.

**Contient :**
- `namespace math_utils` : `lerp`, `clamp`, `PI` — constexpr, zéro dépendance
- Structures : `Vec3`, `BallState`, `CarState`, `GameTickPacket`, `PlayerInput`
- `BallPredictor` / `predict_ball<N>()` : intégration d'Euler semi-implicite, résultat `PredictionPath<N>` sur la pile
- `BallLatencyBuffer` : buffer circulaire 16 × `Vec3` simulant ~83 ms de latence de perception
- `CubicBezier` + `bezier_eval()` : trajectoire de Bézier cubique, zéro allocation
- `BotController::compute(pkt, ctx)` : pipeline complet → steer (lissage + jerk), throttle (lissage + jerk), boost (accumulateur float)

---

### `external_data_manager.hpp`
Lecteur de télémétrie non-intrusif via `ReadProcessMemory`.

**Contient :**
- `ProcessHandle` : RAII wrapper `OpenProcess` / `CloseHandle`
- `SnapshotHandle` : RAII wrapper `CreateToolhelp32Snapshot` / `CloseHandle`
- `MemoryOffsets` : table d'adresses avec `make_default()` et `from_config(cfg)`
- `TelemetryFrame` : snapshot POD avec `to_game_tick_packet(tick_id)`
- `ExternalDataManager` : ouvre, lit, ferme à chaque `read_frame()` — aucun handle résiduel entre deux cycles
- `external_tick()` : adaptateur avec fallback sur `simulate_tick`

---

### `vigem_bridge.hpp`
Abstraction RAII d'un contrôleur Xbox 360 virtuel via ViGEmBus.

**Contient :**
- `namespace vigem_detail` : `apply_drift()`, `to_short_axis()`, `to_byte_trigger()` — constexpr, zéro alloc
- `VigemBridge` : `vigem_alloc` + `vigem_connect` + `vigem_target_add` à la construction ; `vigem_target_remove` + `vigem_free` au destructeur
- `VigemBridge::update(input)` : hot path 120 Hz zéro-alloc — construit `XUSB_REPORT` sur la pile et appelle `vigem_target_x360_update`
- Dérive sinusoïdale ±0.01 % à 1.7 Hz (phase accumulée, un seul `std::sin` par tick)
- `valid()` / `last_error()` pour gestion des erreurs d'init sans crash

---

### `console_ui.hpp`
Interface console Windows haute performance.

**Contient :**
- `namespace ConsoleColor` : constantes `WORD` Windows (`BRIGHT_CYAN`, `BRIGHT_YELLOW`, etc.)
- `ScopedConsoleColor` : RAII — restaure la couleur d'origine à la fin du scope
- `initialize_console_ui(title)` : titre, UTF-8, curseur masqué, High Contrast Mode, buffer 2000 lignes
- `ScopedCursorRestore` : RAII — restaure le curseur visible même sur Ctrl+C
- `console_log(color, tag, msg)` : impression colorée avec restauration automatique

---

### `rl_imgui_panel.hpp`
Panneau de contrôle Dear ImGui (mode graphique uniquement).

**Contient :**
- `SharedDebugData` : tous les champs sont `std::atomic<float>` / `std::atomic<BotState>`, écrits en `relaxed` + un seul `seq_cst` fence par tick
- `PlotRing<N>` : ring buffer POD pour les mini-graphes ImGui
- `ControlPanel::draw()` : 4 sections — Mode (toggle autonome + état FSM coloré), Vecteurs (vue terrain 2D via `ImDrawList`), Métriques (boost, tick ms, steer), Log FSM

---

### `main.cpp`
Point d'entrée et orchestrateur.

**Responsabilités :**
- `on_sigint` : handler `SIGINT`/`SIGTERM` → `g_running = false`
- `build_context(pkt)` : construit `DecisionContext` depuis `GameTickPacket`
- `simulate_tick(tick_id)` : fallback simulé (trajectoire sinusoïdale)
- `game_loop(fsm, bot, edm, debug, bridge)` : boucle 120 Hz — hot path complet
- `render_loop(panel)` : boucle 60 Hz — rendu ImGui
- `main()` : init RAII dans l'ordre LIFO correct, lancement des threads, `join()`

---

## Spécifications techniques

### Standard C++20

| Fonctionnalité | Utilisation |
|---|---|
| `concepts` | `SPSCQueue<T,Cap>` : `trivially_copyable` + puissance de 2 |
| `[[nodiscard]]` | Toutes les fonctions pures (`predict_ball`, `compute`, etc.) |
| `constexpr` lambdas | Gardes de `TRANSITION_TABLE` |
| `std::atomic` | État FSM, toggle autonome, tous les champs `SharedDebugData` |
| Designated initializers | `PlayerInput { .throttle = 1.f, .steer = cmd }` |
| `std::bit_cast` | Stockage float IEEE 754 dans `ConfigLoader` |
| `std::format` | Labels ImGui `ProgressBar` |
| `[[likely]]` / `[[unlikely]]` | `VigemBridge::update()` — branche `!ready_` |

### Prédiction de balle — Euler semi-implicite

```
a(t)     = [ -drag·vx,  -drag·vy,  g - drag·vz ]
v(t+Δt)  = v(t) + a(t)·Δt
x(t+Δt)  = x(t) + v(t+Δt)·Δt     ← semi-implicite : v déjà mis à jour
```

| Constante | Valeur | Description |
|---|---|---|
| `gravity` | −650 UU/s² | Gravité verticale |
| `drag` | 0.03 | Coefficient de traînée |
| `restitution` | 0.6 | Rebond sol |
| `radius` | 92.75 UU | Rayon balle standard |

### Filtre d'inertie (BotController)

| Signal | α (lerp) | τ approx. | Jerk max/tick |
|---|---|---|---|
| `steer` | 0.20 | ~42 ms | 0.05 |
| `throttle` | 0.25 | ~33 ms | 0.07 |
| `boost` (accumulateur) | 0.15 | ~50 ms | 0.08 |

---

## Prérequis

### Compilateur

| Compilateur | Version minimale |
|---|---|
| GCC | 11+ (C++20 complet) |
| Clang | 12.0+ |
| MSVC | VS 2019 16.8+ (`/std:c++20`) |

### ViGEmBus (obligatoire pour le bridge HID)

1. Installer le driver : [ViGEmBus Releases](https://github.com/ViGEm/ViGEmBus/releases)
2. Télécharger le SDK client : [ViGEmClient](https://github.com/ViGEm/ViGEmClient)
3. Placer les headers dans `C:/ViGEmClient/include/` et `ViGEmClient.lib` dans `C:/ViGEmClient/lib/x64/`

> Sans ViGEmBus, compiler avec `-DDISABLE_VIGEM` — le reste du système fonctionne normalement.

### Bibliothèques (mode graphique uniquement)

| Bibliothèque | Version | Rôle |
|---|---|---|
| Dear ImGui | 1.89+ | Interface graphique |
| SDL2 | 2.0.18+ | Fenêtre + événements |
| OpenGL | 3.3 core | Rendu |

---

## Compilation

### Mode headless + ViGEm (recommandé en production)

```bash
g++ -std=c++20 -O2 -DHEADLESS_MODE \
    -I"C:/ViGEmClient/include" \
    main.cpp -o rl_tool.exe \
    -L"C:/ViGEmClient/lib/x64" -lViGEmClient
```

### Mode headless sans ViGEm (tests / CI)

```bash
g++ -std=c++20 -O2 -DHEADLESS_MODE -DDISABLE_VIGEM \
    main.cpp -o rl_tool.exe
```

### Mode graphique (ImGui + SDL2 + ViGEm)

```bash
g++ -std=c++20 -O2 \
    -I"C:/ViGEmClient/include" \
    main.cpp \
    imgui/imgui.cpp imgui/imgui_draw.cpp \
    imgui/imgui_tables.cpp imgui/imgui_widgets.cpp \
    imgui/backends/imgui_impl_sdl2.cpp \
    imgui/backends/imgui_impl_opengl3.cpp \
    $(sdl2-config --cflags --libs) -lGL \
    -L"C:/ViGEmClient/lib/x64" -lViGEmClient \
    -o rl_tool.exe
```

### Mode simulation forcée (lecture mémoire désactivée)

```bash
g++ -std=c++20 -O2 -DHEADLESS_MODE -DFORCE_SIMULATE \
    main.cpp -o rl_tool.exe
```

### Flags utiles

| Flag | Rôle |
|---|---|
| `-O2` | Optimisations standard (production) |
| `-O3 -march=native` | SIMD + optimisations locales |
| `-DNDEBUG` | Désactive les `assert` |
| `-DDISABLE_VIGEM` | Désactive le bridge HID |
| `-DFORCE_SIMULATE` | Désactive la lecture mémoire |
| `-DHEADLESS_MODE` | Désactive Dear ImGui |
| `-fsanitize=thread` | Détection data races (debug) |
| `-fsanitize=address` | Détection corruptions mémoire (debug) |

---

## Fonctionnement — du contexte à la décision

### Construction du DecisionContext

À chaque tick, `build_context()` calcule un snapshot depuis `GameTickPacket` :

```cpp
struct DecisionContext {
    float    ball_dist_sq;           // distance² voiture→balle (UU²)
    float    ball_dist_to_goal_sq;   // distance² balle→cage propre (UU²)
    float    boost_pct;              // réserve boost [0, 100]
    bool     ball_is_behind_car;     // forward · to_ball < 0
    bool     teammate_attacking;
    uint64_t tick_id;
};
```

`ball_is_behind_car` est déterminé par le produit scalaire `forward · (ball - car)`. Si négatif, la balle est dans le demi-espace défensif → évaluation des règles de défense prioritaire.

### Table de transitions — priorités

```
État       Condition                               → Nouvel état
───────────────────────────────────────────────────────────────
IDLE       balle devant + boost > 20%              → ATTACK
IDLE       balle derrière + danger cage            → DEFEND
IDLE       boost < 20%                             → RECOVER

ATTACK     balle derrière + danger cage            → DEFEND  ← priorité défense
ATTACK     boost < 20%                             → RECOVER
ATTACK     balle loin + pas de danger              → IDLE

DEFEND     danger écarté                           → IDLE
DEFEND     boost < 20%                             → RECOVER

RECOVER    boost ≥ 50%                             → IDLE
```

---

## Bridge ViGEm — mapping HID

### Mapping `PlayerInput` → `XUSB_REPORT`

| Champ `PlayerInput` | Registre XInput | Détail |
|---|---|---|
| `throttle ≥ 0` | `bRightTrigger` [0–255] | Accélération |
| `throttle < 0` | `bLeftTrigger` [0–255] | Frein / marche arrière |
| `steer` | `sThumbLX` [−32768, +32767] | Stick gauche X + dérive |
| `roll` | `sThumbLY` [−32768, +32767] | Stick gauche Y |
| `yaw` | `sThumbRX` [−32768, +32767] | Stick droit X |
| `pitch` | `sThumbRY` [−32768, +32767] | Stick droit Y + dérive |
| `boost` | `XUSB_GAMEPAD_A` | Bouton A |
| `jump` | `XUSB_GAMEPAD_B` | Bouton B |
| `handbrake` | `XUSB_GAMEPAD_X` | Bouton X |

### Simulation de dérive analogique

Modélise la dérive résiduelle d'un potentiomètre physique (ADC 12 bits) :

- **Amplitude** : ±0.01 % de l'étendue complète (±3.27 LSB sur 16 bits)
- **Fréquence** : 1.7 Hz (période ≈ 0.59 s) — typique d'un joystick entrée de gamme sous charge thermique
- **Implémentation** : `drift_phase_ += 2π × 1.7 / 120` par tick, un seul `std::sin` évalué, appliqué sur `steer` et `pitch`
- **Hot path** : zéro allocation, zéro branche non-triviale

---

## Seuils de décision calibrés

Tous dans `namespace Thresholds` (`rl_fsm.hpp`) :

| Constante | Valeur | Signification |
|---|---|---|
| `CLOSE_SQ` | 1500² UU² | Balle "proche" de la voiture |
| `VERY_CLOSE_SQ` | 800² UU² | Balle en portée de frappe directe |
| `GOAL_DANGER_SQ` | 3000² UU² | Balle dangereuse pour la cage |
| `BOOST_LOW` | 20 % | Seuil bas → RECOVER |
| `BOOST_OK` | 50 % | Seuil haut → sortie RECOVER |

---

## Extension et contribution

**Ajouter un état FSM** (`KICKOFF`, `SHADOW`…) : ajouter la valeur dans `enum class BotState`, puis les règles dans `TRANSITION_TABLE`. Aucune vtable, aucune hiérarchie.

**Brancher le vrai framework RLBot** : remplacer `simulate_tick()` par `rlbot_interface.get_packet()` et `(void)input` par `rlbot_interface.set_input(input)`.

**Améliorer la prédiction** : étendre `predict_ball()` avec les rebonds murs (±4096 UU), plafond (2044 UU) et surfaces courbes.

**Contrôleur PID** : remplacer `angle_to_steer()` (proportionnel pur) par un régulateur PID complet pour réduire le dépassement en virage serré.

**Ajuster les offsets mémoire** : modifier uniquement `config.txt` après recalibration Cheat Engine — aucune recompilation nécessaire.

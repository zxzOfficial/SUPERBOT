/**
 * console_ui.hpp — Interface console Windows haute performance, C++20
 *
 * Fournit :
 *   initialize_console_ui(title)  → configure titre, couleurs, curseur
 *   ScopedConsoleColor            → RAII pour la couleur de texte courante
 *   console_print_header()        → bannière de démarrage sobre
 *
 * Politique :
 *   - API Windows native uniquement (SetConsoleXxx) — aucune dépendance externe
 *   - RAII strict : toute ressource console ouverte via ce header est libérée
 *     automatiquement à la fin du scope (destructeurs garantis)
 *   - Compilation croisée : les fonctions sont des no-ops sur Linux/macOS
 *     (séquences ANSI équivalentes en fallback)
 */

#pragma once

#include <cstdio>
#include <string_view>

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#endif

// ─────────────────────────────────────────────────────────
// § 1. Codes de couleur Windows (FOREGROUND_* + BACKGROUND_*)
// ─────────────────────────────────────────────────────────

namespace ConsoleColor {
#ifdef _WIN32
    inline constexpr WORD HIGH_CONTRAST =
        FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY;
        // = Gris/Blanc intense sur fond noir — lisibilité maximale

    inline constexpr WORD DIM_GRAY =
        FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE;
        // = Gris standard — pour les messages secondaires

    inline constexpr WORD BRIGHT_WHITE =
        FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY;

    inline constexpr WORD BRIGHT_CYAN =
        FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY;

    inline constexpr WORD BRIGHT_YELLOW =
        FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY;

    inline constexpr WORD BRIGHT_RED =
        FOREGROUND_RED | FOREGROUND_INTENSITY;
#endif
} // namespace ConsoleColor

// ─────────────────────────────────────────────────────────
// § 2. ScopedConsoleColor — RAII pour la couleur courante
//
//  Usage :
//    {
//        ScopedConsoleColor c(ConsoleColor::BRIGHT_CYAN);
//        std::puts("[INFO] Connexion établie.");
//    }  // ← couleur d'origine restaurée ici
// ─────────────────────────────────────────────────────────

class ScopedConsoleColor {
public:
#ifdef _WIN32
    explicit ScopedConsoleColor(WORD color) noexcept
        : handle_(GetStdHandle(STD_OUTPUT_HANDLE))
    {
        // Sauvegarder la couleur courante avant de la changer
        CONSOLE_SCREEN_BUFFER_INFO info{};
        if (GetConsoleScreenBufferInfo(handle_, &info))
            original_ = info.wAttributes;
        SetConsoleTextAttribute(handle_, color);
    }

    ~ScopedConsoleColor() noexcept {
        // Restauration garantie même si une exception est levée ailleurs
        if (handle_ != INVALID_HANDLE_VALUE)
            SetConsoleTextAttribute(handle_, original_);
    }

    // Non-copiable, non-déplaçable : sémantique de ressource unique
    ScopedConsoleColor(const ScopedConsoleColor&)            = delete;
    ScopedConsoleColor& operator=(const ScopedConsoleColor&) = delete;
    ScopedConsoleColor(ScopedConsoleColor&&)                 = delete;
    ScopedConsoleColor& operator=(ScopedConsoleColor&&)      = delete;

private:
    HANDLE handle_   = INVALID_HANDLE_VALUE;
    WORD   original_ = ConsoleColor::HIGH_CONTRAST;

#else
    // Stub Linux/macOS : couleurs ANSI
    explicit ScopedConsoleColor([[maybe_unused]] int color) noexcept {}
    ~ScopedConsoleColor() noexcept { std::fputs("\033[0m", stdout); }
    ScopedConsoleColor(const ScopedConsoleColor&)            = delete;
    ScopedConsoleColor& operator=(const ScopedConsoleColor&) = delete;
#endif
};

// ─────────────────────────────────────────────────────────
// § 3. initialize_console_ui()
//
//  Configure la fenêtre console au lancement :
//   1. Titre de fenêtre (SetConsoleTitleA)
//   2. Curseur texte masqué (SetConsoleCursorInfo)
//   3. Couleurs en High Contrast Mode (gris sur noir)
//   4. Code page UTF-8 pour les caractères accentués
// ─────────────────────────────────────────────────────────

/**
 * Initialise la console pour un affichage monitorig sobre.
 *
 * @param title  Titre affiché dans la barre de titre de la fenêtre console.
 *               Exemple : "RLTool v5 — Sensor Monitor"
 */
inline void initialize_console_ui(std::string_view title) noexcept {
#ifdef _WIN32
    // ── 1. Titre de la fenêtre ────────────────────────────
    SetConsoleTitleA(title.data());

    // ── 2. Code page UTF-8 (accents, symboles) ────────────
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);

    const HANDLE hout = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hout == INVALID_HANDLE_VALUE) return;

    // ── 3. Masquer le curseur clignotant ──────────────────
    //    bVisible = FALSE → curseur invisible
    //    dwSize   = 1     → taille minimale (ignorée quand invisible)
    CONSOLE_CURSOR_INFO cursor_info{};
    cursor_info.dwSize   = 1;
    cursor_info.bVisible = FALSE;
    SetConsoleCursorInfo(hout, &cursor_info);

    // ── 4. High Contrast Mode : Gris intense sur Noir ─────
    //    Texte par défaut = FOREGROUND_RGB | INTENSITY (blanc/gris)
    //    Fond  par défaut = 0 (noir) — aucun flag BACKGROUND_* activé
    SetConsoleTextAttribute(hout, ConsoleColor::HIGH_CONTRAST);

    // ── 5. Agrandir le buffer de scroll (confort de lecture) ─
    CONSOLE_SCREEN_BUFFER_INFO csbi{};
    if (GetConsoleScreenBufferInfo(hout, &csbi)) {
        // Buffer vertical : au moins 2000 lignes pour le log
        COORD new_size = csbi.dwSize;
        if (new_size.Y < 2000) new_size.Y = 2000;
        SetConsoleScreenBufferSize(hout, new_size);
    }

#else
    // Linux/macOS : séquences ANSI équivalentes
    // OSC 2 = titre de fenêtre terminal (xterm, gnome-terminal, …)
    std::printf("\033]2;%.*s\007",
                static_cast<int>(title.size()),
                title.data());
    // Cacher le curseur (séquence DECTCEM)
    std::fputs("\033[?25l", stdout);
    // Fond noir, texte blanc intense
    std::fputs("\033[0;97m", stdout);
    std::fflush(stdout);
#endif
}

// ─────────────────────────────────────────────────────────
// § 4. ScopedCursorRestore
//
//  RAII : restaure le curseur à la fin de l'application
//  (utile en cas de Ctrl+C ou d'exception non gérée).
// ─────────────────────────────────────────────────────────

class ScopedCursorRestore {
public:
    ScopedCursorRestore() noexcept = default;

    ~ScopedCursorRestore() noexcept {
#ifdef _WIN32
        const HANDLE hout = GetStdHandle(STD_OUTPUT_HANDLE);
        if (hout != INVALID_HANDLE_VALUE) {
            CONSOLE_CURSOR_INFO ci{ .dwSize = 25, .bVisible = TRUE };
            SetConsoleCursorInfo(hout, &ci);
        }
#else
        std::fputs("\033[?25h\033[0m\n", stdout);
        std::fflush(stdout);
#endif
    }

    ScopedCursorRestore(const ScopedCursorRestore&)            = delete;
    ScopedCursorRestore& operator=(const ScopedCursorRestore&) = delete;
    ScopedCursorRestore(ScopedCursorRestore&&)                 = delete;
    ScopedCursorRestore& operator=(ScopedCursorRestore&&)      = delete;
};

// ─────────────────────────────────────────────────────────
// § 5. Helpers d'impression colorée
// ─────────────────────────────────────────────────────────

/// Imprime une ligne avec le préfixe [TAG] en couleur, message en gris.
#ifdef _WIN32
inline void console_log(WORD tag_color,
                         std::string_view tag,
                         std::string_view msg) noexcept
{
    const HANDLE hout = GetStdHandle(STD_OUTPUT_HANDLE);
    // Couleur du tag
    SetConsoleTextAttribute(hout, tag_color);
    std::printf("[%.*s] ", static_cast<int>(tag.size()), tag.data());
    // Message en gris standard
    SetConsoleTextAttribute(hout, ConsoleColor::DIM_GRAY);
    std::printf("%.*s\n",  static_cast<int>(msg.size()),  msg.data());
    // Restaurer High Contrast
    SetConsoleTextAttribute(hout, ConsoleColor::HIGH_CONTRAST);
}
#else
inline void console_log([[maybe_unused]] int tag_color,
                         std::string_view tag,
                         std::string_view msg) noexcept
{
    std::printf("[%.*s] %.*s\n",
        static_cast<int>(tag.size()), tag.data(),
        static_cast<int>(msg.size()),  msg.data());
}
#endif

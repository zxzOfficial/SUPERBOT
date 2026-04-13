/**
 * config_loader.hpp — Lecteur de configuration léger, C++20
 *
 * Format du fichier config.txt :
 *   # commentaire (ligne ignorée)
 *   CLE=VALEUR          (VALEUR peut être décimale ou hex 0x…)
 *
 *   OFFSET_BALL=0x02A4B0E0
 *   SCALE=1.5            ← stocké en bits IEEE 754 via std::bit_cast
 *   TICK_HZ=120
 *
 * Architecture :
 *   - Parsing unique au démarrage (cold path)
 *   - Stockage dans std::unordered_map → lookup O(1) amorti
 *   - Toutes les valeurs stockées en uintptr_t
 *     (les floats sont bit_cast'd avant stockage, et bit_cast'd à la lecture)
 *   - get_value_or_default() : noexcept, zéro allocation sur le hot path
 *   - Lignes > MAX_LINE_LEN ignorées silencieusement (protection buffer)
 */

#pragma once

#include <bit>
#include <charconv>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <unordered_map>

class ConfigLoader {
public:
    /// Taille maximale d'une ligne (protection contre les lignes malformées).
    static constexpr std::size_t MAX_LINE_LEN = 512;

    ConfigLoader() = default;

    /**
     * Charge et parse le fichier de configuration.
     * Appelé une seule fois au démarrage — non critique pour la perf.
     *
     * @param path  Chemin du fichier (ex : "config.txt").
     * @return true si le fichier a été ouvert et parsé (même partiellement).
     */
    bool load(const char* path) noexcept {
        std::FILE* f = std::fopen(path, "r");
        if (!f) return false;

        char line[MAX_LINE_LEN];
        while (std::fgets(line, static_cast<int>(MAX_LINE_LEN), f)) {
            parse_line(std::string_view(line, std::strlen(line)));
        }

        std::fclose(f);
        return true;
    }

    /**
     * Récupère la valeur associée à key, ou default_val si absente.
     * O(1) amorti — aucune allocation, noexcept.
     *
     * @param key          Clé à rechercher (ex : "OFFSET_BALL").
     * @param default_val  Valeur retournée si la clé est absente.
     */
    [[nodiscard]] uintptr_t get_value_or_default(
        std::string_view key,
        uintptr_t        default_val) const noexcept
    {
        // Construire une std::string uniquement si nécessaire (find via hétérogène)
        const auto it = map_.find(std::string(key));
        if (it == map_.end()) return default_val;
        return it->second;
    }

    /**
     * Surcharge pour les valeurs float stockées via bit_cast.
     * Usage : float scale = cfg.get_float_or("SCALE", 1.f);
     */
    [[nodiscard]] float get_float_or(
        std::string_view key,
        float            default_val) const noexcept
    {
        const auto it = map_.find(std::string(key));
        if (it == map_.end()) return default_val;

        // Les floats ont été stockés en bits IEEE 754 via bit_cast
        static_assert(sizeof(float) <= sizeof(uintptr_t));
        uint32_t bits = static_cast<uint32_t>(it->second);
        return std::bit_cast<float>(bits);
    }

    /**
     * Nombre d'entrées chargées (utile pour le diagnostic au démarrage).
     */
    [[nodiscard]] std::size_t size() const noexcept { return map_.size(); }

    /**
     * Retourne true si la clé existe.
     */
    [[nodiscard]] bool contains(std::string_view key) const noexcept {
        return map_.count(std::string(key)) > 0;
    }

    /**
     * Stocke (ou écrase) une paire clé/valeur depuis le code.
     * Utile pour injecter des valeurs par défaut avant load().
     */
    void set(std::string key, uintptr_t value) {
        map_.insert_or_assign(std::move(key), value);
    }

private:
    std::unordered_map<std::string, uintptr_t> map_;

    /**
     * Parse une ligne du fichier.
     * Format attendu : CLE=VALEUR\n
     * Commentaires (#) et lignes vides ignorés.
     */
    void parse_line(std::string_view line) noexcept {
        // Supprimer le '\n' / '\r' en fin de ligne
        while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
            line.remove_suffix(1);

        // Ignorer commentaires et lignes vides
        if (line.empty() || line.front() == '#' || line.front() == ';')
            return;

        const auto eq = line.find('=');
        if (eq == std::string_view::npos) return;

        std::string_view raw_key = line.substr(0, eq);
        std::string_view raw_val = line.substr(eq + 1);

        // Trim espaces
        raw_key = trim(raw_key);
        raw_val = trim(raw_val);

        if (raw_key.empty() || raw_val.empty()) return;

        // Parse la valeur — hex (0x…) ou décimale, ou float
        uintptr_t parsed_val = 0;
        if (!parse_value(raw_val, parsed_val)) return;

        map_.insert_or_assign(std::string(raw_key), parsed_val);
    }

    /**
     * Tente de parser raw_val dans out.
     * Ordre d'essai : hexadécimal (0x), décimal entier, float (bit_cast).
     * Retourne false si aucune représentation n'est reconnue.
     */
    [[nodiscard]] static bool parse_value(
        std::string_view raw, uintptr_t& out) noexcept
    {
        if (raw.empty()) return false;

        const char* beg = raw.data();
        const char* end = beg + raw.size();

        // ── Hexadécimal : 0x… ou 0X… ──────────────────────────
        if (raw.size() >= 2 && raw[0] == '0' && (raw[1] == 'x' || raw[1] == 'X')) {
            uintptr_t val = 0;
            auto [ptr, ec] = std::from_chars(beg + 2, end, val, 16);
            if (ec == std::errc{} && ptr == end) { out = val; return true; }
            return false;
        }

        // ── Entier décimal (signé ou non) ─────────────────────
        {
            uintptr_t val = 0;
            auto [ptr, ec] = std::from_chars(beg, end, val);
            if (ec == std::errc{} && ptr == end) { out = val; return true; }
        }

        // ── Float (ex : SCALE=1.5) → stocké en bits IEEE 754 ──
        //    from_chars pour float n'est pas disponible partout ;
        //    on utilise sscanf comme fallback portable.
        {
            float f = 0.f;
            char  buf[64]{};
            const std::size_t n = std::min(raw.size(), sizeof(buf) - 1);
            std::memcpy(buf, raw.data(), n);
            buf[n] = '\0';

            if (std::sscanf(buf, "%f", &f) == 1) {
                static_assert(sizeof(uint32_t) <= sizeof(uintptr_t));
                out = static_cast<uintptr_t>(std::bit_cast<uint32_t>(f));
                return true;
            }
        }

        return false;
    }

    /// Supprime les espaces en début et fin de vue.
    [[nodiscard]] static std::string_view trim(std::string_view sv) noexcept {
        while (!sv.empty() && (sv.front() == ' ' || sv.front() == '\t'))
            sv.remove_prefix(1);
        while (!sv.empty() && (sv.back() == ' ' || sv.back() == '\t'))
            sv.remove_suffix(1);
        return sv;
    }
};

// ============================================================================
//  main.cpp - Helios AI: stale nasłuchiwanie słowa-klucza + rozmowa z LLM
//
//  Nowy przebieg (zamiast ręcznego "/listen"):
//      1. robot STALE nasłuchuje słowa-klucza "Helios" (wątek mikrofonu),
//      2. po usłyszeniu "Helios" zbiera wypowiedź do 1,2 s ciszy,
//      3. rozpoznaje mowę (whisper-server / whisper.cpp lokalnie),
//      4. wysyła tekst do LLM (OpenRouter), wykonuje komendy #...# i mówi (Piper),
//      5. przez kilka sekund można mówić dalej BEZ powtarzania "Helios"
//         (HELIOS_FOLLOW_UP), a potem znowu trzeba zacząć od słowa-klucza.
//
//  Tryby diagnostyczne:
//      ./main --wake-test plik.wav     # wynik detektora dla każdej ramki
//      ./main --stt plik.wav           # tylko rozpoznawanie mowy z pliku
//      HELIOS_NO_WAKEWORD=1 ./main     # bez słowa-klucza (ręczne /listen)
//
//  Konfiguracja: zmienne środowiskowe (patrz README.md, sekcja "Konfiguracja").
// ============================================================================

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <csignal>
#include <cstdio>
#include <cstring>
#include <execinfo.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include "helios_tts.h"
#include "llm_commands.h"
#include "mic.h"
#include "speech_to_text.h"
#include "wakeword.h"

using json = nlohmann::json;

// ---------------------------------------------------------------------------
// Konfiguracja (wszystko przez zmienne środowiskowe)
// ---------------------------------------------------------------------------
namespace {

std::string env(const char* name, const std::string& fallback = {}) {
    const char* value = std::getenv(name);
    return value && *value ? std::string(value) : fallback;
}

double envDouble(const char* name, double fallback) {
    const std::string value = env(name);
    if (value.empty()) {
        return fallback;
    }
    try {
        return std::stod(value);
    } catch (...) {
        return fallback;
    }
}

int envInt(const char* name, int fallback) {
    const std::string value = env(name);
    if (value.empty()) {
        return fallback;
    }
    try {
        return std::stoi(value);
    } catch (...) {
        return fallback;
    }
}

// ---------------------------------------------------------------------------
// Diagnostyka awarii
//
// ROS 2 buduje z _GLIBCXX_ASSERTIONS, więc wyjście poza zakres kontenera
// przerywa program komunikatem bez informacji, GDZIE. Ten handler wypisuje
// zrzut stosu, dzięki czemu od razu widać winowajcę.
// ---------------------------------------------------------------------------
namespace {

void crashHandler(int sig) {
    const char* name = "nieznany sygnał";
    switch (sig) {
        case SIGSEGV: name = "SIGSEGV - zły dostęp do pamięci"; break;
        case SIGABRT: name = "SIGABRT - przerwanie (np. asercja biblioteki)"; break;
        case SIGBUS:  name = "SIGBUS"; break;
        case SIGFPE:  name = "SIGFPE"; break;
        case SIGILL:  name = "SIGILL"; break;
        default: break;
    }
    void* frames[64];
    const int count = backtrace(frames, 64);
    char header[256];
    const int written = std::snprintf(header, sizeof(header),
        "\n=== AWARIA: %s ===\n--- zrzut stosu (wyślij ten fragment) ---\n", name);
    if (written > 0) {
        ssize_t ignored = write(STDERR_FILENO, header, static_cast<size_t>(written));
        (void)ignored;
    }
    backtrace_symbols_fd(frames, count, STDERR_FILENO);
    const char* footer = "--- koniec zrzutu ---\n";
    ssize_t ignored = write(STDERR_FILENO, footer, std::strlen(footer));
    (void)ignored;
    _exit(128 + sig);
}

void installCrashHandler() {
    std::signal(SIGSEGV, crashHandler);
    std::signal(SIGABRT, crashHandler);
    std::signal(SIGBUS, crashHandler);
    std::signal(SIGFPE, crashHandler);
    std::signal(SIGILL, crashHandler);
}

// Konstruktor tego obiektu wykonuje się PRZED main(). Jeśli w logu NIE ma linii
// "[start] ...", to program pada przy statycznym inicjalizowaniu (np. w którymś
// z plików commands/*.cpp) - jeszcze zanim zacznie działać main.
struct StartupProbe {
    StartupProbe() {
        installCrashHandler();
        static const char message[] = "[start] program wystartowal (handler awarii aktywny)\n";
        const ssize_t ignored = write(STDOUT_FILENO, message, sizeof(message) - 1);
        (void)ignored;
    }
};

StartupProbe gStartupProbe;

}  // namespace

// ---------------------------------------------------------------------------
// Weryfikacja słowa-klucza na tekście z STT
//
// Detektor akustyczny (mały model z TTS) nie jest doskonały: przy niskim progu
// ma wysoką czułość, ale też sporo fałszywych pobudek. Dlatego po wykryciu
// "Helios" nagrywamy ~1,6 s wokół pobudki i sprawdzamy transkrypcję - jeśli
// w tekście nie ma nic podobnego do "Helios", wracamy do nasłuchu bez reakcji.
// Koszt: jedno krótkie zapytanie do STT na pobudkę (nie na każdą wypowiedź).
// ---------------------------------------------------------------------------
std::vector<std::string> splitWords(const std::string& text) {
    std::vector<std::string> words;
    std::string current;
    for (unsigned char character : text) {
        if (std::isalnum(character) || character >= 0x80) {
            current += static_cast<char>(character);
        } else if (!current.empty()) {
            words.push_back(current);
            current.clear();
        }
    }
    if (!current.empty()) {
        words.push_back(current);
    }
    return words;
}

std::string lowerPl(const std::string& text) {
    std::string result = text;
    for (char& character : result) {
        const unsigned char c = static_cast<unsigned char>(character);
        if (c >= 'A' && c <= 'Z') {
            character = static_cast<char>(c - 'A' + 'a');
        }
    }
    return result;
}

int editDistance(const std::string& a, const std::string& b) {
    std::vector<int> previous(b.size() + 1), current(b.size() + 1);
    for (size_t j = 0; j <= b.size(); ++j) {
        previous[j] = static_cast<int>(j);
    }
    for (size_t i = 1; i <= a.size(); ++i) {
        current[0] = static_cast<int>(i);
        for (size_t j = 1; j <= b.size(); ++j) {
            const int cost = a[i - 1] == b[j - 1] ? 0 : 1;
            current[j] = std::min({previous[j] + 1, current[j - 1] + 1, previous[j - 1] + cost});
        }
        previous = current;
    }
    return previous[b.size()];
}

// Czy pojedyncze słowo wygląda jak "Helios"?
//
//  - akceptujemy: helios, heliosz, heliusz, helius, heliosem, helio, heljos (przekręcenia Whispera),
//  - odrzucamy:   helikopter, helion (pierwiastek), helikon itp. - słowa, które brzmią podobnie,
//                 ale NIE są słowem-kluczem (to tzw. twarde negatywy).
bool isWakeWordToken(const std::string& word) {
    static const std::vector<std::string> deny = {
        "helion", "helionu", "helionie", "heliony", "helionem",
        "helikopter", "helikoptera", "helikopterem", "helikoptery",
        "helikon", "helikonia", "helikonu", "heliks",
    };
    static const std::vector<std::string> patterns = {"helios", "heliusz", "heljos"};
    if (word.size() < 4) {
        return false;
    }
    for (const std::string& blocked : deny) {
        if (word == blocked) {
            return false;
        }
    }
    const std::string prefix = word.substr(0, 5);
    if (prefix == "helio" || prefix == "heliu" || prefix == "helju") {
        return true;                       // helios, heliosz, heliusz, helio, heljos...
    }
    for (const std::string& pattern : patterns) {
        if (editDistance(word, pattern) <= 1) {
            return true;                   // np. "helioss", "helios" z literówką
        }
    }
    // Whisper dość często gubi albo przekręca pierwszą literę słowa-klucza.
    // Zmierzone na prawdziwych nagraniach (Groq whisper-large-v3, głosy piper):
    //   "Helios" -> "Telios", "Petelios", "Elios"; "Hej Helios" osobno niżej.
    // Wystarczy więc, że w słowie siedzi rdzeń "elios"/"elius"/"eljos" - ale NIE "elion"
    // ani "eliko", dzięki czemu "helion" i "helikopter" nadal są odrzucane.
    if (word.size() >= 5) {
        for (const char* root : {"elios", "elius", "eljos"}) {
            if (word.find(root) != std::string::npos) {
                return true;
            }
        }
    }
    return false;
}

// Czy tekst (transkrypcja fragmentu z pobudki) wygląda jak słowo-klucz?
bool looksLikeWakeWord(const std::string& transcript) {
    for (const std::string& raw : splitWords(transcript)) {
        if (isWakeWordToken(lowerPl(raw))) {
            return true;
        }
    }
    return false;
}

// Czy słowo to "wypełniacz" wstawiany przed słowem-kluczem ("Hej Helios, ...")?
// Usuwamy go tylko wtedy, gdy zaraz po nim naprawdę stoi słowo-klucz.
bool isFillerToken(const std::string& word) {
    static const std::vector<std::string> fillers = {"hej", "ej", "ok", "okej", "halo", "no"};
    for (const std::string& filler : fillers) {
        if (word == filler) {
            return true;
        }
    }
    return false;
}

// Usuwa słowo-klucz (i ewentualny wypełniacz przed nim) z początku wypowiedzi:
//   "Helios, włącz drag"        -> "włącz drag"
//   "Hej Helios, zrób zdjęcie"  -> "zrób zdjęcie"
std::string stripWakeWord(const std::string& text) {
    const std::string separators = " \t\n\r.,!?;:-\"\u201e\u201d'";
    auto nextToken = [&](const std::string& source, size_t from, size_t* begin, size_t* outEnd) -> bool {
        const size_t tokenStart = source.find_first_not_of(separators, from);
        if (tokenStart == std::string::npos) {
            return false;
        }
        size_t tokenEnd = tokenStart;
        while (tokenEnd < source.size() &&
               (std::isalnum(static_cast<unsigned char>(source[tokenEnd])) ||
                static_cast<unsigned char>(source[tokenEnd]) >= 0x80)) {
            ++tokenEnd;
        }
        *begin = tokenStart;
        *outEnd = tokenEnd;
        return true;
    };

    std::string result = text;
    bool stripped = false;
    while (true) {
        size_t begin = 0, tokenEnd = 0;
        if (!nextToken(result, 0, &begin, &tokenEnd)) {
            return std::string();
        }
        const std::string first = lowerPl(result.substr(begin, tokenEnd - begin));
        if (isWakeWordToken(first)) {
            stripped = true;
            result = result.substr(tokenEnd);
            continue;
        }
        size_t begin2 = 0, end2 = 0;   // "Hej Helios" - wypełniacz + słowo-klucz
        if (isFillerToken(first) && nextToken(result, tokenEnd, &begin2, &end2) &&
            isWakeWordToken(lowerPl(result.substr(begin2, end2 - begin2)))) {
            stripped = true;
            result = result.substr(end2);
            continue;
        }
        break;
    }
    if (!stripped) {
        return result;
    }
    const size_t startIndex = result.find_first_not_of(separators);
    return startIndex == std::string::npos ? std::string() : result.substr(startIndex);
}


std::string trimCopy(const std::string& text) {
    const size_t begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const size_t end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

std::vector<std::string> splitList(const std::string& text) {
    std::vector<std::string> items;
    size_t start = 0;
    while (start <= text.size()) {
        const size_t comma = text.find(',', start);
        const std::string item = trimCopy(text.substr(start, comma == std::string::npos
                                                               ? std::string::npos
                                                               : comma - start));
        if (!item.empty()) {
            items.push_back(item);
        }
        if (comma == std::string::npos) {
            break;
        }
        start = comma + 1;
    }
    return items;
}

struct Options {
    std::string audioDevice = env("HELIOS_AUDIO_DEVICE", "plughw:CARD=REF,DEV=0");
    std::string wakeModel = env("HELIOS_WAKE_MODEL", "/home/std/helios/models/helios.onnx");
    std::string melModel = env("HELIOS_MEL_MODEL", "/home/std/helios/models/melspectrogram.onnx");
    std::string embeddingModel = env("HELIOS_EMBEDDING_MODEL",
                                     "/home/std/helios/models/embedding_model.onnx");
    float wakeThreshold = static_cast<float>(envDouble("HELIOS_WAKE_THRESHOLD", 0.40));
    int wakePatience = envInt("HELIOS_WAKE_PATIENCE", 2);
    double wakeDebounce = envDouble("HELIOS_WAKE_DEBOUNCE", 1.2);
    double followUpSeconds = envDouble("HELIOS_FOLLOW_UP", 6.0);
    double preRollSeconds = envDouble("HELIOS_PREROLL", 0.6);
    bool noWakeword = !env("HELIOS_NO_WAKEWORD").empty();
    // "off" = bez weryfikacji, "strict" = odrzuć gdy STT zawiedzie, inaczej: weryfikuj,
    // a przy błędzie STT przyjmij pobudkę (żeby robot nie był "głuchy" offline).
    std::string verifyMode = lowerPl(env("HELIOS_WAKE_VERIFY"));
    bool verifyWake = (verifyMode != "off") && !noWakeword;
    bool verifyStrict = (verifyMode == "strict");
    // Gdy detektor jest BARDZO pewny (użytkownik zmierzył 0.999), weryfikacja przez
    // STT jest zbędna - a to ona dodaje ~1-3 s ciszy na starcie rozmowy. Powyżej
    // tego progu reagujemy natychmiast. Ustaw 1.1, aby wyłączyć tę ścieżkę.
    float wakeFastScore = static_cast<float>(envDouble("HELIOS_WAKE_FAST_SCORE", 0.90));
    // Automatyczne wzmocnienie mikrofonu (patrz klasa MicGain). HELIOS_AGC=0 wyłącza.
    bool agc = env("HELIOS_AGC") != "0";
    // Jak długo po pobudce czekamy na mowę, zanim wrócimy do nasłuchu.
    // W trakcie tego okna robot DALEJ słucha słowa-klucza, więc można spokojnie
    // dać dużo czasu - powtórzone "Helios" i tak zaczyna nagrywanie od nowa.
    double captureArmSeconds = envDouble("HELIOS_CAPTURE_ARM", 4.0);
    // Maksymalne wzmocnienie mikrofonu (AGC). Im większe, tym głośniej słyszy
    // cichą mowę - ale przy bardzo dużych wartościach podbija też szum tła.
    double agcMax = envDouble("HELIOS_AGC_MAX", 10.0);
    // ALSA: domyślnie PUSTE, czyli arecord sam dobiera duży bufor - to ustawienie
    // jest sprawdzone i nie gubi próbek (mały bufor 2048 ramek powodował "overrun"
    // i rzadkie łapanie słowa-klucza). Można eksperymentować, np. 1024/8192.
    std::string alsaPeriod = env("HELIOS_ALSA_PERIOD");
    std::string alsaBuffer = env("HELIOS_ALSA_BUFSIZE");
    // Krótki dzwonek w momencie przyjęcia słowa-klucza - słychać, że robot usłyszał.
    // Wyłączenie: HELIOS_CHIME=0
    bool chime = env("HELIOS_CHIME") != "0";
    std::string openRouterUrl = "https://openrouter.ai/api/v1/chat/completions";
    std::string modelName = env("HELIOS_LLM_MODEL", "nvidia/nemotron-3-ultra-550b-a55b:free");
    // Adres i nazwa zmiennej z kluczem modelu językowego. Puste = OpenRouter.
    std::string llmUrl = env("HELIOS_LLM_URL");
    std::string llmKeyEnv = env("HELIOS_LLM_KEY_ENV");
    // Zapas: Groq (tylko model językowy - rozpoznawanie mowy działa lokalnie).
    // Ten sam klucz GROQ_API_KEY: gdy darmowy limit OpenRoutera się wyczerpie, Helios przełącza się sam.
    std::string groqUrl = env("HELIOS_LLM_GROQ_URL",
                              "https://api.groq.com/openai/v1/chat/completions");
    std::string groqKeyEnv = env("HELIOS_LLM_GROQ_KEY_ENV", "GROQ_API_KEY");
    // Aktualne darmowe modele Groq (stan: wrzesień 2026). Llama 3.3 70B została
    // wycofana z darmowego planu, dlatego domyślnie gpt-oss-120b, a niżej lista
    // rezerwowa - gdyby nazwy się zmieniły.
    std::string groqModel = env("HELIOS_LLM_GROQ_MODEL", "openai/gpt-oss-120b");
    std::vector<std::string> groqFallbackModels = splitList(
        env("HELIOS_LLM_GROQ_FALLBACK", "openai/gpt-oss-20b,llama-3.3-70b-versatile"));
    // Modele zapasowe, używane gdy główny nie odpowie (oddzielone przecinkami).
    std::vector<std::string> fallbackModels = splitList(
        env("HELIOS_LLM_FALLBACK",
            "nvidia/nemotron-3-super-120b-a12b:free,google/gemma-4-31b-it:free"));
    // Ile tokenów może zużyć model na odpowiedź. Modele "myślące" (reasoning)
    // potrzebują więcej - przy zbyt małym limicie ich odpowiedź bywa pusta
    // (dokładnie taki błąd: "brak choices w odpowiedzi").
    int llmMaxTokens = envInt("HELIOS_LLM_MAX_TOKENS", 1024);
    std::string historyDirectory = env("HELIOS_HISTORY_DIR", "historia");
};

const std::string MASTER_PROMPT = R"(
Jesteś humanoidalnym robotem o imieniu Helios. Jesteś przyjaznym asystentem.
Mówisz krótko i naturalnie po polsku.

Lista twoich komend:
#wave# - pomachaj do kogoś
#dragOn:left# - włącz tryb drag dla lewej ręki
#dragOn:right# - włącz tryb drag dla prawej ręki
#dragOn:both# - włącz tryb drag dla obu rąk
#dragOff# - wyłącza tryb drag
#stats# - pokazuje twoje statystyki, informacje itd
#youtube:TYTUŁ AUTOR# - wyszukuje i odtwarza najbardziej pasujący wynik z YouTube
#volume:PROCENTY# - ustawia głośność głośników USB od 0 do 80
#posZero:left# - wraca do pozycji zerowej lewej ręki
#posZero:right# - wraca do pozycji zerowej prawej ręki
#posZero:both# - wraca do pozycji zerowej obu ramion
#posZero:torso# - wraca do pozycji zerowej tułowia
#posZero:all# - wraca do pozycji zerowej wszystkich jointów

Dozwolony zakres głośności wynosi od 0 do 80.

Przykład:
Użytkownik: Odtwórz 18 lat Axotox
Odpowiedź: Jasne, odtwarzam. #youtube:18 lat Axotox#

Przykład:
Użytkownik: Ustaw głośność na 70 procent
Odpowiedź: Ustawiam głośność. #volume:70#

Przykład:
Użytkownik: Włącz drag lewej ręki
Odpowiedź: Jasne, włączam drag lewej ręki. #dragOn:left#

Przykład:
Użytkownik: Wróć do pozycji zerowej
Odpowiedź: Jasne, wracam do pozycji zerowej #posZero:all#

Wstawiaj komendy dokładnie w podanej formie. Nie wymyślaj nowych komend.
System automatycznie wykryje tag i wykona odpowiednią akcję.
Mów czystym tekstem, bez gwiazdek i emotek.
)";

// ---------------------------------------------------------------------------
// Klient OpenRouter (bez zmian względem poprzedniej wersji)
// ---------------------------------------------------------------------------
size_t writeCallback(void* contents, size_t size, size_t nmemb, std::string* output) {
    const size_t total = size * nmemb;
    output->append(static_cast<char*>(contents), total);
    return total;
}

std::string shortenText(const std::string& text, size_t limit = 240) {
    std::string clean;
    clean.reserve(text.size());
    for (const char character : text) {
        clean += (character == '\n' || character == '\r' || character == '\t') ? ' ' : character;
    }
    if (clean.size() <= limit) {
        return clean;
    }
    return clean.substr(0, limit) + "...";
}

// ---------------------------------------------------------------------------
// Rozmowa z modelem językowym (OpenRouter).
//
// Dwa zabezpieczenia, bo wcześniej zdarzał się komunikat "[Błąd OpenRouter:
// brak choices w odpowiedzi]":
//   1. Gdy model nie odpowie (np. "myślący" model zużył cały limit tokenów na
//      swoje rozważania), próbujemy jeszcze raz - tym razem z wyłączonym
//      rozumowaniem, a potem na modelu zapasowym.
//   2. Każdy błąd pokazuje PRAWDZIWY powód z serwera (kod HTTP + treść), więc
//      od razu widać, co się dzieje (limit, brak kredytów, zły model itd.).
// ---------------------------------------------------------------------------
bool wygladaNaLimit(const std::string& error) {
    std::string lower = error;
    for (char& character : lower) {
        character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    }
    return lower.find("429") != std::string::npos ||
           lower.find("rate limit") != std::string::npos ||
           lower.find("quota") != std::string::npos ||
           lower.find("credits") != std::string::npos;
}

// ---------------------------------------------------------------------------
// Rozmowa z modelem językowym.
//
// Program próbuje kolejno kilku dostawców, więc jedna awaria (limit darmowego
// konta, brak klucza, chwilowa przerwa w działaniu serwera) nie odbiera robotowi
// możliwości rozmowy:
//
//   1. OpenRouter - model z HELIOS_LLM_MODEL (domyślnie darmowy Nemotron),
//   2. ten sam model bez "rozumowania" (modele myślące czasem nie odpowiadają),
//   3. modele zapasowe z HELIOS_LLM_FALLBACK,
//   4. Groq - ten sam klucz GROQ_API_KEY (tylko model językowy).
//
// Gdy serwer zwróci błąd przekroczenia limitu (429), próby na tym samym serwerze
// są pomijane - program od razu przechodzi na kolejnego dostawcę.
// ---------------------------------------------------------------------------
class OpenRouterChat {
    const Options& options_;
    json messages_ = json::array();

    struct Attempt {
        std::string label;
        std::string url;
        std::string keyEnv;
        std::string model;
        bool disableReasoning = false;
        bool openRouterHeaders = false;
    };

    std::vector<Attempt> buildAttempts() const {
        std::vector<Attempt> attempts;

        // --- 1. Dostawca główny (OpenRouter albo własny adres) ---------------
        const std::string primaryKeyEnv =
            options_.llmKeyEnv.empty() ? std::string("OPENROUTER_API_KEY") : options_.llmKeyEnv;
        const std::string primaryUrl =
            options_.llmUrl.empty() ? options_.openRouterUrl : options_.llmUrl;
        const bool primaryIsOpenRouter = primaryUrl.find("openrouter.ai") != std::string::npos;
        const bool primaryIsGroq = primaryUrl.find("groq.com") != std::string::npos;

        if (!env(primaryKeyEnv.c_str()).empty()) {
            attempts.push_back({options_.modelName, primaryUrl, primaryKeyEnv, options_.modelName,
                                false, primaryIsOpenRouter});
            attempts.push_back({options_.modelName + " (bez rozumowania)", primaryUrl, primaryKeyEnv,
                                options_.modelName, true, primaryIsOpenRouter});
            for (const auto& fallback : options_.fallbackModels) {
                attempts.push_back({fallback, primaryUrl, primaryKeyEnv, fallback, true,
                                    primaryIsOpenRouter});
            }
        } else {
            std::cout << "[LLM] brak zmiennej " << primaryKeyEnv << " - pomijam "
                      << primaryUrl << "\n";
        }

        // --- 2. Zapas: Groq (jeśli mamy jego klucz) --------------------------
        if (!env(options_.groqKeyEnv.c_str()).empty() && !(primaryIsGroq && !options_.llmUrl.empty())) {
            attempts.push_back({"Groq " + options_.groqModel, options_.groqUrl,
                                options_.groqKeyEnv, options_.groqModel, true, false});
            for (const auto& fallback : options_.groqFallbackModels) {
                attempts.push_back({"Groq " + fallback, options_.groqUrl,
                                    options_.groqKeyEnv, fallback, true, false});
            }
        }
        return attempts;
    }

    std::string requestOnce(const Attempt& attempt, const std::string& apiKey, std::string* error) {
        json request = {
            {"model", attempt.model},
            {"messages", messages_},
            {"stream", false},
            {"temperature", 0.7},
            {"max_tokens", options_.llmMaxTokens},
        };
        if (attempt.disableReasoning) {
            // Wyłączenie "myślenia" modelu. Każdy serwer wymaga swojego parametru:
            //   OpenRouter: "reasoning": {"enabled": false}
            //   Groq:       "reasoning_effort": "low"   (Groq odrzuca "reasoning",
            //               a także wartość "none" - sprawdzone na żywo)
            if (attempt.url.find("groq.com") != std::string::npos) {
                request["reasoning_effort"] = "low";
            } else {
                request["reasoning"] = {{"enabled", false}};
            }
        }

        CURL* curl = curl_easy_init();
        if (curl == nullptr) {
            *error = "błąd CURL init";
            return {};
        }

        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        const std::string authorization = std::string("Authorization: Bearer ") + apiKey;
        headers = curl_slist_append(headers, authorization.c_str());
        if (attempt.openRouterHeaders) {
            headers = curl_slist_append(headers, "HTTP-Referer: http://localhost");
            headers = curl_slist_append(headers, "X-Title: Helios AI");
        }

        const std::string body = request.dump();
        std::string response;
        curl_easy_setopt(curl, CURLOPT_URL, attempt.url.c_str());
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 180L);

        const CURLcode result = curl_easy_perform(curl);
        long httpStatus = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpStatus);
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        if (result != CURLE_OK) {
            *error = std::string("połączenie: ") + curl_easy_strerror(result);
            return {};
        }

        try {
            const json data = json::parse(response);
            // Błąd sprawdzamy NIEZALEŻNIE od kodu HTTP: serwery potrafią zwrócić
            // 200 i opis błędu w treści.
            if (data.contains("error") && !data["error"].is_null()) {
                const auto& errorObject = data["error"];
                const std::string message =
                    errorObject.is_string()
                        ? errorObject.get<std::string>()
                        : errorObject.value("message", std::string("nieznany"));
                *error = "serwer zwrócił błąd (" + std::to_string(httpStatus) + "): " + shortenText(message);
                return {};
            }
            if (httpStatus < 200 || httpStatus >= 300) {
                *error = "HTTP " + std::to_string(httpStatus) + ": " + shortenText(response);
                return {};
            }
            if (!data.contains("choices") || !data["choices"].is_array() || data["choices"].empty()) {
                *error = "brak odpowiedzi w treści: " + shortenText(response);
                return {};
            }
            const auto& choice = data["choices"][0];
            std::string content;
            if (choice.contains("message") && choice["message"].contains("content") &&
                !choice["message"]["content"].is_null()) {
                const auto& raw = choice["message"]["content"];
                content = raw.is_string() ? raw.get<std::string>() : raw.dump();
            }
            if (trimCopy(content).empty()) {
                const std::string finish = choice.value("finish_reason", std::string("?"));
                *error = "model nie zwrócił treści (finish_reason=" + finish + ")";
                return {};
            }
            return content;
        } catch (const std::exception& parseError) {
            *error = std::string("błąd JSON: ") + parseError.what() + " | treść: " +
                     shortenText(response);
            return {};
        }
    }

public:
    explicit OpenRouterChat(const Options& options) : options_(options) {
        messages_.push_back({{"role", "system"}, {"content", MASTER_PROMPT}});
    }

    std::string send(const std::string& userMessage) {
        messages_.push_back({{"role", "user"}, {"content", userMessage}});

        const std::vector<Attempt> attempts = buildAttempts();
        if (attempts.empty()) {
            messages_.erase(messages_.size() - 1);
            return "[Błąd LLM: brak klucza API. Ustaw OPENROUTER_API_KEY albo GROQ_API_KEY.]";
        }

        std::string lastError = "nieznany";
        for (size_t index = 0; index < attempts.size();) {
            const Attempt& attempt = attempts[index];
            const std::string apiKey = env(attempt.keyEnv.c_str());
            if (apiKey.empty()) {
                ++index;
                continue;
            }

            std::string error;
            const std::string content = requestOnce(attempt, apiKey, &error);
            if (!content.empty()) {
                if (index > 0) {
                    std::cout << "[LLM] odpowiedział: " << attempt.label << "\n";
                }
                messages_.push_back({{"role", "assistant"}, {"content", content}});
                return content;
            }

            lastError = error;
            std::cout << "[LLM] nie udało się (" << attempt.label << "): " << error << "\n";

            if (wygladaNaLimit(error)) {
                // OpenRouter liczy limit dla CAŁEGO konta, więc po 429 nie ma sensu
                // pytać go ponownie - przeskakujemy wszystkie jego próby.
                // Groq liczy osobno dla każdego modelu, więc próbujemy kolejny model.
                if (attempt.url.find("openrouter.ai") != std::string::npos) {
                    size_t next = index + 1;
                    while (next < attempts.size() && attempts[next].url == attempt.url) {
                        ++next;
                    }
                    if (next < attempts.size()) {
                        std::cout << "[LLM] limit tego serwera wyczerpany (429) - przechodzę na: "
                                  << attempts[next].label << "\n";
                    }
                    index = next;
                    continue;
                }
            }
            ++index;
        }

        messages_.erase(messages_.size() - 1);   // nieudanej wypowiedzi nie zapisujemy w historii
        if (wygladaNaLimit(lastError)) {
            std::cout << "[LLM] Wyczerpany limit zapytań. Co można zrobić:\n"
                         "[LLM]  * poczekać do północy czasu UTC (limit darmowy się odnawia),\n"
                         "[LLM]  * dodać 10 USD na koncie OpenRouter (1000 zapytań/dzień),\n"
                         "[LLM]  * ustawić GROQ_API_KEY - Helios przełączy się na Groq sam,\n"
                         "[LLM]  * w ostateczności wpisać komendę z klawiatury, np. /drag left\n"
                         "[LLM]    (polecenia z klawiatury działają bez żadnego modelu i internetu).\n";
        }
        return "[Błąd LLM: " + lastError + "]";
    }

    const json& history() const { return messages_; }
};

std::string sessionTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
    std::tm localTime{};
    localtime_r(&currentTime, &localTime);
    std::ostringstream result;
    result << std::put_time(&localTime, "%Y-%m-%d_%H-%M-%S");
    return result.str();
}

void exportHistory(const OpenRouterChat& chat, const std::filesystem::path& directory,
                   const std::string& fileName) {
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error) {
        return;
    }
    std::ofstream jsonFile(directory / (fileName + ".json"));
    if (jsonFile) {
        jsonFile << chat.history().dump(2) << "\n";
    }
    std::ofstream textFile(directory / (fileName + ".txt"));
    if (textFile) {
        for (const auto& message : chat.history()) {
            const std::string role = message.value("role", "unknown");
            if (role == "system") {
                continue;
            }
            textFile << (role == "user" ? "Ty" : "Helios") << ": "
                     << message.value("content", "") << "\n\n";
        }
    }
}

// ---------------------------------------------------------------------------
// Stan programu współdzielony między wątkami
// ---------------------------------------------------------------------------
enum class State { WaitingForWake, VerifyingWake, Capturing, Busy };

struct Shared {
    std::atomic<bool> running{true};
    std::atomic<bool> robotSpeaking{false};
    std::atomic<State> state{State::WaitingForWake};
    std::atomic<double> followUpUntil{0.0};      // czas (s) do kiedy nie trzeba mówić "Helios"
    std::atomic<bool> manualListen{false};       // żądanie z klawiatury: /listen
    std::mutex queueMutex;
    std::condition_variable queueCv;
    std::deque<std::vector<int16_t>> utterances;
    std::deque<std::vector<int16_t>> verifyQueue;   // fragment do sprawdzenia słowa-klucza
    std::deque<std::string> typed;               // tekst wpisany na klawiaturze
    std::atomic<int> verifyVerdict{0};           // 0 = brak, 1 = przyjęte, -1 = odrzucone
    std::atomic<bool> verifyPending{false};
    std::mutex printMutex;
};

double nowSeconds() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// ---------------------------------------------------------------------------
// Dzwonek potwierdzający słowo-klucz
//
// Krótki "dzyń" (dwa tony) odtwarzany natychmiast po przyjęciu słowa-klucza, żeby
// od razu było słychać, że robot usłyszał - zamiast czekać w ciszy na rozpoznanie.
// Plik WAV generujemy raz i zapisujemy w katalogu tymczasowym. Odtwarzanie idzie
// w osobnym wątku, więc nie opóźnia nagrywania polecenia. Można wyłączyć: HELIOS_CHIME=0
// ---------------------------------------------------------------------------
std::string formatScore(float score) {
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(3);
    stream << score;
    return stream.str();
}

std::string shellQuote(const std::string& value) {
    std::string result = "'";
    for (const char character : value) {
        if (character == '\'') {
            result += "'\\''";
        } else {
            result += character;
        }
    }
    return result + "'";
}

void playWakeChime(const std::string& audioDevice, bool enabled) {
    if (!enabled) {
        return;
    }
    const std::string path = (std::filesystem::temp_directory_path() / "helios_chime.wav").string();
    std::error_code error;
    if (!std::filesystem::exists(path, error)) {
        const int rate = 16000;
        const int samplesPerTone = rate * 60 / 1000;         // 60 ms na ton
        std::vector<int16_t> pcm;
        pcm.reserve(static_cast<size_t>(samplesPerTone) * 2);
        const double frequencies[2] = {1046.5, 1568.0};      // C6, G6
        for (double frequency : frequencies) {
            for (int i = 0; i < samplesPerTone; ++i) {
                const double progress = static_cast<double>(i) / samplesPerTone;
                const double envelope = std::sin(3.14159265 * progress);   // bez trzasków
                const double value = std::sin(2.0 * 3.14159265 * frequency * i / rate);
                pcm.push_back(static_cast<int16_t>(value * envelope * 0.20 * 32767.0));   // cicho, by mikrofon go nie zagłuszył
            }
        }
        if (!helios_audio::writeWav(path, pcm, rate)) {
            return;
        }
    }
    // Odtwarzanie w osobnym wątku - nie blokuje nagrywania polecenia.
    std::thread([path, audioDevice] {
        const std::string command = "aplay -D " + shellQuote(audioDevice) + " -q " + shellQuote(path) +
                                    " >/dev/null 2>&1";
        const int ignored = std::system(command.c_str());
        (void)ignored;
    }).detach();
}

void logLine(Shared& shared, const std::string& text) {
    std::lock_guard<std::mutex> lock(shared.printMutex);
    std::cout << text << std::endl;
}

// ---------------------------------------------------------------------------
// Tryby diagnostyczne
// ---------------------------------------------------------------------------
int runWakeTest(const Options& options, const std::string& wavPath) {
    std::vector<int16_t> samples;
    int sampleRate = 16000;
    if (!helios_audio::readWav(wavPath, &samples, &sampleRate)) {
        std::cerr << "Nie mogę przeczytać pliku: " << wavPath << "\n";
        return 1;
    }
    wakeword::Config config;
    config.melModelPath = options.melModel;
    config.embeddingModelPath = options.embeddingModel;
    config.modelPaths = {options.wakeModel};
    config.threshold = options.wakeThreshold;
    config.patience = options.wakePatience;
    config.debounceSeconds = options.wakeDebounce;

    wakeword::Detector detector(config);
    size_t frame = 0;
    double maxScore = 0.0;
    for (size_t offset = 0; offset + 1280 <= samples.size(); offset += 1280) {
        const auto result = detector.processFrame(samples.data() + offset, 1280);
        maxScore = std::max(maxScore, static_cast<double>(result.score));
        if (result.triggered) {
            std::cout << "[WAKE-TEST] wykryto słowo-klucz w ramce " << frame
                      << " (score " << result.score << ")\n";
        }
        ++frame;
    }
    std::cout << "[WAKE-TEST] ramek: " << frame << ", najwyższy score: " << maxScore
              << ", próg: " << options.wakeThreshold << ", patience: " << options.wakePatience
              << "\n";
    return 0;
}

int runSttTest(const Options& /*options*/, const std::string& wavPath) {
    speech_to_text::Config config = speech_to_text::Config::fromEnvironment();
    speech_to_text::Recognizer recognizer(config);
    std::string error;
    std::string backend;
    const std::string text = recognizer.transcribeFile(wavPath, &error, &backend);
    std::cout << "[STT-TEST] silnik: " << backend << "\n";
    if (text.empty()) {
        std::cout << "[STT-TEST] brak rozpoznania. " << error << "\n";
        return 1;
    }
    std::cout << "[STT-TEST] rozpoznano: " << text << "\n";
    return 0;
}

}  // namespace

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Automatyczne wzmocnienie dźwięku z mikrofonu (AGC)
//
// Detektor słowa-klucza i rozpoznawanie mowy działają wyraźnie lepiej, gdy poziom
// dźwięku jest w miarę stały. Gdy mówisz z drugiego końca pokoju albo odwrócony,
// szczyt bywa kilka razy niższy - detektor wtedy "nie słyszy", a Whisper gubi
// miękkie dźwięki (włącz / wyłącz). Tutaj to wyrównujemy, ale ostrożnie:
// wzmacniamy maksymalnie HELIOS_AGC_MAX razy (domyślnie 10x) i tylko wtedy,
// gdy w ramce naprawdę coś słychać. Wzmocnienie rośnie powoli, ale spada
// natychmiast - inaczej obcinałoby początek słowa.
// Wyłączenie: HELIOS_AGC=0
// ---------------------------------------------------------------------------
class MicGain {
public:
    explicit MicGain(bool enabled, double maxGain = 10.0)
        : enabled_(enabled), maxGain_(maxGain < 1.0 ? 1.0 : maxGain) {}

    void process(int16_t* data, size_t count) {
        if (!enabled_) {
            return;
        }
        int peak = 0;
        for (size_t i = 0; i < count; ++i) {
            const int magnitude = std::abs(static_cast<int>(data[i]));
            if (magnitude > peak) {
                peak = magnitude;
            }
        }
        // Szczyt wygładzony wolno (0.97 na ramkę): pojedyncza pauza między słowami
        // nie wywraca wzmocnienia do góry.
        smoothedPeak_ = std::max(static_cast<double>(peak), smoothedPeak_ * 0.97);
        if (smoothedPeak_ < 150.0) {
            return;                     // cisza - nie podbijamy szumu własnego mikrofonu
        }
        const double target = 12000.0;  // ~37% skali
        double desired = target / smoothedPeak_;
        // Limit ustawiany przez HELIOS_AGC_MAX (domyślnie 10x).
        desired = std::min(maxGain_, std::max(1.0, desired));

        // SZYBKI ATAK: gdy w ramce pojawia się głośny dźwięk, wzmocnienie spada
        // NATYCHMIAST. Bez tego (poprzednia wersja zmieniała wzmocnienie powoli)
        // po ciszy zostawało 6-10x i obcinało początek słowa - a obcięte "He-"
        // w "Helios" czy "wy-" w "wyłącz" to dokładnie te słowa, które robot gubił.
        if (desired < gain_) {
            gain_ = desired;
        } else {
            // WOLNE PODNOSZENIE: w ciszy wzmocnienie rośnie powoli, więc nie "pompuje".
            gain_ += (desired - gain_) * 0.05;
        }
        if (gain_ <= 1.02) {
            gain_ = 1.0;
            return;
        }
        for (size_t i = 0; i < count; ++i) {
            const long value = std::lround(static_cast<double>(data[i]) * gain_);
            data[i] = static_cast<int16_t>(std::max(-32768L, std::min(32767L, value)));
        }
    }

    double gain() const { return gain_; }

private:
    bool enabled_;
    double maxGain_;
    double gain_ = 1.0;
    double smoothedPeak_ = 0.0;
};

// ---------------------------------------------------------------------------
// Kolejka ramek audio
//
// Wątek czytający wyciąga dźwięk z mikrofonu i wrzuca go tutaj, a pętla główna
// (z detektorem słowa-klucza) pobiera ramki i przetwarza. Dzięki temu szybkość
// rozpoznawania NIE wpływa na odbiór dźwięku: gdyby czytanie i model były w tej
// samej pętli, model liczący dłużej niż trwa ramka powodowałby przepełnienie
// bufora ALSA i GUBIENIE próbek - a wtedy słowo-klucz łapany jest rzadko
// (dokładnie taki objaw zgłosił użytkownik).
// ---------------------------------------------------------------------------
class FrameQueue {
public:
    explicit FrameQueue(size_t maxFrames) : maxFrames_(maxFrames) {}

    void push(const int16_t* data, size_t count) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (frames_.size() >= maxFrames_) {
            frames_.pop_front();
            ++dropped_;
        }
        frames_.emplace_back(data, data + count);
        condition_.notify_one();
    }

    // Zwraca false, jeśli w podanym czasie nie pojawiła się nowa ramka.
    bool pop(std::vector<int16_t>* out, int timeoutMs) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!condition_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                                 [this] { return !frames_.empty() || closed_; })) {
            return false;
        }
        if (frames_.empty()) {
            return false;
        }
        *out = std::move(frames_.front());
        frames_.pop_front();
        return true;
    }

    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
        condition_.notify_all();
    }

    size_t dropped() {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_;
    }

private:
    size_t maxFrames_;
    std::deque<std::vector<int16_t>> frames_;
    size_t dropped_ = 0;
    bool closed_ = false;
    std::mutex mutex_;
    std::condition_variable condition_;
};

// ---------------------------------------------------------------------------
// Tryb --mic-test: mierzy, jak głośno mikrofon zbiera Twój głos.
//
// Odpowiada na pytanie "dlaczego muszę drzeć mordę?". Pokazuje szczyt i średnią
// poziomu PRZED wzmocnieniem oraz podpowiada, co zrobić.
// ---------------------------------------------------------------------------
void showMixerInfo(const std::string& device) {
    const size_t cardPosition = device.find("CARD=");
    std::string card = cardPosition == std::string::npos
                           ? std::string("0")
                           : device.substr(cardPosition + 5);
    const size_t comma = card.find(',');
    if (comma != std::string::npos) {
        card = card.substr(0, comma);
    }
    std::cout << "\n[MIC-TEST] Ustawienia miksera dla karty " << card << ":\n";
    const std::string listCommand =
        "amixer -c " + shellQuote(card) + " scontrols 2>/dev/null || echo '  (brak narzędzia amixer)'";
    const int ignored = std::system(listCommand.c_str());
    (void)ignored;
}

// ---------------------------------------------------------------------------
// Tryb --llm-test: sprawdza sam model językowy (OpenRouter), bez mikrofonu.
// Pokazuje dokładnie, co odpowiedział serwer - pomocne przy błędach typu
// "brak choices w odpowiedzi".
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Tryb --kalibracja: dopasowanie rozpoznawania mowy do Twojego głosu
//
// Program czyta po kolei zdania, które masz powiedzieć, nagrywa je, rozpoznaje
// i porównuje z tym, co miało być powiedziane. Z różnic buduje słownik poprawek
// (~/.helios_slownik.txt), który potem jest używany automatycznie przy każdym
// rozpoznaniu - bez przebudowywania programu.
//
// To jest praktyczna "nauka" na Twoim głosie: nie zmieniamy modelu Whisper
// (to model u dostawcy), tylko uczymy program Twoich typowych przekłamań.
// ---------------------------------------------------------------------------
std::vector<std::pair<std::string, std::string>> porownajZdania(
    const std::string& oczekiwane, const std::string& uslyszane) {
    // Dzieli oba zdania na słowa i wyrównuje je (odległość Levenshteina na słowach),
    // a potem zwraca pary "usłyszane -> oczekiwane" dla słów, które się różnią.
    auto podziel = [](const std::string& text) {
        std::vector<std::string> words;
        size_t index = 0;
        while (index < text.size()) {
            const unsigned char character = static_cast<unsigned char>(text[index]);
            if (!(std::isalnum(character) != 0 || character >= 0x80)) {
                ++index;
                continue;
            }
            const size_t start = index;
            while (index < text.size()) {
                const unsigned char current = static_cast<unsigned char>(text[index]);
                if (std::isalnum(current) != 0 || current >= 0x80) {
                    ++index;
                } else {
                    break;
                }
            }
            words.push_back(text.substr(start, index - start));
        }
        return words;
    };

    const std::vector<std::string> a = podziel(oczekiwane);   // co miało być
    const std::vector<std::string> b = podziel(uslyszane);    // co usłyszał program
    const size_t n = a.size();
    const size_t m = b.size();
    if (n == 0 || m == 0) {
        return {};
    }

    // aLower/bLower: porównujemy bez wielkości liter
    std::vector<std::string> aLower(n), bLower(m);
    for (size_t i = 0; i < n; ++i) {
        aLower[i] = lowerPl(a[i]);
    }
    for (size_t j = 0; j < m; ++j) {
        bLower[j] = lowerPl(b[j]);
    }

    std::vector<std::vector<int>> cost(n + 1, std::vector<int>(m + 1, 0));
    for (size_t i = 0; i <= n; ++i) {
        cost[i][0] = static_cast<int>(i);
    }
    for (size_t j = 0; j <= m; ++j) {
        cost[0][j] = static_cast<int>(j);
    }
    for (size_t i = 1; i <= n; ++i) {
        for (size_t j = 1; j <= m; ++j) {
            const int substitute = cost[i - 1][j - 1] + (aLower[i - 1] == bLower[j - 1] ? 0 : 1);
            cost[i][j] = std::min({cost[i - 1][j] + 1, cost[i][j - 1] + 1, substitute});
        }
    }

    std::vector<std::pair<std::string, std::string>> pairs;
    size_t i = n;
    size_t j = m;
    while (i > 0 && j > 0) {
        const bool same = aLower[i - 1] == bLower[j - 1];
        if (same && cost[i][j] == cost[i - 1][j - 1]) {
            --i;
            --j;
            continue;
        }
        const int substitute = cost[i - 1][j - 1] + (same ? 0 : 1);
        if (cost[i][j] == substitute) {
            if (!same) {
                pairs.push_back({b[j - 1], a[i - 1]});   // usłyszane -> oczekiwane
            }
            --i;
            --j;
            continue;
        }
        if (cost[i][j] == cost[i - 1][j] + 1) {
            --i;   // słowo pominięte przez program - nie ma czego poprawiać
            continue;
        }
        --j;       // słowo dodane przez program
    }
    return pairs;
}

int runKalibracja(const Options& options, const std::string& phrasesPath) {
    const std::vector<std::string> domyslne = {
        "włącz drag lewej ręki",
        "wyłącz drag lewej ręki",
        "włącz drag prawej ręki",
        "wyłącz drag",
        "pokaż statystyki",
        "zapisz pozycję bazową",
        "jedź do pozycji zerowej",
        "pomachaj",
        "ustaw prędkość na 30 procent",
        "włącz muzykę",
        "zatrzymaj muzykę",
        "zrób zdjęcie",
    };

    std::vector<std::string> phrases = domyslne;
    if (!phrasesPath.empty()) {
        std::ifstream file(phrasesPath);
        if (!file) {
            std::cout << "[KALIBRACJA] Nie mogę przeczytać pliku: " << phrasesPath << "\n";
            return 1;
        }
        phrases.clear();
        std::string line;
        while (std::getline(file, line)) {
            line = trimCopy(line);
            if (!line.empty() && line[0] != '#') {
                phrases.push_back(line);
            }
        }
    }

    speech_to_text::Config sttConfig = speech_to_text::Config::fromEnvironment();
    speech_to_text::Recognizer recognizer(sttConfig);
    std::cout << "\n=== Kalibracja na Twoim głosie ===\n";
    std::cout << "Zdania do powiedzenia: " << phrases.size() << "\n";
    std::cout << "Po każdym zdaniu poczekaj na wynik - trwa to około sekundy.\n";
    std::cout << "Mów normalnym głosem, tak jak mówisz do robota.\n\n";

    // Rozgrzewka połączenia - żeby pierwsze zdanie nie czekało na DNS/TLS.
    recognizer.warmUp();

    helios_audio::Config micConfig;
    micConfig.device = options.audioDevice;
    micConfig.sampleRate = 16000;
    helios_audio::MicStream mic(micConfig);
    std::string micError;
    if (!mic.start(&micError)) {
        std::cout << "[KALIBRACJA] Nie mogę otworzyć mikrofonu: " << micError << "\n";
        std::cout << "Sprawdź: pgrep -af arecord   (i zamknij stary program)\n";
        return 1;
    }

    std::vector<int16_t> frame(mic.frameSamples());
    helios_audio::RingBuffer ring(3.0, 16000);
    speech_to_text::UtteranceCollector collector(sttConfig);

    std::map<std::string, std::pair<std::string, int>> poprawki;   // usłyszane -> (poprawne, ile razy)
    int udane = 0;

    for (size_t index = 0; index < phrases.size(); ++index) {
        const std::string& phrase = phrases[index];
        std::cout << "[" << (index + 1) << "/" << phrases.size() << "] Powiedz: \"" << phrase
                  << "\"\n";

        // Zbieramy wypowiedź: koniec po 1,2 s ciszy (tak samo jak w rozmowie).
        collector.begin({});
        double startedAt = nowSeconds();
        bool finished = false;
        while (nowSeconds() - startedAt < 8.0) {
            if (!mic.waitReadable(200)) {
                continue;
            }
            const size_t got = mic.readFrame(frame.data());
            if (got == 0) {
                break;
            }
            ring.push(frame.data(), got);
            if (collector.feed(frame.data(), got)) {
                finished = true;
                break;
            }
        }
        if (!finished || collector.audio().empty()) {
            std::cout << "     (nie usłyszałem nic - pomijam)\n\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
            continue;
        }

        std::string error;
        std::string backend;
        const std::string heard = trimCopy(recognizer.transcribe(collector.audio(), &error, &backend));
        if (heard.empty()) {
            std::cout << "     (nie rozpoznano: " << (error.empty() ? "cisza" : error) << ")\n\n";
            continue;
        }

        const bool same = lowerPl(heard) == lowerPl(phrase);
        std::cout << "     usłyszałem: \"" << heard << "\""
                  << (same ? "   [OK]" : "   <-- różni się") << "\n";

        const auto pairs = porownajZdania(phrase, heard);
        for (const auto& pair : pairs) {
            const std::string key = lowerPl(pair.first);
            const std::string value = pair.second;
            auto found = poprawki.find(key);
            if (found == poprawki.end()) {
                poprawki[key] = {value, 1};
            } else {
                found->second.second += 1;
            }
            std::cout << "     poprawka: " << pair.first << " -> " << value << "\n";
        }
        if (same || !pairs.empty()) {
            ++udane;
        }
        std::cout << "\n";
    }
    mic.stop();

    // Zapis słownika poprawek.
    const char* home = std::getenv("HOME");
    const std::string outPath = std::string(home != nullptr ? home : "/tmp") + "/.helios_slownik.txt";
    {
        std::ofstream out(outPath);
        out << "# Słownik poprawek Heliosa - zebrany automatycznie podczas kalibracji\n";
        out << "# Format: usłyszane -> poprawne. Działa od razu, bez przebudowania programu.\n";
        out << "# Liczba powtórzeń pokazuje, jak często program się mylił.\n\n";
        if (poprawki.empty()) {
            out << "# (brak poprawek - wszystko rozpoznane poprawnie)\n";
        }
        for (const auto& entry : poprawki) {
            out << entry.first << " -> " << entry.second.first << "      # powtórzeń: "
                << entry.second.second << "\n";
        }
    }

    std::cout << "=== Koniec kalibracji ===\n";
    std::cout << "Zdań wypowiedzianych: " << udane << " z " << phrases.size() << "\n";
    std::cout << "Zebranych poprawek: " << poprawki.size() << "\n";
    std::cout << "Zapisane do: " << outPath << "\n";
    if (poprawki.empty()) {
        std::cout << "Rozpoznawanie działa na Twoim głosie bez zarzutu - nie ma czego poprawiać.\n";
    } else {
        std::cout << "Poprawki działają od razu przy następnym uruchomieniu Heliosa.\n";
    }
    std::cout << "Możesz dopisywać własne linie do tego pliku (usłyszane -> poprawne).\n";
    return 0;
}

int runLlmTest(const Options& options, const std::string& tekst) {
    std::cout << "=== Test modelu językowego ===\n";
    std::cout << "Model główny: " << options.modelName << "\n";
    std::cout << "Klucz główny: "
              << (options.llmKeyEnv.empty() ? "OPENROUTER_API_KEY" : options.llmKeyEnv) << "\n";
    std::cout << "Zapas (Groq): " << options.groqModel << " przez " << options.groqKeyEnv << "\n";
    std::cout << "Modele zapasowe: ";
    for (const auto& model : options.fallbackModels) {
        std::cout << model << " ";
    }
    std::cout << "\nLimit tokenów: " << options.llmMaxTokens << "\n\n";

    // Sprawdzamy wszystkie klucze, których program może użyć (główny + zapasowy).
    const std::string glownyKlucz = options.llmKeyEnv.empty() ? "OPENROUTER_API_KEY" : options.llmKeyEnv;
    if (env(glownyKlucz.c_str()).empty() && env(options.groqKeyEnv.c_str()).empty()) {
        std::cout << "[LLM-TEST] Brak klucza: ustaw " << glownyKlucz << " albo "
                  << options.groqKeyEnv << " - nie ma czego testować.\n";
        return 1;
    }

    OpenRouterChat chat(options);
    std::cout << "[LLM-TEST] Pytam: \"" << tekst << "\"\n\n";
    const std::string answer = chat.send(tekst);
    std::cout << "\n=== Odpowiedź ===\n" << answer << "\n";
    return 0;
}

// ---------------------------------------------------------------------------
// Sprzątanie po poprzedniej sesji
//
// Gdy zamkniesz okno terminala (albo rozłączysz się z robotem) bez wpisania
// "exit", stary Helios może dalej działać w tle. Mikrofon może wtedy obsłużyć
// tylko jeden program naraz - nowa sesja nie wystartuje ("urządzenie zajęte").
// Poniżej: plik z numerem PID, kończenie starej sesji i zwalnianie mikrofonu.
// Wyłączenie przejmowania: HELIOS_PRZEJMIJ=0
// ---------------------------------------------------------------------------
std::atomic<bool> gStopRequested{false};

void requestStopHandler(int) {
    gStopRequested = true;      // tylko flaga - przerwanie obsłuży pętla główna
}

const char* kPlikPid = "/tmp/helios_ai.pid";

// Sprawdza, czy proces o danym PID to naprawdę Helios. Bez tego, po restarcie
// systemu numer PID mogłoby przejąć coś innego i przejmowanie zabiłoby niewinny
// program (numery PID są przydzielane ponownie).
bool processIsHelios(long pid) {
    if (pid <= 1) {
        return false;
    }
    std::ifstream cmdline("/proc/" + std::to_string(pid) + "/cmdline", std::ios::binary);
    if (!cmdline) {
        return false;
    }
    std::string line((std::istreambuf_iterator<char>(cmdline)), std::istreambuf_iterator<char>());
    return line.find("helios") != std::string::npos;
}

bool processRunning(long pid) {
    if (pid <= 1) {
        return false;
    }
    // Proces "zombie" (już zakończony, czeka na odebranie przez rodzica) też
    // liczy się jako nieżyjący - nie ma czego kończyć.
    std::ifstream statFile("/proc/" + std::to_string(pid) + "/stat", std::ios::binary);
    if (statFile) {
        std::string stat((std::istreambuf_iterator<char>(statFile)), std::istreambuf_iterator<char>());
        const size_t closingBracket = stat.rfind(')');
        if (closingBracket != std::string::npos && closingBracket + 2 < stat.size() &&
            stat[closingBracket + 2] == 'Z') {
            return false;
        }
    }
    errno = 0;
    if (::kill(static_cast<pid_t>(pid), 0) == 0) {
        return true;
    }
    return errno == EPERM;      // proces istnieje, ale należy do innego użytkownika
}

long readPidFile() {
    std::ifstream file(kPlikPid);
    long pid = 0;
    file >> pid;
    return pid;
}

void removePidFile() {
    std::remove(kPlikPid);
}

bool takeOverPreviousSession() {
    if (env("HELIOS_PRZEJMIJ") == "0") {
        return false;
    }
    const long pid = readPidFile();
    if (pid <= 0 || pid == static_cast<long>(::getpid()) || !processRunning(pid)) {
        removePidFile();
        return false;
    }
    if (!processIsHelios(pid)) {
        std::cout << "[Start] Numer PID " << pid << " należy teraz do innego programu - "
                     "nie ruszam go.\n";
        removePidFile();
        return false;
    }

    std::cout << "[Start] Poprzednia sesja Heliosa (PID " << pid
              << ") wciąż działa - kończę ją, żeby zwolnić mikrofon.\n";
    ::kill(static_cast<pid_t>(pid), SIGTERM);
    for (int i = 0; i < 30 && processRunning(pid); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (processRunning(pid)) {
        std::cout << "[Start] Nie zareagowała na prośbę - kończę ją siłą.\n";
        ::kill(static_cast<pid_t>(pid), SIGKILL);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    removePidFile();
    return true;
}

// Ubija osierocone arecord, które trzymają NASZE urządzenie audio.
// "[a]record" zamiast "arecord", żeby wzorzec nie pasował do samego polecenia
// (inaczej pkill mógłby zabić powłokę, która go uruchomiła).
void freeAudioDevice(const std::string& device) {
    const std::string pattern = "[a]record.*" + device;
    const std::string command = "pkill -f " + shellQuote(pattern) + " 2>/dev/null";
    const int ignored = std::system(command.c_str());
    (void)ignored;
}

void printMicHelp(const std::string& device, const std::string& error) {
    std::cout << "\n[Błąd] Nie mogę otworzyć mikrofonu: " << error << "\n";
    std::cout << "Ktoś inny trzyma urządzenie " << device << ". Uruchom te polecenia:\n\n";
    std::cout << "    pgrep -af arecord        # stary proces nagrywania\n";
    std::cout << "    pgrep -af helios         # stara sesja Heliosa\n";
    std::cout << "    pkill -f arecord         # zamknij nagrywanie\n";
    std::cout << "    pkill -f helios_ai       # zamknij starą sesję\n\n";
    std::cout << "Potem uruchom program ponownie.\n";
}

int runMicTest(const Options& options) {
    helios_audio::Config config;
    config.device = options.audioDevice;
    config.sampleRate = 16000;
    helios_audio::MicStream mic(config);
    std::string error;
    if (!mic.start(&error)) {
        std::cout << "[MIC-TEST] Mikrofon zajęty - próbuję go zwolnić.\n";
        freeAudioDevice(config.device);
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
        if (!mic.start(&error)) {
            printMicHelp(config.device, error);
            return 1;
        }
    }

    std::cout << "\n=== Pomiar mikrofonu (6 sekund) ===\n";
    std::cout << "Mów teraz NORMALNYM głosem, tak jak chcesz mówić do robota.\n";
    std::cout << "Nie krzycz - krzyk zniekształca dźwięk i pogarsza rozpoznawanie.\n\n";

    std::vector<int16_t> frame(mic.frameSamples());
    MicGain gain(options.agc, options.agcMax);
    const double started = nowSeconds();
    int globalPeak = 0;
    int globalPeakAfter = 0;
    int windowPeak = 0;
    int windowPeakAfter = 0;
    double windowSum = 0.0;
    size_t windowCount = 0;
    int second = 1;
    while (nowSeconds() - started < 6.0) {
        if (!mic.waitReadable(200)) {
            continue;
        }
        const size_t got = mic.readFrame(frame.data());
        if (got == 0) {
            break;
        }
        // Mierzymy PRZED wzmocnieniem - chcemy znać prawdziwy poziom mikrofonu.
        for (size_t i = 0; i < got; ++i) {
            const int magnitude = std::abs(static_cast<int>(frame[i]));
            if (magnitude > globalPeak) {
                globalPeak = magnitude;
            }
            if (magnitude > windowPeak) {
                windowPeak = magnitude;
            }
            windowSum += static_cast<double>(frame[i]) * frame[i];
            ++windowCount;
        }
        gain.process(frame.data(), got);
        // Sprawdzamy też poziom PO wzmocnieniu: jeśli dotyka maksimum, dźwięk jest
        // obcinany i rozpoznawanie będzie gorsze niż na oryginalnym nagraniu.
        for (size_t i = 0; i < got; ++i) {
            const int magnitude = std::abs(static_cast<int>(frame[i]));
            if (magnitude > windowPeakAfter) {
                windowPeakAfter = magnitude;
            }
            if (magnitude > globalPeakAfter) {
                globalPeakAfter = magnitude;
            }
        }
        if (nowSeconds() - started >= second) {
            const double rms = windowCount > 0 ? std::sqrt(windowSum / windowCount) : 0.0;
            std::cout << "[MIC-TEST] sekunda " << second << ": szczyt " << windowPeak
                      << ", średnia " << static_cast<int>(rms) << ", po wzmocnieniu "
                      << windowPeakAfter << "\n";
            windowPeak = 0;
            windowPeakAfter = 0;
            windowSum = 0.0;
            windowCount = 0;
            ++second;
        }
    }
    mic.stop();

    const double percent = 100.0 * static_cast<double>(globalPeak) / 32767.0;
    const double peakDb = globalPeak > 0 ? 20.0 * std::log10(static_cast<double>(globalPeak) / 32768.0)
                                         : -120.0;
    std::cout << "\n=== Wynik ===\n";
    std::cout << "Najwyższy szczyt: " << globalPeak << " z 32767 (" << static_cast<int>(percent)
              << "% skali, " << static_cast<int>(peakDb) << " dB)\n";
    const int afterPercent = static_cast<int>(100.0 * globalPeakAfter / 32767.0);
    std::cout << "Po wzmocnieniu AGC: " << globalPeakAfter << " (" << afterPercent << "% skali)\n";
    if (globalPeakAfter >= 32700) {
        std::cout << "UWAGA: po wzmocnieniu dźwięk dotyka maksimum, czyli jest OBCINANY.\n";
        std::cout << "To pogarsza rozpoznawanie. Zmniejsz limit: HELIOS_AGC_MAX=6\n"
                     "albo ustaw HELIOS_AGC=0.\n";
    }
    std::cout << "Wzmocnienie AGC, które się włączyło: " << formatScore(static_cast<float>(gain.gain()))
              << "x (limit " << options.agcMax << "x)\n\n";
    if (gain.gain() >= options.agcMax - 0.01) {
        std::cout << "UWAGA: wzmocnienie dobiło do limitu - mikrofon jest naprawdę cichy.\n";
        std::cout << "Możesz zwiększyć limit: HELIOS_AGC_MAX=20, ale lepiej podnieść\n";
        std::cout << "czułość mikrofonu w alsamixer (patrz niżej) - to czyściejsze rozwiązanie.\n\n";
    }

    if (globalPeak < 1500) {
        std::cout << "WNIOSEK: mikrofon zbiera BARDZO cicho - stąd to darcie mordy.\n";
        std::cout << "Podnieś czułość mikrofonu raz na zawsze:\n";
        std::cout << "    alsamixer -c 3        # klawisz F4, strzałki w górę na 'Capture',\n";
        std::cout << "                          # cel: 70-85%, nie 100% (100% szumi)\n";
        std::cout << "    sudo alsactl store    # zapamięta po restarcie\n";
    } else if (globalPeak < 4000) {
        std::cout << "WNIOSEK: cicho, ale da się pracować - AGC to podbija do ~37% skali.\n";
        std::cout << "Jeśli robot nadal słabo łapie, podnieś 'Capture' w alsamixer -c 3.\n";
    } else if (globalPeak > 32000) {
        std::cout << "WNIOSEK: mikrofon jest PRZESTEROWANY (szczyt na maksimum).\n";
        std::cout << "Mów trochę ciszej albo zmniejsz 'Capture' w alsamixer -c 3.\n";
        std::cout << "Przesterowany dźwięk rozpoznaje się GORZEJ niż cichy.\n";
    } else {
        std::cout << "WNIOSEK: poziom mikrofonu jest dobry.\n";
        std::cout << "To znaczy, że NIE musisz krzyczeć. Krzyk przesterowuje mikrofon\n";
        std::cout << "i rozpoznawanie wychodzi gorzej niż przy normalnym głosie.\n";
    }
    showMixerInfo(options.audioDevice);
    return 0;
}

int main(int argc, char** argv) {
    std::cout << std::unitbuf;      // nic nie ginie w buforze przy awarii
    installCrashHandler();
    curl_global_init(CURL_GLOBAL_DEFAULT);
    // Ctrl+C i "kill" kończą program po dobroci: sprząta po sobie mikrofon,
    // zamyka wątki i zapisuje historię rozmowy.
    std::signal(SIGINT, requestStopHandler);
    std::signal(SIGTERM, requestStopHandler);

    Options options;
    std::vector<std::string> args(argv + 1, argv + argc);

    for (size_t i = 0; i < args.size(); ++i) {
        if (args[i] == "--wake-test" && i + 1 < args.size()) {
            const int result = runWakeTest(options, args[i + 1]);
            curl_global_cleanup();
            return result;
        }
        if (args[i] == "--stt" && i + 1 < args.size()) {
            const int result = runSttTest(options, args[i + 1]);
            curl_global_cleanup();
            return result;
        }
        if (args[i] == "--llm-test") {
            const std::string tekst = (i + 1 < args.size() && args[i + 1].rfind("--", 0) != 0)
                                          ? args[i + 1]
                                          : std::string("Helios, włącz drag lewej ręki");
            const int result = runLlmTest(options, tekst);
            curl_global_cleanup();
            return result;
        }
        if (args[i] == "--kalibracja") {
            const std::string plik = (i + 1 < args.size() && args[i + 1].rfind("--", 0) != 0)
                                         ? args[i + 1]
                                         : std::string();
            const int result = runKalibracja(options, plik);
            curl_global_cleanup();
            return result;
        }
        if (args[i] == "--mic-test") {
            const int result = runMicTest(options);
            curl_global_cleanup();
            return result;
        }
        if (args[i] == "--help" || args[i] == "-h") {
            std::cout << "Użycie:\n"
                         "  ./main                      # praca normalna (stale nasłuchuje \"Helios\")\n"
                         "  ./main --wake-test plik.wav # test detektora słowa-klucza\n"
                         "  ./main --stt plik.wav       # test rozpoznawania mowy\n"
                         "  ./main --mic-test           # pomiar poziomu mikrofonu (6 s)\n"
                         "  ./main --kalibracja         # kalibracja na Twoim głosie (słownik poprawek)\n"
                         "  ./main --llm-test [tekst]   # test modelu językowego (OpenRouter)\n"
                         "  HELIOS_NO_WAKEWORD=1 ./main # tryb ręczny (/listen)\n"
                         "\nKonfiguracja przez zmienne środowiskowe - patrz README.md\n";
            curl_global_cleanup();
            return 0;
        }
    }

    std::cout << "=== Robot Helios uruchomiony ===\n";
    std::cout << "Model LLM: " << options.modelName;
    if (!env("GROQ_API_KEY").empty()) {
        std::cout << "  |  zapas: Groq " << options.groqModel;
    }
    std::cout << "\n";
    std::cout << "Mikrofon: " << options.audioDevice << "\n";
    std::cout << "Model słowa-klucza: " << options.wakeModel << " (próg "
              << options.wakeThreshold << ", patience " << options.wakePatience << ")\n";

    // --- rozpoznawanie mowy -------------------------------------------------
    speech_to_text::Config sttConfig = speech_to_text::Config::fromEnvironment();
    speech_to_text::Recognizer recognizer(sttConfig);
    std::cout << "Silnik STT: " << sttConfig.backend;
    if (!sttConfig.serverUrl.empty()) {
        std::cout << " (" << sttConfig.serverUrl << ")";
    } else {
        std::cout << " (" << sttConfig.whisperModel << ")";
    }
    std::cout << "\n";

    // Rozgrzewka połączenia z serwerem transkrypcji w tle: pierwsze zapytanie
    // (DNS + TLS) trwa najdłużej, a przy starcie rozmowy liczy się każda sekunda.
    if (!sttConfig.serverUrl.empty()) {
        std::thread([&recognizer] {
            const auto started = std::chrono::steady_clock::now();
            recognizer.warmUp();
            const double ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - started)
                                  .count();
            std::cout << "[STT] rozgrzewka połączenia: " << static_cast<int>(ms) << " ms\n";
        }).detach();
    }

    // --- detektor słowa-klucza ---------------------------------------------
    //  Tworzymy go TYLKO wtedy, gdy naprawdę nasłuchujemy. Dzięki temu tryb ręczny
    //  (HELIOS_NO_WAKEWORD=1) oraz testy --stt nie ładują modeli ONNX w ogóle.
    bool wakewordReady = false;
    std::unique_ptr<wakeword::Detector> detectorPtr;
    if (!options.noWakeword) {
        std::cout << "[Krok] wczytuję modele słowa-klucza (mel + embedding + helios.onnx)...\n";
        detectorPtr = std::make_unique<wakeword::Detector>([&] {
            wakeword::Config config;
            config.melModelPath = options.melModel;
            config.embeddingModelPath = options.embeddingModel;
            config.modelPaths = {options.wakeModel};
            config.threshold = options.wakeThreshold;
            config.patience = options.wakePatience;
            config.debounceSeconds = options.wakeDebounce;
            config.threads = envInt("HELIOS_WAKE_THREADS", 2);
            return config;
        }());
        std::cout << "[Krok] modele słowa-klucza wczytane\n";
    } else {
        std::cout << "[Tryb ręczny] detektor słowa-klucza wyłączony (HELIOS_NO_WAKEWORD)\n";
    }

    if (!options.noWakeword) {
        std::error_code error;
        if (std::filesystem::exists(options.wakeModel, error) &&
            std::filesystem::exists(options.melModel, error) &&
            std::filesystem::exists(options.embeddingModel, error)) {
            wakewordReady = true;
        } else {
            std::cout << "[Uwaga] Brak plików modelu słowa-klucza - działam w trybie ręcznym.\n"
                         "         Sprawdź: " << options.wakeModel << "\n";
        }
    }

    // --- mikrofon -----------------------------------------------------------
    helios_audio::Config micConfig;
    micConfig.device = options.audioDevice;
    if (!options.alsaPeriod.empty()) {
        micConfig.extraArgs += " --period-size=" + options.alsaPeriod;
    }
    if (!options.alsaBuffer.empty()) {
        micConfig.extraArgs += " --buffer-size=" + options.alsaBuffer;
    }
    helios_audio::MicStream mic(micConfig);

    // Kończymy poprzednią sesję (jeśli została otwarta niedbale) i zapisujemy
    // swój numer PID - dzięki temu następne uruchomienie będzie wiedziało,
    // że to my trzymamy mikrofon.
    takeOverPreviousSession();
    {
        std::ofstream pidFile(kPlikPid);
        pidFile << ::getpid();
    }

    std::string micError;
    if (!mic.start(&micError)) {
        // Najczęstsza przyczyna: osierocone arecord z poprzedniej sesji.
        std::cout << "[Start] Mikrofon zajęty - próbuję go zwolnić.\n";
        freeAudioDevice(options.audioDevice);
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
        if (!mic.start(&micError)) {
            printMicHelp(options.audioDevice, micError);
            removePidFile();
            curl_global_cleanup();
            return 1;
        }
    }
    std::cout << "Mikrofon otwarty (" << micConfig.device << ").\n";
    std::cout << "Aby zakończyć rozmowę: napisz exit i Enter (albo Ctrl+C).\n";
    std::cout << "Polecenia z klawiatury bez internetu: /drag left, /drag right, /drag both,\n"
                 "  /dragoff, /stats, /wave, /zero   (albo wprost tag, np. #dragOn:left#)\n";

    std::cout << "[Krok] mikrofon gotowy, zaczynam pętlę nasłuchu\n";

    helios_audio::RingBuffer ring(3.0, 16000);
    speech_to_text::UtteranceCollector collector(sttConfig);

    Shared shared;
    OpenRouterChat chat(options);
    const std::filesystem::path historyDirectory = options.historyDirectory;
    const std::string historyFileName = "rozmowa_" + sessionTimestamp();

    if (wakewordReady) {
        std::cout << "Nasłuchuję słowa-klucza \"Helios\"... (mów: \"Helios, włącz drag lewej ręki\")\n";
    } else {
        std::cout << "Tryb ręczny: wpisz /listen aby nagrać wypowiedź, exit aby zakończyć.\n";
    }
    std::cout << "ALSA: period " << (options.alsaPeriod.empty() ? "auto" : options.alsaPeriod)
              << ", bufor " << (options.alsaBuffer.empty() ? "auto" : options.alsaBuffer) << "\n";
    std::cout << "Wzmocnienie mikrofonu (AGC): ";
    if (options.agc) {
        std::cout << "włączone, maksymalnie " << options.agcMax << "x";
    } else {
        std::cout << "wyłączone";
    }
    std::cout << "  |  okno na polecenie po pobudce: " << options.captureArmSeconds << " s\n";
    std::cout << "Szybka ścieżka (bez weryfikacji) od pewności: " << options.wakeFastScore
              << "  |  dzwonek: " << (options.chime ? "włączony" : "wyłączony") << "\n";
    std::cout << "Weryfikacja słowa-klucza: "
              << (options.verifyWake ? (options.verifyStrict ? "włączona (tryb strict)" : "włączona")
                             : "wyłączona (HELIOS_WAKE_VERIFY=off)")
              << "\n";
    std::cout << "Po odpowiedzi masz jeszcze " << options.followUpSeconds
              << " s na dokończenie rozmowy bez powtarzania \"Helios\".\n\n";

    // Wątek czytający dźwięk - patrz komentarz przy klasie FrameQueue.
    FrameQueue frameQueue(25);   // ~2 s zapasu, potem nadpisujemy najstarsze
    std::thread audioReader([&] {
        std::vector<int16_t> localFrame(mic.frameSamples());
        while (shared.running && !gStopRequested) {
            if (!mic.waitReadable(200)) {
                continue;   // brak danych w tym momencie - sprawdzam, czy kończymy pracę
            }
            const size_t got = mic.readFrame(localFrame.data());
            if (got == 0) {
                if (shared.running) {
                    std::cerr << "[Audio] Strumień mikrofonu się zakończył - próbuję wznowić.\n";
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    std::string restartError;
                    mic.start(&restartError);
                }
                continue;
            }
            frameQueue.push(localFrame.data(), got);
        }
        frameQueue.close();
    });

    // --- wątek roboczy: rozpoznawanie + LLM + mowa --------------------------
    std::thread worker([&] {
        while (shared.running) {
            std::vector<int16_t> utterance;
            std::vector<int16_t> verifyClip;
            std::string typedText;
            bool isVerify = false;
            {
                std::unique_lock<std::mutex> lock(shared.queueMutex);
                shared.queueCv.wait_for(lock, std::chrono::milliseconds(200), [&] {
                    return !shared.verifyQueue.empty() || !shared.utterances.empty() ||
                           !shared.typed.empty() || !shared.running;
                });
                if (!shared.running) {
                    break;
                }
                if (!shared.verifyQueue.empty()) {
                    verifyClip = std::move(shared.verifyQueue.front());
                    shared.verifyQueue.pop_front();
                    isVerify = true;
                } else if (!shared.typed.empty()) {
                    typedText = std::move(shared.typed.front());
                    shared.typed.pop_front();
                } else if (!shared.utterances.empty()) {
                    utterance = std::move(shared.utterances.front());
                    shared.utterances.pop_front();
                } else {
                    continue;
                }
            }

            if (isVerify) {
                // Sprawdzenie, czy to naprawdę "Helios" (patrz komentarz przy looksLikeWakeWord).
                std::string error;
                std::string backend;
                const auto verifyStarted = std::chrono::steady_clock::now();
                const std::string heard = recognizer.transcribe(verifyClip, &error, &backend);
                const double verifyMs = std::chrono::duration<double, std::milli>(
                                            std::chrono::steady_clock::now() - verifyStarted)
                                            .count();
                bool accepted = false;
                std::string reason;
                if (heard.empty()) {
                    accepted = !options.verifyStrict;
                    // Nie mogliśmy sprawdzić słowa (np. brak klucza STT) - przyjmujemy
                    // pobudkę, żeby robot nie był głuchy, ale piszemy to WPROST.
                    reason = std::string("NIE MOGŁEM SPRAWDZIĆ, przyjmuję na wszelki wypadek: ") +
                             (error.empty() ? "brak transkrypcji" : error);
                } else if (looksLikeWakeWord(heard)) {
                    accepted = true;
                    reason = heard;
                } else {
                    accepted = false;
                    reason = "usłyszałem: \"" + heard + "\"";
                }
                std::ostringstream verifyLine;
                verifyLine << "[Słowo-klucz] " << (accepted ? "POTWIERDZONY" : "odrzucony")
                           << " (" << reason << ") - " << static_cast<int>(verifyMs) << " ms";
                logLine(shared, verifyLine.str());
                shared.verifyVerdict = accepted ? 1 : -1;
                shared.verifyPending = false;
                continue;
            }

            std::string text = typedText;
            if (text.empty()) {
                std::string error;
                std::string backend;
                const auto heardStarted = std::chrono::steady_clock::now();
                const std::string heard = recognizer.transcribe(utterance, &error, &backend);
                const double heardMs = std::chrono::duration<double, std::milli>(
                                           std::chrono::steady_clock::now() - heardStarted)
                                           .count();
                {
                    std::ostringstream timing;
                    timing << "[Czas] rozpoznawanie: " << static_cast<int>(heardMs) << " ms ("
                           << (backend.empty() ? "?" : backend) << ")";
                    logLine(shared, timing.str());
                }
                text = stripWakeWord(heard);
                if (text.empty()) {
                    if (!heard.empty()) {
                        logLine(shared, "[STT] To było tylko słowo-klucz - czekam na polecenie.");
                    } else {
                        logLine(shared, "[STT] Nie rozpoznano mowy. " + error);
                    }
                    shared.state = State::WaitingForWake;
                    continue;
                }
            }
            logLine(shared, "Ty: " + text);

            const std::string response = chat.send(text);
            const std::string cleaned = llm_commands::process(response);
            logLine(shared, "\nHelios: " + cleaned + "\n");

            shared.robotSpeaking = true;
            helios_tts::speak(cleaned);
            shared.robotSpeaking = false;
            shared.followUpUntil = nowSeconds() + options.followUpSeconds;

            exportHistory(chat, historyDirectory, historyFileName);
            shared.state = State::WaitingForWake;
        }
    });

    // --- wątek klawiatury (tryb ręczny + wyjście) ---------------------------
    //  UWAGA: ten wątek NIGDY nie czyta mikrofonu (jeden strumień arecord może
    //  być czytany tylko z jednego wątku). Ustawia tylko flagę /listen, a pętla
    //  mikrofonu sama zaczyna nagrywanie.
    std::thread keyboard([&] {
        std::string line;
        while (shared.running && std::getline(std::cin, line)) {
            if (line == "exit" || line == "quit") {
                shared.running = false;
                shared.queueCv.notify_all();
                break;
            }
            if (line == "/listen") {
                shared.manualListen = true;
                continue;
            }

            // Skróty poleceń robota. Działają BEZ internetu i BEZ modelu
            // językowego - wykonują komendę od razu.
            // Pełna lista tagów: #wave#, #dragOn:left#, #dragOff#, #stats# itd.
            {
                const std::string skrot =
                    line == "/drag left"   ? "#dragOn:left#"  :
                    line == "/drag right"  ? "#dragOn:right#" :
                    line == "/drag both"   ? "#dragOn:both#"  :
                    line == "/dragoff"     ? "#dragOff#"      :
                    line == "/stats"       ? "#stats#"        :
                    line == "/wave"        ? "#wave#"         :
                    line == "/zero"        ? "#posZero:all#"  : std::string();
                if (!skrot.empty()) {
                    logLine(shared, "[Klawiatura] Wykonuję: " + skrot + " (bez modelu)");
                    llm_commands::process(skrot);
                    continue;
                }
            }

            // Linia z tagiem, np. #dragOn:left# - wykonujemy bezpośrednio.
            if (line.find('#') != std::string::npos) {
                logLine(shared, "[Klawiatura] Wykonuję polecenie bezpośrednio (bez modelu).");
                const std::string bezTagow = llm_commands::process(line);
                if (!bezTagow.empty()) {
                    shared.robotSpeaking = true;
                    helios_tts::speak(bezTagow);
                    shared.robotSpeaking = false;
                }
                continue;
            }

            if (line.rfind("/", 0) == 0) {
                logLine(shared, "[Klawiatura] Nieznana komenda. Dostępne: /listen, /drag left, "
                                "/drag right, /drag both, /dragoff, /stats, /wave, /zero, exit");
                continue;
            }
            if (!line.empty()) {
                std::lock_guard<std::mutex> lock(shared.queueMutex);
                shared.typed.push_back(line);
                shared.queueCv.notify_all();
            }
        }
    });

    // --- pętla mikrofonu: słowo-klucz -> wypowiedź --------------------------
    std::vector<int16_t> frame;
    double captureStartedAt = 0.0;
    double verifyDeadline = 0.0;      // do kiedy zbieramy fragment do sprawdzenia
    double verifyStartedAt = 0.0;
    bool verifySent = false;          // czy fragment poszedł już do STT
    bool collectorFinished = false;   // czy kolektor zamknął wypowiedź przed werdyktem
    bool firstFrameLogged = false;
    const double captureArmTimeout = options.captureArmSeconds;
    bool announcedFollowUp = false;

    // Statystyki audio - wypisywane raz na 5 s, żeby od razu było widać,
    // czy dźwięk płynie bez przerw i czy detektor wyrabia w czasie rzeczywistym.
    MicGain micGain(options.agc, options.agcMax);
    double statsAt = nowSeconds();
    int statsFrames = 0;
    int statsPeak = 0;
    double statsDetectorMs = 0.0;
    double statsMaxScore = 0.0;
    size_t statsDropped = 0;

    while (shared.running && !gStopRequested) {
        if (!frameQueue.pop(&frame, 500)) {
            continue;   // brak nowej ramki (np. mikrofon w trakcie restartu)
        }
        const size_t got = frame.size();
        micGain.process(frame.data(), got);     // wyrównanie poziomu (patrz MicGain)
        ring.push(frame.data(), got);
        if (!firstFrameLogged) {
            firstFrameLogged = true;
            std::cout << "[Krok] pierwsza ramka audio odebrana (" << got << " próbek)\n";
        }

        // Statystyki: poziom szczytu i liczba ramek w oknie 5 s.
        ++statsFrames;
        for (size_t i = 0; i < got; ++i) {
            const int16_t value = frame[i];
            const int16_t magnitude = static_cast<int16_t>(value < 0 ? -value : value);
            if (magnitude > statsPeak) {
                statsPeak = magnitude;
            }
        }
        if (nowSeconds() - statsAt >= 5.0) {
            const double window = nowSeconds() - statsAt;
            const size_t dropped = frameQueue.dropped();
            std::ostringstream stats;
            stats.setf(std::ios::fixed);
            stats.precision(1);
            stats << "[Audio] ramek/s: " << (statsFrames / window)
                  << ", detektor: " << (statsFrames > 0 ? statsDetectorMs / statsFrames : 0.0)
                  << " ms/ramkę, szczyt: " << statsPeak
                  << ", max score: " << formatScore(static_cast<float>(statsMaxScore))
                  << ", zgubione ramki: " << (dropped - statsDropped)
                  << ", wzmocnienie: " << formatScore(static_cast<float>(micGain.gain())) << "x";
            logLine(shared, stats.str());
            if (statsFrames / window < 10.0) {
                logLine(shared, "[Audio] UWAGA: dociera mniej ramek niż powinno (12,5/s) - "
                                "dźwięk gubi się po drodze.");
            }
            if (statsFrames > 0 && statsDetectorMs / statsFrames > 60.0) {
                logLine(shared, "[Audio] UWAGA: detektor liczy dłużej niż trwa ramka (80 ms) - "
                                "ogranicz wątki: HELIOS_WAKE_THREADS=1");
            }
            statsAt = nowSeconds();
            statsFrames = 0;
            statsPeak = 0;
            statsDetectorMs = 0.0;
            statsMaxScore = 0.0;
            statsDropped = dropped;
        }

        // Gdy robot mówi lub przetwarza - nie nasłuchujemy własnego głosu.
        if (shared.robotSpeaking || shared.state == State::Busy) {
            continue;
        }

        if (shared.state == State::WaitingForWake) {
            // Ręczne /listen pomija słowo-klucz.
            if (shared.manualListen.exchange(false)) {
                logLine(shared, "[Tryb ręczny] Nagrywam... (mów teraz)");
                collector.begin(ring.last(0.3));
                captureStartedAt = nowSeconds();
                shared.state = State::Capturing;
                continue;
            }

            const bool followUpActive = nowSeconds() < shared.followUpUntil.load();
            // W trybie ręcznym (HELIOS_NO_WAKEWORD=1) detektora słowa-klucza nie ma
            // w ogóle - bez tego sprawdzenia program przewracał się po pierwszej
            // ramce audio (SIGSEGV).
            wakeword::FrameResult result;
            if (detectorPtr) {
                const auto detectorStarted = std::chrono::steady_clock::now();
                result = detectorPtr->processFrame(frame.data(), got);
                statsDetectorMs += std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - detectorStarted)
                                       .count();
                if (result.score > statsMaxScore) {
                    statsMaxScore = result.score;
                }
            }
            if (wakewordReady && result.triggered) {
                // Zawsze zaczynamy nagrywać - nawet gdy weryfikujemy słowo-klucz,
                // bo polecenie wypowiadane jest tuż po nim.
                collector.begin(ring.last(options.preRollSeconds));
                collectorFinished = false;
                shared.followUpUntil = 0.0;
                announcedFollowUp = false;
                // Detektor bardzo pewny? Wtedy nie ma po co pytać STT - reagujemy od razu.
                const bool confident = result.score >= options.wakeFastScore;
                if (options.verifyWake && !confident) {
                    logLine(shared, "[Helios] Pobudka - sprawdzam słowo... (pewność " +
                                        formatScore(result.score) + ")");
                    verifyStartedAt = nowSeconds();
                    // Wysyłamy fragment zaraz po pobudce (wcześniej czekaliśmy 0,9 s
                    // "na zapas" - to było główne źródło zwłoki na starcie rozmowy).
                    verifyDeadline = verifyStartedAt + 0.25;
                    verifySent = false;
                    shared.verifyPending = true;
                    shared.verifyVerdict = 0;
                    shared.state = State::VerifyingWake;
                } else {
                    if (confident && options.verifyWake) {
                        logLine(shared, "[Helios] Pewna detekcja (" + formatScore(result.score) +
                                            ") - pomijam weryfikację.");
                    }
                    playWakeChime(options.audioDevice, options.chime);
                    logLine(shared, "[Helios] Słucham...");
                    captureStartedAt = nowSeconds();
                    shared.state = State::Capturing;
                }
            } else if (followUpActive) {
                if (!announcedFollowUp) {
                    announcedFollowUp = true;
                    logLine(shared, "[Helios] Możesz mówić dalej bez słowa \"Helios\"...");
                }
                collector.begin(ring.last(0.3));
                captureStartedAt = nowSeconds();
                shared.state = State::Capturing;
            } else {
                announcedFollowUp = false;
            }
            continue;
        }

        if (shared.state == State::VerifyingWake) {
            // Nagrywamy dalej (nic nie ginie) i czekamy na werdykt z wątku roboczego.
            if (collector.feed(frame.data(), got)) {
                collectorFinished = true;
            }
            const double now = nowSeconds();
            if (!verifySent && now >= verifyDeadline) {
                verifySent = true;
                std::lock_guard<std::mutex> lock(shared.queueMutex);
                shared.verifyQueue.push_back(ring.last(1.6));
                shared.queueCv.notify_all();
            }
            const int verdict = shared.verifyVerdict.load();
            if (verdict == 1) {
                shared.verifyVerdict = 0;
                playWakeChime(options.audioDevice, options.chime);
                logLine(shared, "[Helios] Słucham...");
                captureStartedAt = now;
                if (collectorFinished) {
                    if (!collector.audio().empty()) {
                        std::lock_guard<std::mutex> lock(shared.queueMutex);
                        shared.utterances.push_back(collector.audio());
                        shared.queueCv.notify_all();
                        shared.state = State::Busy;
                    } else {
                        shared.state = State::WaitingForWake;
                    }
                } else {
                    shared.state = State::Capturing;
                }
            } else if (verdict == -1) {
                shared.verifyVerdict = 0;
                collector.finish();
                shared.state = State::WaitingForWake;
            } else if (now - verifyStartedAt > 8.0) {
                // Bezpiecznik: brak odpowiedzi z STT (np. zerwane łącze).
                logLine(shared, "[Helios] Nie udało się sprawdzić słowa - wracam do nasłuchu.");
                shared.verifyPending = false;
                collector.finish();
                shared.state = State::WaitingForWake;
            }
            continue;
        }

        if (shared.state == State::Capturing) {
            // Dopóki nie zaczęła się mowa, dalej słuchamy słowa-klucza. Dzięki temu
            // gdy powtórzysz "Helios" (np. po nieudanej próbie), robot od razu
            // zaczyna nagrywać od nowa, zamiast być "głuchy" przez kilka sekund.
            if (detectorPtr && !collector.inSpeech()) {
                const auto resultAgain = detectorPtr->processFrame(frame.data(), got);
                if (wakewordReady && resultAgain.triggered) {
                    collector.begin(ring.last(options.preRollSeconds));
                    playWakeChime(options.audioDevice, options.chime);
                    logLine(shared, "[Helios] Słyszę Cię ponownie - nagrywam od nowa.");
                    captureStartedAt = nowSeconds();
                    continue;
                }
            }
            if (collector.feed(frame.data(), got)) {
                if (!collector.audio().empty()) {
                    {
                        std::lock_guard<std::mutex> lock(shared.queueMutex);
                        shared.utterances.push_back(collector.audio());
                        shared.queueCv.notify_all();
                    }
                    shared.state = State::Busy;
                } else {
                    logLine(shared, "[Helios] Za krótko - wracam do nasłuchu.");
                    shared.state = State::WaitingForWake;
                }
            } else if (!collector.inSpeech() && nowSeconds() - captureStartedAt > captureArmTimeout) {
                // Nikt nic nie powiedział (np. fałszywy alarm słowa-klucza).
                logLine(shared, "[Helios] Cisza - wracam do nasłuchu.");
                collector.finish();
                shared.state = State::WaitingForWake;
            }
            continue;
        }
    }

    shared.running = false;
    shared.queueCv.notify_all();
    frameQueue.close();
    // Najpierw kończymy wątek czytający, DOPIERO POTEM zamykamy mikrofon.
    // Odwrotna kolejność (pclose przy trwającym czytaniu w innym wątku) jest błędem.
    if (audioReader.joinable()) {
        audioReader.join();
    }
    mic.stop();
    if (keyboard.joinable()) {
        keyboard.detach();   // wątek klawiatury blokuje na getline - nie czekamy na niego
    }
    if (worker.joinable()) {
        worker.join();
    }

    exportHistory(chat, historyDirectory, historyFileName);
    std::cout << "=== Zakończono. Historia: "
              << (historyDirectory / (historyFileName + ".txt")) << " ===\n";
    removePidFile();

    curl_global_cleanup();
    return 0;
}

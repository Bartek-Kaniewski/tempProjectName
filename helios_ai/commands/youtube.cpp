#include "youtube.h"

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace commands {

using json = nlohmann::json;

namespace {

constexpr const char* YOUTUBE_API_URL =
    "https://www.googleapis.com/youtube/v3/search";
constexpr const char* AUDIO_DEVICE = "alsa/plughw:CARD=REF,DEV=0";

size_t writeCallback(void* contents, size_t size, size_t nmemb,
                     std::string* output) {
    const size_t total = size * nmemb;
    output->append(static_cast<char*>(contents), total);
    return total;
}

std::string urlEncode(CURL* curl, const std::string& value) {
    char* encoded = curl_easy_escape(curl, value.c_str(),
                                     static_cast<int>(value.size()));
    if (!encoded) return {};

    std::string result(encoded);
    curl_free(encoded);
    return result;
}

bool playWithMpv(const std::string& url) {
    const pid_t pid = fork();

    if (pid < 0) {
        std::cerr << "[YouTube] Nie uda³o siê uruchomiæ procesu mpv.\n";
        return false;
    }

    if (pid == 0) {
        execlp(
            "mpv",
            "mpv",
            "--no-video",
            "--audio-device=alsa/plughw:CARD=REF,DEV=0",
            "--cache=yes",
            "--cache-secs=30",
            "--cache-pause=yes",
            "--ytdl-format=251",
            url.c_str(),
            static_cast<char*>(nullptr)
        );

        std::cerr << "[YouTube] Nie znaleziono programu mpv.\n";
        _exit(127);
    }

    std::cout << "[YouTube] Uruchomiono mpv, PID: " << pid << "\n";
    return true;
}

} // namespace

void youtube(const std::string& query) {
    if (query.empty()) {
        std::cerr << "[YouTube] Puste zapytanie.\n";
        return;
    }

    const char* apiKey = std::getenv("YOUTUBE_API_KEY");
    if (!apiKey || std::string(apiKey).empty()) {
        std::cerr << "[YouTube] Brak zmiennej œrodowiskowej YOUTUBE_API_KEY.\n";
        std::cerr << "[YouTube] Ustaw j¹ poleceniem: export YOUTUBE_API_KEY=...\n";
        return;
    }

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "[YouTube] Nie mo¿na uruchomiæ CURL.\n";
        return;
    }

    const std::string encodedQuery = urlEncode(curl, query);
    if (encodedQuery.empty()) {
        curl_easy_cleanup(curl);
        std::cerr << "[YouTube] Nie mo¿na zakodowaæ zapytania.\n";
        return;
    }

    const std::string url = std::string(YOUTUBE_API_URL) +
        "?part=snippet&q=" + encodedQuery +
        "&type=video&maxResults=1&order=relevance&key=" + apiKey;

    std::string response;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 20L);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "HeliosAI/1.0");

    const CURLcode result = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    if (result != CURLE_OK) {
        std::cerr << "[YouTube] B³¹d po³¹czenia: "
                  << curl_easy_strerror(result) << "\n";
        return;
    }

    try {
        const json data = json::parse(response);

        if (data.contains("error")) {
            std::cerr << "[YouTube] API zwróci³o b³¹d: "
                      << data["error"].dump() << "\n";
            return;
        }

        if (!data.contains("items") || data["items"].empty()) {
            std::cout << "[YouTube] Nie znaleziono wyniku dla: "
                      << query << "\n";
            return;
        }

        const auto& item = data["items"][0];
        const std::string videoId = item["id"]["videoId"];
        const std::string title = item["snippet"]["title"];
        const std::string channel = item["snippet"]["channelTitle"];
        const std::string videoUrl =
            "https://www.youtube.com/watch?v=" + videoId;

        std::cout << "[YouTube] Znaleziono: " << title
                  << " — " << channel << "\n";
        std::cout << "[YouTube] Odtwarzam: " << videoUrl << "\n";

        playWithMpv(videoUrl);
    } catch (const std::exception& error) {
        std::cerr << "[YouTube] B³¹d odpowiedzi JSON: "
                  << error.what() << "\n";
    }
}

} // namespace commands

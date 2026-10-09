#pragma once

// ============================================================================
//  mic.h - jeden strumień mikrofonu dla całego programu
//
//  Kluczowa zmiana względem poprzedniej wersji: nagrywanie NIE jest już
//  uruchamiane osobno dla słowa-klucza i osobno dla rozpoznawania mowy.
//  Jest JEDEN proces arecord, a ramki trafiają do:
//     - detektora słowa-klucza (stale nasłuchuje "Helios"),
//     - bufora pierścieniowego (ostatnie sekundy audio - "pre-roll"),
//     - kolektora wypowiedzi (gdy Helios już nasłuchuje).
//  Dzięki temu nie ma konfliktów o urządzenie i nie gubi się początku zdania.
// ============================================================================

#include <cstdint>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace helios_audio {

struct Config {
    // Nazwa urządzenia ALSA, np. "plughw:CARD=REF,DEV=0" (patrz: arecord -l)
    std::string device = "plughw:CARD=REF,DEV=0";
    int sampleRate = 16000;
    int channels = 1;
    int frameSamples = 1280;      // 80 ms
    // Dodatkowe opcje arecord, jeśli trzeba (np. "-D hw:1,0")
    std::string extraArgs;
};

// Bufor pierścieniowy ostatnich N sekund (16-bit PCM).
class RingBuffer {
public:
    explicit RingBuffer(double seconds = 3.0, int sampleRate = 16000);
    void push(const int16_t* samples, size_t count);
    std::vector<int16_t> last(double seconds) const;
    void clear();
    void setCapacity(double seconds);

private:
    mutable std::mutex mutex_;
    std::deque<int16_t> data_;
    size_t capacity_ = 0;
    int sampleRate_ = 16000;
};

// Strumień audio z arecord. readFrame() blokuje do zebrania pełnej ramki.
class MicStream {
public:
    explicit MicStream(const Config& config);
    ~MicStream();

    MicStream(const MicStream&) = delete;
    MicStream& operator=(const MicStream&) = delete;

    // Uruchamia arecord. Zwraca false i opis błędu, gdy się nie udało.
    bool start(std::string* error);
    void stop();
    bool running() const;

    // Czyta dokładnie frameSamples próbek (16-bit). Zwraca liczbę próbek
    // (może być mniejsza przy zamykaniu strumienia).
    size_t readFrame(int16_t* out);

    // Czeka maksymalnie timeoutMs na dane z mikrofonu i zwraca true, gdy można
    // czytać (lub gdy strumień się skończył). Dzięki temu wątek czytający nie
    // blokuje się na zawsze w fread i może sprawdzić, czy program się kończy.
    bool waitReadable(int timeoutMs);

    int frameSamples() const { return config_.frameSamples; }
    int sampleRate() const { return config_.sampleRate; }

private:
    Config config_;
    FILE* pipe_ = nullptr;
    bool running_ = false;
    std::vector<char> readBuffer_;
    size_t buffered_ = 0;
};

// Zapis/odczyt WAV 16-bit mono (nagłówki budujemy sami - nie trzeba sox/ffmpeg).
bool writeWav(const std::string& path, const std::vector<int16_t>& samples, int sampleRate);
bool readWav(const std::string& path, std::vector<int16_t>* samples, int* sampleRate);

}  // namespace helios_audio

// ============================================================================
//  mic.cpp - przechwytywanie audio z arecord (Jetson AGX Orin, ALSA)
//
//  arecord uruchamiamy z "-t raw", więc w potoku nie ma nagłówka WAV i można
//  czytać strumień ramka po ramce (1280 próbek = 80 ms) bez żadnych sztuczek.
// ============================================================================

#include "mic.h"
#include <poll.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <sstream>

namespace helios_audio {

// ---------------------------------------------------------------------------
// RingBuffer
// ---------------------------------------------------------------------------
RingBuffer::RingBuffer(double seconds, int sampleRate) : sampleRate_(sampleRate) {
    setCapacity(seconds);
}

void RingBuffer::setCapacity(double seconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    capacity_ = static_cast<size_t>(std::max(0.1, seconds) * static_cast<double>(sampleRate_));
    while (data_.size() > capacity_) {
        data_.pop_front();
    }
}

void RingBuffer::push(const int16_t* samples, size_t count) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < count; ++i) {
        data_.push_back(samples[i]);
    }
    while (data_.size() > capacity_) {
        data_.pop_front();
    }
}

std::vector<int16_t> RingBuffer::last(double seconds) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t want = static_cast<size_t>(std::max(0.0, seconds) * static_cast<double>(sampleRate_));
    const size_t n = std::min(want, data_.size());
    return std::vector<int16_t>(data_.end() - static_cast<long>(n), data_.end());
}

void RingBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    data_.clear();
}

// ---------------------------------------------------------------------------
// MicStream
// ---------------------------------------------------------------------------
MicStream::MicStream(const Config& config) : config_(config) {}

MicStream::~MicStream() {
    stop();
}

bool MicStream::start(std::string* error) {
    if (running_) {
        return true;
    }

    std::ostringstream command;
    command << "arecord -D " << config_.device
            << " -f S16_LE -r " << config_.sampleRate
            << " -c " << config_.channels
            << " -t raw";   // bez --quiet: chcemy widzieć ostrzeżenia ALSA o gubionych próbkach
    if (!config_.extraArgs.empty()) {
        command << " " << config_.extraArgs;
    }

    pipe_ = popen(command.str().c_str(), "r");
    if (pipe_ == nullptr) {
        if (error) {
            *error = "Nie udało się uruchomić arecord (" + config_.device + ")";
        }
        return false;
    }

    // arecord wypisuje błędy na stderr; pierwszy odczyt weryfikuje, czy strumień żyje.
    readBuffer_.assign(static_cast<size_t>(config_.frameSamples) * sizeof(int16_t), 0);
    buffered_ = 0;
    running_ = true;

    const size_t got = fread(readBuffer_.data() + buffered_, 1,
                             readBuffer_.size() - buffered_, pipe_);
    if (got == 0) {
        if (error) {
            *error = "arecord nie zwrócił danych - sprawdź urządzenie '" + config_.device +
                     "' (arecord -l) i czy nie jest zajęte przez inny program";
        }
        stop();
        return false;
    }
    buffered_ += got;
    return true;
}

void MicStream::stop() {
    if (pipe_ != nullptr) {
        // Zamknięcie potoku kończy proces arecord.
        pclose(pipe_);
        pipe_ = nullptr;
    }
    running_ = false;
    buffered_ = 0;
}

bool MicStream::running() const {
    return running_;
}

bool MicStream::waitReadable(int timeoutMs) {
    if (pipe_ == nullptr) {
        return false;
    }
    struct pollfd descriptor;
    descriptor.fd = fileno(pipe_);
    descriptor.events = POLLIN;
    descriptor.revents = 0;
    const int ready = poll(&descriptor, 1, timeoutMs);
    if (ready <= 0) {
        return false;
    }
    // POLLHUP oznacza koniec strumienia - zwracamy true, żeby czytający zobaczył
    // zerowy odczyt i mógł spróbować wznowić mikrofon.
    return (descriptor.revents & (POLLIN | POLLHUP | POLLERR)) != 0;
}

size_t MicStream::readFrame(int16_t* out) {
    if (!running_ || pipe_ == nullptr) {
        return 0;
    }
    const size_t frameBytes = static_cast<size_t>(config_.frameSamples) * sizeof(int16_t);

    while (buffered_ < frameBytes) {
        const size_t got = fread(readBuffer_.data() + buffered_, 1, frameBytes - buffered_, pipe_);
        if (got == 0) {
            running_ = false;
            return 0;
        }
        buffered_ += got;
    }

    std::memcpy(out, readBuffer_.data(), frameBytes);
    buffered_ = 0;
    return static_cast<size_t>(config_.frameSamples);
}

// ---------------------------------------------------------------------------
// WAV 16-bit mono
// ---------------------------------------------------------------------------
bool writeWav(const std::string& path, const std::vector<int16_t>& samples, int sampleRate) {
    std::ofstream file(path, std::ios::binary);
    if (!file) {
        return false;
    }
    const uint32_t dataBytes = static_cast<uint32_t>(samples.size() * sizeof(int16_t));
    const uint32_t byteRate = static_cast<uint32_t>(sampleRate * 2);
    const uint16_t blockAlign = 2;
    const uint16_t bitsPerSample = 16;
    const uint16_t channels = 1;
    const uint16_t audioFormat = 1;

    auto write32 = [&file](uint32_t v) {
        file.write(reinterpret_cast<const char*>(&v), 4);
    };
    auto write16 = [&file](uint16_t v) {
        file.write(reinterpret_cast<const char*>(&v), 2);
    };

    file.write("RIFF", 4);
    write32(36 + dataBytes);
    file.write("WAVE", 4);
    file.write("fmt ", 4);
    write32(16);
    write16(audioFormat);
    write16(channels);
    write32(static_cast<uint32_t>(sampleRate));
    write32(byteRate);
    write16(blockAlign);
    write16(bitsPerSample);
    file.write("data", 4);
    write32(dataBytes);
    file.write(reinterpret_cast<const char*>(samples.data()), dataBytes);
    return static_cast<bool>(file);
}

bool readWav(const std::string& path, std::vector<int16_t>* samples, int* sampleRate) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return false;
    }
    char riff[4], wave[4];
    uint32_t chunkSize = 0;
    file.read(riff, 4);
    file.read(reinterpret_cast<char*>(&chunkSize), 4);
    file.read(wave, 4);
    if (std::string(riff, 4) != "RIFF" || std::string(wave, 4) != "WAVE") {
        return false;
    }
    uint16_t channels = 1, bits = 16;
    uint32_t rate = 16000;
    while (file) {
        char id[4];
        uint32_t size = 0;
        file.read(id, 4);
        file.read(reinterpret_cast<char*>(&size), 4);
        if (!file) {
            break;
        }
        const std::string chunk(id, 4);
        if (chunk == "fmt ") {
            std::vector<char> buffer(size);
            file.read(buffer.data(), size);
            channels = *reinterpret_cast<uint16_t*>(&buffer[2]);
            rate = *reinterpret_cast<uint32_t*>(&buffer[4]);
            bits = *reinterpret_cast<uint16_t*>(&buffer[14]);
        } else if (chunk == "data") {
            if (channels != 1 || bits != 16) {
                return false;
            }
            samples->assign(size / 2, 0);
            file.read(reinterpret_cast<char*>(samples->data()), static_cast<std::streamsize>(size));
            *sampleRate = static_cast<int>(rate);
            return true;
        } else {
            file.seekg(size, std::ios::cur);
        }
    }
    return false;
}

}  // namespace helios_audio

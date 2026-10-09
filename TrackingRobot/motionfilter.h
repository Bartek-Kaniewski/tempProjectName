#pragma once

#include <cmath>

class MotionFilter {
public:
    // alpha: 0.0 (bardzo wolny, super gładki) do 1.0 (brak filtrowania, natychmiastowy)
    explicit MotionFilter(float alpha = 0.25f, float deadband = 0.8f)
        : m_alpha(alpha), m_deadband(deadband), m_initialized(false), m_filteredValue(0.0f) {}

    float filter(float rawValue) {
        if (!m_initialized) {
            m_filteredValue = rawValue;
            m_initialized = true;
            return m_filteredValue;
        }

        // Martwa strefa (deadband): ignoruj mikrodrgania poniżej np. 0.8 stopnia
        if (std::abs(rawValue - m_filteredValue) < m_deadband) {
            return m_filteredValue;
        }

        // Płynne uśrednianie: nowa_wartość = alpha * pomiar + (1 - alpha) * poprzednia
        m_filteredValue = m_alpha * rawValue + (1.0f - m_alpha) * m_filteredValue;
        return m_filteredValue;
    }

    void reset() {
        m_initialized = false;
    }

private:
    float m_alpha;
    float m_deadband;
    bool m_initialized;
    float m_filteredValue;
};
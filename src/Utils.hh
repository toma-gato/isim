#pragma once
#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

inline float cleanFloat(float value, float epsilon = 1e-5f)
{
    return (std::abs(value) < epsilon) ? 0.0f : value;
}

// Function to save framebuffer to PPM
inline void saveToPPM(const std::string& filename, const std::vector<uint8_t>& framebuffer, int width, int height)
{
    std::ofstream file(filename, std::ios::binary);
    file << "P6\n" << width << " " << height << "\n255\n";
    
    for (size_t i = 0; i < framebuffer.size(); i += 4) {
        file.write(reinterpret_cast<const char*>(&framebuffer[i]), 3);
    }
    
    file.close();
}

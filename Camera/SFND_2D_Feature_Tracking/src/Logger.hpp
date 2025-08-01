#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <numeric>
#include <opencv2/opencv.hpp>

class Logger {
public:
    Logger(const std::string& filename = "./logs/performance.csv")
    {
        logFile.open(filename, std::ios::out | std::ios::app);
        if (logFile.tellp() == 0) { // If file is empty, write header
            logFile << "Detector,Descriptor,"
                    << "AvgKeypointsTotal,AvgKeypointsOnVehicle,AvgNeighborhoodSize,"
                    << "AverageMatches,AvgDetectionTimeMs,AvgDescriptionTimeMs\n";
        }
    }

    ~Logger()
    {
        if (logFile.is_open())
            logFile.close();
    }

    void log(const std::string& detector,
             const std::string& descriptor,
             double avgKeypoints,
             double avgVehicleKeypoints,
             double avgNeighborhoodSize,
             int avgMatches,
             double avgDetTime,
             double avgDescTime)
    {
        logFile << detector << "," << descriptor << ","
                << avgKeypoints << "," << avgVehicleKeypoints << "," << avgNeighborhoodSize << ","
                << avgMatches << "," << avgDetTime << "," << avgDescTime << "\n";
    }

private:
    std::ofstream logFile;
};
